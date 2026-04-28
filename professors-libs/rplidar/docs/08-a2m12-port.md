# 08 — Porting the driver from RPLIDAR A2M8 to A2M12

This chapter documents the port of the driver from the original **Slamtec RPLIDAR A2M8** to the **A2M12**. It explains what changed, why, what we left alone, and how to verify the result on real hardware. A future reader who replaces the A2M12 with an A2M13, A3, or S-series unit should read this chapter first — the structure below (compat table, verified-unchanged list, code diff, verification steps) is the pattern to repeat.

## Sources

Every factual claim in this chapter cites one of two sources so future maintainers can verify or update without re-deriving:

**[DS]** — `rplidar-a2m12/LD310_SLAMTEC_rplidar_datasheet_A2M12_v1.1_en.pdf` — official Slamtec A2M12 datasheet, document ID LD310, rev 1.1, 2026-03-12. Referenced by page number and section name.

**[CODE]** — the existing driver source tree under [src/](../src/). Referenced by `file:line`.

The original A2M8 protocol details (command codes, response descriptor layout, 5-byte scan packet fields, 0xF0 SET_MOTOR_PWM frame) come from the Slamtec "RPLIDAR Interface Protocol and Application Notes" that Johannes van Esch used in 2017. We did not re-download that PDF for this port — it is corroborated by the existing implementation at [src/Lidar.cpp:256-286](../src/Lidar.cpp#L256-L286) (now replaced) and [src/Lidar.cpp:190-216](../src/Lidar.cpp#L190-L216), which match the A2M8 spec exactly.

## A2M8 vs A2M12 — what actually differs

| Aspect | A2M8 (original target) | A2M12 (current target) | Source |
|---|---|---|---|
| Baud rate | 115200 bps | **256000 bps**, 8N1 | [DS p.13 "Data communication interface", Figure 2-8] |
| Sample rate | ~4 kHz | **16 kHz** | [DS p.9, Figure 2-1 "RPLIDAR Performance"] |
| Scan rate | 5-15 Hz (typical 10) | 5-15 Hz (typical 10) | [DS p.9, Figure 2-1] |
| Angular resolution @ 10 Hz | ~0.9° | **0.225°** | [DS p.9, Figure 2-1] |
| Range (white target) | 12 m | 12 m, 0.2 m min | [DS p.9, Figure 2-1] |
| UART logic level | 3.3V TTL | 3.3V TTL, RX control-node ≥1.6V | [DS p.13, Figure 2-8] |
| Motor control | `0xF0 SET_MOTOR_PWM` UART command, 6-byte payload with XOR checksum | **MOTOCTL pin** on XH2.54-5P pin 5 (blue wire): active-high, 3.3V logic, internal pull-down. DC high = max speed; PWM 24.5-25.5 kHz sets speed by duty cycle. **0xF0 command not documented for A2M12.** | [DS p.12, Figure 2-6 "External Interface Signal Definition"] and [DS p.13-14 "Scanner Motor Control", Figure 2-9] |
| Cable / connector | XH2.54-5P | XH2.54-5P, pins: VCC, TX, RX, GND, MOTOCTL | [DS p.11-12, Figure 2-5 and 2-6] |
| Power | 5V, ~400 mA steady | 5V (4.9-5.2V), 450 mA typical / 600 mA max, 2500 mA inrush | [DS p.12, Figure 2-7] |
| Protocol compat | — | "Supports former SDK protocols" — STANDARD_SCAN 0x20, STOP 0x25, GET_INFO 0x50, GET_HEALTH 0x52, GET_SAMPLERATE 0x59, RESET 0x40, response descriptor layout, 5-byte standard scan packet — all unchanged. | [DS p.7-8 "High Speed Sampling Protocol and Compatibility"] |
| Full 16 kHz sample rate | — | Requires the "newly extended high speed sampling protocol" — i.e. EXPRESS_SCAN 0x82 with cabin packets. Clients still speaking STANDARD_SCAN receive valid data at a reduced rate in compat mode. | [DS p.7-8] |

**Key insight.** The A2M12 is cable-compatible and protocol-compatible with the subset of commands this driver actually uses. The only two changes that *must* happen in code are (1) the baud rate and (2) removing the `0xF0 SET_MOTOR_PWM` command. Everything else is documentation cleanup.

## What stays the same (verified, do not touch)

Each item below was verified by direct Read during the port. If you're later reusing this driver for yet another RPLIDAR model, re-verify these before assuming they hold.

- **Baud rate `#define`** — [src/Lidar.hpp:15](../src/Lidar.hpp#L15) reads `#define LIDAR_SERIAL_BAUDRATE 256000`. This was updated from 115200 prior to (or as part of) this port.
- **Scan command** — [src/Lidar.cpp:246-247](../src/Lidar.cpp#L246) sends `0xA5 0x20`. A2M12 honors this in compat mode [DS p.7-8].
- **Stop command** — [src/Lidar.cpp:71-72](../src/Lidar.cpp#L71) sends `0xA5 0x25`. Unchanged.
- **5-byte scan data packet layout** — [src/Lidar.cpp:289-310](../src/Lidar.cpp#L289-L310) reads five raw bytes with zero field decoding. The encoding (`byte0 = quality<<2 | S_bit<<1 | !S_bit`, `byte1 = angle_lo<<1 | check_bit`, `byte2 = angle_hi`, `byte3..4 = distance_q2_le`) is preserved by A2M12 standard-scan compat mode — the documented data format in [DS p.7, Figure 1-4 "Sample Point Data Information"] matches the A2M8 spec field for field.
- **S-bit (new-scan flag)** — [src/MapDataStreamer.cpp:120](../src/MapDataStreamer.cpp#L120) uses `scanData.byte0 & 0x01` to detect revolution boundaries. Still valid — this is the only bit of the scan payload the driver actually examines.
- **Response descriptor layout** — [src/Lidar.cpp:190-216](../src/Lidar.cpp#L190-L216) parses the 7-byte header `A5 5A | dataLen[30 bits] | sendMode[2 bits] | dataType`. Unchanged; A2M12 uses the same descriptor format.
- **Scan buffer size** — [src/MapDataStreamer.hpp:20](../src/MapDataStreamer.hpp#L20) is `STREAMER_SCAN_BUFFER_SIZE 8192` packets. Worst case for A2M12 is 16 kHz / 5 Hz = 3200 samples/revolution — 2.5× headroom. No bump.
- **DTR clear on connect** — [src/Lidar.cpp:55-58](../src/Lidar.cpp#L55-L58) does `ioctl(TIOCMBIC, TIOCM_DTR)` after opening the port. On the Slamtec USB adapter the adapter board ties its DTR line to the MOTOCTL wire of the lidar cable, so clearing DTR drives MOTOCTL from idle-low to 3.3V high. Per [DS p.14, note under Figure 2-9] that makes "RPLIDAR A2M12 rotate at the highest speed". This is exactly the behavior we need for the fixed-10-Hz-ish use case; no further motor-speed control is required.
- **TCP framing, threading model, build system, wiringPi dependency** — unchanged. The Makefile at [../makefile](../makefile) builds identically: `g++ -std=c++11 -lpthread -lwiringPi`. No new headers.

## What changed in code

### 1. `setMotorPwm()` is now a no-op

**File**: [src/Lidar.cpp](../src/Lidar.cpp), around line 256.

**Why.** [DS p.13-14 "Scanner Motor Control"] documents motor control exclusively via the MOTOCTL pin. The 0xF0 command is not mentioned. On A2M8 0xF0 was a legitimate motor-control command that returned no response; on A2M12 it is at best silently ignored, and at worst the firmware may emit an error response descriptor that would desync the RX stream and corrupt the first few scan packets until the parser realigns. Safer to never send it.

**Before** (A2M8, ~30 lines):

```cpp
lidar_result_t Lidar::setMotorPwm(int pwm)
{
    if (_serialHandle == -1) { return LIDAR_NOT_CONNECTED_ERROR; }
    else {
        unsigned char message[LIDAR_PWM_MESSAGE_SIZE];
        if (pwm < 0) pwm = 0;
        if (pwm > LIDAR_MAX_MOTOR_PWM) pwm = LIDAR_MAX_MOTOR_PWM;
        message[0] = LIDAR_REQUEST;
        message[1] = LIDAR_SET_MOTOR_PWM_COMMAND;
        message[2] = 2;
        message[3] = pwm & 0xff;
        message[4] = (pwm & 0xff00) >> 8;
        message[5] = message[0] ^ message[1] ^ message[2] ^ message[3] ^ message[4];
        for (int i = 0; i < LIDAR_PWM_MESSAGE_SIZE; i++)
            serialPutchar(_serialHandle, message[i]);
        return LIDAR_SUCCESS;
    }
}
```

**After** (A2M12, 7 lines):

```cpp
lidar_result_t Lidar::setMotorPwm(int pwm)
{
    // A2M12 motor is controlled via the MOTOCTL pin, not the 0xF0 UART command.
    // On the Slamtec USB adapter MOTOCTL is tied to DTR, which connect() clears.
    // Kept as a no-op so A2M8-era callers still compile. See docs/08-a2m12-port.md.
    (void) pwm;
    return (_serialHandle == -1) ? LIDAR_NOT_CONNECTED_ERROR : LIDAR_SUCCESS;
}
```

The signature is preserved so the three existing call sites in [src/rplidar.cpp](../src/rplidar.cpp) — at [line 23 (START)](../src/rplidar.cpp#L23), [line 34 (STOP)](../src/rplidar.cpp#L34), and [line 65 (startup idle)](../src/rplidar.cpp#L65) — still compile unchanged. The `LIDAR_NOT_CONNECTED_ERROR` branch is preserved so callers that rely on it for connection-state detection still get the same return value contract.

### 2. DTR-clear comment rewritten

**File**: [src/Lidar.cpp:53-56](../src/Lidar.cpp#L53-L56).

**Why.** The A2M8-era comment said "For the motor to spin we have to clear the DTR line" — true but context-free. On A2M12 the physical reason is that DTR is wired to MOTOCTL on the Slamtec adapter board and clearing DTR drives MOTOCTL high. A future reader who wonders "where in this code does the motor actually get enabled?" needs that breadcrumb.

**After**:

```cpp
// DTR clear drives MOTOCTL high on the Slamtec USB adapter, spinning
// the A2M12 motor at max speed (~10 Hz). wiringPi doesn't expose DTR,
// so we do it manually. See docs/08-a2m12-port.md.
uint32_t DTRBit = TIOCM_DTR;
ioctl(_serialHandle, TIOCMBIC, &DTRBit);
```

### 3. Call-site comment in `rplidar.cpp`

**File**: [src/rplidar.cpp:21-28](../src/rplidar.cpp#L21-L28).

**Why.** The `_lidar->setMotorPwm(LIDAR_DEFAULT_MOTOR_PWM)` call in the START case is now a no-op at runtime, but deleting it would lose the historical breadcrumb that tells a reader "this is the lifecycle hook where an A2M8 would set motor speed". A one-line comment captures the intent without deleting code.

The other two call sites ([line 34](../src/rplidar.cpp#L34) and [line 65](../src/rplidar.cpp#L65)) were left alone — once the reader sees the note at the START case, the other two are self-explanatory.

## What changed in documentation

| File | Change |
|---|---|
| [docs/07-observations.md](07-observations.md) | Hardcoded constants table: baud 115200 → 256000; note "Motor PWM (660)" and "Idle PWM (0)" are now ignored at runtime; cross-reference to this chapter. |
| [docs/03-rplidar-protocol.md](03-rplidar-protocol.md) | `#define` snippet: 115200 → 256000. Command code table: mark 0xF0 as A2M8-specific / unused on A2M12. Sequence diagram: remove the two `setMotorPwm` arrows. |
| [docs/01-overview.md](01-overview.md) | "115200 baud" → "256000 baud" in both mentions. |
| [docs/README.md](README.md) | "115200 baud" → "256000 baud". |
| [docs/08-a2m12-port.md](08-a2m12-port.md) | **(this file, new)** The port guide itself. |

## Verification

End-to-end procedure on real hardware.

1. **Build.** On the target Pi:
   ```
   cd professors-libs/rplidar
   make clean && make release
   ```
   Expect: clean build, no warnings from the now-unused `(void) pwm` cast or the still-defined `LIDAR_PWM_MESSAGE_SIZE` / `LIDAR_MAX_MOTOR_PWM` / `LIDAR_SET_MOTOR_PWM_COMMAND` macros in [Lidar.hpp](../src/Lidar.hpp). If you want to clean those up later, that's optional and non-blocking.

2. **Hardware bring-up (Slamtec USB adapter path).**
   - Plug A2M12 into the adapter board, adapter into the Pi USB.
   - `ls /dev/ttyUSB*` → expect `/dev/ttyUSB0`.
   - `dmesg | tail` → expect CP210x / CH340 / FTDI probe without errors. No further kernel messages.
   - Motor should **not** be spinning yet (DTR is asserted by default at port open — adapter keeps MOTOCTL low).

3. **Run the daemon and observe the motor.**
   ```
   sudo ./bin/release/rplidar
   ```
   (or `sudo service rplidar start` if `make install` was run)
   - Within ~1 second of startup, `connect()` runs the `TIOCMBIC` ioctl and the A2M12 motor should spin up to ~10 Hz. This is audible as a soft hum and visible as rotation of the top housing.
   - **If the motor does not spin**: the assumption "Slamtec USB adapter ties DTR to MOTOCTL" is wrong for your particular adapter variant. Fix options:
     - Jumper the MOTOCTL wire (blue, pin 5 of XH2.54-5P) directly to the Pi's 3.3V rail — motor runs at max speed unconditionally, which is the same end behavior this code expects.
     - Or wire MOTOCTL to a Pi GPIO and drive it high in software (see "Fallback: bare-UART wiring" below).

4. **TCP stream sanity check.**
   - From another terminal on the Pi:
     ```
     nc localhost 9888 &
     ```
     (data channel consumer — has to be listening *before* we send START)
   - Then:
     ```
     printf '\x10\x00' | nc localhost 9887
     ```
     (sends START to the command channel; the 2 bytes are `commandId=0x10, paramLen=0x00`)
   - The `nc localhost 9888` session should start receiving data: 10-byte frames (5-byte driver header + 5-byte lidar packet). `xxd` it to eyeball.
   - Quick throughput sanity: `timeout 10 nc localhost 9888 | wc -c`. Expect tens to hundreds of KB over 10 seconds depending on what compat sample rate A2M12 emits (the datasheet does not state the compat rate numerically — see "Risks and unknowns").

5. **Revolution closure check.**
   Write a throwaway Python client that connects to port 9888, reads 10-byte chunks, and prints `packet[5] & 0x01` (that is: the first byte of the lidar packet, after the 5-byte driver header — see [04-tcp-protocol.md](04-tcp-protocol.md) for framing). Expect exactly one `1` per ~800-3200 consecutive samples — that is the S-bit fired at the start of each new 360° revolution. If the bit toggles every sample or never toggles, the compat-mode standard-scan framing is not what we think it is and you'll need to scope the UART directly.

6. **10-minute soak.**
   Leave the daemon running for 10 minutes under a typical consumer load. Watch:
   - **syslog** for garbage response descriptors or parse-alignment errors — those would be the canary that removing `0xF0` wasn't enough, or that some other command we send is mis-framed. Expect zero.
   - **Samples per revolution** — should be stable. Drift would indicate thermal, power, or inrush issues, not a driver bug.
   - **Streamer thread CPU** via `top` — should sit comfortably under 20% on a Pi 3, under 10% on a Pi 4.

7. **STOP command.**
   Send STOP:
   ```
   printf '\x20\x00' | nc localhost 9887
   ```
   Scan stream should halt within ~2 seconds. **Motor continues to spin** — expected behavior on the new port, because we no longer drive MOTOCTL down on stop (DTR stays cleared). If halting the motor on STOP is ever required, add a second `ioctl(TIOCMBIS, TIOCM_DTR)` at the end of [Lidar::stop()](../src/Lidar.cpp#L63-L80) to re-assert DTR. That change is out of scope for this port.

## Fallback: bare-UART wiring (A2M12 direct to Pi GPIO)

If a future user moves off the Slamtec USB adapter and wires the A2M12 directly to Pi GPIO14 (TXD) / GPIO15 (RXD) — for example because they want to free up a USB port or remove a layer of USB-serial flakiness — they need to handle MOTOCTL themselves. There is nothing on a bare Pi UART that corresponds to "DTR-wired-to-MOTOCTL".

**Minimum effort (fixed max speed).** Tie the blue MOTOCTL wire directly to the Pi's 3.3V pin (header pin 1 or 17). The motor will spin at max speed whenever the lidar is powered. No code changes. This is what [DS p.14, Figure 2-9 note] describes as the "DC high level signal" case.

**With variable speed control.** Wire MOTOCTL to BCM GPIO18 (header pin 12, hardware PWM capable). In `Lidar::connect()` after the `serialOpen`, add:

```cpp
wiringPiSetupGpio();
pinMode(18, PWM_OUTPUT);
pwmSetMode(PWM_MODE_MS);
pwmSetClock(24);   // 19.2 MHz / 24 = 800 kHz base
pwmSetRange(32);   // 800 kHz / 32 = 25 kHz PWM — matches [DS p.14, Figure 2-9]
```

Then reinstate `setMotorPwm()` so it writes to GPIO18:

```cpp
lidar_result_t Lidar::setMotorPwm(int pwm)
{
    if (_serialHandle == -1) return LIDAR_NOT_CONNECTED_ERROR;
    if (pwm < 0) pwm = 0;
    if (pwm > LIDAR_MAX_MOTOR_PWM) pwm = LIDAR_MAX_MOTOR_PWM;
    pwmWrite(18, (pwm * 32) / LIDAR_MAX_MOTOR_PWM);  // map 0..1023 to 0..32
    return LIDAR_SUCCESS;
}
```

This path was not implemented during the initial port because the lab uses the Slamtec adapter, but it's the only code path that would need work for the bare-UART case. Not a full rewrite — just a `Lidar.cpp` diff.

## Risks and unknowns

Carried forward from the port planning; surface these if something behaves unexpectedly.

- **Adapter MOTOCTL wiring.** We assumed the Slamtec USB adapter ties its DTR line to the MOTOCTL wire of the XH2.54-5P. This is true for the official USB-to-A2 adapter shipped with RPLIDAR A2 kits circa 2020-2023 but has not been verified against the specific adapter currently in the lab. If the assumption is wrong, step 3 of verification will fail and the 30-second fix is to jumper MOTOCTL to the Pi's 3.3V rail.
- **Compat-mode sample rate.** [DS p.7-8] says STANDARD_SCAN is backward-compatible but does not state what sample rate the A2M12 emits when a client speaks the legacy protocol. Could be the full 16 kHz, or throttled to 4-8 kHz for A2M8 parity. Either value is acceptable for the current downstream consumers — they don't depend on a specific rate, only on continuous delivery. If you need the guaranteed 16 kHz rate, you need to implement EXPRESS_SCAN 0x82 (see below).
- **0xF0 behavior on A2M12.** We're *assuming* the firmware silently ignores the 0xF0 command. If it actually responds with an error descriptor, any A2M8-era code path still sending 0xF0 would desync the RX stream. This is why step 1 of this port is "stop sending 0xF0 entirely" rather than "send it and hope". The 10-minute soak in verification step 6 is the canary for this.

## Deferred: EXPRESS_SCAN (0x82) for full 16 kHz

Not implemented in this port. If a downstream consumer later requires the full 16 kHz sample rate, you'll need to:

1. Implement [Lidar::expressScan()](../src/Lidar.cpp#L218-L232) — currently a TODO stub. Send `0xA5 0x82 0x05 0x00 0x00 0x00 0x00 0x00 <xor>` (the 9-byte express-scan request with a 5-byte payload, per the Slamtec protocol reference).
2. Read the 7-byte response descriptor — layout unchanged, `dataType` becomes `0x82`.
3. Read 84-byte **express cabin packets**: `0xA?`, `0x5?`, checksum, startAngle (u15), then 16 cabins each encoding 2 samples' angle deltas and distances. This is significantly more parsing than the 5-byte standard packet.
4. Add a new streaming mode to `MapDataStreamer` — currently it only handles `SINGLE_PACKET_NORMAL` and `COMPLETE_SCAN_NORMAL` (see [docs/07-observations.md §9](07-observations.md) for the existing stubs).
5. Decode the cabins into individual samples before forwarding, or change the TCP protocol to pass the cabin format through. The former preserves downstream compatibility; the latter is less work but requires updating every consumer.

This would be a separate port with its own chapter (say, `09-express-scan.md`). Filed under "future work, only if needed".

---

Back to [README.md](README.md).
