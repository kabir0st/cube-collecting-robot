# 07 — Quirks, hardcoded values, and limitations

This is the "read before you reuse this code" chapter. None of these are bugs so severe the driver doesn't work — it *does* work, on a Raspberry Pi plugged into an RPLIDAR, with a local client. But they're all things you'd want to know.

## Hardcoded constants

Everything that would normally be a config file or CLI flag is baked in at compile time.

| Constant | Value | Where | Notes |
|----------|-------|-------|-------|
| Serial device | `/dev/ttyUSB0` | [Lidar.hpp:14](../src/Lidar.hpp#L14) | No auto-discovery. If your RPLIDAR enumerates as `ttyUSB1`, you recompile. |
| Baud rate | `115200` | [Lidar.hpp:15](../src/Lidar.hpp#L15) | Correct for all A-series devices. |
| Default motor PWM | `660` | [Lidar.hpp:40](../src/Lidar.hpp#L40) | Out of a max of 1023. ~64% duty cycle. |
| Idle motor PWM | `0` | [Lidar.hpp:41](../src/Lidar.hpp#L41) | Motor fully off. |
| Command channel address | `127.0.0.1` | [rplidar.hpp:16](../src/rplidar.hpp#L16) | **Loopback only.** |
| Command channel port | `9887` | [rplidar.hpp:17](../src/rplidar.hpp#L17) | — |
| Data channel address | `127.0.0.1` | [rplidar.hpp:18](../src/rplidar.hpp#L18) | **Loopback only.** |
| Data channel port | `9888` | [rplidar.hpp:19](../src/rplidar.hpp#L19) | — |
| Scan buffer size | `8192` packets | [MapDataStreamer.hpp:20](../src/MapDataStreamer.hpp#L20) | ~40 KB. Enough for ~20 revolutions of an A1 at full speed. |
| Connect retries | `10` | [MapDataStreamer.hpp:18](../src/MapDataStreamer.hpp#L18) | × 10s timeout each = 100s worst case. |
| Connect timeout | `10` seconds | [MapDataStreamer.hpp:17](../src/MapDataStreamer.hpp#L17) | — |
| Retry delay | `10000` µs | [MapDataStreamer.hpp:19](../src/MapDataStreamer.hpp#L19) | 10 ms between tries. |
| Default streaming mode | `SINGLE_PACKET_NORMAL` | [rplidar.cpp:27](../src/rplidar.cpp#L27) | Hardcoded. The START command has no way to request `COMPLETE_SCAN_NORMAL`. |
| Command parameter max | `256` bytes | [CommandInterpreter.hpp:13](../src/CommandInterpreter.hpp#L13), [TCPPortListener.cpp:38](../src/TCPPortListener.cpp#L38) | Matches the 1-byte length field's max value. |
| Syslog facility | `LOG_LOCAL1` | [rplidar.cpp:54](../src/rplidar.cpp#L54) | — |

The practical consequence: this driver is meant to run on **one specific machine** (a Raspberry Pi with one RPLIDAR on `ttyUSB0`, talking to one local client). To run two lidars on one machine, or to expose the lidar over a network, you have to change constants and rebuild.

## Architectural quirks

### 1. Loopback only

Both `DEFAULT_LIDAR_SERVER_TCP_ADDRESS` and `DEFAULT_LIDAR_CLIENT_TCP_ADDRESS` are `"127.0.0.1"`. If you want to feed a lidar stream from a Pi to another machine, you need to either recompile with the right IP baked in or run an SSH tunnel / socat proxy on the side.

### 2. The data channel has inverted roles

The daemon *connects out* to the data consumer on port 9888 rather than accepting connections. This is explained in [04-tcp-protocol.md](04-tcp-protocol.md). The practical consequence is:

- **Your data consumer must start a TCP listener before sending START.** If you send START first, the daemon's retry loop will stall for up to 100 seconds waiting for you to open the port.
- **Exactly one consumer at a time.** There is no multi-subscriber model.
- **A quick client restart is fine** — the daemon will send STOP on your behalf (via TCP EOF → `send` returns ≤0 → `stopStreaming()`), but you'll lose whatever partial revolution was in the buffer.

### 3. No signal handlers

The process installs no `SIGTERM` / `SIGINT` / `SIGHUP` handlers. The "graceful shutdown" path is only reachable through the `0x99` TCP command. If someone `kill`s the process (which is what the init script does on `stop`), the driver dies mid-serial-read without sending STOP to the hardware. The RPLIDAR keeps spinning until it's power-cycled. Annoying but not dangerous.

**Workaround for clean shutdown:**
```bash
printf '\x99\x00' | nc 127.0.0.1 9887
```

### 4. Blocking reads with no timeout

Two places in the code can hang forever:

- **`serialGetchar()`** in `Lidar::readNormalScanDataPacket()` ([Lidar.cpp:299-300](../src/Lidar.cpp#L299-L300)). If the RPLIDAR stops producing data mid-scan, the streamer thread blocks in the kernel. `_running = false` will not wake it. You need a SIGKILL.
- **`TCPStream::receive(..., timeout=0)`** in `TCPPortListener::execute()` ([TCPPortListener.cpp:52, 56](../src/TCPPortListener.cpp#L52)). If a client opens a TCP connection and then never sends its 2-byte command header, the listener thread is stuck in `read()`. The `TCPStream::receive` API *supports* timeouts — they're just not used here.

A robust rewrite would pass a non-zero timeout and convert timeouts into "go check `_running`" events.

### 5. No device state machine

The `Lidar` class has no internal concept of "idle" vs "scanning" vs "expressScanning". Its only state is `_serialHandle != -1` (connected). Callers are expected to know what state the device is in and call the right methods in the right order. If you called `readNormalScanDataPacket()` without first calling `scan()`, the code would happily block waiting for bytes that will never arrive.

The `lidar_mode_t` enum ([Lidar.hpp:54-62](../src/Lidar.hpp#L54-L62)) is defined but never *stored* anywhere — it's decorative.

### 6. No locks on cross-thread state

Several member variables are written by one thread and read by another with no synchronisation:

| Variable | Writer | Reader |
|----------|--------|--------|
| `MapDataStreamer::_isStreaming` | listener thread (via `processCommand`) | streamer thread (execute loop) |
| `MapDataStreamer::_lidar` | listener thread | streamer thread |
| `MapDataStreamer::_tcpStream` | listener thread and streamer thread | both |
| `MapDataStreamer::_streamingMode` | listener thread | streamer thread |
| `TCPPortListener::_isListening` | `rplidar::processCommand` (listener thread) | listener thread itself |
| `Thread::_running` | `rplidar::processCommand` (listener) and `rplidar::run` (main) | the owning thread |

None of them are `std::atomic<T>`, none are protected by a mutex. This works on x86/ARM because:

- A `bool` store/load is a single machine instruction and atomic on both platforms.
- The listener thread's write to `_isStreaming = true` is followed by a syscall (`write` to the lidar for `scan()`), which acts as a memory fence.
- The streamer thread's check of `_isStreaming` is followed by a read (`serialGetchar`), ditto.

But it is **not technically correct** by the C++ memory model, and a smart compiler is allowed to cache `_running` in a register across the loop body (though in practice the presence of syscalls in the body prevents this). A five-minute fix would be `std::atomic<bool>` for all of these.

### 7. The `command_t` struct is read directly from the wire

```cpp
_tcpStream->receive((unsigned char *) &command, 2, 0);
```

— [TCPPortListener.cpp:52](../src/TCPPortListener.cpp#L52)

This only works because:
1. `command_t` contains exactly two `unsigned char` fields.
2. There is no compiler-inserted padding between them.
3. Endianness doesn't matter for single bytes.

If someone ever added a `short` field, or changed the first field to `uint16_t`, the binary protocol would silently break. A robust alternative would be to read into a local `unsigned char[2]` and decode byte-by-byte.

### 8. `TCPStream::receive()` does not loop on short reads

```cpp
ssize_t TCPStream::receive(unsigned char* buffer, size_t len, int timeout)
{
    if (timeout <= 0) return read(m_sd, buffer, len);
    ...
}
```

— [TCPStream.cpp:39-49](../src/TCPStream.cpp#L39-L49)

`read()` can return fewer bytes than requested. The listener does not re-try for the missing bytes. On loopback with 2-byte payloads, the kernel delivers in a single call ~100% of the time, but it's not guaranteed. A correctness-minded rewrite would add a `readExact()` helper.

### 9. Unimplemented express scan path

`SINGLE_PACKET_EXPRESS` and `COMPLETE_SCAN_EXPRESS` are valid enum values for the streaming mode ([MapDataStreamer.hpp:38-40](../src/MapDataStreamer.hpp#L38-L40)), but there is no code path to handle them:

- `MapDataStreamer::execute()` only switches on `SINGLE_PACKET_NORMAL` and `COMPLETE_SCAN_NORMAL`, and falls through to `stopStreaming()` for anything else ([MapDataStreamer.cpp:70-72](../src/MapDataStreamer.cpp#L70-L72)).
- `Lidar::expressScan()` is a TODO stub ([Lidar.cpp:218-232](../src/Lidar.cpp#L218-L232)).
- `LIDAR_EXPRESS_SCAN_COMMAND` (0x82) is defined but never sent.

So if you passed `SINGLE_PACKET_EXPRESS` to `startStreaming()`, the streamer would connect to the client, immediately hit the `default:` case, stop, and you'd see a socket open-then-close with no data. Confusing to debug.

### 10. Unused info/health/sample-rate paths

`getInfo()`, `getHealth()`, `getSampleRate()` are all implemented in `Lidar.cpp` ([lines 103-187](../src/Lidar.cpp#L103-L187)), and the data-channel protocol has dedicated content types for them (`LIDAR_INFO_PACKET`, `LIDAR_HEALTH_PACKET`, `LIDAT_SAMPLE_RATE_PACKET` — note the typo in the third), but nothing ever calls them. They are dead code from the running daemon's point of view.

This is actually the most interesting architectural gap: a fourth TCP command (say, `0x30 GET_HEALTH`) that called `_lidar->getHealth()` and replied on the command connection would be an easy addition and would make the daemon much more useful.

### 11. Manual `new` / `delete` everywhere

No `std::unique_ptr`, no `std::shared_ptr`, no RAII for the allocated objects in `rplidar::run()`. The process doesn't `delete` them on shutdown — it relies on process exit to reclaim memory. Harmless, but inconsistent (the destructor for `MapDataStreamer` does `free(_scanBuffer)`, so *some* cleanup is taken seriously).

## Latent bugs

These are actual small bugs in the code, not just design choices.

### Bug 1: uninitialised `sendBytes` in `streamCompleteScanNormal`

[src/MapDataStreamer.cpp:110-146](../src/MapDataStreamer.cpp#L110-L146):

```cpp
void MapDataStreamer::streamCompleteScanNormal()
{
    lidar_normal_scan_data_packet_t scanData;
    int sendBytes;            // ← uninitialised
    int dataSize;

    _lidar->readNormalScanDataPacket(&scanData);

    if (scanData.byte0 & 0x01)
    {
        if (_scanBufferIndex > 0)
        {
            dataSize = sizeof(scanData) * _scanBufferIndex;

            if (sendHeader(LIDAR_COMPLETE_SCAN_PACKET, dataSize) == HEADER_PACKET_LENGTH)
                sendBytes = _tcpStream->send((unsigned char *) _scanBuffer, dataSize);
            // ← if sendHeader returned short, sendBytes is still undefined

            if (sendBytes <= 0)          // ← potentially reading garbage
            {
                stopStreaming();
            }
            // ...
```

If `sendHeader()` returns anything other than `HEADER_PACKET_LENGTH` (i.e. a short TCP write on the header itself), `sendBytes` keeps whatever garbage value was on the stack. The `if (sendBytes <= 0)` check then has a 50/50 chance of incorrectly stopping or continuing. On loopback with 5-byte writes this never actually happens, which is why nobody has noticed.

**Fix:** `int sendBytes = 0;` at declaration.

### Bug 2: `init.d` status pulls from a potentially stale pidfile

[initdscript/rplidar:163-177](../initdscript/rplidar#L163-L177). If someone manually edits `/var/run/rplidar/rplidar.pid` to contain a PID of a different process, `status` will report that unrelated process as "rplidar running". The script's own TODO comments acknowledge this.

### Bug 3: `getInfo()` does not read the response descriptor

[src/Lidar.cpp:103-126](../src/Lidar.cpp#L103-L126). Unlike `getHealth()` and `getSampleRate()`, which read the 7-byte descriptor before reading the payload, `getInfo()` just drains the port into a static buffer. This means the first 7 bytes returned are actually the descriptor, not the info payload. Since the method is never called at runtime, nobody has noticed.

### Bug 4: Init script's enum typo

`LIDAT_SAMPLE_RATE_PACKET` in [MapDataStreamer.hpp:31](../src/MapDataStreamer.hpp#L31). Not a bug in behaviour, but a typo frozen into an enum name. (`LIDAT` should be `LIDAR`.)

## What would I change if reusing this?

Ranked by effort vs. impact:

1. **Make ports and device path configurable** via a config file or CLI args. 1-hour fix, removes the main reusability limitation.
2. **Add non-zero timeouts to all blocking reads.** 30 minutes, eliminates the hang-forever class of failures.
3. **Wrap cross-thread state in `std::atomic`.** 15 minutes, makes the code correct under any compiler.
4. **Install a SIGTERM handler that sends `0x99` to itself** (or just calls the shutdown path directly). 15 minutes, makes `systemctl stop` actually clean up the motor.
5. **Implement GET_HEALTH as a command**, even if it just logs. The hardware method already exists. 30 minutes, adds real observability.
6. **Initialise `sendBytes`** in `streamCompleteScanNormal`. 10 seconds.

None of these are deal-breakers for a teaching/research driver. They're just the rough edges you'd want to sand off before deploying this on anything you care about.

---

That's the report. Back to [README.md](README.md).
