# 02 — Lidar Protocol & TCP Plumbing

The RPLIDAR A1 is driven by a vendor binary that exposes two TCP endpoints
on `127.0.0.1`. We don't talk to the device directly. Both ports use TCP.

| Port  | Direction              | Owner   | Purpose                                       |
|-------|------------------------|---------|-----------------------------------------------|
| 9887  | our process → driver   | driver  | one-byte control signal (start / stop)         |
| 9888  | driver → our process   | us      | continuous stream of scan packets              |

Constants live in [include/types.hpp](../include/types.hpp) as
`CONTROL_PORT` and `LIDAR_STREAM_PORT`.

## Control protocol (port 9887)

[`send_control_signal()`](../src/utils.cpp) opens a fresh TCP connection,
sends two bytes, closes:

```
   byte 0:  0x10  (start)   or   0x20  (stop)
   byte 1:  0x00            (reserved / null)
```

Used by `LidarDataServer::start_lidar()` and `stop_lidar()`. There is no
ack — we just send and trust. If the connect fails we log and return -1;
the calling code decides what to do.

## Stream protocol (port 9888)

The driver opens a connection to us and pushes "express scan" packets back
to back until told to stop. Each packet has a 5-byte header followed by a
variable-length payload.

```
header (5 bytes):
   0      0xA5         start byte; we ignore packets with any other value
   1      flags        bit 0 = 1 means "I'm sending you a frame"
   2..4   size_be      24-bit big-endian payload length

payload[0]:    quality << 2 | flags
payload[1..2]: angle_q6     (little-endian, 1/64 degree fixed point)
payload[3..4]: distance_q2  (little-endian, 1/4 mm fixed point)
```

We decode each packet to a tuple `(quality, angle_deg, distance_mm)` in
[`LidarDataServer::start_stream`](../src/server.cpp):

```c++
const int quality   = (unsigned char)data[0] >> 2;
const int angle_q6  = ((unsigned char)data[1] >> 1)
                    | ((unsigned char)data[2] << 7);
const int angle_deg = (angle_q6 >> 6) % 360;
const int dist_mm   = (((unsigned char)data[3])
                     | ((unsigned char)data[4] << 8)) >> 2;
```

Notes:

- `quality > 0` means the driver believes the return is real. Anything
  else is dropped at the consumer (`process_point` checks both quality
  and distance > 0).
- We coarsen the angle from q6 (1/64 deg ≈ 0.015°) to integer degrees
  (1°). For our 1 Hz Cox matcher this is more than precise enough — the
  residual noise floor is dominated by `σ_r` (mm), not angular bins.
  If you ever need sub-degree precision (long-range narrow corridors),
  store `angle_q6` directly in `LidarDataFrame` and convert to radians
  with `(angle_q6 / 64.0f) * (π/180)`.
- Distance is straightforward: q2 fixed point in mm. The shift-by-2
  yields raw mm. For RPLIDAR A1 the working range is 150–6000 mm; we
  drop anything < 150 mm as near-zone noise during map building.

## Server lifecycle

[`LidarDataServer`](../include/server.hpp) is a small state machine driven
by atomic flags:

```
   listener_ready      true while accept() can be reached
   recv_active         true while the recv() loop is touching shared state
   stream_server_fd    listening socket (or -1)
   stream_client_fd    accepted connection (or -1)
```

Sequence per phase:

```
[main]                           [listener thread]
ListenerThread(handler, server)
                                 start_stream()
                                   bind/listen
                                   listener_ready = true
server.is_ready() == true
server.start_lidar()
   send_control_signal(1)
                                   accept() returns
                                   recv_active = true
                                   ... recv loop ...
                                   process_point per packet
                                   ...
sleep 5 s
server.stop_stream()
   send_control_signal(0)
   close(client); close(server)
                                   recv() returns 0 (EOF)
                                   listener_ready = false
                                   recv_active = false
   wait recv_active==false
   (listener has stopped writing
    to scan_collection)
calibrate()           ◄── safe to read
~ListenerThread        joins listener
```

The `recv_active`-spin in `stop_stream` is the synchronization point. When
it returns, the listener thread is no longer inside `process_point`. The
scan-collection mutex is a defence-in-depth: even if `recv_active`'s
two-instruction window doesn't quite line up on some platform, the mutex
makes the consumer safe.

## Why TCP and not USB direct?

The vendor driver handles the USB FTDI quirks, baud rate, and the
RPLIDAR start-spin-then-scan handshake. It also lets us run the lidar
driver as a separate process under its own permission set. From our
side it's just `recv()`.

If you want to skip the driver, replace `LidarDataServer` with a class
that opens `/dev/ttyUSB0` and parses the same payload format directly.
Everything downstream of `process_point` is agnostic to the source.

## Replay / unit testing

There is no replay path right now. `lidar_data.txt` is a captured scan
log used for ad hoc inspection. To unit-test the matcher offline you'd
write a small adapter that reads that file and calls
`PositionEstimator::process_point` at lidar-realistic timing — the
estimator doesn't care that the data came over TCP.
