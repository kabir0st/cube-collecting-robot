# RPLIDAR Driver — Reading Guide

This folder is a walk-through of the C++ RPLIDAR driver written by **Johannes van Esch** in March 2017. It is meant to be read top-down: start with the overview, then drill into the parts you care about.

## TL;DR

The driver is a **localhost TCP gateway** for an RPLIDAR A-series laser scanner. It opens `/dev/ttyUSB0` at 115200 baud, speaks the RPLIDAR binary protocol to spin the motor and collect scan samples, and exposes those samples over TCP so other programs on the same machine can consume them. It runs as a multi-threaded daemon (main + listener + streamer threads) and ships with a makefile and a System V init script so it can be managed via `service rplidar start|stop`.

It **does not** try to be a full SDK: it has no object detection, no angle/distance decoding, no ROS integration — it just shuttles raw bytes from the serial port to a TCP client.

## Table of contents

| # | File | What's inside |
|---|------|---------------|
| 01 | [01-overview.md](01-overview.md) | What the driver does, inputs/outputs, end-to-end flow |
| 02 | [02-architecture.md](02-architecture.md) | Component diagram, thread model, object lifecycle |
| 03 | [03-rplidar-protocol.md](03-rplidar-protocol.md) | Serial hardware protocol: commands, response descriptors, scan packets |
| 04 | [04-tcp-protocol.md](04-tcp-protocol.md) | The two TCP channels: command channel (9887) and data channel (9888) |
| 05 | [05-source-walkthrough.md](05-source-walkthrough.md) | File-by-file deep dive with line references |
| 06 | [06-build-and-deploy.md](06-build-and-deploy.md) | Makefile, WiringPi dependency, initdscript daemon management |
| 07 | [07-observations.md](07-observations.md) | Hardcoded values, quirks, limitations, latent bugs |

## How to read this report

- **First-time reader?** Read 01 → 02 → 04. That gives you the mental model: what the daemon does, how it is structured, and what you would need to talk to it.
- **Just trying to understand a specific file?** Jump straight to [05-source-walkthrough.md](05-source-walkthrough.md) — every section has `file:line` links back to the source.
- **Trying to reuse / port the driver?** 03 (hardware protocol) and 07 (quirks) are the sections that will save you the most time.
- **Operating the daemon on a machine?** 06 covers `make`, `make install`, and the init script.

## Diagram rendering

All diagrams use Mermaid inside fenced code blocks. GitHub, GitLab, and the VS Code Markdown preview render them natively — no extra tooling required.

## Source map

The driver lives at [../src/](../src/). Eight source pairs make up the whole program:

```
rplidar.cpp/hpp            ← main(), command dispatcher
Lidar.cpp/hpp              ← hardware serial protocol
MapDataStreamer.cpp/hpp    ← scan acquisition + TCP streaming thread
TCPPortListener.cpp/hpp    ← command reception thread
TCPAcceptor.cpp/hpp        ← BSD server-socket wrapper
TCPConnector.cpp/hpp       ← BSD client-socket wrapper (with timeout)
TCPStream.cpp/hpp          ← read/write wrapper around a connected socket
Thread.cpp/hpp             ← pthread base class
CommandInterpreter.hpp     ← abstract interface for command dispatch
```

Everything outside `src/` is build + deploy plumbing: [../makefile](../makefile) and [../initdscript/rplidar](../initdscript/rplidar).
