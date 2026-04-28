# 05 — Source walkthrough

A file-by-file reading of the driver. Each section: what the file is for, what's in it, and the line references you need to follow along. Use this like a reference — you don't have to read it in order.

---

## `rplidar.cpp` / `rplidar.hpp`

**Purpose:** the application entry point. Owns the three major objects (`Lidar`, `TCPPortListener`, `MapDataStreamer`), wires them together, and implements the `CommandInterpreter` interface so it can receive decoded commands from the listener thread.

### Header — [src/rplidar.hpp](../src/rplidar.hpp)

```cpp
class rplidar: public virtual CommandInterpreter
{
    private:
        Lidar *_lidar;
        TCPPortListener *_tcpPortListener;
        MapDataStreamer *_mapDataStreamer;

    public:
        int run();
        void processCommand(command_t *command, unsigned char *commandParameters);
};
```

— [src/rplidar.hpp:29-42](../src/rplidar.hpp#L29-L42)

Three owned pointers, two public methods. That is the entire class. The macros above it define the three TCP endpoints and the three command IDs (see [04-tcp-protocol.md](04-tcp-protocol.md)).

### `main()` — [src/rplidar.cpp:90-95](../src/rplidar.cpp#L90-L95)

```cpp
int main(int argc, char **argv)
{
    hh::rplidar *lidar = new hh::rplidar();
    return lidar->run();
}
```

That's it. No argument parsing, no config file, no CLI help. The daemon is configured purely through compile-time constants. The `new` is never matched with a `delete` — the process exits right after `run()` returns, so the leak is harmless.

### `run()` — [src/rplidar.cpp:51-86](../src/rplidar.cpp#L51-L86)

Step-by-step:

1. **`openlog(...)`** ([line 54](../src/rplidar.cpp#L54)) — route all logs to syslog facility `LOCAL1` with the `rplidar` tag.
2. **Allocate the three objects** ([lines 58-60](../src/rplidar.cpp#L58-L60)). Note they're allocated with raw `new` and never `delete`d; relies on process exit to reclaim memory.
3. **Bring the hardware to idle** ([lines 63-65](../src/rplidar.cpp#L63-L65)): `connect()`, `stop()`, `setMotorPwm(LIDAR_IDLE_MOTOR_PWM)`. This ensures the driver starts in a known state even if the previous run left the motor spinning.
4. **Start the two threads** ([lines 68-69](../src/rplidar.cpp#L68-L69)). `Thread::start()` calls `pthread_create()` and returns immediately. Neither thread is doing real work yet — the listener is waiting for `_isListening` to flip true, and the streamer is waiting for `_isStreaming` to flip true.
5. **`startListening()`** ([line 72](../src/rplidar.cpp#L72)) — this does NOT block. It calls `TCPAcceptor::start()` (bind + listen) and sets `_isListening = true`, then returns. From this moment on, the listener thread is actively polling for new connections.
6. **`tcpPortListener->join()`** ([line 75](../src/rplidar.cpp#L75)) — **this** is the call that parks the main thread. It waits until the listener thread's `execute()` loop exits.
7. **Cleanup** ([lines 78-83](../src/rplidar.cpp#L78-L83)) — stop the streamer, join it, close syslog.

### `processCommand()` — [src/rplidar.cpp:15-49](../src/rplidar.cpp#L15-L49)

This is the callback hit by the listener thread whenever a command arrives.

```cpp
void rplidar::processCommand(command_t *command, unsigned char *commandParameters)
{
    syslog(LOG_NOTICE, "received command 0x%02x", command->commandId);

    switch (command->commandId)
    {
        case LIDAR_COMMAND_START:
            _lidar->setMotorPwm(LIDAR_DEFAULT_MOTOR_PWM);
            _mapDataStreamer->startStreaming(_lidar,
                                             DEFAULT_LIDAR_CLIENT_TCP_ADDRESS,
                                             DEFAULT_LIDAR_CLIENT_TCP_PORT,
                                             SINGLE_PACKET_NORMAL);
            break;

        case LIDAR_COMMAND_STOP:
            _mapDataStreamer->stopStreaming();
            _lidar->stop();
            _lidar->setMotorPwm(LIDAR_IDLE_MOTOR_PWM);
            break;

        case LIDAR_SERVER_TERMINATE:
            _tcpPortListener->stopListening();
            _tcpPortListener->stop();
            break;

        default:
            break;
    }
}
```

Things to notice:

- **Runs on the listener thread**, not on the main thread. This is the cross-thread boundary where commands reach back into `rplidar`'s owned objects.
- **`SINGLE_PACKET_NORMAL` is hardcoded.** There is no way to request `COMPLETE_SCAN_NORMAL` mode through the TCP protocol as it stands — you'd need to either add a command parameter or change this literal.
- **STOP order matters.** First stop streaming (so the streamer stops reading from the serial port), then send STOP to the hardware, then drop motor PWM. If you swapped the first two, the streamer could read partial data while the lidar was mid-stop.
- **`LIDAR_SERVER_TERMINATE` calls both `stopListening()` and `stop()`.** The first tears down the `TCPAcceptor`; the second sets `_running = false`. Together they unblock `join()` in `run()`.
- **`commandParameters` is ignored** by all three handlers. The parameter buffer exists in the protocol but no command currently uses it.

---

## `Lidar.cpp` / `Lidar.hpp`

**Purpose:** the only file that knows about serial I/O and the RPLIDAR wire format. Everything above this layer just calls methods on a `Lidar*`.

The full wire format is covered in [03-rplidar-protocol.md](03-rplidar-protocol.md) — this section focuses on the C++ surface area.

### Class shape — [src/Lidar.hpp:103-130](../src/Lidar.hpp#L103-L130)

```cpp
class Lidar
{
    private:
        int _serialHandle;
        lidar_result_t readResponseDescriptor(lidar_response_descriptor_t *response);

    public:
        Lidar();
        virtual ~Lidar();

        lidar_result_t connect();
        lidar_result_t connect(const std::string device);
        lidar_result_t stop();
        lidar_result_t reset();
        unsigned char *getInfo();
        lidar_result_t getHealth(lidar_health_t *health);
        lidar_result_t getSampleRate(lidar_samplerate_t *sampleRate);
        lidar_result_t scan();
        lidar_result_t expressScan();
        lidar_result_t setMotorPwm(int pwm);
        lidar_result_t readNormalScanDataPacket(lidar_normal_scan_data_packet_t *scanData);
};
```

The only state is `_serialHandle` — the WiringPi file descriptor returned by `serialOpen()`. It is -1 when disconnected; every method guards against that. The destructor is a no-op, so the serial port leaks if the object is destroyed, but since `rplidar::run()` never destroys it, this never happens.

### Methods actually used by the running daemon

| Method | Called from | What it does |
|--------|-------------|--------------|
| `connect()` | `rplidar::run()` startup | Opens `/dev/ttyUSB0`, clears DTR. |
| `stop()` | startup + STOP handler | Sends `0xA5 0x25`, waits 2ms, flushes. |
| `setMotorPwm(int)` | startup (PWM=0) + START (660) + STOP (0) | Builds 6-byte command with XOR checksum. |
| `scan()` | `MapDataStreamer::startStreaming()` | Sends `0xA5 0x20`, reads+discards response descriptor. |
| `readNormalScanDataPacket(...)` | Streamer's hot loop | Blocks on `serialGetchar` five times, copies bytes into struct. |

### Methods that exist but are never called

`reset()`, `getInfo()`, `getHealth()`, `getSampleRate()`, `expressScan()` — all dead code from the point of view of the running daemon. They're worth reading anyway if you want to understand how response descriptors work, since `getHealth()` and `getSampleRate()` are the only places `readResponseDescriptor()` is used productively. `expressScan()` ([src/Lidar.cpp:218-232](../src/Lidar.cpp#L218-L232)) is literally:

```cpp
lidar_result_t Lidar::expressScan()
{
    if (_serialHandle == -1) return LIDAR_NOT_CONNECTED_ERROR;
    else {
        // TODO
        return LIDAR_SUCCESS;
    }
}
```

So the `SINGLE_PACKET_EXPRESS` / `COMPLETE_SCAN_EXPRESS` streaming modes defined in [src/MapDataStreamer.hpp:38-40](../src/MapDataStreamer.hpp#L38-L40) are enum values for functionality that does not exist.

### The response descriptor parser — [src/Lidar.cpp:190-216](../src/Lidar.cpp#L190-L216)

Worth reading once carefully — it's the one place the code has to untangle a packed bitfield. The tricky line is:

```cpp
response->dataLength = (receivedByte[5] >> 2) << 24 |
                       receivedByte[4] << 16 |
                       receivedByte[3] << 8 |
                       receivedByte[2];
```

This is a 30-bit little-endian length where the top 2 bits of the high byte have been stolen for a `sendMode` field. The `>> 2) << 24` pair first drops the stolen bits, then shifts the remaining 6 into position 24–29. `sendMode` itself is extracted as `receivedByte[5] & 0x03`.

### `readNormalScanDataPacket` — [src/Lidar.cpp:289-310](../src/Lidar.cpp#L289-L310)

```cpp
for (int i = 0; i < LIDAR_NORMAL_SCAN_DATA_RESPONSE_SIZE; i++)
    receivedByte[i] = serialGetchar(_serialHandle);

scanData->byte0 = receivedByte[0];
// ... byte1..byte4 ...
```

Five blocking reads, one per byte. Copy into the output struct, return. No attempt to verify sync or reframe if alignment is lost. If a byte is dropped from the UART, the driver will be out of sync for the rest of eternity (until someone sends STOP + START). This is fine in practice because USB serial is reliable at 115200.

---

## `MapDataStreamer.cpp` / `MapDataStreamer.hpp`

**Purpose:** owns the streamer thread. Pulls packets out of the `Lidar` object and pushes framed data to a TCP socket. Supports two buffering modes.

### State — [src/MapDataStreamer.hpp:54-65](../src/MapDataStreamer.hpp#L54-L65)

```cpp
bool _isStreaming;
Lidar *_lidar;
std::string *_tcpStreamAddress;
int _tcpStreamPort;
streaming_mode_t _streamingMode;

libcpp::TCPStream *_tcpStream;

lidar_normal_scan_data_packet_t *_scanBuffer;
unsigned int _scanBufferIndex;

bool _streamingError;
```

The 8192-packet `_scanBuffer` is allocated in the constructor ([src/MapDataStreamer.cpp:25](../src/MapDataStreamer.cpp#L25)) and freed in the destructor. That's ~40KB for `COMPLETE_SCAN_NORMAL` mode. In `SINGLE_PACKET_NORMAL` mode it is allocated but unused.

### `execute()` — [src/MapDataStreamer.cpp:48-89](../src/MapDataStreamer.cpp#L48-L89)

The main thread loop:

```cpp
while (_running)
{
    if (_isStreaming)
    {
        switch (_streamingMode)
        {
            case SINGLE_PACKET_NORMAL:  streamSinglePacketNormal();  break;
            case COMPLETE_SCAN_NORMAL:  streamCompleteScanNormal();  break;
            default:                    stopStreaming();             break;
        }
    }
    else
    {
        if (_tcpStream != NULL) { delete _tcpStream; _tcpStream = NULL; }
        _scanBufferIndex = 0;
        usleep(100);
    }
}
```

The idle branch is responsible for **closing the TCP socket when streaming stops** (via the `delete _tcpStream`), which is the side-channel by which the client learns streaming is over — EOF on the data socket. There is no explicit goodbye packet.

### `streamSinglePacketNormal()` — [src/MapDataStreamer.cpp:92-107](../src/MapDataStreamer.cpp#L92-L107)

Read one 5-byte packet, prepend a 5-byte header, send. If `send` returns ≤0 (TCP error), call `stopStreaming()` which will cause the next iteration of the main loop to hit the idle branch and clean up the socket.

### `streamCompleteScanNormal()` — [src/MapDataStreamer.cpp:110-146](../src/MapDataStreamer.cpp#L110-L146)

Read one 5-byte packet. If its start-of-revolution bit is set and the buffer has accumulated samples, emit one big header+payload covering the whole previous revolution and reset the buffer. Then store the current packet in the buffer.

```cpp
if (scanData.byte0 & 0x01)
{
    if (_scanBufferIndex > 0)
    {
        dataSize = sizeof(scanData) * _scanBufferIndex;

        if (sendHeader(LIDAR_COMPLETE_SCAN_PACKET, dataSize) == HEADER_PACKET_LENGTH)
            sendBytes = _tcpStream->send((unsigned char *) _scanBuffer, dataSize);

        if (sendBytes <= 0)
            stopStreaming();

        _scanBufferIndex = 0;
    }
}

_scanBuffer[_scanBufferIndex] = scanData;

if (_scanBufferIndex < (STREAMER_SCAN_BUFFER_SIZE - 1))
    _scanBufferIndex++;
```

**Latent bug**: `sendBytes` is declared but not initialised at the top of the function. The `if (sendBytes <= 0)` check is only reached when `(byte0 & 0x01) && _scanBufferIndex > 0`, and in that case `sendBytes` is assigned first — *except* when `sendHeader()` itself returns ≠ `HEADER_PACKET_LENGTH`, in which case `sendBytes` keeps whatever stack garbage it had and the check is wrong. Harmless in practice because a short TCP write on a healthy loopback socket never happens, but it is a real undefined-behaviour read. Flagged again in [07-observations.md](07-observations.md).

### `startStreaming()` — [src/MapDataStreamer.cpp:149-186](../src/MapDataStreamer.cpp#L149-L186)

Runs on the **listener thread** (it's called from `rplidar::processCommand`). This is the cross-thread set-up:

1. Allocate a fresh `TCPConnector`.
2. Store config (lidar pointer, address, port, mode) into member variables.
3. Loop up to `STREAMER_CONNECT_TRIES` (10) times, each with a `STREAMER_CONNECT_TIMEOUT_SECONDS` (10s) timeout, sleeping `STREAMER_WAIT_BETWEEN_CONNECT_RETRIES_US` (10ms) between attempts. Log each failed try.
4. Delete the connector.
5. If no stream after all retries: set `_streamingError = true`, `_isStreaming = false`. Silent failure — the START command that triggered this already got a successful `return` at the TCP-command level.
6. Otherwise: clear the error flag, set `_isStreaming = true`, and call `_lidar->scan()` to put the hardware in scan mode. **The streamer thread will see the flag change on its next loop iteration and start pumping data.**

### `sendHeader()` — [src/MapDataStreamer.cpp:197-211](../src/MapDataStreamer.cpp#L197-L211)

Constructs the 5-byte frame header and sends it. Returns whatever `TCPStream::send` returned (typically 5 on success). See [04-tcp-protocol.md](04-tcp-protocol.md) for the header layout.

---

## `TCPPortListener.cpp` / `TCPPortListener.hpp`

**Purpose:** the command-channel thread. Accepts connections, reads one command per connection, dispatches via `CommandInterpreter`, closes.

### `execute()` — [src/TCPPortListener.cpp:35-75](../src/TCPPortListener.cpp#L35-L75)

```cpp
while (_running)
{
    if (_isListening)
    {
        if (_tcpAcceptor->acceptPending())
            _tcpStream = _tcpAcceptor->accept();
        else
            _tcpStream = NULL;

        if (_tcpStream != NULL)
        {
            _tcpStream->receive((unsigned char *) &command, 2, 0);

            if (command.commandParameterLength > 0)
                _tcpStream->receive((unsigned char *) commandParameters,
                                    command.commandParameterLength, 0);

            _commandInterpreter->processCommand(&command, commandParameters);

            delete _tcpStream;
        }
        else
            usleep(100);
    }
    else
        usleep(100);
}
```

A few subtleties:

- **`acceptPending()` is a polled `select()` with a 1ms timeout**, not a blocking `accept()`. This is how the loop remains responsive to `_running` changes — it wakes up every ~1ms to re-check.
- **The receive calls have `timeout=0`**, which in `TCPStream::receive()` ([src/TCPStream.cpp:39-49](../src/TCPStream.cpp#L39-L49)) means "blocking read, no timeout". So if a client connects and then never sends its 2 bytes, the listener thread is stuck in that `recv()` — it can't even notice a shutdown signal. This is another instance of the blocking-read hazard (see [07-observations.md](07-observations.md)).
- **One command per connection.** The `delete _tcpStream` at the bottom of the `if` closes the socket before the next loop iteration.
- **Command parameters reuse a stack buffer** of 256 bytes ([src/TCPPortListener.cpp:38](../src/TCPPortListener.cpp#L38)). Since `commandParameterLength` is a `uint8_t`, overflow is impossible.

### `startListening()` / `stopListening()` — [src/TCPPortListener.cpp:78-105](../src/TCPPortListener.cpp#L78-L105)

`startListening()`:
1. If already listening, stop first.
2. Store the port + command interpreter.
3. `new TCPAcceptor(port, address)`.
4. `_tcpAcceptor->start()` — this does the `socket()`/`bind()`/`listen()` dance.
5. Set `_isListening = true`.
6. Return. **Does not block.** The actual accept loop is the already-running thread's `execute()`.

`stopListening()`:
1. Set `_isListening = false`.
2. `_tcpAcceptor->stop()` (closes the listening socket).
3. `delete _tcpAcceptor`.

Note these manipulate `_isListening`, `_tcpAcceptor` without locks. Safe here because they are only ever called from `rplidar::processCommand()` which runs on the same listener thread — there is no actual cross-thread contention.

---

## `TCPAcceptor.cpp` / `TCPAcceptor.hpp`

**Purpose:** a thin RAII-style wrapper around the BSD server-socket dance (`socket` → `bind` → `listen` → `accept`). Attribution comment in the header says "Based on code from Vic Hargrave".

Key points from [src/TCPAcceptor.cpp](../src/TCPAcceptor.cpp):

- **`start()`** ([lines 39-83](../src/TCPAcceptor.cpp#L39-L83)) — creates the socket with `PF_INET`, sets `SO_REUSEADDR`, binds (to the given address or `INADDR_ANY` if empty), listens with a backlog of 5.
- **`acceptPending()`** ([lines 86-103](../src/TCPAcceptor.cpp#L86-L103)) — `select()` with a 1ms timeout. This is the polling mechanism that keeps `TCPPortListener`'s loop responsive.
- **`accept()`** ([lines 112-133](../src/TCPAcceptor.cpp#L112-L133)) — blocking `::accept()`, returns a newly-allocated `TCPStream*` on success or `NULL` on error. The caller owns the returned stream.

The destructor closes the listen socket but doesn't null out `m_lsd`, which is harmless as long as the destructor only runs once (it does).

---

## `TCPConnector.cpp` / `TCPConnector.hpp`

**Purpose:** the client-side counterpart to `TCPAcceptor`. Also "Based on code from Vic Hargrave".

Two `connect()` overloads:

### `connect(server, port)` — [src/TCPConnector.cpp:32-65](../src/TCPConnector.cpp#L32-L65)

Blocking connect, no timeout. Hostname → `in_addr` via `getaddrinfo` (with `inet_pton` fallback for dotted-quad literals), then `::connect()`.

### `connect(server, port, timeout)` — [src/TCPConnector.cpp:68-157](../src/TCPConnector.cpp#L68-L157)

The longer path, used by `MapDataStreamer`. Uses the classic non-blocking-connect-with-select pattern:

1. `socket()`, set `O_NONBLOCK`.
2. `::connect()` — expected to return -1 with `errno == EINPROGRESS`.
3. `select()` waits up to `timeout` seconds for the socket to become writable.
4. Check `SO_ERROR` via `getsockopt` — this is the idiomatic way to learn whether the connect actually succeeded after a non-blocking call.
5. Return the socket to blocking mode.
6. Wrap in a `TCPStream*`, or return `NULL` on failure/timeout.

A minor quirk: the blocking flag is restored to blocking mode even on failure paths, just before the `if (result == -1) return NULL;` check. Fine, but means the failed socket momentarily leaks into blocking state before being discarded. No practical impact.

---

## `TCPStream.cpp` / `TCPStream.hpp`

**Purpose:** a thin wrapper around a connected socket. Implements `send`, `receive`, `getPeerIP`, `getPeerPort`. Closes the fd in its destructor.

### Interesting property: private constructors

```cpp
private:
    TCPStream(int sd, struct sockaddr_in* address);
    TCPStream();
    TCPStream(const TCPStream& stream);

public:
    friend class TCPAcceptor;
    friend class TCPConnector;
```

— [src/TCPStream.hpp:33-40](../src/TCPStream.hpp#L33-L40)

You **cannot** create a `TCPStream` directly. Only `TCPAcceptor` and `TCPConnector` can. This enforces the rule "streams are produced by one of the two socket factories" at the type system level. The copy constructor is also private — this prevents accidental double-close on destruction.

### `receive()` — [src/TCPStream.cpp:39-49](../src/TCPStream.cpp#L39-L49)

```cpp
ssize_t TCPStream::receive(unsigned char* buffer, size_t len, int timeout)
{
    if (timeout <= 0) return read(m_sd, buffer, len);

    if (waitForReadEvent(timeout) == true)
        return read(m_sd, buffer, len);

    return connectionTimedOut;
}
```

Important subtlety: **`read()` can return fewer bytes than requested**. `TCPStream::receive` does not loop to drain — a short read is just returned as-is. The caller (`TCPPortListener::execute`) doesn't handle that case either, which means a pathological client that sends the 2-byte command header in two separate TCP segments could cause the listener to read only 1 byte and then call `processCommand` on garbage. In practice, on loopback with tiny payloads, the kernel almost always delivers both bytes in a single `read()`. This is another "accidentally works" thing.

---

## `Thread.cpp` / `Thread.hpp`

**Purpose:** a minimal pthread base class. All the concrete threads (`TCPPortListener`, `MapDataStreamer`) inherit from it.

### Key methods — [src/Thread.cpp](../src/Thread.cpp)

```cpp
int Thread::start(void *args)
{
    setArguments(args);
    _running = true;
    int threadCreateResult = pthread_create(&_theThread, NULL, Thread::threadEntryPoint, this);
    return threadCreateResult;
}

void Thread::stop() { _running = false; }
void Thread::join() { pthread_join(_theThread, NULL); }

void Thread::run(void *arguments) {
    setup();
    execute(arguments);
}

void *Thread::threadEntryPoint(void *pthis) {
    Thread *pt = (Thread *) pthis;
    pt->run(pt->getArguments());
    return 0;
}
```

The static `threadEntryPoint` is the C-ABI trampoline required by `pthread_create`. It casts the `void*` back to `Thread*` and calls the virtual `run()`, which in turn calls the subclass-provided `setup()` and `execute()` hooks.

**`_running` is a plain `bool`, not `std::atomic<bool>`.** This is sloppy by modern standards — on some architectures the compiler could (in theory) keep it in a register across the loop. In practice, x86/ARM with `volatile`-like memory semantics and the memory fences implicit in syscalls (`select`, `usleep`, `read`) make it work. But it is a footgun.

The `setup()`/`execute()` distinction is standard template-method: subclasses override `setup()` for one-time init and `execute()` for the main loop. `MapDataStreamer` and `TCPPortListener` both override both, though `setup()` is empty in each case.

---

## `CommandInterpreter.hpp`

**Purpose:** the tiny abstract base that decouples `TCPPortListener` from knowing anything about what commands do.

```cpp
typedef struct {
    unsigned char commandId;
    unsigned char commandParameterLength;
} command_t;

class CommandInterpreter
{
    public:
        virtual void processCommand(command_t *command,
                                    unsigned char *commandParameters) = 0;
};
```

— [src/CommandInterpreter.hpp:15-34](../src/CommandInterpreter.hpp#L15-L34)

Two observations:

1. **The listener reads 2 bytes directly into a `command_t`** using `receive((unsigned char*)&command, 2, 0)`. This only works because the struct has no padding between its two `unsigned char` members on any sane platform. There is no `#pragma pack` or `alignas`. Portable? Probably. Robust? Not really — if someone ever added a `short` field, the binary format would silently break.
2. **The interface has exactly one method.** There is no `init()`, `shutdown()`, or error callback. Adding those would mean editing every implementor — but there is exactly one implementor (`rplidar`), so it hasn't mattered.

---

## TL;DR of the whole tree

If you only remember five lines from reading this:

1. `main()` → `rplidar::run()` → two threads + `join()` ([rplidar.cpp:51](../src/rplidar.cpp#L51)).
2. `Lidar` is synchronous serial-port code, only meaningfully exercising STOP / SCAN / SET_MOTOR_PWM ([Lidar.cpp:63, 235, 256](../src/Lidar.cpp#L63)).
3. `TCPPortListener` loops on `acceptPending()` → `accept()` → read command → `processCommand()` → close ([TCPPortListener.cpp:35](../src/TCPPortListener.cpp#L35)).
4. `MapDataStreamer` loops on `_isStreaming` and calls the mode-specific handler, which blocks on `readNormalScanDataPacket()` and then forwards via `_tcpStream->send()` ([MapDataStreamer.cpp:48](../src/MapDataStreamer.cpp#L48)).
5. Commands cross threads by the listener thread calling into `rplidar::processCommand()`, which writes to `_mapDataStreamer` members the streamer thread reads — no locks, works by luck of ordering ([rplidar.cpp:15](../src/rplidar.cpp#L15)).

---

Next: [06-build-and-deploy.md](06-build-and-deploy.md).
