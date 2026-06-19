# 01 — Includes & Platform Abstraction

Source: `src/main.cpp` lines 1–41

---

## The full include block

```cpp
#include <cstdio>
#define DR_WAV_IMPLEMENTATION
#define TINYCSOCKET_IMPLEMENTATION

#ifdef _WIN32
#define NOMINMAX
#include <winsock2.h>
#include <windows.h>
#else
#include <arpa/inet.h>
#endif

#include "tinycsocket.h"
#include "cobs.h"
#include "dartt.h"
#include "dartt_sync.h"
#include "checksum.h"
#include "serial_callbacks.h"

#include <Eigen/Dense>

#include <algorithm>
#include <string>
#include <atomic>
#include <cstring>
#include "dartt_mctl_params.h"
#include "motor.h"
#include "serial.h"
#include "lidar.h"
#include "AudioWriter.h"
#include "tick.h"
#include "angle_buffer.h"
#include "udp_forwarder.h"
#include "Smoothing.h"
```

---

## Line-by-line breakdown

### `#include <cstdio>`

Standard C I/O — gives us `printf`, `fprintf`, `perror`. 

**Lesson:** In C++ you can use either `<stdio.h>` (C-style) or `<cstdio>` (C++-style wrapper that puts everything in the `std::` namespace while also guaranteeing it's in the global namespace). For embedded-adjacent work like this, `printf` is preferred over `std::cout` because it's lower overhead and the output is immediate — `cout` can buffer in ways that make debugging confusing.

---

### `#define DR_WAV_IMPLEMENTATION`

This is a **single-header library pattern**. `dr_wav.h` is a library for decoding `.wav` audio files. It's written so that:
- If you just `#include "dr_wav.h"`, you get only the function *declarations* (prototypes).
- If you define `DR_WAV_IMPLEMENTATION` **before** the include (in exactly one `.cpp` file), you also get the function *definitions* (actual compiled code).

**Lesson:** Single-header libraries are common in embedded and game development because they eliminate the need for a build system to link a separate `.a` or `.so` library. You pay zero setup cost — copy one file, define one macro in one place. The `#define` must appear in exactly one translation unit (`.cpp`), otherwise you get "multiple definition" linker errors.

---

### `#define TINYCSOCKET_IMPLEMENTATION`

Same pattern — `tinycsocket.h` is a cross-platform UDP/TCP socket library. Defining this macro includes the actual socket code. Without it, you only get the type declarations and function prototypes.

---

### The `#ifdef _WIN32` block

```cpp
#ifdef _WIN32
#define NOMINMAX
#include <winsock2.h>
#include <windows.h>
#else
#include <arpa/inet.h>
#endif
```

**Lesson: Platform abstraction via preprocessor guards.** The C networking API looks different on every OS:

| OS | Socket header | Notes |
|----|--------------|-------|
| Windows | `<winsock2.h>` | Microsoft's WinSock API |
| Linux/macOS | `<sys/socket.h>`, `<arpa/inet.h>`, `<netinet/in.h>` | POSIX sockets |

`_WIN32` is automatically defined by the MSVC and MinGW compilers whenever you're targeting Windows (even on 64-bit — the name is historical). `#ifdef` lets you write code that compiles correctly on both platforms without duplicating logic.

`NOMINMAX` tells the Windows headers not to define `min` and `max` as macros. Without it, Windows macros would collide with `std::min` / `std::max` and cause bizarre compile errors.

**Why `arpa/inet.h` is needed on Linux:** Functions like `inet_addr()` (converts "192.168.1.1" to a 32-bit integer) live in this header on Linux. On Windows, they're in `winsock2.h`. This was the cause of the build error we fixed earlier.

---

### `#include "tinycsocket.h"`

Cross-platform UDP. The Pi uses this for:
1. Sending modified 1242-byte packets to the Windows viewer (port 9000)
2. Receiving raw 1206-byte VLP-16 packets (port 2381)

**Lesson:** On Linux you *could* use the raw POSIX socket API (`socket()`, `bind()`, `recvfrom()`, etc.). Using a thin wrapper like tinycsocket means the same code compiles on Windows for development/testing without changes.

---

### `#include "cobs.h"`

COBS = **Consistent Overhead Byte Stuffing**. This is a framing protocol for serial data.

**The problem COBS solves:** When you send binary data over a serial port (UART), you need a way to mark where one packet ends and the next begins. A common approach is to use a special byte (e.g., `0x00`) as a packet delimiter. But what if your data *contains* `0x00`? It would look like the end of a packet mid-message.

COBS encodes the data so that `0x00` bytes **never appear in the payload** — only at packet boundaries. The overhead is at most 1 byte per 254 bytes of data.

**Lesson:** Serial framing is a classic embedded problem. UART has no concept of "message boundaries" — it's a raw byte stream. You must layer a framing protocol on top. COBS is one of the cleanest solutions.

---

### `#include "dartt.h"`, `"dartt_sync.h"`, `"checksum.h"`

DARTT is the custom protocol used to communicate with the motor controller MCU. It defines:
- Packet structure (header, address, payload, CRC)
- Data structures for motor state (`dp_periph`: encoder reading, velocity) and control (`dp_ctl`: command word, PID gains)
- `DARTT_PROTOCOL_SUCCESS` return code

**Lesson:** In real embedded systems, motors have their own microcontrollers with defined command/response protocols. You almost never talk directly to hardware registers from a high-level processor like the Pi — there's usually a dedicated firmware layer in between.

---

### `#include <Eigen/Dense>`

Eigen is a C++ linear algebra library. Here it's used for storing 3-D point positions as `Eigen::Vector3f` (a 3-element float vector).

**Lesson:** Even in embedded work on a Linux SBC (single-board computer) like the Pi, it's fine to use heavyweight math libraries. The distinction is between resource-constrained MCUs (the motor controller firmware, which is hand-tuned assembly/C with no heap) and Linux SBCs (which have hundreds of MB of RAM and a full OS).

---

### `#include "angle_buffer.h"`, `"udp_forwarder.h"`, etc.

These are the project's own headers. Each one defines a class that wraps a piece of functionality:

| Header | Class | Purpose |
|--------|-------|---------|
| `angle_buffer.h` | `AngleBuffer` | Thread-safe ring buffer of motor angle samples |
| `udp_forwarder.h` | `UdpForwarder` | Send packets to viewer; serve web config |
| `motor.h` | `Motor` | DARTT motor abstraction |
| `serial.h` | `Serial` | UART serial port (auto-detects port) |
| `lidar.h` | `LidarSystem` | UDP receive + packet injection |
| `tick.h` | — | `get_tick32()` (ms) and `get_microsecond64()` (µs) |
| `Smoothing.h` | — | `smooth_qd()` trajectory generator for GOLDEN_SNAP mode |

---

## Next: `02_iir_filter.md` — The filtfilt (zero-phase IIR) filter
