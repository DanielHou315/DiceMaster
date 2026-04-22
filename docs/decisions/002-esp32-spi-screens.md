# ADR-002: ESP32-S3 + SPI for Driving 6 Independent Screens

## Status

Accepted

## Context

DiceMaster needs 6 independent 480x480 IPS displays, one per face of the dice enclosure. Each screen must be capable of displaying full-color JPEG images, animated GIFs, and multi-language text (Arabic, Chinese, Cyrillic, Devanagari) with correct rotation for the current physical orientation of the face.

The central compute unit is a Raspberry Pi 4. The Pi needs to push content to all 6 screens with low enough latency that the display updates feel responsive to a user shaking or rotating the dice. A 480x480 JPEG image is typically 10–15 KB; a GIF may be larger and requires frame sequencing on the display side.

Key constraints:
- The Pi has a limited number of native SPI buses (4 usable: 0, 1, 3, and one reserved for other use).
- The enclosure is compact; wiring must be minimized.
- Screen rendering (JPEG decode, font rendering, GIF playback) must not block the Pi's CPU.

## Decision

Use **3 ESP32-S3 microcontroller boards**, each driving **2 screens via a shared SPI bus** on the ESP32 side. The Raspberry Pi communicates with each ESP32 over a dedicated SPI bus (Pi as master, ESP32 as slave), using a custom binary protocol.

The screen-to-bus assignment is:
- SPI Bus 0: Screens 1 and 6 (ESP32 #1)
- SPI Bus 1: Screens 3 and 5 (ESP32 #2)
- SPI Bus 3: Screens 2 and 4 (ESP32 #3)

Each ESP32 drives its two screens using the Arduino GFX library and decodes JPEGs on-chip with the JPEGDEC library. Multi-language fonts are rendered via U8g2. GIF playback is handled on the ESP32, freeing the Pi from per-frame transmission after the initial asset is sent.

Screens on the same ESP32 are addressed using a CAN-bus-inspired bitmask: bit k in the Screen ID byte of each protocol packet corresponds to screen k. An ESP32 silently drops packets whose Screen ID byte does not have its screen's bit set, enabling the Pi to target either or both screens on a bus in a single SPI transaction.

The SPI buffer is 2 KB per transaction. Large images are split into numbered chunks by the Pi-side `ScreenBusManager` and reassembled on the ESP32. If not all chunks arrive within `100ms × num_chunks`, the ESP32 discards the partial image.

## Consequences

**Positive:**
- All JPEG decoding, font rendering, and GIF animation run on the ESP32 CPUs, not the Pi. The Pi only needs to transmit compressed asset bytes.
- Three parallel SPI buses allow all three ESP32s to be updated concurrently from separate Python threads.
- The CAN-bus bitmask scheme allows broadcasting the same content to both screens on a bus in one transaction, which is useful for symmetric dice faces.
- The chunked transfer protocol with timeout-based invalidation is robust to partial failures without requiring an explicit acknowledgment per chunk.

**Negative:**
- Requires custom firmware development for the ESP32 (Arduino C++), adding a firmware maintenance burden.
- The custom binary protocol (defined in `docs/protocol.md`) must be kept in sync between the Pi-side Python implementation (`protocol.py`) and the ESP32-side C++ parser. Protocol mismatches cause silent rendering failures.
- The standard `py-spidev` library's 4 KB SPI buffer was too small for image chunks; a custom build supporting 8 KB buffers was required.
- Debugging cross-device issues requires attaching a serial monitor to the ESP32 while the Pi sends data.

## Alternatives Considered

**Direct HDMI or DSI displays from the Pi**
The Raspberry Pi 4 has one HDMI and one DSI output, which would support at most 2 screens without additional hardware. Driving 6 screens this way would require DisplayPort MST hubs or a Compute Module 4 with additional carrier board support. This approach would not easily support per-face orientation-aware rendering and would tie the display stack to the Linux framebuffer, making custom rotation and content dispatch more complex.

**I2C displays**
I2C is limited to ~400 kHz (Fast Mode) or 1 MHz (Fast-Plus). Transmitting a 10 KB JPEG to a single display over I2C at 400 kHz takes approximately 200 ms, making 6-screen simultaneous updates infeasible at interactive frame rates. SPI at the configured speed is roughly 100x faster.
