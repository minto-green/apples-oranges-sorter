# Sorter Arm Firmware — PlatformIO

Firmware for the 5-DOF ESP32-S3 CycloiDrive arm (Sorter Edition). The Python
brain in `../` talks to this over Wi-Fi via `arm_sender.py`.

## Upload from VSCode

1. Install the **PlatformIO IDE** extension in VSCode.
2. Open **this folder** (`ao/firmware`) as the workspace root — *not* the repo
   root. PlatformIO only detects a project when `platformio.ini` sits at the
   top level of the open folder.
3. Plug the ESP32-S3 in via its **USB** port.
4. Click **Upload** (the → arrow in the blue status bar), or run:

   ```
   pio run --target upload
   pio device monitor
   ```

The board is flashed at 115200 baud. On boot the serial monitor prints the
dashboard IP address — open `http://<ip>/` for the jog dashboard, `/ik` for the
inverse-kinematics one.

## Before the first upload

Set your Wi-Fi credentials near the top of `src/main.cpp`:

```cpp
const char* ssid     = "...";
const char* password = "...";
```

Then point the brain at the IP the board prints:
`python arm_sender.py --host <ip> status`

## Dependencies & fixing `#include` errors

If the `#include` lines at the top of `src/main.cpp` show up red (e.g.
`cannot open source file "AccelStepper.h"`) on a new machine, **nothing is
missing from the code** — the toolchain and libraries just haven't been
downloaded and indexed on that device yet. Here's where each include comes from:

| Include(s) | Source | Installed how |
|---|---|---|
| `AccelStepper.h` | `waspinator/AccelStepper` library | Auto-downloaded from `lib_deps` in `platformio.ini` |
| `ESP32Servo.h` | `madhephaestus/ESP32Servo` library | Auto-downloaded from `lib_deps` in `platformio.ini` |
| `Arduino.h`, `WiFi.h`, `WebServer.h`, `ESPmDNS.h`, `freertos/*.h` | ESP32 Arduino core | Comes with the `espressif32` platform, installed on first build |
| `<queue>` | C++ standard library | Ships with the compiler toolchain |
| `dashboard.h`, `ik_dashboard.h` | This project (`src/`) | Already in the repo |

So you never install these by hand. To resolve the errors on a fresh machine:

1. Install the **PlatformIO IDE** extension in VSCode (see below). This also
   installs the PlatformIO Core CLI.
2. Open **this folder** (`ao/firmware`) as the workspace root, so
   `platformio.ini` is detected.
3. **Build once.** The first build downloads the `espressif32` platform, its
   toolchain, and the two libraries in `lib_deps`:

   ```
   pio run
   ```

   (or click the **Build**/✓ button in the blue status bar). This can take
   several minutes the first time while everything downloads.
4. If the red squiggles remain after the build, rebuild the code index:
   run **PlatformIO: Rebuild IntelliSense Index** from the Command Palette
   (`Ctrl+Shift+P`), then reload the window.

To force-refresh the libraries specifically (e.g. after a corrupted download):

```
pio pkg install
```

## Board note

`platformio.ini` targets `esp32-s3-devkitm-1` with a 16MB flash / 8MB PSRAM
(N16R8) layout. If your board differs, change `board`, `board_upload.flash_size`
and `board_build.partitions` to match.

## Layout

```
platformio.ini      build config + library deps
src/main.cpp        firmware
src/dashboard.h     jog dashboard web UI  (served at /)
src/ik_dashboard.h  IK dashboard web UI   (served at /ik)
```
