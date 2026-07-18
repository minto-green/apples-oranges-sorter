# Apples &amp; Oranges — Autonomous Vision Sorter

A closed-loop fruit sorting system: a camera watches a marker-defined platform, a
YOLOv11s model classifies what is on it, and a 5-DOF ESP32-S3 arm picks each item
into the correct bin. A natural-language assistant sits on top, so the whole thing
can also be driven by typing or speaking.

```
Camera → YOLOv11s → ArUco 2×2 zone (S1–S4) → 5-DOF arm (Wi-Fi G-code)
                            ↑
                  AI assistant (chat + voice)
```

---

## Contents

1. [What it does](#1-what-it-does)
2. [Hardware](#2-hardware)
3. [Install](#3-install)
4. [Configure](#4-configure)
5. [Calibration — ArUco markers and lighting](#5-calibration--aruco-markers-and-lighting)
6. [Running the system](#6-running-the-system)
7. [Operating the dashboard](#7-operating-the-dashboard)
8. [Handling the robot arm](#8-handling-the-robot-arm)
9. [How the autonomy works](#9-how-the-autonomy-works)
10. [Tuning](#10-tuning)
11. [Troubleshooting](#11-troubleshooting)
12. [Data and results](#12-data-and-results)
13. [Project layout](#13-project-layout)

---

## 1. What it does

| Stage | Detail |
|---|---|
| **Detect** | YOLOv11s, two classes (Apple, Orange), 640 px inference, live confidence threshold |
| **Localise** | Four ArUco markers define the platform; a homography splits it into a 2×2 grid (S1–S4) and survives camera movement without recalibration |
| **Decide** | A per-sector state machine issues exactly **one** arm command per object, using time-stable readings so detection flicker cannot double-fire |
| **Act** | Pick sequence for the sector, drop sequence for the class: Apple → Bin 1, Orange → Bin 2 (8 combinations total) |
| **Verify** | After the arm finishes, the sector is re-scanned; removal — not the firmware's "motion done" — is what counts as a successful pick |
| **Command** | Text or voice → LLM → strict JSON intent → whitelist validation → arm. The firmware never sees model output directly |

---

## 2. Hardware

- 5-DOF arm with cycloidal-drive joints and a gripper, driven by an **ESP32-S3**
- Any USB webcam (1080p recommended; the app requests MJPG)
- A flat platform with four printed ArUco markers at its corners
- Two bins, one per class
- A Windows/macOS/Linux laptop on the **same network** as the ESP32

---

## 3. Install

Python 3.10–3.12.

```bash
python -m pip install --upgrade pip
pip install ultralytics opencv-contrib-python flask flask-socketio requests
```

| Package | Why |
|---|---|
| `ultralytics` | YOLO inference (pulls in torch, torchvision, numpy, pillow) |
| `opencv-contrib-python` | Camera, drawing, and **ArUco** — the plain `opencv-python` package has no ArUco |
| `flask` / `flask-socketio` | Web server and live dashboard updates |
| `requests` | Talks to the arm over Wi-Fi and to the AI provider |

Optional, only if you want to run the `.onnx` weights instead of the `.pt`:

```bash
pip install onnxruntime
```

### Verify

```bash
python -c "import ultralytics, flask, flask_socketio, requests; print('core OK')"
python -c "import cv2; print('opencv', cv2.__version__); import cv2.aruco; print('aruco OK')"
```

Both must print. If the second fails, you have a plain OpenCV build shadowing the
contrib one:

```bash
pip uninstall -y opencv-python opencv-python-headless opencv-contrib-python opencv-contrib-python-headless
pip install opencv-contrib-python
```

<details>
<summary>Other install failures</summary>

**Torch DLL errors on Windows**
```bash
pip uninstall -y torch torchvision
pip install ultralytics --force-reinstall --no-cache-dir
```

**Flask / SocketIO version clash**
```bash
pip uninstall -y flask flask-socketio python-socketio python-engineio
pip install flask flask-socketio
```

**pip installed into a different Python than the one running app.py**
Always install with the interpreter you launch with:
```bash
path\to\python.exe -m pip install ultralytics opencv-contrib-python flask flask-socketio requests
```
</details>

---

## 4. Configure

Nothing sensitive is committed. Everything machine-specific comes from the
environment.

| Variable | Default | Purpose |
|---|---|---|
| `AO_ARM_HOST` | `ao-arm.local` | Arm hostname or IP |
| `AO_SIMULATE` | `0` | `1` runs the whole pipeline with no hardware attached |
| `AI_PROVIDER` | *(unset)* | `claude`, `gemini`, `groq`, or `openrouter` |
| `<PROVIDER>_API_KEY` | *(unset)* | Key for the chosen provider |

PowerShell, for the current window:

```powershell
$env:AO_ARM_HOST = "<arm-ip-or-hostname>"
$env:AI_PROVIDER = "gemini"
$env:GEMINI_API_KEY = "<your key>"
```

Permanently (`setx`, then open a **new** terminal):

```powershell
setx AO_ARM_HOST "<arm-ip-or-hostname>"
```

Without an AI key the assistant falls back to a keyword parser and tags its
replies *offline mode*. Commands still work. See **AI_PROVIDERS.md** for the free
options.

### Wi-Fi credentials for the firmware

Credentials live in `firmware/src/secrets.h`, which is git-ignored. Copy the
template and fill it in before flashing:

```bash
cp firmware/src/secrets.h.example firmware/src/secrets.h
```

```cpp
#define WIFI_SSID     "your-network"
#define WIFI_PASSWORD "your-password"
```

In `firmware/src/main.cpp`, replace the literal SSID/password lines with:

```cpp
#include "secrets.h"
const char* ssid     = WIFI_SSID;
const char* password = WIFI_PASSWORD;
```

### Finding the arm's address

1. **Serial monitor** — `pio device monitor`; the boot log prints the dashboard
   URL. Typing `IP` + Enter reprints it.
2. **mDNS** — the firmware announces itself as `ao-arm.local`. If that opens the
   jog dashboard in a browser, use it directly as `AO_ARM_HOST` and you never have
   to chase an IP again. (Windows may need Apple Bonjour for `.local` names.)
3. **Router** — check the DHCP lease list for `ao-arm` or an Espressif MAC.

### Changing venue / network

1. Update `secrets.h`, re-flash. This is the only step that needs a re-flash.
2. Find the new address (above) and update `AO_ARM_HOST`.
3. Laptop and ESP32 must be on the **same** network. A phone hotspot is the most
   reliable demo setup — the address usually stays the same each time.
4. Voice recognition needs internet; everything else is LAN-only.

---

## 5. Calibration — ArUco markers and lighting

This is the step that decides whether the system works at all. Budget time for it.

### 5.1 Print the markers

```bash
cd ao
python marker_zone.py     # writes zone_markers.png and zone_markers_inverted.png
```

Use the **inverted** sheet on a dark platform. Print on **matte** paper — glossy
photo paper is the single most common cause of failed detection under a lamp.

### 5.2 Place them

```
ID 0 ────────── ID 1        ID 0  top-left
  │   S1 │ S2    │          ID 1  top-right
  │ ─────┼───── │           ID 2  bottom-right
  │   S3 │ S4    │          ID 3  bottom-left
ID 3 ────────── ID 2
```

Tape one at each corner, flat, with the **white quiet zone intact** — leave about
1 cm of white margin around the black square when you cut. A marker trimmed flush
to the black edge is undetectable, and this trips people up more than anything
else.

The homography computed from these four corners places the S1–S4 pick points on
the platform's white dots automatically. The camera can sit anywhere and even be
nudged mid-run without recalibration, as long as all four markers stay in frame.

### 5.3 Scanning them

Launch the app and watch the pill in the top-right of the dashboard:

| Pill | Meaning |
|---|---|
| `PLACE MARKERS` | Fewer than four markers visible |
| `ZONE LOCKED` | All four found; the grid overlay and pick dots are drawn on the feed |

Hold the camera steady until it locks. The detector runs every 15 frames and
retries on a contrast-boosted copy of each frame, so a lock usually takes a second
or two. The "on platform" counter and the S1–S4 mini-map stay blank until the zone
is locked — that is expected, not a bug.

### 5.4 Lighting — glare is the enemy

Glare hurts **both** stages: a specular highlight erases the black/white pattern
the marker detector needs, and a blown-out highlight on fruit skin destroys the
colour and texture cues the YOLO model was trained on. Waxy apple skin is
especially good at throwing a hotspot straight back into the lens.

If markers will not lock, or fruit detection is unstable, work through this in
order:

1. **Turn the camera's brightness down.** Counter-intuitive, but a slightly dark
   frame with intact edges detects far better than a bright frame with blown
   highlights. The ArUco detector applies its own contrast boost, so it can
   recover a dim image — it cannot recover clipped white.
2. **Turn the gain down.** High gain adds sensor noise, and noise breaks the
   marker's bit pattern before it breaks anything a human would notice. Lower gain
   plus more real light always beats high gain.
3. **Fix exposure manually** if the camera allows it. Auto-exposure hunting as the
   arm swings over the platform is a common cause of "it worked a minute ago".
4. **Move the light source off-axis.** Light from the side or behind the camera at
   an angle; never point a lamp along the camera's line of sight onto glossy
   surfaces.
5. **Diffuse it.** A sheet of baking paper or a white cloth over a desk lamp
   removes most hotspots for free.
6. **Kill overhead point sources** — a single bright ceiling LED directly above the
   platform is worse than a dimmer, broader source.
7. **Matte everything.** Matte marker paper, matte or lightly-textured platform
   surface. Wipe fruit dry; a wet apple is a mirror.

On Windows, camera brightness/gain/exposure live in the vendor's camera utility or
in `Camera` settings; on Linux, `v4l2-ctl --list-ctrls` then
`v4l2-ctl --set-ctrl=brightness=…,gain=…`. Set them **before** launching the app —
the capture backend takes over the device once it opens.

A quick check: if the marker squares look grey and readable rather than glaring
white patches, you are in the right range.

---

## 6. Running the system

Order matters:

1. **Park the arm in its initial upright position** and confirm it against the
   alignment marks on the joints. Everything downstream assumes this pose is
   `0,0,0,0,0,0`.
2. Power the arm and wait for it to join Wi-Fi.
3. Confirm the link:
   ```bash
   python arm_sender.py --host <arm-host> status   # should print IDLE
   python arm_sender.py list                       # the 8 sort combinations
   ```
4. Place the platform, markers, and camera; check lighting (§5).
5. Launch:
   ```bash
   cd ao
   python app.py
   ```
6. Open **http://localhost:5800** in Chrome or Edge.

Use `localhost`, not the LAN IP — browser speech recognition only runs on a secure
origin, so voice control silently dies over `http://192.168.x.x`.

Autosort does not start on its own (`AUTO_START = False`). Press **Start** when
you are ready.

For a demo with no hardware, set `AO_SIMULATE=1`: detection, the state machine, and
the 3D view all run; only the motors are absent.

---

## 7. Operating the dashboard

### Sorter tab

**Feed (left)** — camera picker, live confidence slider, arm state, current speed
and acceleration, FPS. The overlay shows boxes with class and confidence (dimmed
when outside the zone), the zone polygon, the 2×2 grid, and the S1–S4 pick points.

**Control strip**

| Button | Effect |
|---|---|
| **Start** | Begin autosort (any fruit, unlimited) |
| **Stop** | Halt the arm mid-motion, drain the firmware queue, return home, pause autosort |
| **Clear** | Cancel everything pending — the current move finishes safely, the queue is dropped, sector state is forgotten, the arm returns home |
| **Home** | Send the arm to its home pose |

The pipeline indicator to the right shows SCAN → PICK &amp; DROP → VERIFY.

**Statistics (right)** — totals per class, split bar, the live 2×2 map (coloured =
fruit present, white outline = the arm is working it, red = needs attention),
recent commands with object IDs and ✓/✗, failed-pick count, uptime.

**Assistant (right, bottom)** — type, or press the mic and speak; replies are read
aloud. It understands scoped orders:

```
"sort the oranges"          "sort 1 apple on S1"
"grab two apples"           "how many apples do you see?"
"stop"                      "clear the queue"        "arm home"
```

A scoped order appears as a pill in the control strip (e.g. `1 × Apples @ S1`) and
autosort pauses itself once the order is filled.

---

## 8. Handling the robot arm

The **ARM** tab is the calibration workshop: a live 3D model, joint sliders, a
motion section, and a sequence builder.

### 8.1 Speed and acceleration

The **SPD** and **ACC** sliders are the real controls for how fast the arm moves.
They are pushed to the server the moment you release the slider, and every payload
built from that point on — **manual poses and autosort picks alike** — carries the
new values.

- Defaults: **speed 40, accel 25**.
- Ceilings are the firmware's: **50** and **25**. Anything higher is clamped, both
  here and on the board, so the dashboard cannot lie about what the arm was asked
  to do.
- A change takes effect on the **next** pick. A sequence already queued in the
  firmware keeps the speed it was sent with — that is a firmware queue property,
  not a UI lag.
- If the arm still feels slow at 50, the limit is
  `UNIVERSAL_MAX_SPEED_DEG_PER_SEC` in `firmware/src/main.cpp`. Raise it there and
  re-flash; raising it on this side alone does nothing.

### 8.2 Keeping the 3D view and the real arm in agreement

The 3D view is a model, not a measurement. It stays honest by pulling joint angles
from the ESP32's `/api/status` roughly every second, and the **view** badge in the
top bar tells you where its numbers came from:

| Badge | Meaning |
|---|---|
| `live` / `synced from arm` | Angles are coming from the hardware |
| `zeroed manually` | You told it the arm is at zero |
| `not synced` | The arm reported no angles — the view is showing assumptions |
| `arm unreachable` | No status at all |

Three buttons resolve any disagreement:

| Button | Use when | Effect |
|---|---|---|
| **Sync view from arm** | You trust the hardware | Pulls the reported angles and snaps the model to them |
| **Zero the view** | The arm *is* physically at home but the view is not | Resets the model to `0,0,0,0,0,0`. **Does not move the arm.** |
| **Send arm to 0°** | The view is right but the arm is not | Drives the arm to exactly the pose the view calls zero |

The middle one is the fix for the classic failure: you park the arm at home, open
the tab, and the view shows some other pose — from then on the view treats *that*
as home and every angle you read off it is offset. Press **Zero the view** and the
two agree again.

Two guards keep the view from fighting you: status polling is suspended while a
sequence is animating and while you are holding a joint slider, so a poll can no
longer yank a slider out from under your finger mid-adjustment.

### 8.3 Calibration workflow

1. Drag J1–J6 until the gripper sits exactly over a white dot or a bin. The 3D
   model follows live and the pose string updates (e.g. `0,56,-22,24,0,0,X`).
2. **Execute this pose** sends it — the model animates in simulate mode, the real
   arm moves otherwise. Nudge until it is right.
3. **+ Add to sequence** / **+ Delay** build the full pick or drop routine.
4. **Run sequence** test-drives it end to end.
5. **Copy for code**, then paste into `PICK_SEQUENCES` / `DROP_SEQUENCES` in
   `arm_sender.py`.

Command format, one per line:

```
J1,J2,J3,J4,J5,J6,T     T = O (open) | C (close) | X (unchanged)
S:<speed>               A:<accel>              D:<milliseconds>
```

### 8.4 Safety habits

- Keep a hand near the power switch the first time you run a re-tuned sequence.
- Test new drop angles with **Execute this pose** one line at a time before running
  the whole sequence.
- **Stop** freezes mid-motion; **Clear** lets the current move finish. Prefer
  **Clear** when the gripper is holding fruit.
- Both end with the arm back at its initial position, so the state after an abort
  is always known.

---

## 9. How the autonomy works

Each sector runs a state machine driven by *time-stable* readings, so flicker or
the arm passing over the camera cannot confuse it:

```
EMPTY      --(object seen continuously for STABLE_S = 1.0 s)--> OCCUPIED
OCCUPIED   --(exactly ONE command sent)--------------------> COMMANDED
COMMANDED  : camera input is IGNORED until the arm finishes
COMMANDED  --(arm done)------------------------------------> VERIFY
VERIFY     --(reading stable for CLEAR_S = 2.5 s)-->
               fewer objects than before -> SUCCESS -> EMPTY
                                            (or OCCUPIED if one remains:
                                             that is a NEW object, eligible
                                             for one new command)
               same objects as before     -> FAILED  -> ATTENTION
ATTENTION  : no retry, ever, until the sector reads empty for CLEAR_S
             (a human removed or replaced the fruit) -> EMPTY
```

**Late success.** Removal is the truth signal — the firmware reports motion
complete, not grip success. If a sector fails immediate verification but the fired
object disappears later, it is counted as sorted (`✓ late` in the log) and the
unresolved-failure counter goes back down.

**Multiple objects in one sector.** The count comes only from boxes seen
*simultaneously* in a single frame, sustained for `STABLE_S`. Detections in
different frames are never summed, so one wobbling fruit cannot become "two". A
×2 sector sorts one command at a time, each removal confirmed before the next
command becomes eligible.

**Firmware protocol**

| Endpoint | Behaviour |
|---|---|
| `POST /api/sequence` | One order at a time — HTTP 409 `BUSY` while working; duplicate `ID:` lines answered `DUP` and ignored |
| `POST /api/override` | `STOP` / `CLEAR` / `HO` / `GH`, accepted at any time |
| `GET /api/status` | State, current order ID, queue depth, joint angles |

The dashboard tolerates either JSON or plain text from `/api/status` and normalises
it, so a firmware revision that changes the reply format will not blind the 3D view.

**How the assistant reaches the arm** — it never talks to the ESP32 directly:

```
voice/text → LLM (JSON intent only) → app.py whitelist validation
           → arm_sender.py → HTTP → ESP32
```

Whichever provider produced the intent, the firmware only ever receives the same
short list of validated actions.

---

## 10. Tuning

Top of `app.py`:

| Setting | Default | Meaning |
|---|---|---|
| `CONF` | 0.40 | Detection threshold (also live via the UI slider) |
| `STABLE_S` | 1.0 | Seconds a reading must hold to confirm a sector |
| `CLEAR_S` | 2.5 | Seconds a sector must read empty/lower to confirm removal |
| `AUTO_START` | False | Start sorting immediately on launch |
| `SORT_TIMEOUT` | 150 | Watchdog seconds for a stuck arm phase |
| `IMGSZ` | 640 | Inference size (matches training) |
| `CAM_WIDTH` / `CAM_HEIGHT` | 1920 / 1080 | Drop to 1280×720 if the feed lags |

Motion defaults live in `arm_sender.py` (`DEFAULT_SPEED`, `DEFAULT_ACCEL`) and are
overridden at runtime by the ARM tab sliders.

---

## 11. Troubleshooting

| Symptom | Cause and fix |
|---|---|
| `PLACE MARKERS` never clears | In order of likelihood: markers cut without a white margin; glare or blown highlights (§5.4); too dark or too noisy; markers too small or partly out of frame; `import cv2.aruco` failing (§3) |
| Detections appear but nothing fires | The zone must read LOCKED and the fruit's centre must be inside the polygon. Check the filter pill — a scoped "sort apples" order ignores oranges until Stop/Start resets it |
| Speed slider seems to do nothing | It applies to the **next** pick; a queued sequence keeps its original speed. Above 50/25 the firmware clamps — see §8.1 |
| 3D view disagrees with the real arm | §8.2 — **Zero the view** if the arm is at home, **Sync view from arm** if it is not |
| `view: not synced` | The firmware returned no joint angles. Sorting is unaffected; the 3D view is just running open-loop |
| Arm unreachable | Ping the host; confirm both devices are on the same network. Detection keeps running and a toast reports the error. Use `AO_SIMULATE=1` for hardware-free demos |
| Voice button does nothing | Not Chrome/Edge; opened over a LAN IP instead of `localhost`; mic permission denied; no internet; or the wrong system default mic. The status line under the chat box names the exact cause |
| Camera missing from the dropdown | Devices are probed once at launch — restart `app.py` after plugging one in |
| Laggy feed | Lower `CAM_WIDTH`/`CAM_HEIGHT` to 1280×720 |
| `app.py` not recognised (PowerShell) | Run `python app.py`; PowerShell will not execute a script by filename alone |
| `FileNotFoundError` for the weights | The `.pt` must sit beside `app.py`. Paths resolve relative to the script, so the launch directory does not matter |

---

## 12. Data and results

`sort_log.jsonl` accumulates one JSON object per event, timestamped:

- **sorts** — object ID, class, sector, confidence, speed/accel in force, success
  flag, and whether it resolved late
- **assistant** — the raw message and the parsed intent
- **motion** — every speed/acceleration change
- **watchdog_reset** — stuck-phase recoveries

Success rate, failed picks, and throughput per minute come straight out of this
file — no manual tallying during a run.

---

## 13. Project layout

```
ao/
├── app.py                  dashboard, detection loop, sort state machine
├── arm_sender.py           Wi-Fi G-code controller, pick/drop sequences, motion
├── marker_zone.py          ArUco generator, homography, S1–S4 segmentation
├── assistant.py            NL → validated JSON intent, multi-provider + offline
├── templates/index.html    the web UI (Sorter and ARM tabs)
├── AI_PROVIDERS.md         going online, including free providers
├── data.yaml               training class map (not needed at runtime)
├── apples_oranges_yolo11s_best.pt      weights used by default
├── apples_oranges_yolo11s_best.onnx    fallback weights (needs onnxruntime)
└── firmware/
    ├── src/main.cpp        ESP32-S3 firmware (one order at a time, ID dedup)
    ├── src/secrets.h       Wi-Fi credentials — git-ignored, create from example
    └── src/secrets.h.example
```

`sort_log.jsonl` and `zone_markers*.png` are generated at runtime.
