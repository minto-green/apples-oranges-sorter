"""
Apples & Oranges - autonomous vision sorting dashboard.

Camera -> YOLOv11s -> ArUco 2x2 zone (S1-S4) -> 5-DOF arm (Wi-Fi G-code)
with an AI assistant layer (chat + voice).

Sort cycle (fully autonomous, one object at a time - no confirmation step):
    scan -> detection lands in a segment (generation ID assigned) -> fire arm
    immediately -> segment locked while arm busy -> arm idle -> POST-VERIFY
    rescan -> stats -> next object. Each object gets one command; a failed
    pick is parked until the object is moved/replaced, so the arm never
    loops on one stubborn spot.

SETUP:
    pip install ultralytics opencv-contrib-python flask flask-socketio requests
    set AO_ARM_HOST=<arm ip or ao-arm.local>
    set AI_PROVIDER / <PROVIDER>_API_KEY  (optional; offline fallback exists)

MARKERS (once): python marker_zone.py -> print zone_markers.png
RUN:            python app.py -> http://localhost:5800
"""
import datetime
import json
import os
import platform
import re
import threading
import time
from pathlib import Path

import cv2
from flask import Flask, Response, jsonify, render_template, request
from flask_socketio import SocketIO
from ultralytics import YOLO

import arm_sender
from arm_sender import CubicoreArmController, build_sort_payload
from assistant import parse_intent
from marker_zone import SEGMENT_LABELS, MarkerZone

# ---------------- CONFIG ----------------
BASE_DIR = Path(__file__).resolve().parent  # paths work from any launch dir


def _resolve_model():
    """Prefer the .pt; fall back to .onnx (needs onnxruntime installed)."""
    for name in ("apples_oranges_yolo11s_best.pt",
                 "apples_oranges_yolo11s_best.onnx"):
        p = BASE_DIR / name
        if p.exists():
            return str(p)
    raise FileNotFoundError(
        f"No model found in {BASE_DIR}.\n"
        "Put apples_oranges_yolo11s_best.pt (or .onnx) next to app.py.")


MODEL_PATH = _resolve_model()
CAM_WIDTH = 1920
CAM_HEIGHT = 1080
IMGSZ = 640
CONF = 0.40             # UI slider can change at runtime
STABLE_S = 1.0          # seconds a reading must hold to confirm occupancy
CLEAR_S = 2.5           # seconds a sector must read lower/empty to confirm removal
AUTO_START = False      # set True once bin drop angles are verified
ZONE_UPDATE_EVERY = 15
SORT_TIMEOUT = 150      # s; real pick+drop sequences take ~30-60s
CAM_INDEX = 0
MAX_CAMERAS = 5         # probe indices 0..4 for the dropdown

# Arm address. Do NOT hardcode an IP here - set it in the environment:
#   PowerShell:  $env:AO_ARM_HOST = "192.168.x.x"
#   permanent :  setx AO_ARM_HOST "192.168.x.x"
# Leave it unset to use mDNS (the firmware announces itself as ao-arm.local).
ESP32_HOST = os.environ.get("AO_ARM_HOST", "ao-arm.local")

SIMULATE_ARM = os.environ.get("AO_SIMULATE", "0") == "1"
LOG_FILE = str(BASE_DIR / "sort_log.jsonl")

CLASS_NAMES = ["Apple", "Orange"]
CLASS_TO_BIN = {"Apple": 1, "Orange": 2}
# BGR label colors - Apple = red family, Orange = orange family
CLASS_COLORS = {"Apple": (60, 60, 232), "Orange": (43, 140, 255)}
# ----------------------------------------

app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins="*", async_mode="threading")
print(f"Loading model: {MODEL_PATH}")
model = YOLO(MODEL_PATH, task="detect")
arm = CubicoreArmController(ESP32_HOST, simulate=SIMULATE_ARM)
zone = MarkerZone(update_every=ZONE_UPDATE_EVERY)

lock = threading.Lock()
state = {
    "running": AUTO_START,  # autosort enabled
    # active order: filter = class or "all"; segment = "S1".."S4" or None;
    # remaining = int quota or None (unlimited). Set by UI Start or AI intents.
    "order": {"filter": "all", "segment": None, "remaining": None},
    "conf": CONF,
    # Live motion settings. These are the single source of truth for BOTH
    # manual poses and autosort sequences - arm_sender rebuilds its S:/A:
    # preamble from arm_sender.MOTION every time it builds a payload.
    "speed": arm_sender.DEFAULT_SPEED,
    "accel": arm_sender.DEFAULT_ACCEL,
    "camera_index": CAM_INDEX,
    "change_camera": False,
    "phase": "scan" if AUTO_START else "idle",  # idle|scan|sorting|verify
    "active": None,         # {seg, cls, conf, obj_id} while confirming/sorting
    "frame": None,
    "fps": 0.0,
    "stats": {
        "totals": {n: 0 for n in CLASS_NAMES},
        "session_start": time.time(),
        "history": [],      # last 20 sorts: {obj_id, cls, seg, conf, ts, ok}
        "failed_picks": 0,
    },
    "_sort_start": 0.0,
}

# Make sure arm_sender starts from the same numbers the UI shows.
arm_sender.set_motion(state["speed"], state["accel"])

# Per-segment state machine - ONE command per object, flicker-proof.
#   EMPTY      no confirmed object
#   OCCUPIED   object(s) confirmed -> eligible for exactly one command
#   COMMANDED  command sent; detection flicker/occlusion IGNORED until arm done
#   VERIFY     arm finished; waiting for a CLEAR_S-stable reading to judge it
#   ATTENTION  pick failed (count did not drop) -> NO re-fire until the
#              sector reads empty for CLEAR_S (human removed/replaced it)
# count = number of objects seen SIMULTANEOUSLY in one frame (never
# accumulated across frames), stabilized over STABLE_S/CLEAR_S.
segments = {
    s: {"state": "EMPTY", "cls": None, "conf": 0.0,
        "count": 0,           # confirmed simultaneous-object count
        "cand_count": 0, "cand_cls": None, "cand_conf": 0.0,
        "cand_since": 0.0,    # when the candidate reading started
        "obj_id": None, "fired_count": 0, "fired_cls": None,
        "arm_ok": False}
    for s in SEGMENT_LABELS
}
_id_counter = {s: 0 for s in SEGMENT_LABELS}


# ------------------------------------------------------------------ helpers
def log_event(obj):
    obj["ts"] = datetime.datetime.now().isoformat()
    with open(LOG_FILE, "a") as f:
        f.write(json.dumps(obj) + "\n")


def open_camera(index):
    sysname = platform.system()
    if sysname == "Windows":
        cap = cv2.VideoCapture(index, cv2.CAP_DSHOW)
    elif sysname == "Darwin":
        cap = cv2.VideoCapture(index, cv2.CAP_AVFOUNDATION)
    else:
        cap = cv2.VideoCapture(index)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAM_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_HEIGHT)
    return cap


def list_cameras():
    """Probe camera indices once for the UI dropdown."""
    found = []
    for i in range(MAX_CAMERAS):
        cap = open_camera(i)
        ok = cap.isOpened() and cap.read()[0]
        cap.release()
        if ok:
            found.append(i)
    return found or [0]


AVAILABLE_CAMERAS = list_cameras()


def draw_label(img, text, x, y, color):
    """Readable label: filled pill behind bold white text."""
    font, scale, thick = cv2.FONT_HERSHEY_SIMPLEX, 0.85, 2
    (tw, th), base = cv2.getTextSize(text, font, scale, thick)
    y_top = max(0, y - th - base - 10)
    cv2.rectangle(img, (x, y_top), (x + tw + 16, y), color, -1)
    cv2.putText(img, text, (x + 8, y - base - 4), font, scale,
                (255, 255, 255), thick, cv2.LINE_AA)


def platform_counts():
    counts = {n: 0 for n in CLASS_NAMES}
    for s in segments.values():
        if s["state"] in ("OCCUPIED", "COMMANDED", "VERIFY", "ATTENTION") \
                and s["cls"] in counts:
            counts[s["cls"]] += max(1, s["count"])
    return counts


def assistant_context():
    return {
        "running": state["running"],
        "arm_busy": arm.busy,
        "order": dict(state["order"]),
        "platform_counts": platform_counts(),
        "segments": {k: ({"cls": v["cls"], "count": max(1, v["count"]),
                          "state": v["state"]}
                         if v["state"] != "EMPTY" else None)
                     for k, v in segments.items()},
        "sorted_totals": dict(state["stats"]["totals"]),
    }


def emit_state():
    socketio.emit("state", {
        "running": state["running"],
        "phase": state["phase"],
        "arm_busy": arm.busy,
        "order": dict(state["order"]),
        "conf": state["conf"],
        "speed": state["speed"],
        "accel": state["accel"],
        "speed_max": arm_sender.FW_MAX_SPEED,
        "accel_max": arm_sender.FW_MAX_ACCEL,
        "simulate": SIMULATE_ARM,
        "camera_index": state["camera_index"],
        "cameras": AVAILABLE_CAMERAS,
        "fps": round(state["fps"], 1),
        "zone": zone.status,
        "zone_locked": zone.locked,
        "active": state["active"],
        "segments": {k: {"occupied": v["state"] != "EMPTY", "cls": v["cls"],
                         "conf": round(v["conf"], 2), "obj_id": v["obj_id"],
                         "count": max(1, v["count"]) if v["state"] != "EMPTY" else 0,
                         "locked": v["state"] in ("COMMANDED", "VERIFY"),
                         "stuck": v["state"] == "ATTENTION"}
                     for k, v in segments.items()},
        "platform_counts": platform_counts(),
        "stats": {
            "totals": state["stats"]["totals"],
            "failed_picks": state["stats"]["failed_picks"],
            "history": state["stats"]["history"][-12:],
            "uptime_s": int(time.time() - state["stats"]["session_start"]),
        },
    })


# ------------------------------------------------------- occupancy tracking
def _finish_sort(seg, s, success, late=False):
    """Bookkeeping when a sort resolves (called under lock).
    late=True: the object left AFTER the immediate verify had failed -
    counted as sorted, and the earlier unresolved failure is reclaimed."""
    target = {"seg": seg, "cls": s["fired_cls"], "conf": round(s["conf"], 2),
              "obj_id": s["obj_id"], "late": late}
    if success and late:
        state["stats"]["failed_picks"] = max(0, state["stats"]["failed_picks"] - 1)
    if success:
        state["stats"]["totals"][s["fired_cls"]] += 1
        rem = state["order"].get("remaining")
        if rem is not None:
            state["order"]["remaining"] = rem - 1
            if state["order"]["remaining"] <= 0:
                state["running"] = False
                state["order"] = {"filter": "all", "segment": None,
                                  "remaining": None}
                socketio.emit("toast", {"kind": "ok", "msg": "Order complete"})
    else:
        state["stats"]["failed_picks"] += 1
        socketio.emit("toast", {"kind": "err",
                                "msg": f"{seg} needs attention - {s['fired_cls']} was not removed. "
                                       f"No retry until the sector is cleared."})
    state["stats"]["history"].append(
        {**target, "ok": success,
         "ts": datetime.datetime.now().strftime("%H:%M:%S")})
    state["stats"]["history"] = state["stats"]["history"][-50:]
    log_event({"event": "sort", **target, "ok": success,
               "speed": state["speed"], "accel": state["accel"]})
    state["active"] = None


def update_segments(dets):
    """
    dets: list of (cls_name, conf, cx, cy).
    Per-frame update of the flicker-proof per-segment state machine.
    The object COUNT comes only from simultaneous boxes in a single frame -
    two detections in different frames are never summed into "two objects".
    """
    now = time.time()

    # Gather this frame's simultaneous detections per segment
    frame_seg = {seg: [] for seg in SEGMENT_LABELS}
    for cls_name, conf, cx, cy in dets:
        seg = zone.segment_of_point(cx, cy)
        if seg is not None:
            frame_seg[seg].append((cls_name, conf))

    for seg, s in segments.items():
        boxes = frame_seg[seg]
        raw_count = len(boxes)
        if boxes:
            best = max(boxes, key=lambda b: b[1])
            raw_cls, raw_conf = best
        else:
            raw_cls, raw_conf = None, 0.0

        # --- candidate reading: restart the stability clock on any change ---
        if raw_count != s["cand_count"]:
            s["cand_count"] = raw_count
            s["cand_since"] = now
        s["cand_cls"], s["cand_conf"] = raw_cls, raw_conf
        cand_age = now - s["cand_since"]

        st = s["state"]

        if st == "EMPTY":
            if s["cand_count"] > 0 and cand_age >= STABLE_S:
                _id_counter[seg] += 1
                s.update(state="OCCUPIED", count=s["cand_count"],
                         cls=raw_cls, conf=raw_conf,
                         obj_id=f"{seg}#{_id_counter[seg]}")

        elif st == "OCCUPIED":
            if s["cand_count"] == 0 and cand_age >= CLEAR_S:
                # removed by hand before any command
                s.update(state="EMPTY", count=0, cls=None, conf=0.0,
                         obj_id=None)
            elif s["cand_count"] > 0 and cand_age >= STABLE_S:
                s["count"] = s["cand_count"]   # e.g. a 2nd fruit placed
                s["cls"], s["conf"] = raw_cls, raw_conf

        elif st == "COMMANDED":
            pass  # arm may occlude the camera - ignore everything until done

        elif st == "VERIFY":
            # need a CLEAR_S-stable reading before judging the pick
            if cand_age >= CLEAR_S:
                if s["cand_count"] <= s["fired_count"] - 1:
                    _finish_sort(seg, s, success=True)
                    if s["cand_count"] == 0:
                        s.update(state="EMPTY", count=0, cls=None, conf=0.0,
                                 obj_id=None)
                    else:
                        # remaining fruit = a DIFFERENT object -> new ID,
                        # eligible for exactly one new command
                        _id_counter[seg] += 1
                        s.update(state="OCCUPIED", count=s["cand_count"],
                                 cls=raw_cls or s["cls"],
                                 conf=raw_conf,
                                 obj_id=f"{seg}#{_id_counter[seg]}")
                else:
                    _finish_sort(seg, s, success=False)
                    s["state"] = "ATTENTION"
                state["phase"] = "scan" if state["running"] else "idle"

        elif st == "ATTENTION":
            # The fired object leaving (count DROPS below what we fired at)
            # counts as sorted - late. Any remaining fruit is a NEW object
            # with its own single command.
            if cand_age >= CLEAR_S and s["cand_count"] < s["fired_count"]:
                _finish_sort(seg, s, success=True, late=True)
                if s["cand_count"] == 0:
                    s.update(state="EMPTY", count=0, cls=None, conf=0.0,
                             obj_id=None)
                else:
                    _id_counter[seg] += 1
                    s.update(state="OCCUPIED", count=s["cand_count"],
                             cls=raw_cls or s["cls"], conf=raw_conf,
                             obj_id=f"{seg}#{_id_counter[seg]}")


def reset_segments():
    """Clear-queue: forget all sector state; re-derives from live detections."""
    now = time.time()
    for s in segments.values():
        s.update(state="EMPTY", cls=None, conf=0.0, count=0,
                 cand_count=0, cand_cls=None, cand_conf=0.0, cand_since=now,
                 obj_id=None, fired_count=0, fired_cls=None, arm_ok=False)


def pick_target():
    """Next OCCUPIED sector matching the active order (one command each)."""
    best = None
    order = state["order"]
    for seg, s in segments.items():
        if s["state"] != "OCCUPIED":
            continue
        if order["filter"] != "all" and s["cls"] != order["filter"]:
            continue
        if order["segment"] and seg != order["segment"]:
            continue
        if best is None or s["conf"] > segments[best]["conf"]:
            best = seg
    return best


def fire_sort(seg):
    """Exactly ONE command per OCCUPIED object. State -> COMMANDED."""
    s = segments[seg]
    target = {"seg": seg, "cls": s["cls"], "conf": s["conf"],
              "obj_id": s["obj_id"], "count": s["count"]}
    s.update(state="COMMANDED", fired_count=s["count"], fired_cls=s["cls"])
    state["active"] = target
    state["phase"] = "sorting"
    state["_sort_start"] = time.time()

    def on_done(ok):
        with lock:
            s["arm_ok"] = ok
            s["cand_since"] = time.time()  # restart clock: judge fresh frames
            s["state"] = "VERIFY"
            state["phase"] = "verify"
        socketio.emit("toast", {
            "kind": "ok" if ok else "err",
            "msg": f"{target['cls']} from {seg} -> Bin {CLASS_TO_BIN[target['cls']]}"
                   if ok else f"Arm error while sorting {seg}",
        })
        emit_state()

    try:
        # Built here so the ARM tab animation shows the SAME speed preamble
        # the arm is about to receive.
        socketio.emit("arm_sequence", {
            "order_id": target["obj_id"],
            "cls": target["cls"], "seg": seg,
            "lines": build_sort_payload(seg, target["cls"]),
        })
    except Exception:
        pass
    launched = arm.sort_async(seg, s["cls"], on_done,
                             order_id=target["obj_id"])
    if not launched:            # arm busy - stay OCCUPIED, retry later
        s["state"] = "OCCUPIED"
        state["active"] = None
        state["phase"] = "scan"


# -------------------------------------------------------------- main loop
def camera_loop():
    cap = open_camera(state["camera_index"])
    t_prev = time.time()
    while True:
        with lock:
            if state["change_camera"]:
                cap.release()
                cap = open_camera(state["camera_index"])
                state["change_camera"] = False

        ok, frame = cap.read()
        if not ok:
            time.sleep(0.2)
            continue

        zone.update(frame)

        # ------------------------------------------------ detection
        dets = []
        res = model.predict(frame, imgsz=IMGSZ, conf=state["conf"], verbose=False)
        r = res[0]
        if r.boxes is not None:
            for b in r.boxes:
                cls_i = int(b.cls.item())
                conf = float(b.conf.item())
                x1, y1, x2, y2 = (int(v) for v in b.xyxy[0].tolist())
                cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                cls_name = CLASS_NAMES[cls_i] if cls_i < len(CLASS_NAMES) else str(cls_i)
                in_zone = zone.point_in_zone(cx, cy)
                if in_zone:
                    dets.append((cls_name, conf, cx, cy))
                color = CLASS_COLORS.get(cls_name, (200, 200, 200))
                if not in_zone:
                    color = tuple(int(c * 0.45) for c in color)
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 3)
                draw_label(frame, f"{cls_name} {conf*100:.0f}%", x1, y1, color)
                cv2.circle(frame, (cx, cy), 4, color, -1)

        with lock:
            update_segments(dets)

            # ------------------------------------------ state machine
            if state["running"] and not arm.busy and state["phase"] in ("scan", "idle"):
                state["phase"] = "scan"
                target = pick_target()
                if target:
                    fire_sort(target)
            if state["phase"] == "sorting" and \
                    time.time() - state["_sort_start"] > SORT_TIMEOUT:
                # watchdog: unstick a dead arm phase
                for s in segments.values():
                    if s["state"] == "COMMANDED":
                        s["state"] = "ATTENTION"
                state["active"] = None
                state["phase"] = "scan" if state["running"] else "idle"
                log_event({"event": "watchdog_reset"})

        zone.draw_overlay(frame, segments)

        # HUD strip
        now = time.time()
        state["fps"] = 0.9 * state["fps"] + 0.1 * (1.0 / max(1e-3, now - t_prev))
        t_prev = now
        hud = f"{zone.status} | {state['phase'].upper()} | {state['fps']:.1f} FPS"
        cv2.putText(frame, hud, (16, frame.shape[0] - 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2, cv2.LINE_AA)

        state["frame"] = frame

        emit_state()
        socketio.sleep(0.01) if hasattr(socketio, "sleep") else time.sleep(0.01)


# ------------------------------------------------------------------ routes
@app.route("/")
def index():
    return render_template("index.html")


@app.route("/feed")
def feed():
    def gen():
        while True:
            with lock:
                frame = state["frame"]
            if frame is None:
                time.sleep(0.05)
                continue
            ok, jpg = cv2.imencode(".jpg", frame,
                                   [cv2.IMWRITE_JPEG_QUALITY, 82])
            if ok:
                yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n"
                       + jpg.tobytes() + b"\r\n")
            time.sleep(0.03)
    return Response(gen(), mimetype="multipart/x-mixed-replace; boundary=frame")


@app.route("/api/assistant", methods=["POST"])
def api_assistant():
    msg = (request.json or {}).get("message", "").strip()
    if not msg:
        return jsonify({"reply": "Say something first.", "action": "chat"})
    intent = parse_intent(msg, assistant_context())
    action, target = intent["action"], intent["target"]

    segment = intent.get("segment")
    count = intent.get("count")
    with lock:
        if action == "sort":
            state["order"] = {"filter": target or "all",
                              "segment": segment,
                              "remaining": count if isinstance(count, int) and count > 0 else None}
            state["running"] = True
            state["phase"] = "scan"
        elif action == "start":
            state["order"] = {"filter": "all", "segment": None, "remaining": None}
            state["running"] = True
            state["phase"] = "scan"
        elif action == "stop":
            state["running"] = False
            state["phase"] = "idle"
            arm.stop()          # halt motion + drain firmware queue
        elif action == "clear":
            state["running"] = False
            state["phase"] = "idle"
            state["order"] = {"filter": "all", "segment": None, "remaining": None}
            reset_segments()
            arm.clear()         # firmware CLEAR: drop pending cmds
        elif action == "home":
            if not arm.busy:
                arm.home()
    log_event({"event": "assistant", "msg": msg, **intent})
    emit_state()
    return jsonify(intent)


# --------------------------------------------------------- arm status proxy
_NUM = r"-?\d+(?:\.\d+)?"


def _parse_arm_text(txt):
    """Understand a plain-text /api/status reply.

    The firmware answers in text, not JSON. The old proxy called r.json()
    on it, which threw, so the ARM tab saw UNREACHABLE and NEVER received
    joint angles - the 3D view then had nothing to sync to and silently
    treated wherever it happened to be as home. This parses both shapes.
    """
    out = {"status": "UNKNOWN", "order": "", "queue": 0,
           "angles": None, "raw": txt.strip()[:400]}
    up = txt.upper()
    for word in ("EXECUTING", "MOVING", "RUNNING", "BUSY", "HOMING",
                 "ERROR", "IDLE"):
        if word in up:
            out["status"] = word
            break
    m = re.search(r"\bID[:=]\s*(\S+)", txt, re.I)
    if m:
        out["order"] = m.group(1)
    m = re.search(r"QUEUE[:=]\s*(\d+)", up)
    if m:
        out["queue"] = int(m.group(1))

    # angles as "J1:12.0 J2:-4 ..." (preferred)
    js = re.findall(rf"J\s*([1-6])\s*[:=]\s*({_NUM})", txt, re.I)
    if len(js) >= 5:
        ang = [0.0] * 6
        for idx, val in js:
            ang[int(idx) - 1] = float(val)
        out["angles"] = ang
    else:
        # or a bare comma list of six numbers somewhere in the reply
        m = re.search(rf"((?:{_NUM}\s*,\s*){{5,}}{_NUM})", txt)
        if m:
            nums = [float(x) for x in m.group(1).split(",")]
            if len(nums) >= 6:
                out["angles"] = nums[:6]
    return out


@app.route("/api/arm/status")
def api_arm_status():
    """Proxy the ESP32 status for the ARM tab (avoids browser CORS).

    Returns JSON no matter what the firmware speaks, always with the same
    keys: status, order, queue, angles (list of 6 or null), simulated.
    """
    if SIMULATE_ARM:
        return jsonify({"status": "MOVING" if arm.busy else "IDLE",
                        "order": "", "queue": 0, "angles": None,
                        "simulated": True, "host": ESP32_HOST})
    try:
        import requests as _rq
        r = _rq.get(f"http://{ESP32_HOST}/api/status", timeout=3)
        try:
            data = r.json()
            if not isinstance(data, dict):
                raise ValueError("not an object")
            data.setdefault("status", "UNKNOWN")
            data.setdefault("order", "")
            data.setdefault("queue", 0)
            data.setdefault("angles", None)
        except Exception:
            data = _parse_arm_text(r.text)
        data["simulated"] = False
        data["host"] = ESP32_HOST
        return jsonify(data)
    except Exception as e:
        return jsonify({"status": "UNREACHABLE", "error": str(e),
                        "order": "", "queue": 0,
                        "angles": None, "simulated": False,
                        "host": ESP32_HOST})


@app.route("/api/arm/send", methods=["POST"])
def api_arm_send():
    """Manual pose/sequence sender for the ARM tab pose configurator."""
    body = request.json or {}
    lines = body.get("lines", [])
    lines = [str(l).strip() for l in lines if str(l).strip()][:200]
    if not lines:
        return jsonify({"ok": False, "error": "no lines"})
    if arm.busy:
        return jsonify({"ok": False, "error": "arm busy with a sort"})
    # animate in the simulator tab regardless of mode
    socketio.emit("arm_sequence", {"order_id": "manual",
                                   "cls": "-", "seg": "-", "lines": lines})
    if SIMULATE_ARM:
        log_event({"event": "manual_pose", "lines": lines, "sim": True})
        return jsonify({"ok": True, "sim": True})
    ok = arm._post_sequence(lines)
    log_event({"event": "manual_pose", "lines": lines, "ok": ok})
    return jsonify({"ok": ok, "sim": False,
                    "error": None if ok else (arm.last_error or "send failed")})


# ------------------------------------------------------------- socket ctrl
@socketio.on("ctrl")
def on_ctrl(data):
    cmd = data.get("cmd")
    with lock:
        if cmd == "start":
            state["running"] = True
            state["order"] = {"filter": "all", "segment": None, "remaining": None}
            state["phase"] = "scan"
        elif cmd == "stop":
            state["running"] = False
            state["phase"] = "idle"
            arm.stop()
        elif cmd == "clear":
            state["running"] = False
            state["phase"] = "idle"
            state["order"] = {"filter": "all", "segment": None, "remaining": None}
            reset_segments()
            arm.clear()
        elif cmd == "home" and not arm.busy:
            arm.home()
        elif cmd == "go_zero" and not arm.busy:
            # Drive the arm to the exact pose the 3D view calls zero, so the
            # simulator and the hardware can be re-synced deliberately.
            arm.go_zero()
            socketio.emit("arm_sequence", {"order_id": "zero", "cls": "-",
                                           "seg": "-",
                                           "lines": arm_sender.home_sequence()})
        elif cmd == "clear_log":
            state["stats"]["history"] = []
        elif cmd == "set_conf":
            state["conf"] = max(0.1, min(0.9, float(data.get("value", CONF))))
        elif cmd == "set_speed":
            eff = arm_sender.set_motion(speed=data.get("value"))
            state["speed"] = eff["speed"]
            log_event({"event": "motion", **eff})
        elif cmd == "set_accel":
            eff = arm_sender.set_motion(accel=data.get("value"))
            state["accel"] = eff["accel"]
            log_event({"event": "motion", **eff})
        elif cmd == "set_camera":
            idx = int(data.get("value", 0))
            if idx != state["camera_index"]:
                state["camera_index"] = idx
                state["change_camera"] = True
    emit_state()


@socketio.on("connect")
def on_connect():
    emit_state()


if __name__ == "__main__":
    threading.Thread(target=camera_loop, daemon=True).start()
    print(f"Arm host: {ESP32_HOST}  (simulate={SIMULATE_ARM})")
    print("Apples & Oranges dashboard -> http://localhost:5800")
    socketio.run(app, host="0.0.0.0", port=5800,
                 debug=False, allow_unsafe_werkzeug=True)
