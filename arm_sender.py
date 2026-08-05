"""
arm_sender.py - CubicoreArmController for the Apples & Oranges sorter.
Wi-Fi G-code sender for the 5-DOF ESP32-S3 CycloiDrive arm (+ gripper).

Works with the patched "Sorter Edition" firmware:
  - every order is sent with an "ID:<obj_id>" first line (the segment
    generation ID, e.g. S2#7);
  - the firmware runs ONE order at a time (HTTP 409 "BUSY" while working)
    and ignores duplicate order IDs ("DUP"), so a retry or double-send can
    never make the arm repeat a sort or grab at an empty spot;
  - STOP / HO / GH go through /api/override, which is always accepted.

Brain (this script) -> Wi-Fi -> Muscle (ESP32-S3). 5-DOF: J6 channel unused.

6 named sequences (4 PICK + 2 DROP) cover all 8 segment x class combos:
    Any sort = PICK[segment] then DROP[class_name]

MOTION (v10): speed and acceleration are no longer frozen constants. They
live in the MOTION dict and are changed at runtime by set_motion(), which
the dashboard calls when you move the SPD / ACC sliders in the ARM tab.
build_sort_payload() rebuilds the S:/A: preamble on every call, so a slider
change now actually reaches the next sort instead of being ignored.

Host: set the AO_ARM_HOST environment variable, or pass --host.

CLI test:
    python arm_sender.py --host ao-arm.local status
    python arm_sender.py --host ao-arm.local sort S2 Apple
    python arm_sender.py list
"""
import argparse
import logging
import os
import threading
import time
from typing import Callable, Dict, List, Optional

import requests

logger = logging.getLogger(__name__)

# Default arm address. Override with:  set AO_ARM_HOST=192.168.x.x
DEFAULT_HOST = os.environ.get("AO_ARM_HOST", "ao-arm.local")

# ---------------------------------------------------------------------------
# 4 PICK sequences - one per platform segment
#   S1 | S2   <- top row (away from arm)
#   S3 | S4   <- bottom row (near arm)
# Command format: J1,J2,J3,J4,J5,J6,T
#   T = O (open gripper) | C (close gripper) | X (no change)
# ---------------------------------------------------------------------------
PICK_SEQUENCES: dict = {
    # Two-pose picks: approach (gripper open, wrist high), then descend to
    # the fruit. The grab happens on the first line of the DROP sequence
    # (gripper closes as the transit move starts).
    "S1": [
        "45,90,80,-40,-17,-100,O",
        "90,85,32,42,-17,0,O",
    ],
    "S2": [
        "45,90,80,-40,7,-100,O",
        "90,85,32,40,7,0,O",
    ],
    "S3": [
        "90,90,90,-40,-25,-100,O",
        "90,80,85,3,-25,0,O",
    ],
    "S4": [
        "90,90,90,-40,10,-100,O",
        "90,80,85,3,10,0,O",
    ],
}

# ---------------------------------------------------------------------------
# BINS - the explicit sorting destinations:
#   BIN 1 = APPLES (right side)
#   BIN 2 = ORANGES (left side)
# Re-tune these joint angles to your own bin positions before deploying.
# ---------------------------------------------------------------------------
BIN_FOR_CLASS = {"Apple": 1, "Orange": 2}
BIN_LABELS = {1: "Bin 1 (Apples)", 2: "Bin 2 (Oranges)"}

DROP_SEQUENCES: dict = {
    1: [  # BIN 1 (BIN A) - APPLES
        "D:300",
        "0,50,80,-40,-17,-90,C",
        "90,50,0,60,-17,-100,C",
        "90,50,0,60,-17,-100,O",
        "D:500",
        "45,90,80,-40,0,-100,O",
        "0,0,0,0,0,0,O",
    ],
    2: [  # BIN 2 (BIN B) - ORANGES
        "D:300",
        "0,50,80,-40,15,-90,C",
        "90,50,0,60,15,-100,C",
        "90,50,0,60,15,-100,O",
        "D:500",
        "45,90,80,-40,0,-100,O",
        "0,0,0,0,0,0,O",
    ],
}

# ---------------------------------------------------------------------------
# MOTION - live speed / acceleration
#
# The firmware clamps anything above its own ceiling on receipt
# (UNIVERSAL_MAX_SPEED_DEG_PER_SEC / UNIVERSAL_MAX_ACCEL_DEG_PER_SEC_SQ in
# main.cpp, currently 50 / 25), so we clamp here too. That keeps the log and
# the dashboard honest about what the arm was actually asked to do, instead
# of reporting a number the firmware silently threw away.
#
# If the arm still feels slow at SPEED = 50, the ceiling is in the firmware,
# not here - raise UNIVERSAL_MAX_SPEED_DEG_PER_SEC in main.cpp and re-flash.
# ---------------------------------------------------------------------------
FW_MAX_SPEED = 50      # firmware ceiling, deg/s
FW_MAX_ACCEL = 25      # firmware ceiling, deg/s^2
DEFAULT_SPEED = 40
DEFAULT_ACCEL = 25

MOTION: Dict[str, int] = {"speed": DEFAULT_SPEED, "accel": DEFAULT_ACCEL}


def set_motion(speed=None, accel=None) -> Dict[str, int]:
    """Change the speed/accel used by every sequence built from now on.

    Called by the dashboard when the SPD / ACC sliders move. Values are
    clamped to the firmware ceiling. Returns the effective values so the UI
    can show what was really applied.
    """
    if speed is not None:
        try:
            MOTION["speed"] = int(max(1, min(FW_MAX_SPEED, round(float(speed)))))
        except (TypeError, ValueError):
            pass
    if accel is not None:
        try:
            MOTION["accel"] = int(max(1, min(FW_MAX_ACCEL, round(float(accel)))))
        except (TypeError, ValueError):
            pass
    return dict(MOTION)


def get_motion() -> Dict[str, int]:
    return dict(MOTION)


def preamble() -> List[str]:
    """Fresh S:/A: header - built per call so slider changes take effect."""
    return [f"S:{MOTION['speed']}", f"A:{MOTION['accel']}"]


def home_sequence() -> List[str]:
    """Explicit move to the mechanical zero pose (0,0,0,0,0,0, gripper open).

    This is what the ARM tab's "Send arm to 0" button uses. Unlike the GH
    override (which runs the firmware's own homing routine), this puts the
    arm exactly where the 3D view's zero pose is, so the two always agree.
    """
    return preamble() + ["0,0,0,0,0,0,O"]


def build_sort_payload(segment: str, class_name: str) -> List[str]:
    """
    Full ordered G-code string list: PICK[segment] -> DROP[bin].
    Exactly 8 combinations exist (4 sectors x 2 bins):
        S1->Bin1  S1->Bin2  S2->Bin1  S2->Bin2
        S3->Bin1  S3->Bin2  S4->Bin1  S4->Bin2
    The class decides the bin: Apple -> Bin 1, Orange -> Bin 2.
    """
    pick = PICK_SEQUENCES.get(segment)
    bin_no = BIN_FOR_CLASS.get(class_name)
    if pick is None:
        raise ValueError(f"Unknown segment {segment!r}. Valid: {list(PICK_SEQUENCES)}")
    if bin_no is None:
        raise ValueError(f"Unknown class {class_name!r}. Valid: {list(BIN_FOR_CLASS)}")
    return preamble() + pick + DROP_SEQUENCES[bin_no]


def list_combos() -> List[str]:
    """All 8 sector->bin combinations with their payload sizes."""
    out = []
    for seg in PICK_SEQUENCES:
        for cls, b in BIN_FOR_CLASS.items():
            n = len(build_sort_payload(seg, cls))
            out.append(f"{seg} -> {BIN_LABELS[b]} [{cls}] ({n} lines)")
    return out


# ---------------------------------------------------------------------------
class CubicoreArmController:
    """
    HTTP client for the ESP32-S3 arm (Cubicore Muscle Layer).

    Endpoints:
        POST /api/sequence  - inject G-code command queue (text/plain)
        POST /api/override  - STOP / CLEAR / HO / GH, accepted any time
        GET  /api/status    - poll arm state

    Non-blocking sort via sort_async(); guard re-entry with .busy.
    """

    _STATUS_EXECUTING = ("EXECUTING", "BUSY", "RUNNING", "MOVING")

    def __init__(self, host: str, port: int = 80, simulate: bool = False,
                 timeout: float = 5.0):
        self.host, self.port = host, port
        self.simulate = simulate
        self.timeout = timeout
        self._busy = False
        self._lock = threading.Lock()
        self.last_error: Optional[str] = None

    # ------------------------------------------------------------- plumbing
    @property
    def base(self) -> str:
        return f"http://{self.host}:{self.port}"

    @property
    def busy(self) -> bool:
        return self._busy

    def _post_sequence(self, commands: List[str]) -> bool:
        if self.simulate:
            logger.info("[SIM] sequence (%d cmds) @ S:%d A:%d",
                        len(commands), MOTION["speed"], MOTION["accel"])
            time.sleep(4.0)  # pretend the arm is moving
            return True
        try:
            r = requests.post(f"{self.base}/api/sequence",
                              data="\n".join(commands),
                              headers={"Content-Type": "text/plain"},
                              timeout=self.timeout)
            if r.status_code == 409:
                self.last_error = "arm rejected order: BUSY"
                logger.warning("Arm BUSY - order not accepted")
                return False
            if r.ok and r.text.strip() == "DUP":
                logger.info("Arm reports duplicate order - treated as done")
                return True
            return r.ok
        except requests.RequestException as e:
            self.last_error = str(e)
            logger.error("Arm POST failed: %s", e)
            return False

    def status(self) -> str:
        if self.simulate:
            return "IDLE (sim)"
        try:
            r = requests.get(f"{self.base}/api/status", timeout=self.timeout)
            return r.text.strip() if r.ok else f"HTTP {r.status_code}"
        except requests.RequestException as e:
            self.last_error = str(e)
            return "UNREACHABLE"

    def is_executing(self) -> bool:
        if self.simulate:
            return self._busy
        s = self.status().upper()
        return any(k in s for k in self._STATUS_EXECUTING)

    def wait_idle(self, timeout: float = 180.0, poll: float = 0.5) -> bool:
        t0 = time.time()
        while time.time() - t0 < timeout:
            if not self.is_executing():
                return True
            time.sleep(poll)
        return False

    # ------------------------------------------------------------- commands
    def sort_async(self, segment: str, class_name: str,
                   on_done: Optional[Callable[[bool], None]] = None,
                   order_id: Optional[str] = None) -> bool:
        """Fire one full pick+drop. Returns False if arm busy/invalid.
        order_id (e.g. 'S2#7') is prepended as an ID line for firmware dedup.

        The payload is built HERE, at fire time, so it always carries the
        speed/accel currently set in MOTION.
        """
        with self._lock:
            if self._busy:
                return False
            self._busy = True

        def run():
            ok = False
            try:
                payload = build_sort_payload(segment, class_name)
                if order_id:
                    payload = [f"ID:{order_id}"] + payload
                ok = self._post_sequence(payload)
                if ok and not self.simulate:
                    time.sleep(1.0)  # let ESP start executing
                    ok = self.wait_idle()
            finally:
                self._busy = False
                if on_done:
                    on_done(ok)

        threading.Thread(target=run, daemon=True).start()
        return True

    def _post_override(self, cmd: str) -> bool:
        """STOP / CLEAR / HO / GH via the always-accepted override endpoint."""
        if self.simulate:
            logger.info("[SIM] override %s", cmd)
            return True
        try:
            r = requests.post(f"{self.base}/api/override", data=cmd,
                              headers={"Content-Type": "text/plain"},
                              timeout=self.timeout)
            if r.status_code == 404:  # old firmware without /api/override
                return self._post_sequence([cmd])
            return r.ok
        except requests.RequestException as e:
            self.last_error = str(e)
            return False

    def home(self) -> bool:
        """Firmware homing routine (GH override)."""
        if self._busy:
            return False
        return self._post_override("GH")

    def go_zero(self) -> bool:
        """Move to the explicit 0,0,0,0,0,0 pose the 3D view calls home.

        Use this when the physical arm and the live view disagree: it drives
        the arm to the exact pose the simulator draws at zero, so you can
        re-sync both sides without guessing.
        """
        if self._busy:
            return False
        return self._post_sequence(home_sequence())

    def stop(self, go_home: bool = True) -> bool:
        """Immediate halt + queue drain, then return to the home pose.
        (STOP freezes mid-motion; the follow-up GH brings the arm back to
        its initial position so a stop/clear always ends in a known state.)"""
        ok = self._post_override("STOP")
        if ok and go_home:
            time.sleep(0.2)  # let the halt settle
            ok = self._post_override("GH") and ok
        return ok

    def clear(self, go_home: bool = True) -> bool:
        """Drop the pending queue (current move finishes safely), then home.
        The GH is enqueued behind the in-progress move, so the arm completes
        its motion and returns to the initial position."""
        ok = self._post_override("CLEAR")
        if ok and go_home:
            ok = self._post_override("GH") and ok
        return ok


# ---------------------------------------------------------------- CLI test
if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default=DEFAULT_HOST,
                    help="arm hostname or IP (default: $AO_ARM_HOST or ao-arm.local)")
    ap.add_argument("--port", type=int, default=80)
    ap.add_argument("--speed", type=int, default=None, help=f"1..{FW_MAX_SPEED}")
    ap.add_argument("--accel", type=int, default=None, help=f"1..{FW_MAX_ACCEL}")
    ap.add_argument("cmd", choices=["status", "sort", "home", "zero", "list"])
    ap.add_argument("args", nargs="*")
    a = ap.parse_args()

    set_motion(a.speed, a.accel)

    if a.cmd == "list":
        print("Segments:", list(PICK_SEQUENCES))
        print("Bins    :", BIN_LABELS)
        print("Motion  :", get_motion())
        print("\nAll 8 sort combinations:")
        for line in list_combos():
            print("  ", line)
    else:
        ctrl = CubicoreArmController(a.host, a.port)
        if a.cmd == "status":
            print(ctrl.status())
        elif a.cmd == "home":
            print("home ->", ctrl.home())
        elif a.cmd == "zero":
            print("zero ->", ctrl.go_zero())
        elif a.cmd == "sort":
            seg, cls = a.args[0], a.args[1]
            done = threading.Event()
            ctrl.sort_async(seg, cls, lambda ok: (print("done:", ok), done.set()))
            done.wait()
