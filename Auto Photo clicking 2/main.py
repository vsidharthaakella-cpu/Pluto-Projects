"""
Pluto Human Detection (Camera Wi-Fi mode)
==========================================
Detects humans in the drone's live camera feed (YOLOv8/YOLO11) and lets you
fly with keyboard keys. Movement commands are "pulsed": each key press
triggers a fixed-duration move, then auto-resets — giving precise,
repeatable steps instead of relying on continuous key-hold detection.

UNIQUE-PERSON CAPTURE (real ReID model, via Ultralytics BoT-SORT)
--------------------------------------------------------------------
Earlier versions tried: (a) a plain time cooldown, (b) a hand-rolled
centroid position-tracker, (c) a hand-rolled color-histogram "fingerprint."
All three had real weakness on a moving drone camera. This version uses an
actual trained appearance embedding model instead of a heuristic:

  Ultralytics' BoT-SORT tracker has a built-in Re-ID mode (OSNet) — see
  botsort_reid.yaml next to this script. Instead of `model.predict()`, we
  call `model.track(..., tracker="botsort_reid.yaml", persist=True)`.
  BoT-SORT then does two things per detection:
    1. Motion/position matching between frames (like before), PLUS
    2. A learned appearance-embedding comparison (OSNet) so that even
       when a person disappears and reappears — occlusion, drone yaw,
       walking off-frame — they're re-linked to their SAME track ID
       instead of getting a new one.

  A track is only "confirmed" (and captured) once it's persisted for a
  few frames, to filter one-frame flicker/false positives. Each track ID
  is captured exactly once, ever, for the run.

First run note: Ultralytics will auto-download the small OSNet ReID
weights the first time `model.track()` runs with `with_reid: true` — make
sure the machine running this has internet access at least once, or
place the weights file where Ultralytics expects it beforehand.

Requires: `pip install lapx` (BoT-SORT's matching backend) in addition to
your existing ultralytics/opencv/av install.
"""

import os
import sys
import threading
import time
from datetime import datetime

import cv2
import numpy as np

try:
    import av
except ImportError:
    av = None

from plutocam import LWDrone
from plutocontrol import Pluto


MODEL_PATH = "yolo11n.onnx"           # nano — only variant available; precision comes from imgsz/conf/iou tuning below
TRACKER_CONFIG = "botsort_reid.yaml"  # BoT-SORT + OSNet ReID (see file next to this script)
CONFIDENCE = 0.45            # higher = fewer false positives, may miss partial/occluded people
IOU_THRESHOLD = 0.45         # lower = stricter box merging, fewer duplicate/overlapping boxes
IMG_SIZE = 640               # fixed — this yolo11n.onnx was exported at 640x640, changing this will error or no-op
DISPLAY_WIDTH = 480          # window display size (detection still runs at IMG_SIZE)
PULSE_DURATION = 0.5         # seconds each movement command stays active before auto-reset

CAPTURE_DIR = "captures"     # folder where detection photos are saved
MIN_CONSEC_FRAMES = 3        # frames a track must persist before we trust it enough to capture

latest_frame = None
frame_lock = threading.Lock()
stop_event = threading.Event()

_pulse_timer = None
_pulse_lock = threading.Lock()

# Arrow key codes differ by platform (cv2.waitKeyEx)
ARROW_LEFT = (65361, 2424832)
ARROW_RIGHT = (65363, 2555904)
ARROW_UP = (65362, 2490368)
ARROW_DOWN = (65364, 2621440)


# --------------------------------------------------------------------------
# Unique-person capture logic — driven by BoT-SORT+ReID track IDs
# --------------------------------------------------------------------------
class UniquePersonCapturer:
    def __init__(self):
        self.confirm_counts = {}   # track_id -> consecutive visible-frame count
        self.captured_ids = set()  # track_ids already saved (capture once per id, ever)
        os.makedirs(CAPTURE_DIR, exist_ok=True)

    def _save(self, frame, box, conf, track_id):
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
        filepath = os.path.join(CAPTURE_DIR, f"person{track_id}_{timestamp}.jpg")
        try:
            cv2.imwrite(filepath, frame)
            print(f"[CAPTURE] New person (track {track_id}, conf {conf:.2f}) -> {filepath}")
        except Exception as exc:
            print(f"[WARN] Failed to save capture: {type(exc).__name__}: {exc}")

    def process(self, frame, tracked_detections):
        """tracked_detections: list of (track_id, box, conf).
        Returns list of (box, conf, is_new_confirmed_person) for drawing."""
        seen_ids = set()
        annotated = []

        for track_id, box, conf in tracked_detections:
            seen_ids.add(track_id)
            self.confirm_counts[track_id] = self.confirm_counts.get(track_id, 0) + 1

            just_confirmed = False
            if (track_id not in self.captured_ids
                    and self.confirm_counts[track_id] >= MIN_CONSEC_FRAMES):
                self._save(frame, box, conf, track_id)
                self.captured_ids.add(track_id)
                just_confirmed = True

            annotated.append((box, conf, just_confirmed))

        # Drop bookkeeping for tracks that vanished this frame (BoT-SORT's own
        # track_buffer already decides when an ID is truly gone vs. just
        # briefly occluded — we only clean up our local counters here).
        for tid in list(self.confirm_counts.keys()):
            if tid not in seen_ids:
                self.confirm_counts.pop(tid, None)

        return annotated


def video_loop(cam):
    """Decode raw H.264 packets from the drone cam into BGR frames."""
    global latest_frame
    codec = av.CodecContext.create("h264", "r")

    for chunk in cam.start_video_stream():
        if stop_event.is_set():
            break
        try:
            for packet in codec.parse(chunk.frame_bytes):
                for frame in codec.decode(packet):
                    img = frame.to_ndarray(format="bgr24")
                    with frame_lock:
                        latest_frame = img
        except Exception:
            continue


def safe_drone_call(pluto, method_name):
    try:
        getattr(pluto, method_name)()
    except Exception as exc:
        print(f"[WARN] {method_name} failed: {type(exc).__name__}: {exc}")


def pulse_command(pluto, method_name, duration=PULSE_DURATION):
    """Run a movement command for a fixed duration, then auto-reset.

    Cancels any in-flight pulse first, so each key press gives one
    precise, self-terminating movement step.
    """
    global _pulse_timer
    with _pulse_lock:
        if _pulse_timer is not None:
            _pulse_timer.cancel()

        safe_drone_call(pluto, method_name)

        def _finish():
            safe_drone_call(pluto, "reset")

        _pulse_timer = threading.Timer(duration, _finish)
        _pulse_timer.daemon = True
        _pulse_timer.start()


def main():
    if av is None:
        print("[FATAL] Missing package 'av'. Install with: pip install av")
        sys.exit(1)

    if not os.path.exists(TRACKER_CONFIG):
        print(f"[FATAL] Missing tracker config '{TRACKER_CONFIG}'. "
              f"Keep it in the same folder as this script.")
        sys.exit(1)

    from ultralytics import YOLO

    print(f"[1] Loading YOLO model ({MODEL_PATH})...")
    model = YOLO(MODEL_PATH, task="detect")
    model.predict(np.zeros((IMG_SIZE, IMG_SIZE, 3), dtype="uint8"), verbose=False)  # warm-up
    print("[OK] Model loaded. (ReID weights will auto-download on first track() call "
          "if not already cached.)")

    print("[2] Connecting to Pluto (camera Wi-Fi)...")
    pluto = Pluto()
    pluto.cam()          # must be before connect(): targets 192.168.0.1:9060
    pluto.connect()
    if not pluto.connected:
        print("[FATAL] Could not connect. Join the camera module's Wi-Fi and retry.")
        sys.exit(1)
    print("[OK] Connected.")

    cam = LWDrone()       # video host defaults to 192.168.0.1
    vthread = threading.Thread(target=video_loop, args=(cam,), daemon=True)
    vthread.start()

    capturer = UniquePersonCapturer()

    cv2.namedWindow("Pluto Human Detection")
    print("\nControls:")
    print("SPACE=Arm+Takeoff  X=Land  W/S=Fwd/Back  A/D=Left/Right")
    print("UP/DOWN=Throttle  LEFT/RIGHT=Yaw  Q=Quit")
    print(f"(each move key = one {PULSE_DURATION}s precise pulse, auto-resets)")
    print(f"(photos auto-saved to '{CAPTURE_DIR}/' once per unique person, "
          f"via BoT-SORT+ReID tracking, confirm={MIN_CONSEC_FRAMES}f)\n")

    airborne = False

    try:
        while True:
            with frame_lock:
                frame = latest_frame.copy() if latest_frame is not None else None

            if frame is None:
                key = cv2.waitKeyEx(50)  # waiting for first decoded frame
            else:
                results = model.track(
                    frame,
                    persist=True,
                    tracker=TRACKER_CONFIG,
                    classes=[0],          # class 0 = person
                    conf=CONFIDENCE,
                    iou=IOU_THRESHOLD,
                    imgsz=IMG_SIZE,
                    verbose=False,
                )[0]

                tracked_detections = []
                if results.boxes is not None and results.boxes.id is not None:
                    xyxy = results.boxes.xyxy.cpu().numpy()
                    confs = results.boxes.conf.cpu().numpy()
                    ids = results.boxes.id.cpu().numpy().astype(int)
                    for box, conf, track_id in zip(xyxy, confs, ids):
                        x1, y1, x2, y2 = map(int, box)
                        tracked_detections.append((int(track_id), (x1, y1, x2, y2), float(conf)))

                annotated = capturer.process(frame, tracked_detections)

                for box, conf, is_new in annotated:
                    x1, y1, x2, y2 = box
                    color = (0, 200, 255) if is_new else (0, 255, 0)
                    label = f"NEW {conf:.2f}" if is_new else f"{conf:.2f}"
                    cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                    cv2.putText(frame, label, (x1, max(20, y1 - 10)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

                h, w = frame.shape[:2]
                disp_h = int(h * (DISPLAY_WIDTH / w))
                small = cv2.resize(frame, (DISPLAY_WIDTH, disp_h))
                cv2.imshow("Pluto Human Detection", small)
                key = cv2.waitKeyEx(1)
            
            key_char = key & 0xFF  # for regular ASCII keys

            if key_char == ord("q"):
                break
            elif key_char == ord(" "):
                print("[CMD] Arming...")
                safe_drone_call(pluto, "arm")
                time.sleep(1.5)
                print("[CMD] Taking off...")
                safe_drone_call(pluto, "take_off")
                airborne = True
            elif key_char == ord("x"):
                safe_drone_call(pluto, "land")
                airborne = False
            elif key_char == ord("w"):
                pulse_command(pluto, "forward")
            elif key_char == ord("s"):
                pulse_command(pluto, "backward")
            elif key_char == ord("a"):
                pulse_command(pluto, "left")
            elif key_char == ord("d"):
                pulse_command(pluto, "right")
            elif key in ARROW_UP:
                pulse_command(pluto, "increase_height")
            elif key in ARROW_DOWN:
                pulse_command(pluto, "decrease_height")
            elif key in ARROW_LEFT:
                pulse_command(pluto, "left_yaw")
            elif key in ARROW_RIGHT:
                pulse_command(pluto, "right_yaw")

    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        with _pulse_lock:
            if _pulse_timer is not None:
                _pulse_timer.cancel()
        cam.stop_video_stream()
        if airborne:
            safe_drone_call(pluto, "land")
        safe_drone_call(pluto, "disarm")
        safe_drone_call(pluto, "disconnect")
        cv2.destroyAllWindows()
        print("[OK] Stopped, disarmed, disconnected.")


if __name__ == "__main__":
    main()