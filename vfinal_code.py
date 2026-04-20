"""
LiDAR-Driven Haptic Backpack — Python Main Controller
─────────────────────────────────────────────────────
Streams RGB+Depth from iPhone (via Record3D app) → processes depth data →
commands two servos on an Arduino Mega 2560 via serial.

Hardware:
  - iPhone with LiDAR (12 Pro+ / 13 Pro+ / 14 Pro / 15 Pro / 16 Pro)
    running Record3D app in USB (or Wi-Fi) streaming mode
  - Arduino Mega 2560 with two servos on pins 9 (left) and 10 (right)
    flashed with the matching firmware (expects "L:<angle>\\n" / "R:<angle>\\n")

Usage:
  1. Plug iPhone into laptop, open Record3D, start USB streaming
  2. Plug Arduino Mega into laptop
  3. Update SERIAL_PORT below to match your Mega's COM port
  4. Run: python lidar_haptic.py
  5. Keypresses in the OpenCV windows:
       q - quit
       s - save snapshot PNG
       p - pause/resume processing
"""

import threading
import time
from collections import deque

import cv2
import numpy as np
import serial
from record3d import Record3DStream


# ══════════════════════════════════════════════════════════════════
# CONFIG — TUNE THESE
# ══════════════════════════════════════════════════════════════════

# Serial port of Arduino Mega.
#   Windows: "COM3", "COM4", etc. — check Device Manager
#   Mac:     "/dev/cu.usbmodem1401" or similar
#   Linux:   "/dev/ttyACM0"
SERIAL_PORT = "COM8"
SERIAL_BAUD = 115200

# Detection thresholds
OBSTACLE_DISTANCE_M = 1.5       # Trigger haptic if obstacle closer than this (meters)
DROPOFF_FLOOR_M     = 1.2       # Expected floor distance when phone pointed down
DROPOFF_THRESHOLD_M = 0.3       # How much farther than floor = dropoff detected
CLOSE_OBJECT_M      = 0.2       # Ignore readings closer than this (noise)

# Smoothing — how many frames must agree before haptic state changes
SMOOTHING_WINDOW = 5            # Keep last N frames of detections
SMOOTHING_MAJORITY = 3          # Need this many agreeing to flip state

# Servo angles (must match Arduino firmware)
SERVO_REST  = 0
SERVO_PULSE = 90

# Display options
SHOW_DEPTH_WINDOW   = True
SHOW_FPS            = True
PRINT_STATS         = True


# ══════════════════════════════════════════════════════════════════
# SERVO CONTROLLER — talks to Arduino over serial
# ══════════════════════════════════════════════════════════════════

class ServoController:
    """Drives two haptic servos on left/right straps via Arduino Mega."""

    def __init__(self, port, baud=115200):
        print(f"[Servo] Opening {port} at {baud} baud...")
        self.ser = serial.Serial(port, baud, timeout=0.1)
        time.sleep(2)  # Mega resets when serial opens — wait for boot
        self.ser.reset_input_buffer()
        self._last_state = {'L': None, 'R': None}
        print("[Servo] Ready")

    def set(self, side, blocked):
        """Command a servo. Only sends if state actually changed (avoids spam)."""
        assert side in ('L', 'R')
        angle = SERVO_PULSE if blocked else SERVO_REST
        if self._last_state[side] == angle:
            return
        self._last_state[side] = angle
        try:
            self.ser.write(f"{side}:{angle}\n".encode())
        except serial.SerialException as e:
            print(f"[Servo] Write error: {e}")

    def close(self):
        """Reset both servos and close port."""
        try:
            self.set('L', False)
            self.set('R', False)
            time.sleep(0.2)
            self.ser.close()
            print("[Servo] Closed")
        except Exception as e:
            print(f"[Servo] Close error: {e}")


# ══════════════════════════════════════════════════════════════════
# PERCEPTION — pure functions of a depth array
# ══════════════════════════════════════════════════════════════════

def clean_depth(depth, confidence):
    """Zero-out low-confidence pixels. Always call this first."""
    d = depth.copy()
    if confidence is not None:
        d[confidence < 2] = 0
    return d


def distance_ahead(depth, region_size=30):
    """Median distance in a small center region (meters). None if no valid data."""
    h, w = depth.shape
    cy, cx = h // 2, w // 2
    roi = depth[cy - region_size:cy + region_size,
                cx - region_size:cx + region_size]
    valid = roi[roi > 0]
    return float(np.median(valid)) if len(valid) > 10 else None


def closest_point(depth, min_dist=CLOSE_OBJECT_M):
    """Return (row, col, distance) of closest valid point, or None."""
    d = depth.copy()
    d[d < min_dist] = np.inf
    d[d == 0] = np.inf
    min_d = np.min(d)
    if not np.isfinite(min_d):
        return None
    r, c = np.unravel_index(np.argmin(d), d.shape)
    return int(r), int(c), float(min_d)


def obstacle_directions(depth, threshold=OBSTACLE_DISTANCE_M):
    """Check left/center/right thirds. Returns (L_blocked, C_blocked, R_blocked)."""
    h, w = depth.shape
    strip = depth[int(h * 0.3):int(h * 0.7), :]
    third = strip.shape[1] // 3
    regions = [strip[:, :third],
               strip[:, third:2 * third],
               strip[:, 2 * third:]]

    blocked = []
    for r in regions:
        valid = r[r > 0]
        if len(valid) < 10:
            blocked.append(False)
        else:
            close = np.sum(valid < threshold)
            blocked.append(close > len(valid) * 0.15)
    return tuple(blocked)


def detect_dropoff(depth,
                   floor_dist=DROPOFF_FLOOR_M,
                   threshold=DROPOFF_THRESHOLD_M):
    """Detect downward drop in the bottom-center of the frame."""
    h, w = depth.shape
    strip = depth[int(h * 0.85):, w // 3:2 * w // 3]
    valid = strip[strip > 0]
    if len(valid) < 20:
        return False
    far_pixels = np.sum(valid > floor_dist + threshold)
    return bool(far_pixels > len(valid) * 0.3)


# ══════════════════════════════════════════════════════════════════
# VISUALIZATION
# ══════════════════════════════════════════════════════════════════

def make_depth_vis(depth):
    """Colorize depth map (0-5m range) for display."""
    depth_vis = np.clip(depth * 51, 0, 255).astype(np.uint8)
    return cv2.applyColorMap(depth_vis, cv2.COLORMAP_PLASMA)


def draw_overlays(img, zones, smooth_zones, dropoff, closest, ahead_m, fps):
    """Annotate RGB image with perception output."""
    h, w = img.shape[:2]
    third = w // 3
    left, center, right = zones
    smooth_left, smooth_center, smooth_right = smooth_zones

    # Red tints on blocked zones (smoothed ones are darker red)
    overlay = img.copy()
    if smooth_left:    cv2.rectangle(overlay, (0, 0),           (third, h),     (0, 0, 200), -1)
    elif left:         cv2.rectangle(overlay, (0, 0),           (third, h),     (0, 0, 120), -1)
    if smooth_center:  cv2.rectangle(overlay, (third, 0),       (2 * third, h), (0, 0, 200), -1)
    elif center:       cv2.rectangle(overlay, (third, 0),       (2 * third, h), (0, 0, 120), -1)
    if smooth_right:   cv2.rectangle(overlay, (2 * third, 0),   (w, h),         (0, 0, 200), -1)
    elif right:        cv2.rectangle(overlay, (2 * third, 0),   (w, h),         (0, 0, 120), -1)
    img = cv2.addWeighted(overlay, 0.3, img, 0.7, 0)

    # Closest-point marker
    if closest:
        r, c, d = closest
        cv2.circle(img, (c, r), 8, (0, 255, 255), 2)
        cv2.putText(img, f"{d:.2f}m", (c + 12, r),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

    # Top status bar
    y = 25
    if ahead_m is not None:
        cv2.putText(img, f"Ahead: {ahead_m:.2f}m", (10, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        y += 25
    if dropoff:
        cv2.putText(img, "!! DROPOFF !!", (10, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 140, 255), 2)
        y += 25
    if fps is not None:
        cv2.putText(img, f"FPS: {fps:.1f}", (w - 110, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

    # Haptic state indicator (bottom)
    hap_y = h - 20
    cv2.putText(img, f"L:{'ON ' if smooth_left else 'off'}   R:{'ON ' if smooth_right else 'off'}",
                (10, hap_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                (0, 255, 0), 2)

    return img


# ══════════════════════════════════════════════════════════════════
# MAIN APP
# ══════════════════════════════════════════════════════════════════

class LidarHapticApp:

    def __init__(self):
        self.session = None
        self.event = threading.Event()
        self.servos = None
        self.zone_history = deque(maxlen=SMOOTHING_WINDOW)
        self.paused = False
        self.last_frame_time = time.time()
        self.fps = 0.0

    # ── Record3D callbacks ──────────────────────────────────────

    def on_new_frame(self):
        self.event.set()

    def on_stream_stopped(self):
        print("[Record3D] Stream stopped")

    # ── Setup ───────────────────────────────────────────────────

    def connect_iphone(self):
        print("[Record3D] Searching for devices...")
        devs = Record3DStream.get_connected_devices()
        if not devs:
            raise RuntimeError(
                "No iPhone detected. Open the Record3D app and start USB streaming."
            )
        print(f"[Record3D] Found {len(devs)} device(s): "
              f"{[str(d.product_id) for d in devs]}")
        self.session = Record3DStream()
        self.session.on_new_frame = self.on_new_frame
        self.session.on_stream_stopped = self.on_stream_stopped
        self.session.connect(devs[0])
        print("[Record3D] Connected")

    def connect_servos(self):
        self.servos = ServoController(port=SERIAL_PORT, baud=SERIAL_BAUD)

    # ── Main loop ───────────────────────────────────────────────

    def _update_fps(self):
        now = time.time()
        dt = now - self.last_frame_time
        if dt > 0:
            inst_fps = 1.0 / dt
            self.fps = 0.9 * self.fps + 0.1 * inst_fps  # exponential smoothing
        self.last_frame_time = now

    def _smooth(self, zones):
        """Apply majority-vote smoothing over recent frames."""
        self.zone_history.append(zones)
        if len(self.zone_history) < SMOOTHING_MAJORITY:
            return zones  # not enough history yet
        smooth_l = sum(s[0] for s in self.zone_history) >= SMOOTHING_MAJORITY
        smooth_c = sum(s[1] for s in self.zone_history) >= SMOOTHING_MAJORITY
        smooth_r = sum(s[2] for s in self.zone_history) >= SMOOTHING_MAJORITY
        return (smooth_l, smooth_c, smooth_r)

    def run(self):
        print("\n[App] Running. Key bindings (focus an OpenCV window first):")
        print("      q - quit")
        print("      s - save snapshot")
        print("      p - pause/resume\n")

        try:
            while True:
                self.event.wait()
                self._update_fps()

                # ── 1. GRAB FRAME ──
                depth = self.session.get_depth_frame()
                rgb   = self.session.get_rgb_frame()
                try:
                    confidence = self.session.get_confidence_frame()
                except Exception:
                    confidence = None

                # ── 2. CLEAN ──
                depth = clean_depth(depth, confidence)

                # Rotate 90° clockwise if phone is held portrait
                depth = np.rot90(depth, k=-1)
                rgb   = np.rot90(rgb,   k=-1)

                # ── 3. PERCEPTION ──
                if self.paused:
                    zones = (False, False, False)
                    smooth_zones = zones
                    closest = None
                    ahead = None
                    dropoff = False
                else:
                    ahead   = distance_ahead(depth)
                    closest = closest_point(depth)
                    zones   = obstacle_directions(depth)
                    dropoff = detect_dropoff(depth)
                    smooth_zones = self._smooth(zones)

                # ── 4. ACTUATE ──

                ### previous v0 of the code:::
                # self.servos.set('L', smooth_zones[0])
                # self.servos.set('C', smooth_zones[1])  # Added
                # self.servos.set('R', smooth_zones[2])
                # (center zone could trigger both, drive LCD, or be ignored — up to you)


                ### v2:
                # ── 4. ACTUATE ──
                # Left servo fires on left OR center obstacle
                # Right servo fires on right OR center obstacle
                # Center obstacle = both buzz simultaneously (full-stop warning)
                left_active  = smooth_zones[0] or smooth_zones[1]
                right_active = smooth_zones[2] or smooth_zones[1]

                self.servos.set('L', left_active)
                self.servos.set('R', right_active)

                # ── 5. PRINT ──
                if PRINT_STATS:
                    ahead_s = f"{ahead:.2f}m" if ahead else "  —  "
                    zones_s = (f"L={'X' if smooth_zones[0] else '.'} "
                               f"C={'X' if smooth_zones[1] else '.'} "
                               f"R={'X' if smooth_zones[2] else '.'}")
                    drop_s = "DROPOFF" if dropoff else "       "
                    print(f"\rAhead: {ahead_s:>7} | {zones_s} | {drop_s} | "
                          f"FPS: {self.fps:4.1f}", end="", flush=True)

                # ── 6. VISUALIZE ──
                rgb_bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
                annotated = draw_overlays(
                    rgb_bgr, zones, smooth_zones, dropoff, closest, ahead,
                    self.fps if SHOW_FPS else None
                )
                cv2.imshow("RGB + Perception", annotated)

                if SHOW_DEPTH_WINDOW:
                    cv2.imshow("Depth", make_depth_vis(depth))

                # ── 7. HANDLE KEYS ──
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    print("\n[App] Quit requested")
                    break
                elif key == ord('s'):
                    fn = f"snapshot_{int(time.time())}.png"
                    cv2.imwrite(fn, annotated)
                    print(f"\n[App] Saved {fn}")
                elif key == ord('p'):
                    self.paused = not self.paused
                    print(f"\n[App] {'Paused' if self.paused else 'Resumed'}")

                self.event.clear()
        finally:
            self.shutdown()

    def shutdown(self):
        print("\n[App] Shutting down...")
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        if self.servos:
            self.servos.close()
        print("[App] Clean exit")


# ══════════════════════════════════════════════════════════════════
# ENTRY POINT
# ══════════════════════════════════════════════════════════════════

def main():
    app = LidarHapticApp()
    app.connect_iphone()
    app.connect_servos()
    app.run()


if __name__ == "__main__":
    main()