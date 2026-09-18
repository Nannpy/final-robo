"""
RoboMaster EP - ตรวจจับเป้าหมายสี + รูปทรงเรขาคณิต แล้วเล็งและยิงอัตโนมัติ

ลำดับการทำงาน
  1. เชื่อมต่อหุ่น -> ดับไฟ Top/Blaster -> Recenter Gimbal -> เปิดกล้อง
  2. Setup Phase : ลากเมาส์วาดกรอบพื้นที่ตาบอดเพิ่มเติม
  3. Tuning Phase: จูน HSV ทีละสี แดง -> เขียว -> น้ำเงิน -> เหลือง แล้วตรวจรวมทุกสีอีกหน้า
  4. Recenter แล้วเข้า Loop หลัก (เริ่มที่โหมด PREVIEW รอคำสั่ง)

โหมดใน Loop หลัก
  PREVIEW : Gimbal นิ่ง แสดงเป้าทุกสีที่มองเห็น
  TRACK   : เล็ง/ยิงเฉพาะสีที่เลือก
  MISSION : กวาดหัวหาเป้ารอบตัว แล้วยิงทุกเป้าตามลำดับสี แดง -> เขียว -> น้ำเงิน -> เหลือง
            (สีเดียวกันยิงจากซ้ายไปขวา) ถ้า Auto-Fire ปิดอยู่ = ซ้อมเล็งอย่างเดียว ไม่ยิง

ปุ่มควบคุม (ต้องคลิกเลือกหน้าต่างภาพก่อน)
  m             : เริ่มภารกิจยิงทุกเป้าทุกสี
  r / g / b / y : โหมด TRACK สีนั้น (ถ้าเปิดจูนอยู่ = สลับสีที่จะจูน)
  x / SPACE     : หยุด กลับโหมด PREVIEW (ยกเลิกภารกิจ)
  s             : เปิด/ปิด Auto-Fire (Safety Toggle)
  t             : เปิด/ปิด Live HSV Tuning (ระหว่างจูน ระบบยิงถูกล็อก)
  c             : Recenter Gimbal
  q / ESC       : ออกจากโปรแกรมอย่างปลอดภัย

ค่า HSV ที่จูนและพื้นที่ตาบอดที่วาด จะถูกบันทึกไว้ใน autoaim_config.json ข้างไฟล์นี้
"""
import json
import math
import queue
import time
from dataclasses import asdict, dataclass, field, fields
from pathlib import Path
from typing import Dict, List, Optional, Set, Tuple

import cv2
import numpy as np
# pyrefly: ignore [missing-import]
from robomaster import robot, blaster, led

# (x1, y1, x2, y2) เป็นสัดส่วน 0.0-1.0 ของความกว้าง/สูงภาพ
Zone = Tuple[float, float, float, float]

# ==========================================
# 1. การตั้งค่า (ปรับค่าได้ที่ส่วนนี้ส่วนเดียว)
# ==========================================
# --- การเชื่อมต่อ / กล้อง ---
CONN_TYPE = "ap"                # "ap" = ต่อ Wi-Fi ตรงกับหุ่น, "sta" = ผ่านเราเตอร์
STREAM_RESOLUTION = "720p"      # "360p" / "540p" / "720p"
FRAME_TIMEOUT_S = 0.5           # รอเฟรมนานสุดกี่วินาที (สั้นไว้ให้คีย์บอร์ดยังตอบสนองตอน Wi-Fi สะดุด)
ROBOT_MODE = robot.FREE         # FREE = หมุนแค่ Gimbal ฐานล้อไม่หมุนตาม
RECENTER_TIMEOUT_S = 3.0

# --- Field of View ของกล้อง RoboMaster (องศา) ---
FOV_X = 96.0
FOV_Y = 54.0

# จุดที่กระสุนตกจริง เทียบกับกึ่งกลางภาพ (องศา) ใช้ชดเชยระยะกล้อง-ลำกล้อง และวิถีกระสุนตก
# ตัวอย่าง: ยิงแล้วกระสุนตกต่ำกว่ากลางจอ 2 องศา -> AIM_OFFSET_Y_DEG = -2.0
AIM_OFFSET_X_DEG = 0.0          # + = ขวา
AIM_OFFSET_Y_DEG = 0.0          # + = บน

# --- Vision ---
COLOR_ORDER = ("red", "green", "blue", "yellow")  # ลำดับการจูนและลำดับการยิงในภารกิจ
MIN_AREA_RATIO = 0.001          # พื้นที่คอนทัวร์ขั้นต่ำเทียบกับทั้งภาพ (0.1% ~ 920 px ที่ 720p)
POLY_EPSILON_RATIO = 0.04       # ค่า epsilon ของ approxPolyDP เป็นสัดส่วนของเส้นรอบรูป
MIN_VERTICES, MAX_VERTICES = 3, 6
MIN_SOLIDITY = 0.80             # พื้นที่คอนทัวร์ / พื้นที่ Convex Hull (ต่ำ = รูปแหว่ง/ก้อนสีมั่ว)

# ช่วง HSV เริ่มต้น: (H_min, H_max, S_min, S_max, V_min, V_max)
# ถ้า H_min > H_max แปลว่าช่วง Hue วนรอบวงล้อสี เช่นสีแดง 170..179 + 0..10
DEFAULT_HSV = {
    "red":    (170, 10, 120, 255, 70, 255),
    "green":  (35, 90, 50, 255, 30, 255),
    "blue":   (95, 130, 80, 255, 50, 255),     # สีน้ำเงินผ่านกล้องมักซีดกว่าสีอื่น จึงใช้ S_min ต่ำ
    "yellow": (20, 35, 100, 255, 100, 255),
}

# บังพลาสติกสีแดง/ส้มของหัว Gimbal (ด้านล่างกลางจอ) = กล่อง ±250 px สูง 180 px ที่ 720p
AUTO_BLIND_ZONE: Zone = (0.30, 0.75, 0.70, 1.00)
MIN_ZONE_PX = 10                # กรอบที่ลากเล็กกว่านี้ถือว่าคลิกพลาด ไม่บันทึก

# --- Tuning Phase ---
NUDGE_DEG = 15                  # ปุ่ม i/j/k/l หมุนหัวครั้งละกี่องศา (ใช้ตอนเป้าสีนั้นอยู่นอกภาพ)

# --- PID (input = องศา, output = deg/s) ---
YAW_PID = {"kp": 2.5, "ki": 0.1, "kd": 0.05}
PITCH_PID = {"kp": 2.5, "ki": 0.1, "kd": 0.05}
MAX_GIMBAL_SPEED = 250.0        # deg/s
PID_INTEGRAL_LIMIT = 50.0       # deg*s
PID_DEADBAND_DEG = 0.3          # error เล็กกว่านี้ถือว่าตรงเป้า กันหัวสั่น
TRACK_WHILE_TUNING = False      # False = Gimbal หยุดนิ่งระหว่างจูนสี ภาพนิ่งจูนง่ายกว่า

# --- Auto-Fire ---
AUTO_FIRE_ON_START = False      # เริ่มโปรแกรมในสถานะ SAFE เสมอ
LOCK_TOLERANCE_DEG = 5.0        # error X และ Y ต้องน้อยกว่านี้
SETTLE_TIME_S = 0.2             # และต้องค้างในระยะนานเกินนี้ถึงจะยิง
FIRE_COOLDOWN_S = 1.0           # เว้นระยะระหว่างนัด (ไม่ใช้ sleep จึงไม่ทำให้ภาพค้าง)

# --- Mission: ยิงทุกเป้าทุกสี ---
# มุม yaw ที่หยุดถ่ายภาพตอนกวาดหาเป้า (องศา, + = ขวา) กล้องกว้าง 96° จึงเห็นได้ถึง ±48° จากแต่ละมุม
# ค่าเริ่มต้นครอบคลุม -138° ถึง +138° | ถ้าต้องการรอบตัว 360° ใช้ (-150, -90, -30, 30, 90, 150)
# (ระวัง: หุ่นจะยิงไปด้านหลังได้ ห้ามมีคนยืนด้านหลัง)
SCAN_YAW_ANGLES = (-90, -45, 0, 45, 90)
# มุม pitch ของแต่ละแถวที่กวาด (0 = มองตรง เห็นสูง/ต่ำได้ ±27°) ถ้ามีเป้าอยู่สูงมากใช้ เช่น (0, 30)
SCAN_PITCH_ANGLES = (0,)
PITCH_LIMITS = (-20.0, 35.0)    # ช่วง pitch ที่ Gimbal หมุนได้จริง
MOVE_SPEED = 150                # deg/s ตอนหมุนไปยังมุมกวาด/เป้าถัดไป
MOVE_TIMEOUT_S = 4.0
MOVE_SETTLE_S = 0.4             # รอหลังหมุนถึง ให้ภาพจากกล้อง (มี delay ผ่าน Wi-Fi) ตามทัน
SCAN_FRAMES = 6                 # จำนวนเฟรมที่เก็บต่อมุมกวาด
SCAN_MIN_HITS = 3               # ต้องเห็นเป้าอย่างน้อยกี่เฟรมถึงนับเป็นเป้าจริง (กัน noise)
POSE_MERGE_DEG = 3.0            # ในมุมกวาดเดียวกัน จุดที่ห่างกันน้อยกว่านี้ = เป้าเดียวกัน
CROSS_POSE_MERGE_DEG = 12.0     # ข้ามมุมกวาด ค่ามุมเพี้ยนได้จากเลนส์มุมกว้าง จึงเผื่อรัศมีมากกว่า
MATCH_RADIUS_DEG = 8.0          # ตอนเล็งละเอียด หาเป้าในรัศมีนี้รอบจุดเล็ง
LOST_REPOINT_S = 1.0            # เป้าหายจากภาพนานเกินนี้ -> หันกลับไปตำแหน่งตามแผนแล้วหาใหม่
ENGAGE_TIMEOUT_S = 6.0          # ใช้เวลากับเป้าเดียวนานเกินนี้ -> ข้ามเป้านั้น
SHOTS_PER_TARGET = 1
DRY_RUN_HOLD_S = 0.5            # Auto-Fire ปิด: ค้างเล็งให้ดูกี่วินาทีก่อนไปเป้าถัดไป

# --- LED ---
TOP_LED_REFRESH_S = 3.0         # สั่งดับไฟ Top ซ้ำทุกกี่วินาที เผื่อ firmware เปิดเอง (เช่นตอนโดนยิง)

# สีขาวใช้ตอน PREVIEW/MISSION: ความอิ่มสีต่ำ แสงสะท้อนจึงไม่ถูกจับเป็นเป้า
LED_COLORS = {"red": (255, 0, 0), "green": (0, 255, 0), "blue": (0, 0, 255), "yellow": (255, 255, 0),
              "white": (255, 255, 255)}  # RGB
DISPLAY_COLORS = {"red": (0, 0, 255), "green": (0, 255, 0), "blue": (255, 120, 0), "yellow": (0, 255, 255)}  # BGR
KEY_TO_COLOR = {"r": "red", "g": "green", "b": "blue", "y": "yellow"}
NUDGE_KEYS = {"i": (NUDGE_DEG, 0), "k": (-NUDGE_DEG, 0), "j": (0, -NUDGE_DEG), "l": (0, NUDGE_DEG)}  # (pitch, yaw)

MODE_PREVIEW, MODE_TRACK, MODE_MISSION = "PREVIEW", "TRACK", "MISSION"

CONFIG_PATH = Path(__file__).resolve().with_name("autoaim_config.json")
MAIN_WINDOW = "RoboMaster Auto-Aim"
SETUP_WINDOW = "SETUP - Blind Zones"
FONT = cv2.FONT_HERSHEY_SIMPLEX


# ==========================================
# 2. ช่วงสี HSV และการบันทึกค่า
# ==========================================
@dataclass
class HSVRange:
    """ ช่วงสี HSV หนึ่งชุด (OpenCV: H 0-179, S/V 0-255)
    รองรับ Hue วนรอบ (h_min > h_max) ทำให้สีแดงใช้ Trackbar ชุดเดียวจูนได้ครบทั้งสองฝั่ง
    """
    h_min: int
    h_max: int
    s_min: int
    s_max: int
    v_min: int
    v_max: int

    def __post_init__(self):
        # บีบค่าให้อยู่ในช่วงที่ OpenCV รับได้ กันค่าเพี้ยนจากไฟล์ config
        for f in fields(self):
            limit = 179 if f.name.startswith("h") else 255
            setattr(self, f.name, int(min(max(int(getattr(self, f.name)), 0), limit)))

    @property
    def wraps(self) -> bool:
        return self.h_min > self.h_max

    def _in_range(self, hsv: np.ndarray, h_lo: int, h_hi: int) -> np.ndarray:
        lower = np.array([h_lo, self.s_min, self.v_min], dtype=np.uint8)
        upper = np.array([h_hi, self.s_max, self.v_max], dtype=np.uint8)
        return cv2.inRange(hsv, lower, upper)

    def mask(self, hsv: np.ndarray) -> np.ndarray:
        if not self.wraps:
            return self._in_range(hsv, self.h_min, self.h_max)
        return cv2.bitwise_or(self._in_range(hsv, self.h_min, 179), self._in_range(hsv, 0, self.h_max))

    def __str__(self) -> str:
        wrap = " (wrap)" if self.wraps else ""
        return f"H {self.h_min}-{self.h_max}{wrap}  S {self.s_min}-{self.s_max}  V {self.v_min}-{self.v_max}"


def default_color_ranges() -> Dict[str, HSVRange]:
    return {name: HSVRange(*values) for name, values in DEFAULT_HSV.items()}


def load_config(path: Path) -> Tuple[Dict[str, HSVRange], List[Zone]]:
    """ โหลดค่า HSV + พื้นที่ตาบอดที่ผู้ใช้วาดไว้ ถ้าไม่มีไฟล์หรือไฟล์เสียจะใช้ค่าเริ่มต้น """
    ranges = default_color_ranges()
    if not path.exists():
        return ranges, []
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
        for name, values in data.get("hsv", {}).items():
            if name in ranges:
                ranges[name] = HSVRange(**{f.name: values[f.name] for f in fields(HSVRange)})
        zones = []
        for zone in data.get("user_blind_zones", []):
            x1, y1, x2, y2 = (float(v) for v in zone)
            if 0.0 <= x1 < x2 <= 1.0 and 0.0 <= y1 < y2 <= 1.0:
                zones.append((x1, y1, x2, y2))
    except (OSError, ValueError, KeyError, TypeError) as e:
        print(f"[CONFIG] ⚠️ อ่าน {path.name} ไม่ได้ ({e}) ใช้ค่าเริ่มต้นแทน")
        return default_color_ranges(), []
    print(f"[CONFIG] โหลดค่า HSV และพื้นที่ตาบอด {len(zones)} กรอบ จาก {path.name}")
    return ranges, zones


def save_config(path: Path, ranges: Dict[str, HSVRange], zones: List[Zone]):
    data = {
        "hsv": {name: asdict(rng) for name, rng in ranges.items()},
        "user_blind_zones": [[round(v, 4) for v in zone] for zone in zones],
    }
    try:
        path.write_text(json.dumps(data, indent=2), encoding="utf-8")
        print(f"[CONFIG] 💾 บันทึกค่าลง {path.name}")
    except OSError as e:
        print(f"[CONFIG] ⚠️ บันทึก {path.name} ไม่ได้: {e}")


# ==========================================
# 3. Helper ทั่วไป
# ==========================================
def safe_call(label: str, fn, *args, **kwargs):
    """ เรียกคำสั่ง SDK โดยไม่ให้โปรแกรมล่ม ถ้าการสื่อสารกับหุ่นสะดุดชั่วคราว """
    try:
        return fn(*args, **kwargs)
    except Exception as e:  # SDK โยน exception ได้หลายแบบเมื่อ Wi-Fi มีปัญหา
        print(f"[WARN] {label}: {e}")
        return None


def read_frame(ep_camera) -> Optional[np.ndarray]:
    """ อ่านเฟรมล่าสุด; SDK โยน queue.Empty เมื่อหมดเวลา (ไม่ได้คืนค่า None) """
    try:
        return ep_camera.read_cv2_image(timeout=FRAME_TIMEOUT_S, strategy="newest")
    except queue.Empty:
        return None


def key_char(key: int) -> str:
    """ แปลงค่าจาก cv2.waitKey เป็นตัวอักษรพิมพ์เล็ก (กด Caps Lock ก็ยังใช้ได้) """
    return chr(key).lower() if key < 128 else ""


def put_text(img, text, org, color=(255, 255, 255), scale=0.6, thickness=2):
    """ เขียนตัวหนังสือมีขอบดำ อ่านง่ายทุกพื้นหลัง (putText รองรับเฉพาะ ASCII) """
    cv2.putText(img, text, org, FONT, scale, (0, 0, 0), thickness + 2, cv2.LINE_AA)
    cv2.putText(img, text, org, FONT, scale, color, thickness, cv2.LINE_AA)


def pixel_to_degrees(px: float, py: float, w: int, h: int) -> Tuple[float, float]:
    """ แปลงพิกัดพิกเซล -> มุม (องศา) เทียบกับกึ่งกลางภาพ

    ใช้องศาต่อพิกเซลคงที่ (FOV / ความกว้างภาพ) ซึ่งสอดคล้องกับค่า FOV 96x54
    ที่เป็นสัดส่วน 16:9 พอดี (เลนส์มุมกว้างที่มี distortion ให้มุมเกือบแปรผันตรงกับระยะพิกเซล)
    X: + = เป้าอยู่ทางขวา   Y: + = เป้าอยู่ด้านบน (แกน y ของภาพชี้ลง จึงกลับเครื่องหมาย)
    """
    deg_x = (px - w / 2.0) * (FOV_X / w)
    deg_y = (h / 2.0 - py) * (FOV_Y / h)
    return deg_x, deg_y


def degrees_to_pixel(deg_x: float, deg_y: float, w: int, h: int) -> Tuple[int, int]:
    """ แปลงกลับจากองศา -> พิกเซล (ใช้วาดจุดเล็งบนจอ) """
    return int(round(w / 2.0 + deg_x * w / FOV_X)), int(round(h / 2.0 - deg_y * h / FOV_Y))


# ==========================================
# 4. พื้นที่ตาบอด (Blind Zones)
# ==========================================
class BlindZones:
    """ ถมสีดำลงบน Mask เพื่อให้ Vision มองไม่เห็นบริเวณนั้น
    - auto_zone : บังหัว Gimbal ของหุ่นเอง ใช้งานเสมอ
    - user_zones: กรอบที่ผู้ใช้ลากเมาส์วาดเพิ่มตอน Setup
    เก็บพิกัดเป็นสัดส่วน 0-1 จึงใช้ได้ทุกความละเอียดกล้อง
    """

    def __init__(self, auto_zone: Zone, user_zones: Optional[List[Zone]] = None):
        self.auto_zone = auto_zone
        self.user_zones: List[Zone] = list(user_zones or [])

    @staticmethod
    def to_pixels(zone: Zone, w: int, h: int) -> Tuple[Tuple[int, int], Tuple[int, int]]:
        x1, y1, x2, y2 = zone
        return (int(x1 * w), int(y1 * h)), (int(x2 * w), int(y2 * h))

    def all_zones(self) -> List[Zone]:
        return [self.auto_zone] + self.user_zones

    def apply(self, mask: np.ndarray):
        h, w = mask.shape[:2]
        for zone in self.all_zones():
            p1, p2 = self.to_pixels(zone, w, h)
            cv2.rectangle(mask, p1, p2, 0, thickness=-1)

    def draw(self, frame: np.ndarray, fill_alpha: float = 0.0):
        h, w = frame.shape[:2]
        if fill_alpha > 0:
            overlay = frame.copy()
            for zone in self.all_zones():
                p1, p2 = self.to_pixels(zone, w, h)
                cv2.rectangle(overlay, p1, p2, (0, 0, 0), thickness=-1)
            cv2.addWeighted(overlay, fill_alpha, frame, 1.0 - fill_alpha, 0, frame)
        for i, zone in enumerate(self.all_zones()):
            p1, p2 = self.to_pixels(zone, w, h)
            label, color = ("AUTO BLIND ZONE", (0, 165, 255)) if i == 0 else (f"BLIND ZONE {i}", (180, 180, 180))
            cv2.rectangle(frame, p1, p2, color, 2)
            put_text(frame, label, (p1[0] + 8, p1[1] + 22), color, 0.5, 1)


class ZoneDrawer:
    """ Mouse callback ตอน Setup: กดค้าง-ลาก-ปล่อย = เพิ่มกรอบพื้นที่ตาบอด 1 กรอบ """

    def __init__(self, blind_zones: BlindZones):
        self.blind_zones = blind_zones
        self.frame_size = (1, 1)  # (w, h) อัปเดตทุกเฟรม
        self.drag_start: Optional[Tuple[int, int]] = None
        self.drag_end: Optional[Tuple[int, int]] = None

    def on_mouse(self, event, x, y, flags, param):
        w, h = self.frame_size
        x, y = min(max(x, 0), w - 1), min(max(y, 0), h - 1)
        if event == cv2.EVENT_LBUTTONDOWN:
            self.drag_start = self.drag_end = (x, y)
        elif event == cv2.EVENT_MOUSEMOVE and self.drag_start is not None:
            self.drag_end = (x, y)
        elif event == cv2.EVENT_LBUTTONUP and self.drag_start is not None:
            (x1, y1), (x2, y2) = self.drag_start, (x, y)
            self.drag_start = self.drag_end = None
            if abs(x2 - x1) >= MIN_ZONE_PX and abs(y2 - y1) >= MIN_ZONE_PX:
                self.blind_zones.user_zones.append(
                    (min(x1, x2) / w, min(y1, y2) / h, max(x1, x2) / w, max(y1, y2) / h))
                print(f">> เพิ่มพื้นที่ตาบอด #{len(self.blind_zones.user_zones)}")


def run_setup_phase(ep_camera, blind_zones: BlindZones) -> bool:
    """ ให้ผู้ใช้วาดพื้นที่ตาบอดเพิ่ม คืนค่า False ถ้าผู้ใช้กดออกจากโปรแกรม """
    drawer = ZoneDrawer(blind_zones)
    cv2.namedWindow(SETUP_WINDOW)
    cv2.setMouseCallback(SETUP_WINDOW, drawer.on_mouse)

    print("\n" + "=" * 60)
    print(" 🛠️ SETUP: พื้นที่ตาบอด (Blind Zone)")
    print(" - กล่องสีส้มคือ AUTO BLIND ZONE บังหัว Gimbal ให้อัตโนมัติ")
    print(" - กดเมาส์ค้างแล้วลาก เพื่อวาดกรอบบังเพิ่ม (วาดได้หลายกรอบ)")
    print(" - [u] ลบกรอบล่าสุด  [x] ลบกรอบที่วาดทั้งหมด")
    print(" - [ENTER/SPACE] ไปขั้นจูนสี  [q/ESC] ออก")
    print("=" * 60 + "\n")

    while True:
        frame = read_frame(ep_camera)
        if frame is None:
            continue
        h, w = frame.shape[:2]
        drawer.frame_size = (w, h)

        view = frame.copy()
        blind_zones.draw(view, fill_alpha=0.6)
        if drawer.drag_start is not None and drawer.drag_end is not None:
            cv2.rectangle(view, drawer.drag_start, drawer.drag_end, (0, 255, 255), 2)
        put_text(view, "SETUP: drag mouse to add blind zones", (15, 30), (0, 255, 0), 0.7)
        put_text(view, "[u] undo  [x] clear  [ENTER/SPACE] next  [q] quit", (15, 60), (255, 255, 255), 0.6)
        put_text(view, f"User zones: {len(blind_zones.user_zones)}", (15, 90), (255, 255, 255), 0.6)
        cv2.imshow(SETUP_WINDOW, view)

        key = cv2.waitKey(1) & 0xFF
        if key in (13, 10, 32):
            break
        if key in (ord("q"), 27):
            cv2.destroyWindow(SETUP_WINDOW)
            return False
        if key == ord("u") and blind_zones.user_zones:
            blind_zones.user_zones.pop()
        elif key == ord("x"):
            blind_zones.user_zones.clear()

    cv2.destroyWindow(SETUP_WINDOW)
    print(f">> ✅ ใช้พื้นที่ตาบอด 1 (อัตโนมัติ) + {len(blind_zones.user_zones)} (ผู้ใช้วาด)")
    return True


# ==========================================
# 5. Vision: Mask สี + ตรวจรูปทรงเรขาคณิต
# ==========================================
@dataclass
class Target:
    color: str
    cx: int
    cy: int
    area: float
    vertices: int
    approx: np.ndarray


_MORPH_KERNEL = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))


def prepare_hsv(frame_bgr: np.ndarray) -> np.ndarray:
    """ ทำครั้งเดียวต่อเฟรม แล้วใช้ร่วมกันทุกสี """
    blurred = cv2.GaussianBlur(frame_bgr, (5, 5), 0)  # ลด noise ของ sensor ก่อนกรองสี
    return cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)


def build_mask(hsv: np.ndarray, hsv_range: HSVRange, blind_zones: BlindZones) -> np.ndarray:
    mask = hsv_range.mask(hsv)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, _MORPH_KERNEL)   # ลบจุดสีเล็กๆ
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, _MORPH_KERNEL)  # อุดรูในเป้า ขอบเรียบขึ้น
    blind_zones.apply(mask)  # ทำเป็นขั้นสุดท้าย พื้นที่ตาบอดจึงดำสนิทแน่นอน
    return mask


def find_geometric_targets(mask: np.ndarray, color: str) -> List[Target]:
    """ หาทุกเป้าที่เป็นรูปทรง 3-6 มุมและทึบพอ เรียงจากใหญ่ไปเล็ก
    ไม่มี fallback ไปเลือกก้อนสีใหญ่สุด เพราะจะทำให้ตัวกรองรูปทรงไม่มีความหมาย
    """
    h, w = mask.shape[:2]
    min_area = MIN_AREA_RATIO * w * h
    contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]  # รองรับ OpenCV 3 และ 4

    found: List[Target] = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < min_area:
            continue

        approx = cv2.approxPolyDP(cnt, POLY_EPSILON_RATIO * cv2.arcLength(cnt, True), True)
        if not MIN_VERTICES <= len(approx) <= MAX_VERTICES:
            continue

        hull_area = cv2.contourArea(cv2.convexHull(cnt))
        if hull_area <= 0 or area / hull_area < MIN_SOLIDITY:
            continue

        m = cv2.moments(cnt)
        if m["m00"] == 0:
            continue
        # ใช้จุดศูนย์กลางมวล (centroid) นิ่งกว่ากึ่งกลาง bounding box
        found.append(Target(color, int(m["m10"] / m["m00"]), int(m["m01"] / m["m00"]), area, len(approx), approx))
    return sorted(found, key=lambda t: t.area, reverse=True)


def detect_all_colors(hsv: np.ndarray, ranges: Dict[str, HSVRange], blind_zones: BlindZones) -> List[Target]:
    found: List[Target] = []
    for color in COLOR_ORDER:
        found.extend(find_geometric_targets(build_mask(hsv, ranges[color], blind_zones), color))
    return found


def draw_targets(frame: np.ndarray, targets: List[Target]):
    for t in targets:
        bgr = DISPLAY_COLORS[t.color]
        cv2.drawContours(frame, [t.approx], -1, bgr, 3)
        cv2.circle(frame, (t.cx, t.cy), 5, (0, 0, 255), -1)
        put_text(frame, f"{t.color.upper()} {t.vertices}c", (t.cx + 10, t.cy - 10), bgr, 0.5, 1)


# ==========================================
# 6. PID Controller
# ==========================================
class PIDController:
    """ PID ของ Gimbal หนึ่งแกน
    Input : error เป็นองศา (มุมระหว่างเป้ากับจุดเล็ง)
    Output: ความเร็วเชิงมุม deg/s สำหรับ gimbal.drive_speed()
    """
    STALE_DT_S = 0.5  # ห่างจากรอบก่อนนานเกินนี้ (เช่นหลัง recenter) ถือว่าเริ่มใหม่

    def __init__(self, kp, ki, kd, output_limit=MAX_GIMBAL_SPEED, integral_limit=PID_INTEGRAL_LIMIT,
                 deadband=PID_DEADBAND_DEG, d_smoothing=0.5):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.output_limit = output_limit
        self.integral_limit = integral_limit
        self.deadband = deadband
        self.d_smoothing = d_smoothing  # 0 = ไม่กรอง, ใกล้ 1 = กรองแรง (ภาพจากกล้องมี noise)
        self.reset()

    def reset(self):
        self._integral = 0.0
        self._derivative = 0.0
        self._last_error = 0.0
        self._last_time: Optional[float] = None

    @staticmethod
    def _clamp(value, limit):
        return max(-limit, min(limit, value))

    def compute(self, error: float) -> float:
        now = time.monotonic()
        if abs(error) < self.deadband:
            error = 0.0

        if self._last_time is None or now - self._last_time > self.STALE_DT_S:
            # รอบแรกยังไม่มี dt ที่เชื่อถือได้: ใช้แค่ P กัน derivative kick ตอนเจอเป้าใหม่
            dt = 0.0
            self._derivative = 0.0
        else:
            dt = max(now - self._last_time, 1e-3)
            raw_d = (error - self._last_error) / dt
            self._derivative = self.d_smoothing * self._derivative + (1.0 - self.d_smoothing) * raw_d

        p = self.kp * error
        d = self.kd * self._derivative

        # Anti-windup: สะสม I เฉพาะตอนที่ output ยังไม่อิ่มตัว หรือการสะสมช่วยดึงออกจากจุดอิ่มตัว
        candidate = self._clamp(self._integral + error * dt, self.integral_limit)
        unsaturated = p + self.ki * candidate + d
        if abs(unsaturated) <= self.output_limit or unsaturated * error < 0:
            self._integral = candidate

        self._last_error = error
        self._last_time = now
        return self._clamp(p + self.ki * self._integral + d, self.output_limit)


# ==========================================
# 7. ลอจิกการยิง (Settling Time + Safety)
# ==========================================
class FireController:
    """ ตัดสินว่ายิงได้หรือยัง
    ยิงเมื่อ |err_x| และ |err_y| < LOCK_TOLERANCE_DEG ต่อเนื่องนานเกิน SETTLE_TIME_S
    และ Auto-Fire เปิด และไม่อยู่ในโหมดจูน และพ้น cooldown
    """

    def __init__(self):
        self.armed = AUTO_FIRE_ON_START
        self.settled = False  # เล็งนิ่งครบเวลาแล้วหรือยัง (ใช้ตอนซ้อมเล็งในภารกิจ)
        self._lock_start: Optional[float] = None
        self._last_fire = float("-inf")

    def reset_lock(self):
        self._lock_start = None
        self.settled = False

    def evaluate(self, err_x: float, err_y: float, tuning: bool) -> Tuple[str, bool]:
        """ คืนค่า (ข้อความสถานะสำหรับ HUD, ควรยิงหรือไม่) """
        now = time.monotonic()
        self.settled = False
        if abs(err_x) >= LOCK_TOLERANCE_DEG or abs(err_y) >= LOCK_TOLERANCE_DEG:
            self._lock_start = None
            return "TRACKING", False

        if self._lock_start is None:
            self._lock_start = now
        held = now - self._lock_start
        if held <= SETTLE_TIME_S:
            return f"SETTLING {held:.2f}s", False
        self.settled = True
        if tuning:
            return "LOCKED - TUNING (FIRE BLOCKED)", False
        if not self.armed:
            return "LOCKED - SAFE", False
        if now - self._last_fire < FIRE_COOLDOWN_S:
            return "LOCKED - RELOADING", False
        return "FIRE", True

    def mark_fired(self):
        self._last_fire = time.monotonic()
        self.reset_lock()


# ==========================================
# 8. ไฟ LED
# ==========================================
class LedManager:
    """ คุมไฟทั้งหมดของหุ่น
    - ไฟ Top (บน Gimbal) และไฟ Blaster: ดับตลอด กันแสงสะท้อนเข้ากล้อง
    - ไฟ Bottom (ฐานล้อ): แสดงสถานะ
        สี  : TRACK = สีเป้าหมาย, PREVIEW/MISSION = ขาว
        แบบ : ติดค้าง = SAFE, กระพริบ = Auto-Fire พร้อมยิง, หายใจ = โหมดจูน
    """

    def __init__(self, ep_led, ep_blaster):
        self._led = ep_led
        self._blaster = ep_blaster
        self._last_top_off = float("-inf")
        self._bottom_state = None

    def _top_off(self):
        safe_call("top LED off", self._led.set_led, comp=led.COMP_TOP_ALL, r=0, g=0, b=0, effect=led.EFFECT_OFF)
        self._last_top_off = time.monotonic()

    def silence_gimbal_lights(self):
        self._top_off()
        safe_call("blaster LED off", self._blaster.set_led, brightness=0, effect=blaster.LED_OFF)

    def keep_top_off(self):
        if time.monotonic() - self._last_top_off >= TOP_LED_REFRESH_S:
            self._top_off()

    def show_status(self, color: str, armed: bool, tuning: bool):
        if tuning:
            effect = led.EFFECT_BREATH
        elif armed:
            effect = led.EFFECT_FLASH
        else:
            effect = led.EFFECT_ON
        state = (color, effect)
        if state == self._bottom_state:
            return  # ส่งคำสั่งเฉพาะตอนสถานะเปลี่ยน ไม่ถ่วง loop
        r, g, b = LED_COLORS[color]
        safe_call("bottom LED", self._led.set_led, comp=led.COMP_BOTTOM_ALL, r=r, g=g, b=b, effect=effect, freq=2)
        self._bottom_state = state

    def all_off(self):
        safe_call("all LED off", self._led.set_led, comp=led.COMP_ALL, r=0, g=0, b=0, effect=led.EFFECT_OFF)
        self._bottom_state = None


# ==========================================
# 9. Live HSV Tuning
# ==========================================
class TuningPanel:
    """ หน้าต่าง Trackbar ปรับค่า HSV แบบ Real-time โดย Loop หลักยังทำงานต่อ """
    WINDOW = "Live HSV Tuning"
    PREVIEW_WINDOW = "Tuning Preview (masked)"
    TRACKBARS = (("H Min", "h_min", 179), ("H Max", "h_max", 179),
                 ("S Min", "s_min", 255), ("S Max", "s_max", 255),
                 ("V Min", "v_min", 255), ("V Max", "v_max", 255))

    def __init__(self):
        self.is_open = False

    def open(self, rng: HSVRange):
        cv2.namedWindow(self.WINDOW)
        cv2.resizeWindow(self.WINDOW, 400, 300)
        for label, field_name, max_val in self.TRACKBARS:
            cv2.createTrackbar(label, self.WINDOW, getattr(rng, field_name), max_val, lambda _: None)
        self.is_open = True

    def load(self, rng: HSVRange):
        """ เปลี่ยนสีที่จูน: ย้ายค่าของสีใหม่ขึ้น Trackbar """
        for label, field_name, _ in self.TRACKBARS:
            cv2.setTrackbarPos(label, self.WINDOW, getattr(rng, field_name))

    def read(self) -> Optional[HSVRange]:
        """ คืน None ถ้าหน้าต่างถูกปิด (เช่นกดปุ่ม X) """
        try:
            values = {name: cv2.getTrackbarPos(label, self.WINDOW) for label, name, _ in self.TRACKBARS}
        except cv2.error:
            return None
        if any(v < 0 for v in values.values()):
            return None
        return HSVRange(**values)

    def show_preview(self, frame: np.ndarray, mask: np.ndarray):
        # แสดงเฉพาะพิกเซลที่ผ่าน Mask ในสีจริง ย่อครึ่งหนึ่งไม่ให้เต็มจอ
        preview = cv2.bitwise_and(frame, frame, mask=mask)
        cv2.imshow(self.PREVIEW_WINDOW, cv2.resize(preview, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA))

    def close(self):
        for name in (self.WINDOW, self.PREVIEW_WINDOW):
            try:
                cv2.destroyWindow(name)
            except cv2.error:
                pass
        self.is_open = False


# ==========================================
# 10. ภารกิจยิงทุกเป้าทุกสี
# ==========================================
@dataclass
class PlannedTarget:
    """ เป้าหนึ่งเป้า ตำแหน่งเป็นมุมเทียบกับหน้าหุ่น (yaw + = ขวา, pitch + = บน) """
    color: str
    yaw: float
    pitch: float
    area: float
    offset: float                                  # ระยะจากกึ่งกลางภาพตอนวัด (น้อย = แม่น ขอบเลนส์บิดเบี้ยว)
    hits: int = 1
    poses: Set[int] = field(default_factory=set)   # มุมกวาดที่เห็นเป้านี้
    status: str = "PENDING"
    shots: int = 0


def angular_distance(a: PlannedTarget, b: PlannedTarget) -> float:
    return math.hypot(a.yaw - b.yaw, a.pitch - b.pitch)


class TargetMap:
    """ รวมผลการกวาดหาเป้าจากหลายมุมกล้อง ให้เป็นรายการเป้าที่ไม่ซ้ำกัน

    - ในมุมกวาดเดียวกัน: หลายเฟรมที่เห็นจุดเดียวกัน (< POSE_MERGE_DEG) = เป้าเดียว
    - ข้ามมุมกวาด: ค่ามุมเพี้ยนได้หลายองศาเพราะเลนส์มุมกว้าง จึงรวมในรัศมี CROSS_POSE_MERGE_DEG
      แต่ถ้ามุมกวาดเดียวกันเห็นเป็น 2 ชิ้นแยกกัน ถือว่าเป็นคนละเป้าเสมอ
    """

    def __init__(self):
        self._current: List[PlannedTarget] = []
        self._by_pose: List[List[PlannedTarget]] = []

    def add_sighting(self, sighting: PlannedTarget):
        for obs in self._current:
            if obs.color == sighting.color and angular_distance(obs, sighting) < POSE_MERGE_DEG:
                obs.hits += 1
                return
        self._current.append(sighting)

    def end_pose(self) -> int:
        """ ปิดมุมกวาดปัจจุบัน คืนจำนวนเป้าที่เจอในมุมนี้ """
        seen = [o for o in self._current if o.hits >= SCAN_MIN_HITS]
        kept: List[PlannedTarget] = []
        # วัตถุชิ้นเดียวอาจผ่านช่วงสีได้ 2 สี (เช่นส้มผ่านทั้งแดงและเหลือง) เก็บสีที่เห็นบ่อยกว่า
        for obs in sorted(seen, key=lambda o: (o.hits, o.area), reverse=True):
            if all(angular_distance(obs, k) >= POSE_MERGE_DEG for k in kept):
                kept.append(obs)
        self._by_pose.append(kept)
        self._current = []
        return len(kept)

    def build_plan(self) -> List[PlannedTarget]:
        observations = sorted(((i, obs) for i, objs in enumerate(self._by_pose) for obs in objs),
                              key=lambda item: item[1].offset)  # ค่าที่วัดใกล้กลางภาพที่สุดเป็นตัวแทน
        clusters: List[PlannedTarget] = []
        for pose_idx, obs in observations:
            candidates = [c for c in clusters if c.color == obs.color and pose_idx not in c.poses
                          and angular_distance(c, obs) < CROSS_POSE_MERGE_DEG]
            if candidates:
                nearest = min(candidates, key=lambda c: angular_distance(c, obs))
                nearest.poses.add(pose_idx)
                nearest.hits += obs.hits
            else:
                obs.poses = {pose_idx}
                clusters.append(obs)
        # ลำดับยิง: ตามลำดับสี แล้วซ้ายไปขวา (บนลงล่างถ้า yaw เท่ากัน)
        return sorted(clusters, key=lambda c: (COLOR_ORDER.index(c.color), c.yaw, -c.pitch))


class Mission:
    """ ภารกิจยิงทุกเป้าทุกสี แบบ non-blocking (เรียก step() ทุกเฟรม ภาพและปุ่มจึงไม่ค้าง)

    SCAN   : หมุนไปทีละมุมใน SCAN_PITCH_ANGLES x SCAN_YAW_ANGLES เก็บตำแหน่งเป้าทุกสี
    ENGAGE : หมุนไปยังเป้าตามแผน (moveto) -> เล็งละเอียดด้วยภาพ + PID -> ยิง -> เป้าถัดไป
    DONE   : จบภารกิจ

    ใช้ gimbal.moveto ซึ่งอ้างอิง yaw กับฐานล้อ และ pitch กับแนวระนาบ จึงเป็นพิกัดเดียวกับ recenter
    """
    STATUS_SHORT = {"PENDING": "..", "ENGAGING": ">>", "FIRED": "SHOT", "AIMED-SAFE": "AIM", "SKIPPED": "SKIP"}
    MOVE_RETRY_S = 0.5

    def __init__(self, app: "AutoAimApp"):
        self.app = app
        self.poses = [(float(p), float(y)) for p in SCAN_PITCH_ANGLES for y in SCAN_YAW_ANGLES]
        self.target_map = TargetMap()
        self.plan: List[PlannedTarget] = []
        self.phase = "SCAN"
        self.sub = "MOVE"  # MOVE -> SETTLE -> COLLECT (SCAN) หรือ AIM (ENGAGE)
        self.index = 0     # มุมกวาดปัจจุบัน (SCAN) / เป้าปัจจุบัน (ENGAGE)
        self.pose = (0.0, 0.0)

        # ผลของเฟรมล่าสุด ให้ App วาด HUD
        self.state = ""
        self.visible: List[Target] = []
        self.match: Optional[Target] = None
        self.error: Optional[Tuple[float, float]] = None

        self._action = None
        self._sub_t0 = 0.0
        self._last_move_try = float("-inf")
        self._collected = 0
        self._target_t0 = 0.0
        self._last_match_deg: Optional[Tuple[float, float]] = None
        self._lost_since: Optional[float] = None
        self._hold_since: Optional[float] = None
        self._move_to(*self.poses[0])

    @property
    def done(self) -> bool:
        return self.phase == "DONE"

    # ---------- การหมุนไปยังตำแหน่ง ----------
    def _move_to(self, pitch: float, yaw: float):
        self.pose = (min(max(pitch, PITCH_LIMITS[0]), PITCH_LIMITS[1]), yaw)
        self.sub = "MOVE"
        self._sub_t0 = time.monotonic()
        self._last_match_deg = None
        safe_call("gimbal stop", self.app.ep_gimbal.drive_speed, pitch_speed=0, yaw_speed=0)
        self._send_move()

    def _send_move(self):
        self._last_move_try = time.monotonic()
        self._action = safe_call("gimbal moveto", self.app.ep_gimbal.moveto, pitch=self.pose[0], yaw=self.pose[1],
                                 pitch_speed=MOVE_SPEED, yaw_speed=MOVE_SPEED)

    def _ready(self) -> bool:
        """ อัปเดตขั้น MOVE -> SETTLE คืน True เมื่อหมุนถึงและรอภาพตามทันครบแล้ว """
        now = time.monotonic()
        if self.sub == "MOVE":
            if self._action is None and now - self._last_move_try >= self.MOVE_RETRY_S:
                self._send_move()  # คำสั่งก่อนหน้าโดนปฏิเสธ (เช่น action เดิมยังไม่จบ) ลองใหม่
            arrived = self._action is not None and self._action.is_completed
            if arrived or now - self._sub_t0 > MOVE_TIMEOUT_S:
                if not arrived:
                    print("[MISSION] ⚠️ Gimbal หมุนไม่เสร็จตามเวลา ใช้ตำแหน่งปัจจุบันต่อ")
                self.sub, self._sub_t0 = "SETTLE", now
            return False
        if self.sub == "SETTLE":
            if now - self._sub_t0 < MOVE_SETTLE_S:
                return False
            self.sub = "COLLECT" if self.phase == "SCAN" else "AIM"
        return True

    # ---------- วนทุกเฟรม ----------
    def step(self, frame: np.ndarray, hsv: np.ndarray):
        self.visible, self.match, self.error = [], None, None
        if self.phase == "SCAN":
            self._step_scan(frame, hsv)
        elif self.phase == "ENGAGE":
            self._step_engage(frame, hsv)

    def _step_scan(self, frame: np.ndarray, hsv: np.ndarray):
        pitch, yaw = self.pose
        ready = self._ready()
        self.state = f"SCAN {self.index + 1}/{len(self.poses)} (pitch {pitch:+.0f}, yaw {yaw:+.0f}) {self.sub}"
        if not ready:
            return

        h, w = frame.shape[:2]
        self.visible = detect_all_colors(hsv, self.app.color_ranges, self.app.blind_zones)
        for t in self.visible:
            dx, dy = pixel_to_degrees(t.cx, t.cy, w, h)
            self.target_map.add_sighting(
                PlannedTarget(t.color, yaw=yaw + dx, pitch=pitch + dy, area=t.area, offset=math.hypot(dx, dy)))
        self._collected += 1
        if self._collected < SCAN_FRAMES:
            return

        found = self.target_map.end_pose()
        print(f"[MISSION] 🔍 มุม {self.index + 1}/{len(self.poses)} (pitch {pitch:+.0f}°, yaw {yaw:+.0f}°): "
              f"เห็น {found} เป้า")
        self._collected = 0
        self.index += 1
        if self.index < len(self.poses):
            self._move_to(*self.poses[self.index])
            return

        self.plan = self.target_map.build_plan()
        if not self.plan:
            print("[MISSION] ไม่พบเป้าหมายเลย")
            self.phase = "DONE"
            return
        print(f"[MISSION] 📋 แผนยิง {len(self.plan)} เป้า:")
        for i, t in enumerate(self.plan, 1):
            print(f"   {i}. {t.color.upper():<6} yaw {t.yaw:+6.1f}°  pitch {t.pitch:+5.1f}°")
        self.phase = "ENGAGE"
        self.index = 0
        self._begin_target()

    def _begin_target(self):
        t = self.plan[self.index]
        t.status = "ENGAGING"
        self._target_t0 = time.monotonic()
        self._lost_since = None
        self._hold_since = None
        self.app._reset_aim()
        print(f"[MISSION] 🎯 เป้า {self.index + 1}/{len(self.plan)}: {t.color.upper()}")
        # หันให้เป้าไปตกที่จุดเล็ง (ไม่ใช่กลางภาพ) จึงลบ AIM_OFFSET
        self._move_to(t.pitch - AIM_OFFSET_Y_DEG, t.yaw - AIM_OFFSET_X_DEG)

    def _finish_target(self, status: str):
        self.plan[self.index].status = status
        print(f"[MISSION]    -> {status}")
        self.app._stop_gimbal()
        self.index += 1
        if self.index < len(self.plan):
            self._begin_target()
        else:
            self.phase = "DONE"

    def _step_engage(self, frame: np.ndarray, hsv: np.ndarray):
        app = self.app
        t = self.plan[self.index]
        now = time.monotonic()
        label = f"TARGET {self.index + 1}/{len(self.plan)} {t.color.upper()}"
        if now - self._target_t0 > ENGAGE_TIMEOUT_S:
            self._finish_target("SKIPPED")
            return
        if not self._ready():
            self.state = f"{label} {self.sub}"
            return

        # หาเป้าสีนี้ที่ใกล้จุดอ้างอิงที่สุด: เฟรมแรกใช้จุดเล็ง เฟรมต่อไปใช้ตำแหน่งที่เจอล่าสุด (ไม่กระโดดไปเป้าข้างๆ)
        h, w = frame.shape[:2]
        self.visible = find_geometric_targets(build_mask(hsv, app.color_ranges[t.color], app.blind_zones), t.color)
        ref = self._last_match_deg or (AIM_OFFSET_X_DEG, AIM_OFFSET_Y_DEG)
        best, best_deg, best_dist = None, None, MATCH_RADIUS_DEG
        for cand in self.visible:
            deg = pixel_to_degrees(cand.cx, cand.cy, w, h)
            dist = math.hypot(deg[0] - ref[0], deg[1] - ref[1])
            if dist < best_dist:
                best, best_deg, best_dist = cand, deg, dist

        if best is None:
            app._stop_gimbal()
            self._last_match_deg = None
            self._hold_since = None
            if self._lost_since is None:
                self._lost_since = now
            elif now - self._lost_since > LOST_REPOINT_S:
                self._lost_since = None
                self._move_to(t.pitch - AIM_OFFSET_Y_DEG, t.yaw - AIM_OFFSET_X_DEG)
            self.state = f"{label} LOST"
            return

        self._lost_since = None
        self.match, self._last_match_deg = best, best_deg
        err_x, err_y = best_deg[0] - AIM_OFFSET_X_DEG, best_deg[1] - AIM_OFFSET_Y_DEG
        self.error = (err_x, err_y)
        safe_call("gimbal drive", app.ep_gimbal.drive_speed,
                  pitch_speed=app.pid_pitch.compute(err_y), yaw_speed=app.pid_yaw.compute(err_x))

        fire_state, should_fire = app.fire_ctrl.evaluate(err_x, err_y, app.tuning.is_open)
        self.state = f"{label} {fire_state}"
        if should_fire:
            if app._fire():
                t.shots += 1
            if t.shots >= SHOTS_PER_TARGET:
                self._finish_target("FIRED")
            return
        if app.fire_ctrl.settled and not app.fire_ctrl.armed:
            # Auto-Fire ปิด = ซ้อมเล็ง: ค้างให้ดูสักครู่แล้วไปเป้าถัดไป
            if self._hold_since is None:
                self._hold_since = now
            elif now - self._hold_since >= DRY_RUN_HOLD_S:
                self._finish_target("AIMED-SAFE")
        else:
            self._hold_since = None

    # ---------- สรุปผล ----------
    def progress_lines(self, per_line: int = 6) -> List[str]:
        tokens = [f"{i}.{t.color[0].upper()}:{self.STATUS_SHORT[t.status]}" for i, t in enumerate(self.plan, 1)]
        return ["  ".join(tokens[i:i + per_line]) for i in range(0, len(tokens), per_line)]

    def print_summary(self, aborted: bool = False):
        print("\n=== สรุปภารกิจ" + (" (ถูกยกเลิก)" if aborted else "") + " ===")
        if not self.plan:
            print("  ยังกวาดหาเป้าไม่เสร็จ" if self.phase == "SCAN" else "  ไม่พบเป้าหมาย")
            return
        for i, t in enumerate(self.plan, 1):
            print(f"  {i}. {t.color.upper():<6} yaw {t.yaw:+6.1f}°  pitch {t.pitch:+5.1f}°  -> {t.status}")
        fired = sum(t.status == "FIRED" for t in self.plan)
        print(f"  ยิงแล้ว {fired}/{len(self.plan)} เป้า\n")


# ==========================================
# 11. แอปหลัก
# ==========================================
class AutoAimApp:
    def __init__(self):
        self.color_ranges, user_zones = load_config(CONFIG_PATH)
        self.blind_zones = BlindZones(AUTO_BLIND_ZONE, user_zones)
        self.mode = MODE_PREVIEW
        self.target_color = COLOR_ORDER[0]
        self.mission: Optional[Mission] = None

        self.pid_yaw = PIDController(**YAW_PID)
        self.pid_pitch = PIDController(**PITCH_PID)
        self.fire_ctrl = FireController()
        self.tuning = TuningPanel()

        self.ep_robot = None
        self.ep_gimbal = None
        self.ep_camera = None
        self.ep_blaster = None
        self.leds: Optional[LedManager] = None
        self._stream_started = False

        self._fps = 0.0
        self._last_frame_time: Optional[float] = None
        self._fire_banner_until = 0.0

    # ---------- วงจรชีวิตโปรแกรม ----------
    def run(self):
        try:
            if not self._connect():
                return
            if not run_setup_phase(self.ep_camera, self.blind_zones):
                print(">> ออกจากโปรแกรมตั้งแต่ขั้น Setup")
                return
            if not self._run_tuning_phase():
                print(">> ออกจากโปรแกรมตั้งแต่ขั้นจูนสี")
                return
            save_config(CONFIG_PATH, self.color_ranges, self.blind_zones.user_zones)
            self._recenter("ก่อนเริ่ม Loop หลัก")
            self._main_loop()
        except KeyboardInterrupt:
            print("\n>> ได้รับ Ctrl+C")
        finally:
            self._shutdown()

    def _connect(self) -> bool:
        print("=== กำลังเชื่อมต่อ RoboMaster... ===")
        self.ep_robot = robot.Robot()
        if not self.ep_robot.initialize(conn_type=CONN_TYPE):
            print("❌ เชื่อมต่อหุ่นไม่สำเร็จ ตรวจสอบ Wi-Fi / CONN_TYPE")
            return False

        self.ep_gimbal = self.ep_robot.gimbal
        self.ep_camera = self.ep_robot.camera
        self.ep_blaster = self.ep_robot.blaster
        safe_call("set robot mode", self.ep_robot.set_robot_mode, mode=ROBOT_MODE)

        # ดับไฟ Top + Blaster ก่อนเปิดกล้อง แล้วใช้แค่ไฟฐานล้อแสดงสถานะ
        self.leds = LedManager(self.ep_robot.led, self.ep_blaster)
        self.leds.silence_gimbal_lights()
        self._update_status_led()

        self._recenter("หลังเชื่อมต่อ")
        self.ep_camera.start_video_stream(display=False, resolution=STREAM_RESOLUTION)
        self._stream_started = True
        return True

    def _shutdown(self):
        print("\n=== กำลังปิดระบบอย่างปลอดภัย... ===")
        self.fire_ctrl.armed = False  # ปลดอาวุธก่อนทำอย่างอื่น
        if self.mission is not None:
            self.mission.print_summary(aborted=True)
            self.mission = None
        if self.tuning.is_open:
            self.tuning.close()
            save_config(CONFIG_PATH, self.color_ranges, self.blind_zones.user_zones)

        if self.ep_gimbal is not None:
            safe_call("gimbal stop", self.ep_gimbal.drive_speed, pitch_speed=0, yaw_speed=0)
            action = safe_call("gimbal recenter", self.ep_gimbal.recenter)
            if action is not None:
                safe_call("recenter wait", action.wait_for_completed, timeout=RECENTER_TIMEOUT_S)
        if self.leds is not None:
            self.leds.all_off()
        if self._stream_started:
            safe_call("stop video", self.ep_camera.stop_video_stream)
        if self.ep_robot is not None:
            safe_call("robot close", self.ep_robot.close)
        cv2.destroyAllWindows()
        print("=== ปิดระบบเรียบร้อย ===")

    # ---------- Tuning Phase: จูนทุกสีให้เสร็จก่อนเริ่ม ----------
    def _run_tuning_phase(self) -> bool:
        """ จูน HSV ทีละสีตาม COLOR_ORDER แล้วมีหน้าตรวจรวมทุกสี คืน False ถ้าผู้ใช้กดออก """
        steps: List[Optional[str]] = list(COLOR_ORDER) + [None]  # None = หน้าตรวจรวมทุกสี
        idx = 0
        self.tuning.open(self.color_ranges[COLOR_ORDER[0]])
        self._update_status_led()

        print("\n" + "=" * 60)
        print(" 🎨 TUNING: จูนสีทีละสี " + " -> ".join(c.upper() for c in COLOR_ORDER))
        print(" - เลื่อน Trackbar จนหน้าต่าง Preview เห็นเฉพาะเป้าสีนั้นเป็นก้อนเต็ม")
        print(" - [ENTER/SPACE/n] สีถัดไป  [p] ย้อนกลับ")
        print(" - [i/k] หันขึ้น/ลง  [j/l] หันซ้าย/ขวา (ถ้าเป้าสีนั้นอยู่นอกภาพ)  [c] Recenter")
        print(" - หน้าสุดท้ายตรวจรวมทุกสี: ทุกเป้าควรมีป้ายสีเดียวที่ถูกต้อง แล้วกด ENTER")
        print(" - [q/ESC] ออก")
        print("=" * 60 + "\n")

        while True:
            frame = read_frame(self.ep_camera)
            if frame is not None:
                color = steps[idx]
                hsv = prepare_hsv(frame)
                if color is not None:
                    rng = self.tuning.read()
                    if rng is None:  # ผู้ใช้ปิดหน้าต่าง Trackbar ไป -> เปิดใหม่ด้วยค่าล่าสุด
                        self.tuning.open(self.color_ranges[color])
                    else:
                        self.color_ranges[color] = rng
                    mask = build_mask(hsv, self.color_ranges[color], self.blind_zones)
                    targets = find_geometric_targets(mask, color)
                    self.tuning.show_preview(frame, mask)
                    title = f"TUNING {idx + 1}/{len(COLOR_ORDER)}: {color.upper()} - {len(targets)} target(s)"
                    detail = str(self.color_ranges[color])
                    title_color = DISPLAY_COLORS[color]
                else:
                    targets = detect_all_colors(hsv, self.color_ranges, self.blind_zones)
                    title = f"CHECK ALL COLORS - {len(targets)} target(s)"
                    detail = "Each target should have ONE correct label. ENTER = start"
                    title_color = (0, 255, 0)

                self.blind_zones.draw(frame)
                draw_targets(frame, targets)
                put_text(frame, title, (15, 30), title_color, 0.7)
                put_text(frame, detail, (15, 60), (255, 255, 255), 0.6)
                put_text(frame, "[ENTER] next  [p] back  [i/j/k/l] move gimbal  [c] center  [q] quit",
                         (15, 90), (200, 200, 200), 0.6)
                cv2.imshow(MAIN_WINDOW, frame)

            key = cv2.waitKey(1) & 0xFF
            ch = key_char(key)
            if ch == "q" or key == 27:
                self.tuning.close()
                return False
            if key in (13, 10, 32) or ch == "n":
                idx += 1
                if idx == len(steps):
                    break
                if steps[idx] is None:
                    self.tuning.close()
                else:
                    self.tuning.load(self.color_ranges[steps[idx]])
            elif ch == "p" and idx > 0:
                idx -= 1
                if self.tuning.is_open:
                    self.tuning.load(self.color_ranges[steps[idx]])
                else:
                    self.tuning.open(self.color_ranges[steps[idx]])
            elif ch in NUDGE_KEYS:
                pitch, yaw = NUDGE_KEYS[ch]
                safe_call("gimbal move", self.ep_gimbal.move, pitch=pitch, yaw=yaw, pitch_speed=90, yaw_speed=90)
            elif ch == "c":
                self._recenter("ระหว่างจูนสี")

        self._update_status_led()
        print(">> ✅ จูนสีครบทุกสีแล้ว")
        return True

    # ---------- Gimbal ----------
    def _reset_aim(self):
        self.pid_yaw.reset()
        self.pid_pitch.reset()
        self.fire_ctrl.reset_lock()

    def _stop_gimbal(self):
        safe_call("gimbal stop", self.ep_gimbal.drive_speed, pitch_speed=0, yaw_speed=0)
        self._reset_aim()

    def _recenter(self, reason: str):
        print(f"[GIMBAL] Recenter ({reason})")
        safe_call("gimbal stop", self.ep_gimbal.drive_speed, pitch_speed=0, yaw_speed=0)
        action = safe_call("gimbal recenter", self.ep_gimbal.recenter)
        if action is not None:
            safe_call("recenter wait", action.wait_for_completed, timeout=RECENTER_TIMEOUT_S)
        self._reset_aim()

    def _fire(self) -> bool:
        # ตรวจซ้ำอีกชั้นก่อนลั่นไก: โหมดจูนหรือ SAFE ต้องไม่ยิงเด็ดขาด
        if self.tuning.is_open or not self.fire_ctrl.armed:
            return False
        safe_call("gimbal stop", self.ep_gimbal.drive_speed, pitch_speed=0, yaw_speed=0)
        color = self.plan_color() if self.mode == MODE_MISSION else self.target_color
        print(f"🔥 ยิงเป้า {color.upper()}")
        safe_call("blaster fire", self.ep_blaster.fire, fire_type=blaster.WATER_FIRE, times=1)
        # ไม่สั่งไฟ Blaster กระพริบ; ดับไฟซ้ำเผื่อ firmware เปิดเองตอนยิง
        self.leds.silence_gimbal_lights()
        self.fire_ctrl.mark_fired()
        self.pid_yaw.reset()
        self.pid_pitch.reset()
        self._fire_banner_until = time.monotonic() + 0.3
        return True

    def plan_color(self) -> str:
        return self.mission.plan[self.mission.index].color

    # ---------- โหมดการทำงาน ----------
    def _enter_mode(self, mode: str):
        if self.mode == MODE_MISSION and mode != MODE_MISSION:
            self._abort_mission()
        if mode != self.mode:
            print(f">> โหมด: {mode}")
        self.mode = mode
        self._stop_gimbal()
        self._update_status_led()

    def _start_mission(self):
        if self.tuning.is_open:
            print(">> ⚠️ ปิดโหมดจูนสีก่อน (กด t) แล้วค่อยเริ่มภารกิจ")
            return
        if self.mode == MODE_MISSION:
            return
        self._enter_mode(MODE_MISSION)
        self.mission = Mission(self)
        print(f">> 🚀 เริ่มภารกิจ: กวาดหาเป้า {len(self.mission.poses)} มุม แล้วยิงตามลำดับ "
              + " -> ".join(c.upper() for c in COLOR_ORDER))
        if not self.fire_ctrl.armed:
            print("   Auto-Fire ปิดอยู่ = ซ้อมเล็งอย่างเดียว (กด s เปิดยิงได้ระหว่างภารกิจ)")

    def _abort_mission(self):
        self.mission.print_summary(aborted=True)
        self.mission = None
        if self.fire_ctrl.armed:
            self.fire_ctrl.armed = False
            print(">> Auto-Fire: OFF (SAFE) อัตโนมัติหลังยกเลิกภารกิจ")

    def _finish_mission(self):
        self.mission.print_summary()
        self.mission = None
        self.mode = MODE_PREVIEW
        if self.fire_ctrl.armed:
            self.fire_ctrl.armed = False
            print(">> Auto-Fire: OFF (SAFE) อัตโนมัติหลังจบภารกิจ")
        self._recenter("จบภารกิจ")
        self._update_status_led()

    # ---------- Loop หลัก ----------
    def _main_loop(self):
        print(">> 🎯 พร้อมทำงาน | [m] ยิงทุกเป้า  [r/g/b/y] เล็งสีเดียว  [x] หยุด  [s] Auto-Fire  "
              "[t] จูน  [c] Recenter  [q] ออก")
        print(f">> Auto-Fire: {'ON' if self.fire_ctrl.armed else 'OFF (SAFE)'}")
        cv2.namedWindow(MAIN_WINDOW)
        while True:
            frame = read_frame(self.ep_camera)
            if frame is None:
                # ไม่มีภาพ = ห้ามขยับตามข้อมูลเก่า (แต่ไม่ขัดจังหวะ moveto ที่กำลังหมุนอยู่)
                if self.mode == MODE_TRACK or (self.mission is not None and self.mission.sub == "AIM"):
                    self._stop_gimbal()
            else:
                self._process_frame(frame)
            if not self._handle_key(cv2.waitKey(1) & 0xFF):
                break

    def _process_frame(self, frame: np.ndarray):
        h, w = frame.shape[:2]
        now = time.monotonic()
        if self._last_frame_time is not None and now > self._last_frame_time:
            self._fps = 0.9 * self._fps + 0.1 / (now - self._last_frame_time)
        self._last_frame_time = now

        # 1) อ่านค่า Trackbar (ถ้าเปิดโหมดจูนอยู่)
        if self.tuning.is_open:
            rng = self.tuning.read()
            if rng is None:
                self._set_tuning(False)  # หน้าต่างจูนถูกปิดด้วยปุ่ม X
            else:
                self.color_ranges[self.target_color] = rng
        tuning = self.tuning.is_open

        hsv = prepare_hsv(frame)
        if tuning:
            self.tuning.show_preview(frame, build_mask(hsv, self.color_ranges[self.target_color], self.blind_zones))

        # 2) Vision + ควบคุมตามโหมด
        if self.mode == MODE_MISSION:
            self.mission.step(frame, hsv)
            state, targets = self.mission.state, self.mission.visible
            match, error = self.mission.match, self.mission.error
            if self.mission.done:
                self._finish_mission()
        elif self.mode == MODE_TRACK:
            state, targets, match, error = self._step_track(hsv, w, h, tuning)
        else:
            targets, match, error = detect_all_colors(hsv, self.color_ranges, self.blind_zones), None, None
            state = f"{len(targets)} target(s) in view - press [m] to fire at all"

        self.leds.keep_top_off()
        self._draw_hud(frame, targets, match, error, state)
        cv2.imshow(MAIN_WINDOW, frame)

    def _step_track(self, hsv: np.ndarray, w: int, h: int, tuning: bool):
        mask = build_mask(hsv, self.color_ranges[self.target_color], self.blind_zones)
        targets = find_geometric_targets(mask, self.target_color)
        best = targets[0] if targets else None
        if best is None:
            self._stop_gimbal()
            return "SEARCHING", targets, None, None
        if tuning and not TRACK_WHILE_TUNING:
            self._stop_gimbal()
            return "TUNING - GIMBAL HOLD", targets, best, None

        deg_x, deg_y = pixel_to_degrees(best.cx, best.cy, w, h)
        error = (deg_x - AIM_OFFSET_X_DEG, deg_y - AIM_OFFSET_Y_DEG)
        yaw_speed = self.pid_yaw.compute(error[0])
        pitch_speed = self.pid_pitch.compute(error[1])
        safe_call("gimbal drive", self.ep_gimbal.drive_speed, pitch_speed=pitch_speed, yaw_speed=yaw_speed)

        state, should_fire = self.fire_ctrl.evaluate(error[0], error[1], tuning)
        if should_fire:
            self._fire()
        return state, targets, best, error

    def _draw_hud(self, frame, targets: List[Target], match: Optional[Target],
                  error: Optional[Tuple[float, float]], state: str):
        h, w = frame.shape[:2]
        self.blind_zones.draw(frame)

        # จุดเล็ง + กรอบระยะ LOCK_TOLERANCE_DEG
        ax, ay = degrees_to_pixel(AIM_OFFSET_X_DEG, AIM_OFFSET_Y_DEG, w, h)
        tol_x = int(LOCK_TOLERANCE_DEG * w / FOV_X)
        tol_y = int(LOCK_TOLERANCE_DEG * h / FOV_Y)
        cv2.rectangle(frame, (ax - tol_x, ay - tol_y), (ax + tol_x, ay + tol_y), (200, 200, 200), 1)
        cv2.drawMarker(frame, (ax, ay), (255, 255, 255), markerType=cv2.MARKER_CROSS, markerSize=24, thickness=2)

        draw_targets(frame, targets)
        if match is not None:
            bgr = DISPLAY_COLORS[match.color]
            cv2.circle(frame, (match.cx, match.cy), 14, bgr, 2)
            cv2.line(frame, (ax, ay), (match.cx, match.cy), bgr, 2)

        if time.monotonic() < self._fire_banner_until:
            put_text(frame, "FIRE!", (ax - 45, ay - tol_y - 15), (0, 0, 255), 1.1, 3)

        if state.endswith("FIRE"):
            state_color = (0, 0, 255)
        elif "SETTLING" in state or "LOCKED" in state:
            state_color = (0, 255, 255)
        else:
            state_color = (255, 255, 255)

        mode_text = f"TRACK {self.target_color.upper()}" if self.mode == MODE_TRACK else self.mode
        fire_text = "ON" if self.fire_ctrl.armed else "OFF (SAFE)"
        lines = [
            (f"Mode: {mode_text} | Auto-Fire: {fire_text} | FPS: {self._fps:.0f}", (255, 255, 255)),
            (f"State: {state}", state_color),
        ]
        if error is not None:
            lines.append((f"Error: X {error[0]:+.1f} deg  Y {error[1]:+.1f} deg", (255, 255, 255)))
        if self.mission is not None:
            lines.extend((text, (255, 255, 0)) for text in self.mission.progress_lines())
        if self.tuning.is_open:
            lines.append((f"TUNING {self.target_color.upper()} - FIRE DISABLED", (0, 165, 255)))
            lines.append((str(self.color_ranges[self.target_color]), (0, 165, 255)))
        lines.append(("[m] mission  [r/g/b/y] track  [x] stop  [s] fire  [t] tune  [c] center  [q] quit",
                      (200, 200, 200)))

        y = 30
        for text, color in lines:
            put_text(frame, text, (15, y), color, 0.6)
            y += 28

    # ---------- คีย์บอร์ด ----------
    def _handle_key(self, key: int) -> bool:
        """ คืนค่า False เมื่อต้องการออกจากโปรแกรม """
        ch = key_char(key)
        if ch == "q" or key == 27:
            return False
        if ch == "m":
            self._start_mission()
        elif ch in KEY_TO_COLOR:
            if not self.tuning.is_open:
                self._enter_mode(MODE_TRACK)
            self._select_color(KEY_TO_COLOR[ch])
        elif ch == "x" or key == 32:
            self._enter_mode(MODE_PREVIEW)
        elif ch == "t":
            if self.mode == MODE_MISSION:
                self._enter_mode(MODE_PREVIEW)
            self._set_tuning(not self.tuning.is_open)
        elif ch == "s":
            self._toggle_auto_fire()
        elif ch == "c":
            if self.mode == MODE_MISSION:
                self._enter_mode(MODE_PREVIEW)
            self._recenter("กดปุ่ม c")
        return True

    def _select_color(self, color: str):
        self.target_color = color
        self._reset_aim()
        if self.tuning.is_open:
            self.tuning.load(self.color_ranges[color])
        self._update_status_led()
        print(f">> เลือกสี: {color.upper()} [{self.color_ranges[color]}]")

    def _set_tuning(self, enabled: bool):
        if enabled:
            self.tuning.open(self.color_ranges[self.target_color])
            print(f">> 🛠️ เปิดโหมดจูนสี {self.target_color.upper()} (ระบบยิงถูกล็อก, กด r/g/b/y เพื่อสลับสี)")
        else:
            self.tuning.close()
            save_config(CONFIG_PATH, self.color_ranges, self.blind_zones.user_zones)
            print(">> ปิดโหมดจูนสี")
        self._reset_aim()
        self._update_status_led()

    def _toggle_auto_fire(self):
        self.fire_ctrl.armed = not self.fire_ctrl.armed
        self.fire_ctrl.reset_lock()  # ต้องนิ่งครบ SETTLE_TIME_S ใหม่หลังเปิดระบบยิง
        print(f">> Auto-Fire: {'ON ⚠️' if self.fire_ctrl.armed else 'OFF (SAFE)'}")
        if self.fire_ctrl.armed and self.tuning.is_open:
            print("   (จะยังไม่ยิงจนกว่าจะปิดโหมดจูนสี)")
        self._update_status_led()

    def _update_status_led(self):
        color = self.target_color if self.mode == MODE_TRACK else "white"
        self.leds.show_status(color, self.fire_ctrl.armed, self.tuning.is_open)


if __name__ == "__main__":
    AutoAimApp().run()
