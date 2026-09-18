"""
RoboMaster EP - ตรวจจับป้ายสี แล้วยิงทุกเป้าต่อเนื่องตามลำดับสีที่เลือก

รวมของสองคน: ลอจิกจับป้ายสีและค่าการเล็ง/ล็อกเป้าจากโค้ดเพื่อน (grimball_shoot_3targets: vision.py, control.py)
+ โปรแกรม GUI หน้าต่างเดียว, calibrate, เลือกลำดับยิง และภารกิจกวาดหาเป้าจากไฟล์นี้

วิธีรัน
  python gimbalshoot.py --calibrate : วาดพื้นที่ตาบอด + จูนสีครบทุกสี แล้วบันทึก (ทำครั้งแรก หรือเมื่อแสงเปลี่ยน)
  python gimbalshoot.py             : ใช้ค่าที่ calibrate ไว้ เข้าหน้า READY เลือกลำดับแล้วยิงได้ทันที
  (รันครั้งแรกที่ยังไม่มีไฟล์ config จะเข้าโหมด calibrate ให้อัตโนมัติ)

ทุกอย่างอยู่ในหน้าต่างเดียว (1280x720): ภาพกล้องซ้ายบน, แผงปุ่มด้านขวา, แถบล่างเป็น slider จูนสี/ผลภารกิจ
คลิกปุ่มด้วยเมาส์ หรือใช้ปุ่มลัดคีย์บอร์ดที่เขียนในวงเล็บบนปุ่มก็ได้ (ต้องคลิกเลือกหน้าต่างก่อน)

หน้าจอ
  SETUP   (calibrate 1/2): ลากเมาส์บนภาพกล้อง = วาดกรอบพื้นที่ตาบอด       [u] ลบกรอบล่าสุด [x] ล้าง [ENTER] ถัดไป
  TUNE    (calibrate 2/2): เลือกสี แล้วลาก slider H/S/V จนภาพ MASK เห็นแต่เป้า
                           [r/g/b/y] เลือกสี [v] ตรวจรวมทุกสี [n/ENTER] ถัดไป [p] ย้อน [t] บันทึก
                           [i/k/j/l] หันหัว ขึ้น/ลง/ซ้าย/ขวา [c] หันกลับตรง
  READY                  : [r/g/b/y] เรียงลำดับสีที่จะยิง (คลิกสีแรก = เริ่มลำดับใหม่, คลิกซ้ำ = เอาออก)
                           [u/Backspace] ลบตัวท้าย [x] ล้าง [ENTER] เริ่มยิง [d] ซ้อมเล็งไม่ยิง
                           [t] จูนสี [c] หันกลับตรง  ปุ่ม CALIBRATE ALL = วาดพื้นที่ตาบอด + จูนใหม่ทั้งหมด
  MISSION                : กวาดหัวหาเป้ารอบตัว แล้วยิงทุกเป้าต่อกันตามลำดับสี (สีเดียวกันยิงซ้ายไปขวา)
                           [x/SPACE] หยุดทันที [s] เปิด/ปิดระบบยิง [c] หยุดแล้วหันกลับตรง
  ทุกหน้า                 : [q/ESC] ออกจากโปรแกรมอย่างปลอดภัย

ค่า HSV, พื้นที่ตาบอด และลำดับยิงล่าสุด บันทึกไว้ใน autoaim_config.json ข้างไฟล์นี้
(หน้า SETUP/TUNE บันทึกเมื่อกด SAVE & FINISH เท่านั้น)
"""
import argparse
import json
import math
import queue
import textwrap
import threading
import time
from collections import deque
from dataclasses import asdict, dataclass, field, fields, replace
from functools import lru_cache
from pathlib import Path
from typing import Callable, Dict, List, Optional, Set, Tuple

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
STREAM_RESOLUTION = "540p"      # 960x540 = ขนาดภาพใน GUI พอดี ส่งผ่าน Wi-Fi/ถอดรหัสเบากว่า 720p ("360p"/"540p"/"720p")
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

# --- Vision: ลอจิกจับป้ายสีจากโค้ดเพื่อน (grimball_shoot_3targets/vision.py) ---
# กรองสี HSV -> Open/Close -> คอนทัวร์จาก mask + คอนทัวร์จากขอบภาพ (Canny)
# -> รับเฉพาะก้อนที่ "หน้าตาเป็นป้าย": ขนาดพอดี, สัดส่วนไม่ยาวเกิน, ทึบเต็มกรอบ, ข้างในเป็นสีนั้นจริง
COLOR_ORDER = ("red", "green", "blue", "yellow")  # ลำดับการจูนสี และลำดับยิงเริ่มต้น (เปลี่ยนได้ในหน้า READY)
PROCESS_W = 960                 # ประมวลผลที่ความกว้างนี้ (540p = 960 พอดี ไม่ต้องย่อ) ภาพใหญ่กว่านี้จะถูกย่อก่อน
MIN_AREA_RATIO = 0.0004         # ป้ายเล็กสุด เทียบกับพื้นที่ภาพ
MAX_AREA_RATIO = 0.025          # ป้ายใหญ่สุด กันจับผนัง/ของใหญ่สีเดียวกัน
MAX_SIGN_W, MAX_SIGN_H = 0.18, 0.22  # ความกว้าง/สูงสูงสุดของป้าย เทียบกับภาพ
SIGN_ASPECT = (0.55, 2.0)       # ช่วงสัดส่วน กว้าง/สูง ของป้าย
MIN_EXTENT = 0.65               # พื้นที่คอนทัวร์ / พื้นที่กรอบ (สี่เหลี่ยม ~1.0, สามเหลี่ยม ~0.5 -> เป้าสามเหลี่ยมใช้ 0.45)
MIN_FILL = 0.75                 # ข้างในกรอบ (ตัดขอบออก 1/5) ต้องเป็นสีนั้นอย่างน้อยเท่านี้ (สามเหลี่ยมใช้ ~0.5)
BORDER_PX = 3                   # ป้ายที่ชิดขอบภาพ (เห็นไม่เต็มแผ่น) ไม่นับ
DEDUP_RATIO = 0.012             # จุดกลางห่างกันน้อยกว่านี้ (สัดส่วนภาพ) = ป้ายเดียวกัน
MAX_CANDIDATES = 12             # เก็บป้ายต่อสีได้สูงสุดกี่อัน (เรียงจากใหญ่ไปเล็ก)
CANNY_LOW, CANNY_HIGH = 40, 120  # ขอบภาพช่วยแยกป้ายออกจากพื้นหลังสีเดียวกัน (เช่นป้ายเหลืองบนผนังเหลือง)

# ช่วง HSV เริ่มต้น (ค่าที่จูนไว้ในโค้ดเพื่อน): (H_min, H_max, S_min, S_max, V_min, V_max)
# ถ้า H_min > H_max แปลว่าช่วง Hue วนรอบวงล้อสี เช่นสีแดง 170..179 + 0..10
# ใช้เมื่อยังไม่เคย calibrate หรือกดปุ่ม RESET TO DEFAULT ในหน้าจูนสี
DEFAULT_HSV = {
    "red":    (170, 10, 120, 255, 70, 255),
    "green":  (35, 90, 70, 255, 25, 255),
    "blue":   (90, 140, 50, 255, 20, 255),
    "yellow": (20, 35, 153, 255, 100, 255),
}

# บังพลาสติกสีแดง/ส้มของหัว Gimbal (ด้านล่างกลางจอ) = กล่อง ±250 px สูง 180 px ที่ 720p
AUTO_BLIND_ZONE: Zone = (0.30, 0.75, 0.70, 1.00)
MIN_ZONE_PX = 10                # กรอบที่ลากเล็กกว่านี้ถือว่าคลิกพลาด ไม่บันทึก

# --- Tuning Phase ---
NUDGE_DEG = 15                  # ปุ่ม i/j/k/l หมุนหัวครั้งละกี่องศา (ใช้ตอนเป้าสีนั้นอยู่นอกภาพ)

# --- เล็ง: ค่าจากโค้ดเพื่อน (control.py + autoaim_config.json) input = องศา, output = deg/s ---
# yaw = PD (kp 2.5, kd 0.05, กรอง D 0.7) ช้าสุด 25 deg/s | pitch = P (kp 2.0) ช้าสุด 10 deg/s
# ความเร็วต่ำ = หัวไม่เลยเป้า เพราะภาพจากกล้องผ่าน Wi-Fi มาช้ากว่าความจริงเล็กน้อย
YAW_PID = {"kp": 2.5, "ki": 0.0, "kd": 0.05, "output_limit": 25.0, "d_smoothing": 0.7}
PITCH_PID = {"kp": 2.0, "ki": 0.0, "kd": 0.0, "output_limit": 10.0}
PID_INTEGRAL_LIMIT = 50.0       # deg*s (ใช้เมื่อ ki > 0)

# --- Auto-Fire (ระบบยิงเปิดเฉพาะตอนกด ENTER เริ่มภารกิจ และปิดเองเมื่อจบ/หยุด) ---
LOCK_TOLERANCE_X_DEG = 0.25     # ค่าจากโค้ดเพื่อน: error แกน X และ Y ต้องน้อยกว่านี้
LOCK_TOLERANCE_Y_DEG = 0.25
MIN_LOCK_HITS = 3               # ต้องเห็นเป้าติดกันอย่างน้อยกี่เฟรมก่อนเริ่มนับเวลานิ่ง
SETTLE_TIME_S = 0.5             # และต้องค้างในระยะนานเกินนี้ถึงจะยิง
FIRE_COOLDOWN_S = 1.0           # เว้นระยะระหว่างนัด (ไม่ใช้ sleep จึงไม่ทำให้ภาพค้าง)
START_KEY_GUARD_S = 1.0         # หลังเข้าหน้า READY ไม่รับปุ่มเริ่มยิงช่วงนี้ กัน ENTER ที่กดซ้ำมาจากขั้นก่อน

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
MATCH_RADIUS_DEG = 8.0          # ตอนเล็งละเอียด หาเป้าในรัศมีนี้รอบจุดเล็ง (ค่าจากโค้ดเพื่อน)
MATCH_AREA_RATIO = (0.35, 2.8)  # ขนาดป้ายต้องใกล้กับตอนกวาดหา กันสลับไปเล็งป้ายอื่น (ค่าจากโค้ดเพื่อน)
LOST_REPOINT_S = 1.0            # เป้าหายจากภาพนานเกินนี้ -> หันกลับไปตำแหน่งตามแผนแล้วหาใหม่
ENGAGE_TIMEOUT_S = 10.0         # ใช้เวลากับเป้าเดียวนานเกินนี้ -> ข้ามเป้านั้น
SHOTS_PER_TARGET = 1
DRY_RUN_HOLD_S = 0.5            # Auto-Fire ปิด: ค้างเล็งให้ดูกี่วินาทีก่อนไปเป้าถัดไป

# --- LED ---
TOP_LED_REFRESH_S = 3.0         # สั่งดับไฟ Top ซ้ำทุกกี่วินาที เผื่อ firmware เปิดเอง (เช่นตอนโดนยิง)

# ไฟฐานล้อใช้สีขาว: ความอิ่มสีต่ำ แสงสะท้อนจึงไม่ถูกจับเป็นเป้า
LED_COLORS = {"red": (255, 0, 0), "green": (0, 255, 0), "blue": (0, 0, 255), "yellow": (255, 255, 0),
              "white": (255, 255, 255)}  # RGB
DISPLAY_COLORS = {"red": (0, 0, 255), "green": (0, 255, 0), "blue": (255, 120, 0), "yellow": (0, 255, 255)}  # BGR
KEY_TO_COLOR = {"r": "red", "g": "green", "b": "blue", "y": "yellow"}
NUDGE_KEYS = {"i": (NUDGE_DEG, 0), "k": (-NUDGE_DEG, 0), "j": (0, -NUDGE_DEG), "l": (0, NUDGE_DEG)}  # (pitch, yaw)

COLOR_SHORT = {"red": "RED", "green": "GRN", "blue": "BLU", "yellow": "YEL"}

# --- GUI หน้าต่างเดียว ---
WINDOW_NAME = "RoboMaster Auto-Aim"
VIEW_W, VIEW_H = 960, 540       # ภาพกล้องในหน้าต่าง (ย่อจาก 720p)
PANEL_W = 320                   # แผงปุ่มด้านขวา
STRIP_H = 180                   # แถบล่าง: slider จูนสี / คำแนะนำ / ผลภารกิจ
CANVAS_W, CANVAS_H = VIEW_W + PANEL_W, VIEW_H + STRIP_H  # 1280 x 720
UI_BG = (30, 30, 30)            # สีทั้งหมดเป็น BGR
UI_PANEL = (45, 45, 45)
UI_BTN = (85, 85, 85)
UI_TEXT = (235, 235, 235)
UI_DIM = (150, 150, 150)
UI_GREEN = (60, 140, 60)
UI_OK = (120, 220, 120)
UI_RED = (40, 40, 200)
UI_BLUE = (150, 100, 40)
UI_ORANGE = (0, 130, 230)
SCREEN_SETUP, SCREEN_TUNE, SCREEN_READY, SCREEN_MISSION = "SETUP", "TUNE", "READY", "MISSION"

CONFIG_PATH = Path(__file__).resolve().with_name("autoaim_config.json")
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


def load_config(path: Path) -> Tuple[Dict[str, HSVRange], List[Zone], List[str], bool]:
    """ โหลดค่าที่ calibrate ไว้: HSV, พื้นที่ตาบอด และลำดับยิงล่าสุด
    ค่าสุดท้าย (found) เป็น False ถ้าไม่มีไฟล์หรือไฟล์เสีย = ต้อง calibrate ใหม่
    """
    ranges = default_color_ranges()
    if not path.exists():
        return ranges, [], list(COLOR_ORDER), False
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
        order = [c for c in data.get("fire_order", COLOR_ORDER) if c in ranges]
        order = list(dict.fromkeys(order))  # ตัดสีซ้ำ คงลำดับเดิม
    except (OSError, ValueError, KeyError, TypeError) as e:
        print(f"[CONFIG] ⚠️ อ่าน {path.name} ไม่ได้ ({e}) ใช้ค่าเริ่มต้นแทน")
        return default_color_ranges(), [], list(COLOR_ORDER), False
    print(f"[CONFIG] โหลดค่า HSV และพื้นที่ตาบอด {len(zones)} กรอบ จาก {path.name}")
    return ranges, zones, order, True


def save_config(path: Path, ranges: Dict[str, HSVRange], zones: List[Zone], order: List[str]):
    data = {
        "hsv": {name: asdict(rng) for name, rng in ranges.items()},
        "user_blind_zones": [[round(v, 4) for v in zone] for zone in zones],
        "fire_order": list(order),
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


# ==========================================
# 5. Vision: จับป้ายสี (ลอจิกจากโค้ดเพื่อน grimball_shoot_3targets/vision.py)
# ==========================================
@dataclass
class Target:
    color: str
    cx: float                         # จุดกลางกรอบป้าย (พิกเซลของภาพที่ประมวลผล)
    cy: float
    area: float
    bbox: Tuple[int, int, int, int]   # x, y, w, h


@dataclass
class FrameData:
    """ ข้อมูลที่คำนวณครั้งเดียวต่อเฟรม แล้วใช้ร่วมกันทุกสี """
    bgr: np.ndarray                   # ภาพขนาดที่ประมวลผล (พิกัดเป้าทั้งหมดอ้างอิงภาพนี้)
    hsv: np.ndarray
    edge_boxes: List[Tuple[float, Tuple[int, int, int, int]]]  # (พื้นที่, กรอบ) จากขอบภาพที่รูปร่างเป็นป้าย

    @property
    def size(self) -> Tuple[int, int]:
        h, w = self.hsv.shape[:2]
        return w, h


_MORPH_KERNEL = np.ones((3, 3), np.uint8)


def _sign_box(contour, w: int, h: int) -> Optional[Tuple[float, Tuple[int, int, int, int]]]:
    """ เกณฑ์รูปร่างป้าย (ไม่ขึ้นกับสี) คืน (พื้นที่, กรอบ) หรือ None ถ้าไม่เหมือนป้าย """
    x, y, bw, bh = cv2.boundingRect(contour)
    frame_area = w * h
    if bw * bh < MIN_AREA_RATIO * frame_area:
        return None  # เศษเล็กๆ ตัดทิ้งก่อนคำนวณอย่างอื่น (ภาพที่มีลายเยอะมีคอนทัวร์จากขอบเป็นพันเส้น)
    if bw > w * MAX_SIGN_W or bh > h * MAX_SIGN_H or not SIGN_ASPECT[0] <= bw / bh <= SIGN_ASPECT[1]:
        return None
    if x < BORDER_PX or y < BORDER_PX or x + bw >= w - BORDER_PX or y + bh >= h - BORDER_PX:
        return None  # ชิดขอบภาพ = เห็นป้ายไม่เต็มแผ่น
    area = cv2.contourArea(contour)
    if not MIN_AREA_RATIO * frame_area <= area <= MAX_AREA_RATIO * frame_area:
        return None
    if area / (bw * bh) < MIN_EXTENT:
        return None  # ไม่ทึบเต็มกรอบ = ก้อนสีรูปร่างมั่ว
    return area, (x, y, bw, bh)


def prepare_frame(frame_bgr: np.ndarray) -> FrameData:
    """ ย่อภาพ (ถ้าใหญ่กว่า PROCESS_W) -> HSV และขอบภาพ ทำครั้งเดียวต่อเฟรม
    โค้ดเพื่อนคำนวณขอบภาพซ้ำทุกสี ที่นี่คำนวณครั้งเดียวและคัดรูปร่างป้ายไว้ก่อน จึงเร็วกว่ามาก
    """
    h, w = frame_bgr.shape[:2]
    small = frame_bgr
    if w > PROCESS_W:
        small = cv2.resize(frame_bgr, (PROCESS_W, int(round(h * PROCESS_W / w))), interpolation=cv2.INTER_AREA)
    sh, sw = small.shape[:2]
    hsv = cv2.cvtColor(cv2.GaussianBlur(small, (5, 5), 0), cv2.COLOR_BGR2HSV)
    edges = cv2.Canny(cv2.cvtColor(small, cv2.COLOR_BGR2GRAY), CANNY_LOW, CANNY_HIGH)
    edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, _MORPH_KERNEL)
    edge_boxes = []
    for contour in cv2.findContours(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)[-2]:
        box = _sign_box(contour, sw, sh)
        if box is not None:
            edge_boxes.append(box)
    return FrameData(small, hsv, edge_boxes)


def build_mask(hsv: np.ndarray, hsv_range: HSVRange, blind_zones: BlindZones) -> np.ndarray:
    mask = hsv_range.mask(hsv)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, _MORPH_KERNEL)   # ลบจุดสีเล็กๆ
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, _MORPH_KERNEL)  # อุดรูในป้าย
    blind_zones.apply(mask)  # ทำเป็นขั้นสุดท้าย พื้นที่ตาบอดจึงดำสนิทแน่นอน
    return mask


def find_targets(fd: FrameData, mask: np.ndarray, color: str) -> List[Target]:
    """ ป้ายสี color ทั้งหมดในภาพ เรียงจากใหญ่ไปเล็ก (สูงสุด MAX_CANDIDATES)
    ผู้สมัคร = คอนทัวร์จาก mask สีนั้น + คอนทัวร์จากขอบภาพ ที่รูปร่างเป็นป้าย
    แล้วข้างในกรอบ (ตัดขอบออก 1/5) ต้องเป็นสีนั้นอย่างน้อย MIN_FILL
    """
    w, h = fd.size
    boxes = []
    for contour in cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]:
        box = _sign_box(contour, w, h)
        if box is not None:
            boxes.append(box)
    boxes += fd.edge_boxes

    found: List[Target] = []
    for area, (x, y, bw, bh) in boxes:
        inset = max(2, min(bw, bh) // 5)
        interior = mask[y + inset:y + bh - inset, x + inset:x + bw - inset]
        if interior.size == 0 or cv2.countNonZero(interior) / interior.size < MIN_FILL:
            continue
        cx, cy = x + bw / 2.0, y + bh / 2.0
        if any(abs(cx - t.cx) < DEDUP_RATIO * w and abs(cy - t.cy) < DEDUP_RATIO * h for t in found):
            continue  # ป้ายเดียวกันที่เจอทั้งจาก mask และจากขอบภาพ
        found.append(Target(color, cx, cy, area, (x, y, bw, bh)))
    found.sort(key=lambda t: t.area, reverse=True)
    return found[:MAX_CANDIDATES]


def detect_all_colors(fd: FrameData, ranges: Dict[str, HSVRange], blind_zones: BlindZones) -> List[Target]:
    found: List[Target] = []
    for color in COLOR_ORDER:
        found.extend(find_targets(fd, build_mask(fd.hsv, ranges[color], blind_zones), color))
    return found


def draw_targets(frame: np.ndarray, targets: List[Target], scale: float = 1.0):
    """ scale = ขนาดภาพที่วาด / ขนาดภาพที่ประมวลผล """
    for t in targets:
        bgr = DISPLAY_COLORS[t.color]
        x, y, bw, bh = (int(round(v * scale)) for v in t.bbox)
        cv2.rectangle(frame, (x, y), (x + bw, y + bh), bgr, 2)
        cv2.circle(frame, (int(t.cx * scale), int(t.cy * scale)), 3, (255, 255, 255), -1)
        put_text(frame, t.color.upper(), (x, max(14, y - 6)), bgr, 0.5, 1)


# ==========================================
# 6. PID Controller
# ==========================================
class PIDController:
    """ PID ของ Gimbal หนึ่งแกน
    Input : error เป็นองศา (มุมระหว่างเป้ากับจุดเล็ง)
    Output: ความเร็วเชิงมุม deg/s สำหรับ gimbal.drive_speed()
    """
    STALE_DT_S = 0.5  # ห่างจากรอบก่อนนานเกินนี้ (เช่นหลัง recenter) ถือว่าเริ่มใหม่

    def __init__(self, kp, ki, kd, output_limit=25.0, integral_limit=PID_INTEGRAL_LIMIT,
                 deadband=0.0, d_smoothing=0.5):
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
    """ ตัดสินว่ายิงได้หรือยัง (เกณฑ์ล็อกจากโค้ดเพื่อน)
    ยิงเมื่อเห็นเป้าติดกันอย่างน้อย MIN_LOCK_HITS เฟรม และ |err_x| <= LOCK_TOLERANCE_X_DEG,
    |err_y| <= LOCK_TOLERANCE_Y_DEG ต่อเนื่องนานเกิน SETTLE_TIME_S
    และ Auto-Fire เปิด และไม่อยู่ในโหมดจูน และพ้น cooldown
    """

    def __init__(self):
        self.armed = False    # เปิดเฉพาะตอนสั่งเริ่มยิง
        self.settled = False  # เล็งนิ่งครบเวลาแล้วหรือยัง (ใช้ตอนซ้อมเล็งในภารกิจ)
        self._lock_start: Optional[float] = None
        self._hits = 0
        self._last_fire = float("-inf")

    def reset_lock(self):
        self._lock_start = None
        self._hits = 0
        self.settled = False

    def evaluate(self, err_x: float, err_y: float, tuning: bool) -> Tuple[str, bool]:
        """ คืนค่า (ข้อความสถานะสำหรับ HUD, ควรยิงหรือไม่) เรียกเฉพาะเฟรมที่เห็นเป้า """
        now = time.monotonic()
        self.settled = False
        self._hits += 1
        aligned = abs(err_x) <= LOCK_TOLERANCE_X_DEG and abs(err_y) <= LOCK_TOLERANCE_Y_DEG
        if not aligned or self._hits < MIN_LOCK_HITS:
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
    - ไฟ Bottom (ฐานล้อ): แสดงสถานะด้วยสีขาว
        ติดค้าง = SAFE, กระพริบ = Auto-Fire พร้อมยิง, หายใจ = โหมดจูน

    คำสั่งไฟทุกคำสั่งส่งจาก thread แยก เพราะ set_led ต้องรอหุ่นตอบกลับ (Wi-Fi สะดุดรอได้ถึง 3 วินาที)
    ถ้าส่งจาก Loop หลัก ภาพและปุ่มจะค้างทุกครั้งที่ส่ง
    """
    _STOP = object()

    def __init__(self, ep_led, ep_blaster):
        self._led = ep_led
        self._blaster = ep_blaster
        self._bottom_state = None
        self._jobs: "queue.Queue" = queue.Queue()
        self._thread = threading.Thread(target=self._worker, name="led-worker", daemon=True)
        self._thread.start()

    def _worker(self):
        next_top_off = 0.0
        while True:
            try:
                job = self._jobs.get(timeout=0.2)
            except queue.Empty:
                job = None
            if job is self._STOP:
                return
            if job is not None:
                job()
            if time.monotonic() >= next_top_off:  # ดับไฟ Top ซ้ำเป็นระยะ เผื่อ firmware เปิดเอง (เช่นตอนโดนยิง)
                self._top_off()
                next_top_off = time.monotonic() + TOP_LED_REFRESH_S

    def _top_off(self):
        safe_call("top LED off", self._led.set_led, comp=led.COMP_TOP_ALL, r=0, g=0, b=0, effect=led.EFFECT_OFF)

    def silence_gimbal_lights(self):
        def job():
            self._top_off()
            safe_call("blaster LED off", self._blaster.set_led, brightness=0, effect=blaster.LED_OFF)
        self._jobs.put(job)

    def show_status(self, color: str, armed: bool, tuning: bool):
        if tuning:
            effect = led.EFFECT_BREATH
        elif armed:
            effect = led.EFFECT_FLASH
        else:
            effect = led.EFFECT_ON
        state = (color, effect)
        if state == self._bottom_state:
            return  # ส่งคำสั่งเฉพาะตอนสถานะเปลี่ยน
        self._bottom_state = state
        r, g, b = LED_COLORS[color]
        self._jobs.put(lambda: safe_call("bottom LED", self._led.set_led, comp=led.COMP_BOTTOM_ALL,
                                         r=r, g=g, b=b, effect=effect, freq=2))

    def close(self):
        """ หยุด thread แล้วดับไฟทั้งหมด (ตอนปิดโปรแกรม) """
        self._jobs.put(self._STOP)
        self._thread.join(timeout=5)
        safe_call("all LED off", self._led.set_led, comp=led.COMP_ALL, r=0, g=0, b=0, effect=led.EFFECT_OFF)
        self._bottom_state = None


# ==========================================
# 9. GUI หน้าต่างเดียว (ปุ่ม/slider วาดเองด้วย OpenCV คลิกเมาส์ได้ และมีปุ่มลัดคีย์บอร์ด)
# ==========================================
def text_color_for(bg: Tuple[int, int, int]) -> Tuple[int, int, int]:
    """ ตัวหนังสือดำบนพื้นสว่าง (เช่นปุ่มสีเหลือง) และขาวบนพื้นเข้ม """
    b, g, r = bg
    return (20, 20, 20) if 0.114 * b + 0.587 * g + 0.299 * r > 150 else UI_TEXT


def label(canvas, text, x, y, color=UI_TEXT, scale=0.5, thickness=1, max_w: Optional[int] = None):
    """ เขียนข้อความบนแผง GUI ถ้ากำหนด max_w จะย่อตัวอักษรจนพอดีความกว้าง """
    if max_w is not None:
        while scale > 0.35 and cv2.getTextSize(text, FONT, scale, thickness)[0][0] > max_w:
            scale -= 0.03
    cv2.putText(canvas, text, (x, y), FONT, scale, color, thickness, cv2.LINE_AA)


def dim(color: Tuple[int, int, int], factor: float = 0.35) -> Tuple[int, int, int]:
    return tuple(int(c * factor) for c in color)


@lru_cache(maxsize=4)
def hue_bar(w: int, h: int) -> np.ndarray:
    """ แถบสีรุ้งใต้ slider ของ H ให้เห็นว่าค่า Hue แต่ละช่วงคือสีอะไร """
    hues = np.tile(np.linspace(0, 179, w).astype(np.uint8), (h, 1))
    full = np.full((h, w), 255, np.uint8)
    return cv2.cvtColor(np.dstack([hues, full, full]), cv2.COLOR_HSV2BGR)


@dataclass
class Button:
    x: int
    y: int
    w: int
    h: int
    text: str
    action: str
    color: Tuple[int, int, int] = UI_BTN
    enabled: bool = True
    selected: bool = False

    def contains(self, px: int, py: int) -> bool:
        return self.x <= px < self.x + self.w and self.y <= py < self.y + self.h

    def draw(self, canvas: np.ndarray, hover: bool):
        color = self.color if self.enabled else (55, 55, 55)
        if hover and self.enabled:
            color = tuple(min(255, c + 35) for c in color)
        cv2.rectangle(canvas, (self.x, self.y), (self.x + self.w, self.y + self.h), color, -1)
        if self.selected:
            cv2.rectangle(canvas, (self.x, self.y), (self.x + self.w, self.y + self.h), (255, 255, 255), 2)
        big = self.h >= 50
        scale, thickness = (0.62, 2) if big else (0.5, 1)
        while scale > 0.35 and cv2.getTextSize(self.text, FONT, scale, thickness)[0][0] > self.w - 10:
            scale -= 0.03
        (tw, th), _ = cv2.getTextSize(self.text, FONT, scale, thickness)
        fg = text_color_for(color) if self.enabled else (110, 110, 110)
        cv2.putText(canvas, self.text, (self.x + (self.w - tw) // 2, self.y + (self.h + th) // 2),
                    FONT, scale, fg, thickness, cv2.LINE_AA)


@dataclass
class Slider:
    x: int
    y: int
    w: int
    text: str
    value: int
    max_value: int
    on_change: Callable[[int], None]
    hue: bool = False
    TRACK_DY = 20   # ระยะจากบรรทัดชื่อลงมาถึงราง slider
    TRACK_H = 14

    def contains(self, px: int, py: int) -> bool:
        ty = self.y + self.TRACK_DY
        return self.x - 8 <= px <= self.x + self.w + 8 and ty - 10 <= py <= ty + self.TRACK_H + 10

    def set_from_x(self, px: int):
        value = int(round((px - self.x) / self.w * self.max_value))
        value = min(max(value, 0), self.max_value)
        if value != self.value:
            self.value = value
            self.on_change(value)

    def draw(self, canvas: np.ndarray):
        ty = self.y + self.TRACK_DY
        label(canvas, f"{self.text}: {self.value}", self.x, self.y + 12, UI_TEXT, 0.5)
        if self.hue:
            canvas[ty:ty + self.TRACK_H, self.x:self.x + self.w] = hue_bar(self.w, self.TRACK_H)
        else:
            cv2.rectangle(canvas, (self.x, ty), (self.x + self.w, ty + self.TRACK_H), (85, 85, 85), -1)
            fill = self.x + int(self.value / self.max_value * self.w)
            cv2.rectangle(canvas, (self.x, ty), (fill, ty + self.TRACK_H), (160, 160, 160), -1)
        kx = self.x + int(self.value / self.max_value * self.w)
        cv2.rectangle(canvas, (kx - 4, ty - 4), (kx + 4, ty + self.TRACK_H + 4), (255, 255, 255), -1)
        cv2.rectangle(canvas, (kx - 4, ty - 4), (kx + 4, ty + self.TRACK_H + 4), (0, 0, 0), 1)


class Ui:
    """ เก็บปุ่ม/slider ของเฟรมล่าสุด (วาดใหม่ทุกเฟรม) และแปลงการคลิกเมาส์เป็น action
    action ถูกเก็บเข้าคิว แล้วให้ Loop หลักเป็นคนทำ (ไม่สั่งหุ่นจากใน mouse callback)
    """

    def __init__(self):
        self.buttons: List[Button] = []
        self.sliders: List[Slider] = []
        self.actions = deque()
        self.mouse = (-1, -1)
        self.zone_drawer: Optional[ZoneDrawer] = None  # เปิดใช้เฉพาะหน้า SETUP
        self._dragging: Optional[Slider] = None

    def begin(self):
        self.buttons, self.sliders = [], []

    def button(self, canvas, x, y, w, h, text, action, color=UI_BTN, enabled=True, selected=False):
        b = Button(x, y, w, h, text, action, color, enabled, selected)
        b.draw(canvas, b.contains(*self.mouse))
        self.buttons.append(b)

    def slider(self, canvas, x, y, w, text, value, max_value, on_change, hue=False):
        s = Slider(x, y, w, text, value, max_value, on_change, hue)
        s.draw(canvas)
        self.sliders.append(s)

    def on_mouse(self, event, x, y, flags, param):
        self.mouse = (x, y)
        if event == cv2.EVENT_LBUTTONDOWN:
            for b in self.buttons:
                if b.enabled and b.contains(x, y):
                    self.actions.append(b.action)
                    return
            for s in self.sliders:
                if s.contains(x, y):
                    self._dragging = s
                    s.set_from_x(x)
                    return
            if self.zone_drawer is not None and x < VIEW_W and y < VIEW_H:
                self.zone_drawer.on_mouse(event, x, y, flags, param)
        elif event == cv2.EVENT_MOUSEMOVE:
            if self._dragging is not None:
                self._dragging.set_from_x(x)
            elif self.zone_drawer is not None:
                self.zone_drawer.on_mouse(event, x, y, flags, param)
        elif event == cv2.EVENT_LBUTTONUP:
            if self._dragging is not None:
                self._dragging = None
            elif self.zone_drawer is not None:
                self.zone_drawer.on_mouse(event, x, y, flags, param)


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

    def build_plan(self, order: List[str]) -> List[PlannedTarget]:
        """ order = สีที่จะยิงเรียงตามลำดับ สีที่ไม่อยู่ใน order จะไม่ถูกยิง """
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
        # กวาดหาทุกสีแล้วค่อยกรอง: วัตถุที่ผ่าน 2 ช่วงสีถูกตัดสินเป็นสีที่เห็นบ่อยกว่าไปแล้วใน end_pose()
        # ลำดับยิง: ตามลำดับสีที่เลือก แล้วซ้ายไปขวา (บนลงล่างถ้า yaw เท่ากัน)
        chosen = [c for c in clusters if c.color in order]
        return sorted(chosen, key=lambda c: (order.index(c.color), c.yaw, -c.pitch))


class Mission:
    """ ภารกิจยิงทุกเป้าตามลำดับสี แบบ non-blocking (เรียก step() ทุกเฟรม ภาพและปุ่มจึงไม่ค้าง)

    SCAN   : หมุนไปทีละมุมใน SCAN_PITCH_ANGLES x SCAN_YAW_ANGLES เก็บตำแหน่งเป้าทุกสี
    ENGAGE : ยิงตามลำดับสีใน order: หมุนไปยังเป้า (moveto) -> เล็งละเอียดด้วยภาพ + PID -> ยิง -> เป้าถัดไป
    DONE   : จบภารกิจ

    ใช้ gimbal.moveto ซึ่งอ้างอิง yaw กับฐานล้อ และ pitch กับแนวระนาบ จึงเป็นพิกัดเดียวกับ recenter
    """
    STATUS_SHORT = {"PENDING": "..", "ENGAGING": ">>", "FIRED": "SHOT", "AIMED-SAFE": "AIM", "SKIPPED": "SKIP"}
    MOVE_RETRY_S = 0.5

    def __init__(self, app: "AutoAimApp", order: List[str]):
        self.app = app
        self.order = list(order)
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
    def step(self, fd: FrameData):
        self.visible, self.match, self.error = [], None, None
        if self.phase == "SCAN":
            self._step_scan(fd)
        elif self.phase == "ENGAGE":
            self._step_engage(fd)

    def _step_scan(self, fd: FrameData):
        pitch, yaw = self.pose
        ready = self._ready()
        self.state = f"SCAN {self.index + 1}/{len(self.poses)} (pitch {pitch:+.0f}, yaw {yaw:+.0f}) {self.sub}"
        if not ready:
            return

        w, h = fd.size
        self.visible = detect_all_colors(fd, self.app.color_ranges, self.app.blind_zones)
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

        self.plan = self.target_map.build_plan(self.order)
        if not self.plan:
            print("[MISSION] ไม่พบเป้าสีที่เลือก")
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

    def _step_engage(self, fd: FrameData):
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
        # และขนาดต้องใกล้กับตอนกวาดหา (แบบโค้ดเพื่อน) กันสลับไปเล็งป้ายสีเดียวกันที่อยู่ติดกัน
        w, h = fd.size
        mask = build_mask(fd.hsv, app.color_ranges[t.color], app.blind_zones)
        self.visible = find_targets(fd, mask, t.color)
        ref = self._last_match_deg or (AIM_OFFSET_X_DEG, AIM_OFFSET_Y_DEG)
        best, best_deg, best_dist = None, None, MATCH_RADIUS_DEG
        for cand in self.visible:
            if not MATCH_AREA_RATIO[0] < cand.area / t.area < MATCH_AREA_RATIO[1]:
                continue
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

        fire_state, should_fire = app.fire_ctrl.evaluate(err_x, err_y, app.is_tuning)
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
            print("  ยังกวาดหาเป้าไม่เสร็จ" if self.phase == "SCAN" else "  ไม่พบเป้าสีที่เลือก")
            return
        for i, t in enumerate(self.plan, 1):
            print(f"  {i}. {t.color.upper():<6} yaw {t.yaw:+6.1f}°  pitch {t.pitch:+5.1f}°  -> {t.status}")
        fired = sum(t.status == "FIRED" for t in self.plan)
        print(f"  ยิงแล้ว {fired}/{len(self.plan)} เป้า\n")


# ==========================================
# 11. แอปหลัก
# ==========================================
class AutoAimApp:
    """ หน้าต่างเดียว 4 หน้าจอ: SETUP -> TUNE (= calibrate) , READY <-> MISSION """

    def __init__(self, calibrate: bool = False):
        self.color_ranges, user_zones, self.fire_order, found = load_config(CONFIG_PATH)
        self.blind_zones = BlindZones(AUTO_BLIND_ZONE, user_zones)
        # รันครั้งแรก (ยังไม่มีค่าที่ calibrate ไว้) ต้อง calibrate ก่อนเสมอ
        self.calibrate = calibrate or not found
        if not found:
            print("[CONFIG] ยังไม่มีค่าสีที่ calibrate ไว้ -> เข้าโหมด CALIBRATE อัตโนมัติ")
        elif not calibrate:
            print(f"[CONFIG] ใช้ค่าที่ calibrate ไว้ (calibrate ใหม่: กดปุ่ม CALIBRATE ALL "
                  f"หรือรัน python {Path(__file__).name} --calibrate)")

        self.screen = SCREEN_READY
        self.mission: Optional[Mission] = None
        self.ui = Ui()
        self.running = True
        self.tune_tab = 0              # หน้า TUNE: 0-3 = สีตาม COLOR_ORDER, 4 = ตรวจรวมทุกสี
        self._calibrating = False      # อยู่ในขั้น calibrate เต็ม (SETUP -> TUNE)
        self._order_fresh = True       # คลิกสีแรกหลังเข้าหน้า READY = เริ่มเรียงลำดับใหม่
        self._ready_since = 0.0
        self._last_result: List[str] = []

        self.pid_yaw = PIDController(**YAW_PID)
        self.pid_pitch = PIDController(**PITCH_PID)
        self.fire_ctrl = FireController()

        self.ep_robot = None
        self.ep_gimbal = None
        self.ep_camera = None
        self.ep_blaster = None
        self.leds: Optional[LedManager] = None
        self._stream_started = False

        self._fps = 0.0
        self._last_frame_time: Optional[float] = None
        self._fire_banner_until = 0.0

    @property
    def is_tuning(self) -> bool:
        """ อยู่ในหน้า calibrate/จูนสี = ห้ามยิงเด็ดขาด """
        return self.screen in (SCREEN_SETUP, SCREEN_TUNE)

    # ---------- วงจรชีวิตโปรแกรม ----------
    def run(self):
        try:
            if not self._connect():
                return
            if self.calibrate:
                self._enter_setup()                # มี recenter อยู่ในนี้
            else:
                self._enter_ready(recenter=True)   # recenter ก่อนเริ่ม Loop หลักเสมอ
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
        if self.is_tuning:
            print("[CONFIG] ออกระหว่าง calibrate/จูนสี: ค่าที่ยังไม่ได้กด SAVE จะไม่ถูกบันทึก")

        if self.ep_gimbal is not None:
            safe_call("gimbal stop", self.ep_gimbal.drive_speed, pitch_speed=0, yaw_speed=0)
            action = safe_call("gimbal recenter", self.ep_gimbal.recenter)
            if action is not None:
                safe_call("recenter wait", action.wait_for_completed, timeout=RECENTER_TIMEOUT_S)
        if self.leds is not None:
            self.leds.close()
        if self._stream_started:
            safe_call("stop video", self.ep_camera.stop_video_stream)
        if self.ep_robot is not None:
            safe_call("robot close", self.ep_robot.close)
        cv2.destroyAllWindows()
        print("=== ปิดระบบเรียบร้อย ===")

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
        # ตรวจซ้ำอีกชั้นก่อนลั่นไก: ยิงได้เฉพาะระหว่างภารกิจ ไม่อยู่ในหน้าจูนสี และระบบยิงเปิดอยู่
        if self.screen != SCREEN_MISSION or self.is_tuning or not self.fire_ctrl.armed:
            return False
        safe_call("gimbal stop", self.ep_gimbal.drive_speed, pitch_speed=0, yaw_speed=0)
        print(f"🔥 ยิงเป้า {self.mission.plan[self.mission.index].color.upper()}")
        safe_call("blaster fire", self.ep_blaster.fire, fire_type=blaster.WATER_FIRE, times=1)
        # ไม่สั่งไฟ Blaster กระพริบ; ดับไฟซ้ำเผื่อ firmware เปิดเองตอนยิง
        self.leds.silence_gimbal_lights()
        self.fire_ctrl.mark_fired()
        self.pid_yaw.reset()
        self.pid_pitch.reset()
        self._fire_banner_until = time.monotonic() + 0.3
        return True

    # ---------- เปลี่ยนหน้าจอ ----------
    def _enter_setup(self):
        self.screen = SCREEN_SETUP
        self._calibrating = True
        self.ui.zone_drawer = ZoneDrawer(self.blind_zones)
        self.ui.zone_drawer.frame_size = (VIEW_W, VIEW_H)
        self._recenter("เริ่ม calibrate")
        self._update_status_led()
        print(">> 🛠️ CALIBRATE 1/2: ลากเมาส์บนภาพกล้องเพื่อบังสิ่งที่ไม่ใช่เป้า แล้วกด NEXT")

    def _enter_tune(self, calibrating: bool):
        self.screen = SCREEN_TUNE
        self._calibrating = calibrating
        self.ui.zone_drawer = None
        self.tune_tab = 0
        self._stop_gimbal()
        self._update_status_led()
        print(">> 🎨 จูนสี: ลาก slider จนช่อง MASK PREVIEW เห็นเฉพาะเป้าสีนั้น แล้วกด NEXT / SAVE")

    def _finish_tuning(self):
        self._save()
        print(">> ✅ บันทึกค่าสีและพื้นที่ตาบอดแล้ว")
        self._enter_ready(recenter=True)

    def _cancel_calibration(self):
        self.color_ranges, zones, _, _ = load_config(CONFIG_PATH)
        self.blind_zones.user_zones = zones
        print(">> ยกเลิก (ใช้ค่าเดิมที่บันทึกไว้)")
        self._enter_ready(recenter=True)

    def _enter_ready(self, recenter: bool = False):
        """ กลับหน้า READY: ปลดระบบยิง หยุดหัว แล้วรอเลือกลำดับยิงรอบใหม่ """
        self.screen = SCREEN_READY
        self.mission = None
        self._calibrating = False
        self.ui.zone_drawer = None
        if self.fire_ctrl.armed:
            self.fire_ctrl.armed = False
            print(">> Auto-Fire: OFF (SAFE)")
        if recenter:
            self._recenter("กลับหน้า READY")
        else:
            self._stop_gimbal()
        self._order_fresh = True
        self._ready_since = time.monotonic()
        self._update_status_led()
        self._print_order()

    # ---------- ภารกิจ ----------
    def _start_mission(self, fire: bool):
        if not self.fire_order:
            print(">> ⚠️ ยังไม่ได้เลือกสีที่จะยิง")
            return
        if time.monotonic() - self._ready_since < START_KEY_GUARD_S:
            return  # ไม่รับ ENTER ที่กดซ้ำ/ค้างมาจากหน้าก่อน กันเริ่มยิงโดยไม่ตั้งใจ

        self._save()  # จำลำดับยิงไว้เป็นค่าเริ่มต้นของรอบหน้า
        self._stop_gimbal()
        self.fire_ctrl.armed = fire
        self.screen = SCREEN_MISSION
        self.mission = Mission(self, self.fire_order)
        print(f">> 🚀 {'เริ่มยิง' if fire else 'ซ้อมเล็ง (ไม่ยิง)'} ตามลำดับ {self._order_text()} "
              f"| กวาดหาเป้า {len(self.mission.poses)} มุม")
        self._update_status_led()

    def _mission_result(self, aborted: bool) -> List[str]:
        m = self.mission
        fired = sum(t.status == "FIRED" for t in m.plan)
        head = f"Last mission{' (STOPPED)' if aborted else ''}: fired {fired}/{len(m.plan)} target(s)"
        if not m.plan:
            head += " - stopped while scanning" if m.phase == "SCAN" else " - no target of the chosen colors"
        return [head] + m.progress_lines(per_line=8)

    def _abort_mission(self):
        self._last_result = self._mission_result(aborted=True)
        self.mission.print_summary(aborted=True)
        self._enter_ready()

    def _finish_mission(self):
        self._last_result = self._mission_result(aborted=False)
        self.mission.print_summary()
        self._enter_ready(recenter=True)

    def _toggle_auto_fire(self):
        self.fire_ctrl.armed = not self.fire_ctrl.armed
        self.fire_ctrl.reset_lock()  # ต้องนิ่งครบ SETTLE_TIME_S ใหม่หลังเปิดระบบยิง
        print(f">> Auto-Fire: {'ON ⚠️' if self.fire_ctrl.armed else 'OFF (SAFE) - ซ้อมเล็งต่อจนจบ'}")
        self._update_status_led()

    # ---------- ลำดับยิง ----------
    def _toggle_order(self, color: str):
        if self._order_fresh:
            self.fire_order = []  # คลิกสีแรกหลังเข้าหน้า READY = เริ่มเรียงลำดับใหม่
            self._order_fresh = False
        if color in self.fire_order:
            self.fire_order.remove(color)  # คลิกสีที่เลือกไว้แล้ว = เอาออก
        else:
            self.fire_order.append(color)
        self._print_order()

    def _undo_order(self):
        self._order_fresh = False
        if self.fire_order:
            self.fire_order.pop()
        self._print_order()

    def _clear_order(self):
        self._order_fresh = False
        self.fire_order = []
        self._print_order()

    def _order_text(self, sep: str = " -> ") -> str:
        return sep.join(c.upper() for c in self.fire_order)

    def _print_order(self):
        print(f">> ลำดับยิง: {self._order_text() or '(ยังไม่ได้เลือก)'}")

    # ---------- ปุ่ม (เมาส์ + คีย์บอร์ด ใช้ action เดียวกัน) ----------
    def _key_to_action(self, key: int) -> Optional[str]:
        ch = key_char(key)
        if ch == "q" or key == 27:
            return "quit"
        enter = key in (13, 10)
        if self.screen == SCREEN_READY:
            if ch in KEY_TO_COLOR:
                return "order:" + KEY_TO_COLOR[ch]
            if ch == "u" or key in (8, 127):  # Backspace บน macOS = 127
                return "undo"
            if enter:
                return "start"
            return {"x": "clear", "d": "dry", "t": "tune", "c": "center"}.get(ch)
        if self.screen == SCREEN_MISSION:
            if ch == "x" or key == 32:
                return "stop"
            return {"s": "fire_toggle", "c": "stop_center"}.get(ch)
        if self.screen == SCREEN_SETUP:
            if enter or key == 32:
                return "next"
            return {"u": "zone_undo", "x": "zone_clear"}.get(ch)
        if self.screen == SCREEN_TUNE:
            if ch in KEY_TO_COLOR:
                return "tab:" + KEY_TO_COLOR[ch]
            if ch in NUDGE_KEYS:
                return "nudge:" + ch
            if enter or key == 32 or ch == "n":
                return "next"
            return {"v": "tab:check", "p": "back", "t": "done", "c": "center"}.get(ch)
        return None

    def _do(self, action: str):
        if action == "quit":
            self.running = False
        elif self.screen == SCREEN_READY:
            if action.startswith("order:"):
                self._toggle_order(action.split(":", 1)[1])
            elif action == "undo":
                self._undo_order()
            elif action == "clear":
                self._clear_order()
            elif action == "start":
                self._start_mission(fire=True)
            elif action == "dry":
                self._start_mission(fire=False)
            elif action == "tune":
                self._enter_tune(calibrating=False)
            elif action == "calibrate":
                self._enter_setup()
            elif action == "center":
                self._recenter("กดปุ่ม RECENTER")
        elif self.screen == SCREEN_MISSION:
            if action == "stop":
                self._abort_mission()
            elif action == "fire_toggle":
                self._toggle_auto_fire()
            elif action == "stop_center":
                self._abort_mission()
                self._recenter("กดปุ่ม STOP + RECENTER")
        elif self.screen == SCREEN_SETUP:
            if action == "zone_undo" and self.blind_zones.user_zones:
                self.blind_zones.user_zones.pop()
            elif action == "zone_clear":
                self.blind_zones.user_zones.clear()
            elif action == "next":
                self._enter_tune(calibrating=True)
            elif action == "cancel":
                self._cancel_calibration()
        elif self.screen == SCREEN_TUNE:
            if action.startswith("tab:"):
                tab = action.split(":", 1)[1]
                self.tune_tab = len(COLOR_ORDER) if tab == "check" else COLOR_ORDER.index(tab)
            elif action == "next":
                if self.tune_tab >= len(COLOR_ORDER):
                    self._finish_tuning()
                else:
                    self.tune_tab += 1
            elif action == "back":
                self.tune_tab = max(0, self.tune_tab - 1)
            elif action == "done":
                self._finish_tuning()
            elif action == "cancel":
                self._cancel_calibration()
            elif action == "reset_color" and self.tune_tab < len(COLOR_ORDER):
                color = COLOR_ORDER[self.tune_tab]
                self.color_ranges[color] = HSVRange(*DEFAULT_HSV[color])
                print(f">> คืนค่าเริ่มต้นสี {color.upper()} [{self.color_ranges[color]}] (กด SAVE เพื่อบันทึก)")
            elif action.startswith("nudge:"):
                pitch, yaw = NUDGE_KEYS[action.split(":", 1)[1]]
                safe_call("gimbal move", self.ep_gimbal.move, pitch=pitch, yaw=yaw, pitch_speed=90, yaw_speed=90)
            elif action == "center":
                self._recenter("กดปุ่ม CENTER")

    # ---------- Loop หลัก ----------
    def _main_loop(self):
        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_AUTOSIZE)
        cv2.setMouseCallback(WINDOW_NAME, self.ui.on_mouse)
        while self.running:
            frame = read_frame(self.ep_camera)
            if frame is None:
                # ไม่มีภาพ = ห้ามขยับตามข้อมูลเก่า (แต่ไม่ขัดจังหวะ moveto ที่กำลังหมุนอยู่)
                if self.mission is not None and self.mission.sub == "AIM":
                    self._stop_gimbal()
            else:
                self._process_frame(frame)
            action = self._key_to_action(cv2.waitKey(1) & 0xFF)
            if action is not None:
                self.ui.actions.append(action)
            while self.ui.actions and self.running:
                self._do(self.ui.actions.popleft())

    def _process_frame(self, frame: np.ndarray):
        now = time.monotonic()
        if self._last_frame_time is not None and now > self._last_frame_time:
            fps = 1.0 / (now - self._last_frame_time)
            self._fps = fps if self._fps == 0 else 0.9 * self._fps + 0.1 * fps
        self._last_frame_time = now

        fd = prepare_frame(frame)
        targets: List[Target] = []
        match, error, mask = None, None, None
        if self.screen == SCREEN_MISSION:
            self.mission.step(fd)
            state, targets = self.mission.state, self.mission.visible
            match, error = self.mission.match, self.mission.error
            if self.mission.done:
                self._finish_mission()
        elif self.screen == SCREEN_SETUP:
            state = f"{len(self.blind_zones.user_zones)} blind zone(s) drawn"
        elif self.screen == SCREEN_TUNE and self.tune_tab < len(COLOR_ORDER):
            color = COLOR_ORDER[self.tune_tab]
            mask = build_mask(fd.hsv, self.color_ranges[color], self.blind_zones)
            targets = find_targets(fd, mask, color)
            state = f"Tuning {color.upper()}: {len(targets)} target(s) found"
        else:  # READY และหน้าตรวจรวมทุกสี
            targets = detect_all_colors(fd, self.color_ranges, self.blind_zones)
            state = f"{len(targets)} target(s) in view"

        self._render(fd, targets, match, error, state, mask)

    # ---------- วาดหน้าจอ ----------
    def _render(self, fd: FrameData, targets, match, error, state, mask):
        canvas = np.full((CANVAS_H, CANVAS_W, 3), UI_BG, np.uint8)
        w, h = fd.size
        if (w, h) == (VIEW_W, VIEW_H):  # สตรีม 540p = ขนาดช่องภาพพอดี ไม่ต้องย่อ
            canvas[:VIEW_H, :VIEW_W] = fd.bgr
        else:
            canvas[:VIEW_H, :VIEW_W] = cv2.resize(fd.bgr, (VIEW_W, VIEW_H), interpolation=cv2.INTER_AREA)
        self._draw_view(canvas[:VIEW_H, :VIEW_W], VIEW_W / w, targets, match)
        cv2.rectangle(canvas, (VIEW_W, 0), (CANVAS_W, CANVAS_H), UI_PANEL, -1)
        self.ui.begin()
        self._draw_panel(canvas, state, error)
        self._draw_strip(canvas, fd.bgr, mask, targets)
        cv2.imshow(WINDOW_NAME, canvas)

    def _draw_view(self, view, scale, targets, match):
        """ ภาพกล้อง: พื้นที่ตาบอด, จุดเล็ง, เป้าที่ตรวจเจอ """
        if self.screen == SCREEN_SETUP:
            self.blind_zones.draw(view, fill_alpha=0.55)
            d = self.ui.zone_drawer
            if d is not None and d.drag_start is not None and d.drag_end is not None:
                cv2.rectangle(view, d.drag_start, d.drag_end, (0, 255, 255), 2)
            put_text(view, "Drag on this image to hide anything that is NOT a target", (12, 28), (0, 255, 255), 0.6)
            return

        self.blind_zones.draw(view)
        ax, ay = degrees_to_pixel(AIM_OFFSET_X_DEG, AIM_OFFSET_Y_DEG, VIEW_W, VIEW_H)
        tol_x = max(3, int(LOCK_TOLERANCE_X_DEG * VIEW_W / FOV_X))
        tol_y = max(3, int(LOCK_TOLERANCE_Y_DEG * VIEW_H / FOV_Y))
        cv2.rectangle(view, (ax - tol_x, ay - tol_y), (ax + tol_x, ay + tol_y), (200, 200, 200), 1)
        cv2.drawMarker(view, (ax, ay), (255, 255, 255), markerType=cv2.MARKER_CROSS, markerSize=22, thickness=2)

        draw_targets(view, targets, scale)
        if match is not None:
            bgr = DISPLAY_COLORS[match.color]
            mx, my = int(match.cx * scale), int(match.cy * scale)
            cv2.circle(view, (mx, my), 12, bgr, 2)
            cv2.line(view, (ax, ay), (mx, my), bgr, 2)
        if time.monotonic() < self._fire_banner_until:
            put_text(view, "FIRE!", (ax - 40, ay - tol_y - 12), (0, 0, 255), 1.0, 3)

    def _screen_badge(self) -> Tuple[str, Tuple[int, int, int]]:
        if self.screen == SCREEN_SETUP:
            return "CALIBRATE 1/2: BLIND ZONES", UI_ORANGE
        if self.screen == SCREEN_TUNE:
            return ("CALIBRATE 2/2: COLORS" if self._calibrating else "TUNE COLORS"), UI_ORANGE
        if self.screen == SCREEN_MISSION:
            return ("MISSION - FIRING", UI_RED) if self.fire_ctrl.armed else ("MISSION - DRY RUN", UI_BLUE)
        return "READY", UI_GREEN

    def _draw_panel(self, canvas, state: str, error):
        """ แผงขวา: สถานะ + ปุ่มของหน้าจอปัจจุบัน """
        px, bw = VIEW_W + 16, PANEL_W - 32
        label(canvas, "ROBOMASTER AUTO-AIM", px, 28, UI_TEXT, 0.6, 2)
        badge, badge_color = self._screen_badge()
        cv2.rectangle(canvas, (px, 40), (px + bw, 70), badge_color, -1)
        label(canvas, badge, px + 10, 61, text_color_for(badge_color), 0.55, 1, max_w=bw - 20)

        fire_text, fire_color = ("FIRE: ARMED", (90, 90, 255)) if self.fire_ctrl.armed else ("FIRE: SAFE", UI_OK)
        label(canvas, f"FPS {self._fps:.0f}", px, 94, UI_DIM)
        label(canvas, fire_text, px + 100, 94, fire_color, 0.5, 2)
        for i, line in enumerate(textwrap.wrap(state, 36)[:2]):
            label(canvas, line, px, 116 + i * 18, UI_TEXT, 0.45)
        if error is not None:
            label(canvas, f"Error  X {error[0]:+.1f}  Y {error[1]:+.1f} deg", px, 152, UI_DIM, 0.45)

        panels = {SCREEN_READY: self._panel_ready, SCREEN_MISSION: self._panel_mission,
                  SCREEN_SETUP: self._panel_setup, SCREEN_TUNE: self._panel_tune}
        panels[self.screen](canvas, px, 172, bw)
        self.ui.button(canvas, px, CANVAS_H - 50, bw, 36, "QUIT (q)", "quit", (70, 60, 110))

    def _color_chips(self, canvas, px, y, bw, action_prefix, is_selected, caption_for):
        chip_w = (bw - 3 * 8) // 4
        for i, color in enumerate(COLOR_ORDER):
            selected = is_selected(color)
            self.ui.button(canvas, px + i * (chip_w + 8), y, chip_w, 42, caption_for(color),
                           action_prefix + color, DISPLAY_COLORS[color] if selected else dim(DISPLAY_COLORS[color]),
                           selected=selected)

    def _panel_ready(self, canvas, px, y, bw):
        half = (bw - 8) // 2
        label(canvas, "FIRE ORDER - click colors in order", px, y, UI_DIM, 0.45, max_w=bw)

        def caption(color):
            short = COLOR_SHORT[color]
            return f"{self.fire_order.index(color) + 1}:{short}" if color in self.fire_order else short

        self._color_chips(canvas, px, y + 10, bw, "order:", lambda c: c in self.fire_order, caption)
        y += 72
        label(canvas, f"Order: {self._order_text(' > ') or '(none)'}", px, y, (0, 255, 255), 0.5, 1, max_w=bw)
        y += 12
        self.ui.button(canvas, px, y, half, 34, "UNDO (u)", "undo")
        self.ui.button(canvas, px + half + 8, y, half, 34, "CLEAR (x)", "clear")
        y += 48
        can_start = bool(self.fire_order)
        self.ui.button(canvas, px, y, bw, 58, "START FIRING (ENTER)", "start", UI_RED, enabled=can_start)
        y += 68
        self.ui.button(canvas, px, y, bw, 38, "DRY RUN - AIM ONLY (d)", "dry", UI_BLUE, enabled=can_start)
        y += 56
        self.ui.button(canvas, px, y, half, 38, "TUNE COLORS (t)", "tune")
        self.ui.button(canvas, px + half + 8, y, half, 38, "CALIBRATE ALL", "calibrate")
        y += 48
        self.ui.button(canvas, px, y, bw, 36, "RECENTER GIMBAL (c)", "center")

    def _panel_mission(self, canvas, px, y, bw):
        self.ui.button(canvas, px, y, bw, 70, "STOP (x / SPACE)", "stop", UI_RED)
        y += 84
        if self.fire_ctrl.armed:
            self.ui.button(canvas, px, y, bw, 44, "FIRE ON - click for SAFE (s)", "fire_toggle", (40, 40, 150))
        else:
            self.ui.button(canvas, px, y, bw, 44, "SAFE - click to ARM FIRE (s)", "fire_toggle", UI_GREEN)
        y += 56
        self.ui.button(canvas, px, y, bw, 38, "STOP + RECENTER (c)", "stop_center")
        y += 60
        label(canvas, f"Order: {self._order_text(' > ')}", px, y, (0, 255, 255), 0.5, 1, max_w=bw)

    def _panel_setup(self, canvas, px, y, bw):
        half = (bw - 8) // 2
        for line in ("Drag on the camera image to", "hide things that are NOT targets.",
                     "Orange box = robot's own gimbal", "(always hidden)."):
            label(canvas, line, px, y, UI_TEXT, 0.48)
            y += 20
        y += 6
        label(canvas, f"Blind zones drawn: {len(self.blind_zones.user_zones)}", px, y, (0, 255, 255))
        y += 14
        self.ui.button(canvas, px, y, half, 34, "UNDO ZONE (u)", "zone_undo")
        self.ui.button(canvas, px + half + 8, y, half, 34, "CLEAR (x)", "zone_clear")
        y += 50
        self.ui.button(canvas, px, y, bw, 56, "NEXT: TUNE COLORS (ENTER)", "next", UI_GREEN)
        y += 70
        self.ui.button(canvas, px, y, bw, 34, "CANCEL (keep saved values)", "cancel")

    def _panel_tune(self, canvas, px, y, bw):
        half = (bw - 8) // 2
        third = (bw - 16) // 3
        label(canvas, "COLOR TO TUNE (r/g/b/y)", px, y, UI_DIM, 0.45)
        tab = self.tune_tab
        self._color_chips(canvas, px, y + 10, bw, "tab:",
                          lambda c: tab < len(COLOR_ORDER) and COLOR_ORDER[tab] == c, lambda c: COLOR_SHORT[c])
        y += 60
        self.ui.button(canvas, px, y, bw, 32, "CHECK ALL COLORS (v)", "tab:check",
                       selected=tab == len(COLOR_ORDER))
        y += 50
        label(canvas, "MOVE GIMBAL (target off-screen?)", px, y, UI_DIM, 0.45, max_w=bw)
        y += 10
        mid = px + third + 8
        self.ui.button(canvas, mid, y, third, 30, "UP (i)", "nudge:i")
        y += 36
        self.ui.button(canvas, px, y, third, 30, "LEFT (j)", "nudge:j")
        self.ui.button(canvas, mid, y, third, 30, "CENTER (c)", "center")
        self.ui.button(canvas, px + 2 * (third + 8), y, third, 30, "RIGHT (l)", "nudge:l")
        y += 36
        self.ui.button(canvas, mid, y, third, 30, "DOWN (k)", "nudge:k")
        y += 46
        last = tab >= len(COLOR_ORDER)
        self.ui.button(canvas, px, y, half, 34, "< BACK (p)", "back", enabled=tab > 0)
        self.ui.button(canvas, px + half + 8, y, half, 34, "FINISH (n)" if last else "NEXT > (n)", "next")
        y += 46
        self.ui.button(canvas, px, y, bw, 44, "SAVE & FINISH (t)", "done", UI_GREEN)
        y += 54
        self.ui.button(canvas, px, y, bw, 32, "CANCEL (keep saved values)", "cancel")
        y += 42
        self.ui.button(canvas, px, y, bw, 32, "RESET THIS COLOR TO DEFAULT", "reset_color",
                       enabled=tab < len(COLOR_ORDER))

    def _draw_strip(self, canvas, frame, mask, targets: List[Target]):
        """ แถบล่าง: slider จูนสี + ภาพ Mask หรือคำแนะนำ/ผลภารกิจ """
        y0 = VIEW_H
        cv2.line(canvas, (0, y0), (VIEW_W, y0), (80, 80, 80), 1)
        if self.screen == SCREEN_TUNE and self.tune_tab < len(COLOR_ORDER):
            self._strip_sliders(canvas, frame, mask, COLOR_ORDER[self.tune_tab])
            return

        counts = " | ".join(f"{c.upper()} {sum(t.color == c for t in targets)}" for c in COLOR_ORDER)
        if self.screen == SCREEN_SETUP:
            lines = [("STEP 1/2 - BLIND ZONES", UI_ORANGE),
                     ("Mouse: press, drag and release on the camera image = add one blind zone", UI_TEXT),
                     ("The robot will never see or shoot inside the dark boxes.", UI_TEXT)]
            keys = "u = undo zone   x = clear zones   ENTER = next step   q = quit"
        elif self.screen == SCREEN_TUNE:
            lines = [("CHECK ALL COLORS", UI_ORANGE), (f"In view: {counts}", (0, 255, 255)),
                     ("Every target should show exactly ONE correct label.", UI_TEXT),
                     ("Wrong or double label? Click that color above and tune it again.", UI_TEXT)]
            keys = "r/g/b/y = tune color   p = back   t = save & finish   q = quit"
        elif self.screen == SCREEN_MISSION:
            m = self.mission
            lines = [(m.state, UI_TEXT)] + [(text, (0, 255, 255)) for text in m.progress_lines(per_line=8)]
            keys = "x / SPACE = stop now   s = fire on/off   c = stop + recenter   q = quit"
        else:
            lines = [(f"In view: {counts}", (0, 255, 255)),
                     ("Click color chips (or r/g/b/y) in firing order, then START FIRING.", UI_TEXT),
                     ("The robot scans around first, then fires at every target of those colors.", UI_TEXT)]
            lines += [(text, UI_DIM) for text in self._last_result]
            keys = "r/g/b/y = order  u = undo  x = clear  ENTER = fire  d = dry run  t = tune  c = center  q = quit"

        for i, (text, color) in enumerate(lines[:6]):
            label(canvas, text, 16, y0 + 26 + i * 22, color, 0.5, 1, max_w=VIEW_W - 32)
        label(canvas, keys, 16, CANVAS_H - 12, UI_DIM, 0.45, 1, max_w=VIEW_W - 32)

    def _strip_sliders(self, canvas, frame, mask, color: str):
        y0 = VIEW_H
        rng = self.color_ranges[color]
        sliders = (("H min", "h_min", 179, True), ("H max", "h_max", 179, True),
                   ("S min", "s_min", 255, False), ("S max", "s_max", 255, False),
                   ("V min", "v_min", 255, False), ("V max", "v_max", 255, False))
        for i, (text, name, max_value, hue) in enumerate(sliders):
            x = 20 + (i % 2) * 310
            y = y0 + 12 + (i // 2) * 50
            self.ui.slider(canvas, x, y, 290, text, getattr(rng, name), max_value,
                           self._slider_setter(color, name), hue)
        hint = "H min > H max = hue wraps around (used for RED)" if rng.wraps else \
            "S min up = ignore pale colors   V min up = ignore shadows"
        label(canvas, hint, 20, CANVAS_H - 12, UI_DIM, 0.45)

        # ภาพเฉพาะพิกเซลที่ผ่านช่วงสี (สีจริง) ด้านขวาของแถบล่าง
        preview = cv2.bitwise_and(frame, frame, mask=mask)
        canvas[y0:CANVAS_H, VIEW_W - 320:VIEW_W] = cv2.resize(preview, (320, STRIP_H), interpolation=cv2.INTER_AREA)
        cv2.rectangle(canvas, (VIEW_W - 320, y0), (VIEW_W - 1, CANVAS_H - 1), (90, 90, 90), 1)
        put_text(canvas, f"MASK PREVIEW: {color.upper()}", (VIEW_W - 312, y0 + 20), (255, 255, 255), 0.5, 1)

    def _slider_setter(self, color: str, name: str) -> Callable[[int], None]:
        def setter(value: int):
            self.color_ranges[color] = replace(self.color_ranges[color], **{name: value})
        return setter

    # ---------- อื่นๆ ----------
    def _update_status_led(self):
        self.leds.show_status("white", self.fire_ctrl.armed, self.is_tuning)

    def _save(self):
        save_config(CONFIG_PATH, self.color_ranges, self.blind_zones.user_zones, self.fire_order)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="RoboMaster EP ยิงเป้าตามลำดับสีอัตโนมัติ")
    parser.add_argument("--calibrate", action="store_true",
                        help="เริ่มที่หน้า calibrate (วาดพื้นที่ตาบอด + จูนสีครบทุกสี) รันครั้งแรกจะทำให้อัตโนมัติ")
    AutoAimApp(calibrate=parser.parse_args().calibrate).run()
