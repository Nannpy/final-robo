"""
RoboMaster EP - สำรวจเขาวงกต (รอบ 1) แล้ววิ่งไปยิงเป้าสี (รอบ 2)

รวมสองโปรแกรมเดิมโดยไม่แก้ไฟล์เดิม:
  - การเดินทีละช่อง + แผนที่ + A*  : แนวเดียวกับ assignment2.py / path.py (IMU คุมหัวตรง, ToF เป็นกันชน)
  - จับป้ายสี + เล็ง + ยิง + ไฟ LED : import จาก gimbalshoot.py (ใช้ค่าสีที่ calibrate ไว้ใน autoaim_config.json)

ฮาร์ดแวร์ที่ใช้: ToF 1 ตัวติดบนหัว gimbal (หันตามหัว) + กล้อง (ไม่มี Sharp IR)
  - กำแพงบาง/บล็อกทึบ: หันหัวไปทิศนั้นแล้ววัด ToF (ใกล้กว่า WALL_TOF_MM = มีกำแพงกั้น)
  - เป้า (แผ่นอะคริลิคสีบนขาตั้ง กลางช่อง สูง ~6-8 ซม.): ToF มองข้ามเพราะเตี้ย จึงก้มกล้องหา
    แล้วคำนวณจากมุมก้มว่าเป้าอยู่ช่องติดกันจริงไหม -> ช่องเป้าห้ามเดินเข้า ยิงจากช่องข้างๆ

รอบ 1 (EXPLORE): เดินให้ครบทุกช่องที่ไปถึงได้ ทุกช่องหันหัววัดกำแพงและหาเป้าทุกทิศ -> ได้แผนที่ + ตำแหน่งเป้า
รอบ 2 (SHOOT)  : เริ่มจาก start (หุ่นวิ่งกลับเอง) หรือจากจุดที่อยู่ -> ไปทีละเป้า ยิง -> จบตามที่เลือก
                 ลำดับเป้า: ทางสั้นสุดครบทุกเป้า (ค่าเริ่มต้น) หรือเรียงตามสีที่เลือกเอง

วิธีรัน (calibrate สีด้วย gimbalshoot.py --calibrate ก่อน ถ้ายังไม่เคย)
  python maze_shooter.py          # ต่อหุ่นจริง
  python maze_shooter.py --sim    # จำลองบนคอม ไม่ต่อหุ่น (สนาม 4x4 ตัวอย่าง)

ปุ่ม (คลิกในหน้าต่าง หรือกดคีย์ในวงเล็บ)
  e  สำรวจ (รอบ 1)          ENTER  ยิง (รอบ 2)         d  ซ้อมรอบ 2 ไม่ยิง
  1  จุดเริ่มรอบ 2           2  จบรอบ 2 ที่ไหน           3  ลำดับยิง (ทางสั้นสุด / ตามสี)
  r/g/b/y  เรียงลำดับสี     u  ลบสีตัวท้าย             คลิกช่องบนแผนที่ = ตั้ง GOAL
  x / SPACE  หยุดทันที      c  หันหัวกลับตรง           q / ESC  ออก
"""
import argparse
import heapq
import itertools
import json
import math
import threading
import time
import traceback
from collections import Counter, deque
from pathlib import Path
from typing import Dict, List, Optional, Set, Tuple

import cv2
import numpy as np
# pyrefly: ignore [missing-import]
from robomaster import blaster, robot

import gimbalshoot as gs  # จับสี/เล็ง/ยิง/ไฟ/ปุ่ม GUI จากโปรแกรมยิงเดิม (ไม่ได้แก้ไฟล์นั้น)

Cell = Tuple[int, int]

# ==========================================
# 1. การตั้งค่า
# ==========================================
# --- สนาม ---
CELL_M = 0.60                   # ขนาดช่อง (ม.)
DIRS = [(0, 1), (1, 0), (0, -1), (-1, 0)]          # 0=N, 1=E, 2=S, 3=W (หมุนตามเข็ม = +1)
DIR_NAMES = ["N", "E", "S", "W"]
YAW_TARGETS = [0.0, 90.0, 180.0, -90.0]            # มุม IMU ของแต่ละทิศ (แบบ assignment2.py)

# --- การเดิน (ค่าจาก assignment2.py / path.py) ---
BASE_SPEED = 0.25               # m/s
TURN_SPEED = 50                 # deg/s
KP_YAW, KD_YAW = 1.4, 0.08      # คุมหัวรถให้ตรงด้วย IMU ระหว่างเดิน
BUMPER_MM = 150                 # ToF ใกล้กว่านี้ระหว่างเดิน = เบรกฉุกเฉิน
MIN_TRAVEL_FRACTION = 0.8       # เบรกก่อนเดินได้เท่านี้ = ไม่นับว่าถึงช่องถัดไป (ตำแหน่งในแผนที่ไม่เพี้ยน)
MOVE_TIMEOUT_S = 6.0

# --- ตรวจกำแพงด้วย ToF บนหัว ---
WALL_TOF_MM = 400               # ToF ใกล้กว่านี้ = มีกำแพง/บล็อกกั้นระหว่างช่อง (ผนังขอบช่องห่าง ~250 มม.)
TOF_MIN_VALID_MM = 20           # ค่า 0 หรือต่ำมาก = อ่านพลาด ไม่นับ
TOF_SAMPLE_S = 0.35             # เก็บค่า ToF นานเท่านี้แล้วใช้ค่ามัธยฐาน
GIMBAL_SETTLE_S = 0.35          # รอหัวนิ่ง/ภาพตามทันหลังหันหัว
GIMBAL_SPEED = 180              # deg/s ตอนหันหัวไปแต่ละทิศ

# --- หาเป้าด้วยกล้อง (วัดค่าจริงของหุ่นแล้วแก้ 2 ค่านี้) ---
CAMERA_HEIGHT_M = 0.22          # ความสูงเลนส์กล้องจากพื้น
TARGET_HEIGHT_M = 0.05          # ความสูงจุดกลางแผ่นเป้าจากพื้น
CAMERA_FORWARD_M = 0.10         # กล้องอยู่หน้าจุดกลางหุ่นเท่านี้ (ตอนหัวหันทิศไหนก็ตาม)
TARGET_SCAN_PITCH = -15.0       # ก้มกล้องเท่านี้ตอนหาเป้า (เป้าเตี้ย อยู่ต่ำกว่าระดับกล้อง)
TARGET_SCAN_FRAMES = 5
TARGET_MIN_HITS = 3             # ต้องเห็นกี่เฟรมจากทั้งหมดถึงนับว่ามีเป้าจริง
TARGET_CELL_RANGE = (0.35, 0.90)  # ระยะจากกลางช่องหุ่นถึงเป้า (ม.) ที่นับว่าเป้าอยู่ "ช่องติดกัน"
TARGET_MAX_LATERAL_M = 0.25     # เป้าต้องเยื้องซ้าย/ขวาจากแนวกลางไม่เกินนี้

# --- ยิง (เกณฑ์ล็อก/PID ใช้ของ gimbalshoot.py) ---
PITCH_MIN_DEG = gs.PITCH_LIMITS[0]  # หัวก้มได้สุด -20°
PITCH_MARGIN_DEG = 2.0
MAX_BACKOFF_M = 0.12            # ถ้าเป้าต่ำเกินกว่าหัวจะก้มถึง ถอยรถได้สูงสุดเท่านี้ (ไม่ชนผนังหลังช่อง)
AIM_TIMEOUT_S = 12.0            # เล็งเป้าเดียวนานเกินนี้ -> ข้าม
SHOTS_PER_TARGET = 1
DRY_HOLD_S = 0.8                # ซ้อมรอบ 2: ค้างเล็งให้ดูกี่วินาที

# --- ตัวเลือกรอบ 2 ---
START_MODES = ("FROM START", "FROM HERE")
END_MODES = ("STOP", "BACK TO START", "GO TO GOAL")
ORDER_MODES = ("SHORTEST", "COLOR ORDER")
MAX_PERMUTE_TARGETS = 7         # เป้าไม่เกินนี้ = ลองทุกลำดับหาทางสั้นสุดจริง, มากกว่านี้ = เลือกใกล้สุดก่อน

MAP_FILE = Path(__file__).resolve().with_name("maze_map.json")
WINDOW = "RoboMaster Maze Shooter"


class Aborted(Exception):
    """ ผู้ใช้กด STOP / QUIT ระหว่างที่หุ่นทำงาน """


# ==========================================
# 2. แผนที่เขาวงกต
# ==========================================
class MazeMap:
    """ แผนที่ W x H ช่อง
    edges  : กำแพงระหว่างช่องที่ติดกัน  -1 = กั้น (กำแพงบางหรือบล็อกทึบ), 1 = เปิด, ไม่มีคีย์ = ยังไม่รู้
    targets: ช่องที่มีเป้า -> สี (เดินเข้าไม่ได้ ยิงจากช่องข้างๆ ที่ไม่มีกำแพงกั้น)
    ช่องที่สำรวจครบแล้วยังไปไม่ถึง = บล็อกทึบ/ห้องปิด
    """

    def __init__(self, width: int, height: int):
        self.w, self.h = width, height
        self.edges: Dict[Tuple[Cell, Cell], int] = {}
        self.visited: Set[Cell] = set()
        self.targets: Dict[Cell, str] = {}
        self.explored = False
        for x in range(width):  # ขอบนอกสนามเป็นกำแพงเสมอ
            self.set_edge((x, 0), (x, -1), -1)
            self.set_edge((x, height - 1), (x, height), -1)
        for y in range(height):
            self.set_edge((0, y), (-1, y), -1)
            self.set_edge((width - 1, y), (width, y), -1)

    @staticmethod
    def _key(a: Cell, b: Cell) -> Tuple[Cell, Cell]:
        return (a, b) if a <= b else (b, a)

    def inside(self, c: Cell) -> bool:
        return 0 <= c[0] < self.w and 0 <= c[1] < self.h

    def edge(self, a: Cell, b: Cell) -> int:
        return self.edges.get(self._key(a, b), 0)

    def set_edge(self, a: Cell, b: Cell, status: int):
        self.edges[self._key(a, b)] = status

    @staticmethod
    def step(c: Cell, d: int) -> Cell:
        return c[0] + DIRS[d][0], c[1] + DIRS[d][1]

    def can_drive(self, a: Cell, b: Cell) -> bool:
        return self.inside(b) and self.edge(a, b) == 1 and b not in self.targets

    def unknown_dirs(self, c: Cell) -> List[int]:
        """ ทิศที่ยังไม่รู้ว่ามีกำแพงไหม (ขอบสนามรู้อยู่แล้ว) """
        return [d for d in range(4) if self.edge(c, self.step(c, d)) == 0]

    def shooting_spots(self, target: Cell) -> List[Tuple[Cell, int]]:
        """ ช่องที่ยิงเป้านี้ได้: ติดกัน, ไม่มีกำแพงกั้น, เคยไปถึงแล้ว -> (ช่อง, ทิศที่ต้องหัน) """
        spots = []
        for d in range(4):
            s = self.step(target, d)
            if s in self.visited and self.edge(s, target) == 1:
                spots.append((s, (d + 2) % 4))
        return spots

    # ---------- วางเส้นทาง (A* + ค่าปรับการเลี้ยว แบบ assignment2.py) ----------
    def plan(self, start: Cell, heading: int, goal_test, heuristic_to: Optional[Cell] = None
             ) -> Optional[List[Cell]]:
        pq = [(0.0, 0.0, start, heading, [start])]
        seen = set()
        while pq:
            _, cost, cell, head, path = heapq.heappop(pq)
            if cell != start and goal_test(cell):
                return path
            if (cell, head) in seen:
                continue
            seen.add((cell, head))
            for d in range(4):
                nxt = self.step(cell, d)
                if not self.can_drive(cell, nxt):
                    continue
                turn = (d - head) % 4
                new_cost = cost + 1.0 + (0.0 if turn == 0 else 1.5 if turn == 2 else 0.8)
                h = abs(heuristic_to[0] - nxt[0]) + abs(heuristic_to[1] - nxt[1]) if heuristic_to else 0
                heapq.heappush(pq, (new_cost + h, new_cost, nxt, d, path + [nxt]))
        return None

    def path_to(self, start: Cell, heading: int, goal: Cell) -> Optional[List[Cell]]:
        if start == goal:
            return [start]
        return self.plan(start, heading, lambda c: c == goal, goal)

    def path_to_unvisited(self, start: Cell, heading: int) -> Optional[List[Cell]]:
        return self.plan(start, heading, lambda c: c not in self.visited)

    def distances_from(self, start: Cell) -> Dict[Cell, int]:
        """ จำนวนก้าวจาก start ไปทุกช่องที่ไปถึงได้ (ใช้จัดลำดับเป้า) """
        dist = {start: 0}
        queue = deque([start])
        while queue:
            c = queue.popleft()
            for d in range(4):
                n = self.step(c, d)
                if n not in dist and self.can_drive(c, n):
                    dist[n] = dist[c] + 1
                    queue.append(n)
        return dist

    # ---------- บันทึก ----------
    def to_json(self) -> dict:
        return {
            "width": self.w, "height": self.h, "explored": self.explored,
            "visited": sorted(self.visited),
            "walls": [[list(a), list(b)] for (a, b), s in sorted(self.edges.items()) if s == -1],
            "open": [[list(a), list(b)] for (a, b), s in sorted(self.edges.items()) if s == 1],
            "targets": [{"cell": list(c), "color": col} for c, col in sorted(self.targets.items())],
        }


def normalize_angle(angle: float) -> float:
    while angle > 180.0:
        angle -= 360.0
    while angle <= -180.0:
        angle += 360.0
    return angle


# ==========================================
# 3. ฮาร์ดแวร์หุ่น (ของจริง)
# ==========================================
class RealRobot:
    """ คำสั่ง SDK ทั้งหมดที่โปรแกรมนี้ใช้: ToF บนหัว, IMU, กล้อง, gimbal, ล้อ, blaster, ไฟ """
    sim = False

    def __init__(self):
        self.tof_mm = 0
        self._tof_log: deque = deque(maxlen=60)  # (เวลา, มม.)
        self.yaw = 0.0
        self.ep = robot.Robot()
        print("[INFO] Connecting to RoboMaster (AP)...")
        if not self.ep.initialize(conn_type=gs.CONN_TYPE):
            raise ConnectionError("Cannot connect to the robot - check Wi-Fi")
        self.chassis, self.gimbal = self.ep.chassis, self.ep.gimbal
        self.camera, self.blaster = self.ep.camera, self.ep.blaster
        # FREE = หันหัวแยกจากตัวรถได้ (ต้องใช้ตอนหันหัววัดกำแพงซ้าย/ขวา)
        gs.safe_call("set robot mode", self.ep.set_robot_mode, mode=robot.FREE)
        self.leds = gs.LedManager(self.ep.led, self.blaster)  # ไฟ Top/Blaster ดับตลอด (จาก gimbalshoot)
        self.leds.silence_gimbal_lights()
        self.leds.show_status("white", False, False)
        self.ep.sensor.sub_distance(freq=20, callback=self._on_tof)
        self.chassis.sub_attitude(freq=20, callback=self._on_attitude)
        self.camera.start_video_stream(display=False, resolution=gs.STREAM_RESOLUTION)
        self.gimbal_to(0, 0)
        time.sleep(1.0)  # รอค่า ToF/IMU ชุดแรก

    def _on_tof(self, info):
        self.tof_mm = info[0]
        self._tof_log.append((time.monotonic(), info[0]))

    def _on_attitude(self, info):
        self.yaw = info[0]

    def read_tof_mm(self) -> Optional[float]:
        """ ค่ามัธยฐานของ ToF ช่วง TOF_SAMPLE_S ถัดจากนี้ (ตัดค่า 0/อ่านพลาดทิ้ง) """
        t0 = time.monotonic()
        time.sleep(TOF_SAMPLE_S)
        values = [v for t, v in list(self._tof_log) if t >= t0 and v >= TOF_MIN_VALID_MM]
        return float(np.median(values)) if values else None

    def read_frame(self) -> Optional[np.ndarray]:
        return gs.read_frame(self.camera)

    def gimbal_to(self, pitch: float, yaw: float):
        """ หันหัวไปมุมที่กำหนดแล้วรอจนถึง (yaw เทียบตัวรถ + = ขวา, pitch + = เงย) """
        gs.safe_call("gimbal stop", self.gimbal.drive_speed, pitch_speed=0, yaw_speed=0)
        action = gs.safe_call("gimbal moveto", self.gimbal.moveto, pitch=pitch, yaw=yaw,
                              pitch_speed=GIMBAL_SPEED, yaw_speed=GIMBAL_SPEED)
        if action is not None:
            gs.safe_call("gimbal wait", action.wait_for_completed, timeout=3)

    def gimbal_speed(self, pitch_speed: float, yaw_speed: float):
        gs.safe_call("gimbal drive", self.gimbal.drive_speed, pitch_speed=pitch_speed, yaw_speed=yaw_speed)

    def rotate(self, steps: int):
        """ หมุนตัวรถทีละ 90° แบบ assignment2.py (1 = ขวา, 2 = กลับหลัง, 3 = ซ้าย) """
        steps %= 4
        if steps == 0:
            return
        self.stop_wheels()
        time.sleep(0.1)
        z = {1: -90, 2: 180, 3: 90}[steps]  # chassis.move: z บวก = หมุนซ้าย
        action = gs.safe_call("chassis turn", self.chassis.move, x=0, y=0, z=z, z_speed=TURN_SPEED)
        if action is not None:
            gs.safe_call("turn wait", action.wait_for_completed, timeout=MOVE_TIMEOUT_S)
        time.sleep(0.3)

    def forward_one_cell(self, target_yaw: float, check) -> float:
        """ เดินหน้า 1 ช่อง: IMU คุมหัวรถตรง + ToF กันชน คืนสัดส่วนระยะที่เดินได้ (1.0 = ครบช่อง) """
        duration = CELL_M / BASE_SPEED
        t0 = time.monotonic()
        last_err, elapsed = 0.0, 0.0
        try:
            while True:
                check()
                elapsed = time.monotonic() - t0
                if elapsed >= duration or TOF_MIN_VALID_MM <= self.tof_mm <= BUMPER_MM:
                    break
                err = normalize_angle(target_yaw - self.yaw)
                z_speed = KP_YAW * err + KD_YAW * (err - last_err)
                last_err = err
                self.chassis.drive_speed(x=BASE_SPEED, y=0, z=z_speed)
                time.sleep(0.02)
        finally:
            self.stop_wheels()
            time.sleep(0.25)
        return min(1.0, elapsed / duration)

    def move_straight(self, dx_m: float):
        """ เดินหน้า (+) / ถอย (-) ระยะสั้นในช่องเดิม """
        action = gs.safe_call("chassis move", self.chassis.move, x=dx_m, y=0, z=0, xy_speed=0.3)
        if action is not None:
            gs.safe_call("move wait", action.wait_for_completed, timeout=4)

    def fire(self) -> bool:
        ok = gs.safe_call("blaster fire", self.blaster.fire, fire_type=blaster.WATER_FIRE, times=1)
        self.leds.silence_gimbal_lights()  # ไม่ใช้ไฟกระพริบตอนยิง กันแสงเข้ากล้อง
        return bool(ok)

    def set_status_led(self, armed: bool, busy: bool):
        self.leds.show_status("white", armed, busy)

    def stop_wheels(self):
        gs.safe_call("wheels stop", self.chassis.drive_wheels, w1=0, w2=0, w3=0, w4=0)

    def stop_all(self):
        self.stop_wheels()
        self.gimbal_speed(0, 0)

    def close(self):
        self.stop_all()
        self.gimbal_to(0, 0)
        gs.safe_call("unsub tof", self.ep.sensor.unsub_distance)
        gs.safe_call("unsub imu", self.chassis.unsub_attitude)
        self.leds.close()
        gs.safe_call("stop video", self.camera.stop_video_stream)
        gs.safe_call("robot close", self.ep.close)


# ==========================================
# 4. หุ่นจำลอง (--sim)
# ==========================================
class SimRobot:
    """ สนาม 4x4 ตัวอย่าง: กำแพงบาง, บล็อกทึบ 1 ช่อง, เป้า 3 สี
    ToF และภาพกล้องสร้างจากตำแหน่งหุ่น/มุมหัว แล้วส่งเข้าระบบจับสีตัวจริงของ gimbalshoot
    """
    sim = True
    WALLS = {((0, 1), (1, 1)), ((2, 2), (2, 3)), ((2, 0), (2, 1)), ((3, 2), (3, 3))}  # ช่อง (2,3),(3,3) = ห้องปิด
    BLOCKS = {(1, 3)}
    TARGETS = {(2, 1): "red", (0, 3): "blue", (3, 0): "green"}
    PLATE_W_M, PLATE_H_M = 0.10, 0.08
    BGR = {"red": (0, 0, 255), "green": (0, 255, 0), "blue": (255, 0, 0), "yellow": (0, 255, 255)}
    W, H = 960, 540

    def __init__(self, start: Cell, heading: int, width: int, height: int):
        self.cell, self.heading = start, heading
        self.width, self.height = width, height
        self.pitch = self.yaw_rel = 0.0          # มุมหัวเทียบตัวรถ
        self._speed = (0.0, 0.0)
        self._t = time.monotonic()
        self.yaw = YAW_TARGETS[heading]           # IMU
        self.backoff = 0.0                        # ถอยจากกลางช่อง (ม.)
        self.shots: List[Tuple[Optional[Cell], float]] = []
        self.tof_mm = 9999

    # ---------- โลกจำลอง ----------
    def _blocked(self, a: Cell, b: Cell) -> bool:
        key = (a, b) if a <= b else (b, a)
        inside = 0 <= b[0] < self.width and 0 <= b[1] < self.height
        return not inside or key in self.WALLS or b in self.BLOCKS or a in self.BLOCKS

    def _integrate(self):
        now = time.monotonic()
        dt, self._t = now - self._t, now
        self.pitch = min(max(self.pitch + self._speed[0] * dt, PITCH_MIN_DEG), gs.PITCH_LIMITS[1])
        self.yaw_rel += self._speed[1] * dt

    def _camera_pose(self) -> Tuple[float, float, float]:
        """ ตำแหน่งกล้องบนพื้น (x, y) และมุมหัวเทียบทิศเหนือ (องศา ตามเข็ม) """
        a = self.heading * 90 + self.yaw_rel
        hx, hy = DIRS[self.heading]
        cx = (self.cell[0] + 0.5) * CELL_M - hx * self.backoff
        cy = (self.cell[1] + 0.5) * CELL_M - hy * self.backoff
        r = math.radians(a)
        return cx + CAMERA_FORWARD_M * math.sin(r), cy + CAMERA_FORWARD_M * math.cos(r), a

    def _visible_targets(self):
        """ เป้าที่อยู่แนวตรงเดียวกับหุ่นโดยไม่มีกำแพงกั้น """
        for cell, color in self.TARGETS.items():
            dx, dy = cell[0] - self.cell[0], cell[1] - self.cell[1]
            if (dx != 0) == (dy != 0):
                continue
            d = DIRS.index((int(math.copysign(1, dx)) if dx else 0, int(math.copysign(1, dy)) if dy else 0))
            c, ok = self.cell, True
            while c != cell:
                n = MazeMap.step(c, d)
                if self._blocked(c, n):
                    ok = False
                    break
                c = n
            if ok:
                yield cell, color

    def _target_angles(self, cell: Cell) -> Tuple[float, float]:
        """ มุมเป้าเทียบแนวกล้อง: (ซ้ายขวา + = ขวา, ขึ้นลง + = บน เทียบแนวราบ) """
        x, y, a = self._camera_pose()
        tx, ty = (cell[0] + 0.5) * CELL_M, (cell[1] + 0.5) * CELL_M
        r = math.radians(a)
        fwd = (tx - x) * math.sin(r) + (ty - y) * math.cos(r)
        lat = (tx - x) * math.cos(r) - (ty - y) * math.sin(r)
        bearing = math.degrees(math.atan2(lat, fwd))
        elev = math.degrees(math.atan2(TARGET_HEIGHT_M - CAMERA_HEIGHT_M, max(fwd, 1e-3)))
        return bearing, elev, fwd

    # ---------- คำสั่งเหมือน RealRobot ----------
    def read_tof_mm(self) -> Optional[float]:
        time.sleep(0.05)
        self._integrate()
        if abs(self.pitch) > 5:
            return 400.0  # ก้มหัว ToF เจอพื้น
        d = int(round((self.heading * 90 + self.yaw_rel) / 90.0)) % 4
        c, k = self.cell, 0
        while k < 6 and not self._blocked(c, MazeMap.step(c, d)):
            c, k = MazeMap.step(c, d), k + 1
        return (CELL_M / 2 + k * CELL_M - 0.05) * 1000

    def read_frame(self) -> np.ndarray:
        self._integrate()
        img = np.full((self.H, self.W, 3), 70, np.uint8)
        horizon = gs.degrees_to_pixel(0, -self.pitch, self.W, self.H)[1]
        cv2.rectangle(img, (0, max(0, horizon)), (self.W, self.H), (40, 60, 90), -1)
        for cell, color in self._visible_targets():
            bearing, elev, fwd = self._target_angles(cell)
            if fwd < 0.15:
                continue
            px, py = gs.degrees_to_pixel(bearing, elev - self.pitch, self.W, self.H)
            hw = int(math.degrees(math.atan2(self.PLATE_W_M / 2, fwd)) * self.W / gs.FOV_X)
            hh = int(math.degrees(math.atan2(self.PLATE_H_M / 2, fwd)) * self.H / gs.FOV_Y)
            cv2.rectangle(img, (px - hw, py - hh), (px + hw, py + hh), self.BGR[color], -1)
        return img

    def gimbal_to(self, pitch: float, yaw: float):
        self._integrate()
        self._speed = (0.0, 0.0)
        self.pitch, self.yaw_rel = min(max(pitch, PITCH_MIN_DEG), gs.PITCH_LIMITS[1]), yaw
        time.sleep(0.05)

    def gimbal_speed(self, pitch_speed: float, yaw_speed: float):
        self._integrate()
        self._speed = (pitch_speed, yaw_speed)

    def rotate(self, steps: int):
        self.heading = (self.heading + steps) % 4
        self.yaw = YAW_TARGETS[self.heading]
        time.sleep(0.15)

    def forward_one_cell(self, target_yaw: float, check) -> float:
        check()
        nxt = MazeMap.step(self.cell, self.heading)
        if nxt in self.TARGETS:
            raise RuntimeError(f"SIM: robot drove into target cell {nxt}")
        if self._blocked(self.cell, nxt):
            return 0.1  # ชนกำแพง -> กันชนเบรก
        time.sleep(0.2)
        self.cell = nxt
        return 1.0

    def move_straight(self, dx_m: float):
        self.backoff -= dx_m

    def fire(self) -> bool:
        """ บันทึกว่ากระสุนไปโดนเป้าไหน (ใกล้แนวเล็งไม่เกิน 3°) """
        self._integrate()
        best, best_err = None, 3.0
        for cell, _ in self._visible_targets():
            bearing, elev, _ = self._target_angles(cell)
            err = math.hypot(bearing, elev - self.pitch)
            if err < best_err:
                best, best_err = cell, err
        self.shots.append((best, round(best_err, 2)))
        return True

    def set_status_led(self, armed: bool, busy: bool):
        pass

    def stop_wheels(self):
        pass

    def stop_all(self):
        self.gimbal_speed(0, 0)

    def close(self):
        pass


class FrameGrabber(threading.Thread):
    """ อ่านกล้องใน thread เดียวตลอดเวลา ให้ทั้ง GUI และตัวคุมหุ่นหยิบภาพล่าสุดไปใช้
    (ถ้าอ่านกล้องหลายที่ จะแย่งเฟรมกันจนภาพกระตุก)
    """

    def __init__(self, io):
        super().__init__(name="frame-grabber", daemon=True)
        self.io = io
        self.frame: Optional[np.ndarray] = None
        self.seq = 0
        self._stop_event = threading.Event()

    def run(self):
        while not self._stop_event.is_set():
            frame = self.io.read_frame()
            if frame is not None:
                self.frame, self.seq = frame, self.seq + 1
            if self.io.sim:
                time.sleep(1 / 30)

    def wait_new(self, after_seq: int, timeout: float = 1.5) -> Tuple[Optional[np.ndarray], int]:
        """ รอภาพที่ใหม่กว่า after_seq (ภาพหลังหันหัวเสร็จ ไม่ใช่ภาพเก่าที่ค้างอยู่) """
        t0 = time.monotonic()
        while self.seq <= after_seq and time.monotonic() - t0 < timeout:
            time.sleep(0.01)
        return self.frame, self.seq

    def stop(self):
        self._stop_event.set()


# ==========================================
# 5. ตัวคุมภารกิจ (ทำงานใน thread แยก ส่วน GUI แค่อ่านสถานะไปแสดง)
# ==========================================
class MazeMission:
    def __init__(self, io, grabber: FrameGrabber, maze: MazeMap, start: Cell, heading: int):
        self.io, self.grabber, self.maze = io, grabber, maze
        self.start_cell = start
        self.cell, self.heading = start, heading
        self.yaw_offset = io.yaw - YAW_TARGETS[heading]  # มุม IMU ตอนเริ่ม = ทิศเริ่มต้น
        self.color_ranges, zones, _, _ = gs.load_config(gs.CONFIG_PATH)
        self.blind_zones = gs.BlindZones(gs.AUTO_BLIND_ZONE, zones)

        self.stop_event = threading.Event()
        self.busy = False
        self.phase = "IDLE"
        self.status = "Ready"
        self.logs: deque = deque(maxlen=9)
        self.look_dir: Optional[int] = None       # ทิศที่หัวกำลังมอง (วาดบนแผนที่)
        self.overlay: List[gs.Target] = []        # เป้าที่เห็นในภาพล่าสุด (วาดบนภาพกล้อง)
        self.overlay_w = gs.PROCESS_W
        self.route: List[Cell] = []
        self.results: Dict[Cell, str] = {}
        self.armed = False
        self.pid_yaw = gs.PIDController(**gs.YAW_PID)
        self.pid_pitch = gs.PIDController(**gs.PITCH_PID)
        self.fire_ctrl = gs.FireController()

    # ---------- งานเบื้องหลัง ----------
    def log(self, msg: str):
        print(f"[{time.strftime('%H:%M:%S')}] {msg}")
        self.logs.append(msg)

    def check(self):
        if self.stop_event.is_set():
            raise Aborted()

    def start_job(self, name: str, fn, *args) -> bool:
        if self.busy:
            return False
        self.stop_event.clear()
        self.busy, self.phase = True, name

        def runner():
            try:
                fn(*args)
            except Aborted:
                self.log("STOPPED by user")
            except Exception as e:  # แสดงบนจอด้วย ไม่ให้ thread ตายเงียบ
                traceback.print_exc()
                self.log(f"ERROR: {e}")
            finally:
                self.armed = self.fire_ctrl.armed = False
                self.io.stop_all()
                self.io.set_status_led(False, False)
                self.look_dir, self.overlay, self.route = None, [], []
                self.busy, self.phase = False, "IDLE"
                self.status = "Ready"

        threading.Thread(target=runner, name=name, daemon=True).start()
        return True

    def stop(self):
        self.stop_event.set()

    # ---------- การเคลื่อนที่ ----------
    def face(self, d: int):
        """ หมุนตัวรถไปทิศ d แล้วหันหัวตรงตัวรถ (ToF ด้านหน้าเป็นกันชนตอนเดิน) """
        steps = (d - self.heading) % 4
        if steps:
            self.io.rotate(steps)
            self.heading = d
        self.io.gimbal_to(0, 0)

    def drive_path(self, path: List[Cell], on_arrive=None) -> bool:
        """ เดินตาม path ทีละช่อง ถ้ากันชนเบรกกลางทาง บันทึกเป็นกำแพงแล้วคืน False ให้วางเส้นทางใหม่ """
        self.route = list(path)
        for nxt in path[1:]:
            self.check()
            d = DIRS.index((nxt[0] - self.cell[0], nxt[1] - self.cell[1]))
            self.face(d)
            self.status = f"Drive {self.cell} -> {nxt}"
            fraction = self.io.forward_one_cell(self.yaw_offset + YAW_TARGETS[d], self.check)
            if fraction < MIN_TRAVEL_FRACTION:
                self.log(f"Bumper stop {self.cell}->{nxt}: wall, replan")
                if fraction > 0.05:
                    self.io.move_straight(-fraction * CELL_M)  # กลับไปกลางช่องเดิม
                self.maze.set_edge(self.cell, nxt, -1)
                return False
            self.cell = nxt
            self.route = self.route[1:]
            if on_arrive is not None:
                on_arrive()
        return True

    def go_to(self, goal: Cell) -> bool:
        for _ in range(6):  # วางเส้นทางใหม่ได้หลายรอบถ้าเจอกำแพงไม่คาดคิด
            self.check()
            if self.cell == goal:
                return True
            path = self.maze.path_to(self.cell, self.heading, goal)
            if path is None:
                self.log(f"No path to {goal}")
                return False
            if self.drive_path(path):
                return True
        return self.cell == goal

    # ---------- รอบ 1: สำรวจ ----------
    def explore(self):
        self.log("ROUND 1: explore")
        self.io.set_status_led(False, True)
        self.maze.explored = False
        self.scan_cell()
        while True:
            self.check()
            path = self.maze.path_to_unvisited(self.cell, self.heading)
            if path is None:
                break
            self.drive_path(path, on_arrive=self.scan_cell)
        self.maze.explored = True
        self.save_map()
        found = ", ".join(f"{c.upper()}@{cell}" for cell, c in sorted(self.maze.targets.items())) or "none"
        self.log(f"ROUND 1 done: {len(self.maze.visited)} cells, targets: {found}")

    def scan_cell(self):
        """ หันหัวไปทุกทิศที่ยังไม่รู้: ToF วัดกำแพง ถ้าทางเปิดก้มกล้องหาเป้าในช่องติดกัน """
        c = self.cell
        self.maze.visited.add(c)
        rel_of = {0: 0, 1: 90, 2: 180, 3: -90}
        dirs = sorted(self.maze.unknown_dirs(c), key=lambda d: rel_of[(d - self.heading) % 4])
        for d in dirs:
            self.check()
            rel, nb = rel_of[(d - self.heading) % 4], MazeMap.step(c, d)
            self.look_dir = d
            self.status = f"Scan {c} {DIR_NAMES[d]}"
            self.io.gimbal_to(0, rel)
            time.sleep(GIMBAL_SETTLE_S if not self.io.sim else 0.02)
            tof = self.io.read_tof_mm()
            if tof is not None and tof < WALL_TOF_MM:
                self.maze.set_edge(c, nb, -1)
                continue
            if tof is None:
                self.log(f"ToF no reading at {c} {DIR_NAMES[d]}: assume open")
            self.maze.set_edge(c, nb, 1)
            if nb not in self.maze.visited:
                color = self.look_for_target(rel)
                if color is not None and self.maze.targets.get(nb) != color:
                    self.maze.targets[nb] = color
                    self.log(f"Target {color.upper()} at {nb}")
        if dirs:
            self.look_dir = None
            self.io.gimbal_to(0, 0)

    def look_for_target(self, rel_yaw: float) -> Optional[str]:
        self.io.gimbal_to(TARGET_SCAN_PITCH, rel_yaw)
        seq = self.grabber.seq
        time.sleep(GIMBAL_SETTLE_S if not self.io.sim else 0.02)
        votes: Counter = Counter()
        for _ in range(TARGET_SCAN_FRAMES):
            self.check()
            frame, seq = self.grabber.wait_new(seq)
            if frame is None:
                continue
            fd = gs.prepare_frame(frame)
            found = gs.detect_all_colors(fd, self.color_ranges, self.blind_zones)
            self.overlay, self.overlay_w = found, fd.size[0]
            color = self.adjacent_target(fd, found)
            if color is not None:
                votes[color] += 1
        self.overlay = []
        self.io.gimbal_to(0, rel_yaw)
        if votes:
            color, hits = votes.most_common(1)[0]
            if hits >= TARGET_MIN_HITS:
                return color
        return None

    @staticmethod
    def adjacent_target(fd, found: List[gs.Target]) -> Optional[str]:
        """ เลือกป้ายที่อยู่กลางช่องติดกันจริง: คำนวณระยะบนพื้นจากมุมก้ม (กล้องสูงกว่าเป้า) """
        w, h = fd.size
        best = None
        for t in found:
            bearing, dy = gs.pixel_to_degrees(t.cx, t.cy, w, h)
            depression = -(TARGET_SCAN_PITCH + dy)
            if depression <= 1.0:
                continue  # อยู่ระดับกล้องหรือสูงกว่า = ไม่ใช่เป้าบนพื้น
            ground = (CAMERA_HEIGHT_M - TARGET_HEIGHT_M) / math.tan(math.radians(depression))
            distance = ground + CAMERA_FORWARD_M
            lateral = ground * math.tan(math.radians(bearing))
            if TARGET_CELL_RANGE[0] <= distance <= TARGET_CELL_RANGE[1] and abs(lateral) <= TARGET_MAX_LATERAL_M:
                if best is None or t.area > best.area:
                    best = t
        return best.color if best is not None else None

    # ---------- รอบ 2: ยิง ----------
    def shoot_run(self, fire: bool, start_mode: str, end_mode: str, order_mode: str,
                  color_order: List[str], goal: Optional[Cell]):
        if not self.maze.explored:
            self.log("Explore first (round 1)")
            return
        self.armed = self.fire_ctrl.armed = fire
        self.io.set_status_led(fire, False)
        self.results = {}
        self.log(f"ROUND 2: {'FIRE' if fire else 'DRY RUN'} | {start_mode} | {order_mode} | end: {end_mode}")
        if start_mode == "FROM START" and self.cell != self.start_cell:
            self.log(f"Back to START {self.start_cell}")
            if not self.go_to(self.start_cell):
                return
        end_cell = self.start_cell if end_mode == "BACK TO START" else goal if end_mode == "GO TO GOAL" else None
        order = self.plan_targets(order_mode, color_order, end_cell)
        if not order:
            self.log("No reachable target to shoot")
        else:
            self.log("Order: " + " > ".join(f"{self.maze.targets[t][0].upper()}{t}" for t in order))
        for target in order:
            self.check()
            self.results[target] = "GOING"
            spot = self.best_spot(target)
            if spot is None or not self.go_to(spot[0]):
                self.results[target] = "SKIPPED"
                self.log(f"Skip {target}: cannot reach a shooting cell")
                continue
            self.face(spot[1])
            self.results[target] = self.aim_and_fire(target)
            self.log(f"{self.maze.targets[target].upper()} {target}: {self.results[target]}")
        if end_mode == "BACK TO START":
            self.go_to(self.start_cell)
        elif end_mode == "GO TO GOAL":
            if goal is None:
                self.log("No GOAL set: stay here (click a map cell to set GOAL)")
            else:
                self.go_to(goal)
        shot = sum(r == "SHOT" for r in self.results.values())
        self.log(f"ROUND 2 done: shot {shot}/{len(order)}")
        self.save_map()

    def best_spot(self, target: Cell) -> Optional[Tuple[Cell, int]]:
        """ ช่องยิงที่เดินไปถึงใกล้สุดจากตำแหน่งปัจจุบัน """
        dist = self.maze.distances_from(self.cell)
        spots = [s for s in self.maze.shooting_spots(target) if s[0] in dist]
        return min(spots, key=lambda s: dist[s[0]]) if spots else None

    def plan_targets(self, order_mode: str, color_order: List[str], end_cell: Optional[Cell]) -> List[Cell]:
        """ SHORTEST: ลองทุกลำดับ (เป้าไม่เยอะ) เลือกลำดับที่เดินรวมน้อยสุด นับรวมทางไปจุดจบด้วย
        COLOR ORDER: ยิงตามลำดับสีที่เลือก (สีที่ไม่เลือกไม่ยิง) สีเดียวกันไปเป้าที่ใกล้กว่าก่อน
        """
        targets = [t for t in self.maze.targets if self.maze.shooting_spots(t)]
        if order_mode == "COLOR ORDER":
            targets = [t for t in targets if self.maze.targets[t] in color_order]
        cache: Dict[Cell, Dict[Cell, int]] = {}

        def dist(a: Cell, b: Cell) -> float:
            if a not in cache:
                cache[a] = self.maze.distances_from(a)
            return cache[a].get(b, math.inf)

        def reach(pos: Cell, t: Cell) -> Tuple[float, Cell]:
            return min(((dist(pos, s), s) for s, _ in self.maze.shooting_spots(t)), default=(math.inf, pos))

        def greedy(pos: Cell, pool: List[Cell]) -> Tuple[List[Cell], Cell]:
            """ ไปเป้าที่ใกล้สุดก่อนเรื่อยๆ คืน (ลำดับ, ช่องที่หุ่นอยู่ตอนจบ) """
            out, pool = [], list(pool)
            while pool:
                t = min(pool, key=lambda t: reach(pos, t)[0])
                out.append(t)
                pos = reach(pos, t)[1]
                pool.remove(t)
            return out, pos

        if order_mode == "COLOR ORDER":
            order, pos = [], self.cell
            for color in color_order:
                group, pos = greedy(pos, [t for t in targets if self.maze.targets[t] == color])
                order += group
            return order
        if len(targets) > MAX_PERMUTE_TARGETS:
            return greedy(self.cell, targets)[0]
        best_cost, best = math.inf, []
        for perm in itertools.permutations(targets):
            pos, total = self.cell, 0.0
            for t in perm:
                cost, pos = reach(pos, t)
                total += cost
                if total >= best_cost:
                    break
            if end_cell is not None:
                total += dist(pos, end_cell)
            if total < best_cost:
                best_cost, best = total, list(perm)
        return best

    def aim_and_fire(self, target: Cell) -> str:
        """ เล็งละเอียดด้วยกล้อง + PID ของ gimbalshoot แล้วยิงตามเกณฑ์ล็อกเดียวกัน """
        color = self.maze.targets[target]
        drop = CAMERA_HEIGHT_M - TARGET_HEIGHT_M
        ground = CELL_M - CAMERA_FORWARD_M
        # เป้าเตี้ยและใกล้: ถ้าต้องก้มเกินที่หัวทำได้ ถอยรถในช่องเล็กน้อยให้มุมก้มพอดี
        max_depression = math.radians(-PITCH_MIN_DEG - PITCH_MARGIN_DEG)
        backoff = min(MAX_BACKOFF_M, max(0.0, drop / math.tan(max_depression) - ground))
        if backoff > 0.01:
            self.io.move_straight(-backoff)
        self.io.gimbal_to(max(PITCH_MIN_DEG, -math.degrees(math.atan2(drop, ground + backoff))), 0)
        self.pid_yaw.reset()
        self.pid_pitch.reset()
        self.fire_ctrl.reset_lock()
        result, shots, hold_since, last_deg = "SKIPPED", 0, None, None
        seq, t0 = self.grabber.seq, time.monotonic()
        while time.monotonic() - t0 < AIM_TIMEOUT_S:
            self.check()
            frame, seq = self.grabber.wait_new(seq)
            if frame is None:
                continue
            fd = gs.prepare_frame(frame)
            w, h = fd.size
            cands = gs.find_targets(fd, gs.build_mask(fd.hsv, self.color_ranges[color], self.blind_zones), color)
            self.overlay, self.overlay_w = cands, w
            ref = last_deg or (gs.AIM_OFFSET_X_DEG, gs.AIM_OFFSET_Y_DEG)
            best, best_deg, best_dist = None, None, gs.MATCH_RADIUS_DEG
            for cand in cands:
                deg = gs.pixel_to_degrees(cand.cx, cand.cy, w, h)
                d = math.hypot(deg[0] - ref[0], deg[1] - ref[1])
                if d < best_dist:
                    best, best_deg, best_dist = cand, deg, d
            if best is None:
                self.io.gimbal_speed(0, 0)
                self.pid_yaw.reset()
                self.pid_pitch.reset()
                self.fire_ctrl.reset_lock()
                last_deg, hold_since = None, None
                self.status = f"Aim {color.upper()}: target not in view"
                continue
            last_deg = best_deg
            ex, ey = best_deg[0] - gs.AIM_OFFSET_X_DEG, best_deg[1] - gs.AIM_OFFSET_Y_DEG
            self.io.gimbal_speed(self.pid_pitch.compute(ey), self.pid_yaw.compute(ex))
            state, should_fire = self.fire_ctrl.evaluate(ex, ey, False)
            self.status = f"Aim {color.upper()}: {state}  err X {ex:+.2f} Y {ey:+.2f}"
            if should_fire:
                self.io.gimbal_speed(0, 0)
                if self.io.fire():
                    shots += 1
                    self.log(f"FIRE at {color.upper()} {target}")
                self.fire_ctrl.mark_fired()
                if shots >= SHOTS_PER_TARGET:
                    result = "SHOT"
                    break
            elif self.fire_ctrl.settled and not self.armed:
                hold_since = hold_since or time.monotonic()
                if time.monotonic() - hold_since >= DRY_HOLD_S:
                    result = "AIMED"
                    break
            else:
                hold_since = None
        self.io.gimbal_speed(0, 0)
        self.overlay = []
        if backoff > 0.01:
            self.io.move_straight(backoff)  # กลับกลางช่อง
        self.io.gimbal_to(0, 0)
        return result

    def recenter(self):
        self.io.gimbal_to(0, 0)

    def save_map(self):
        data = self.maze.to_json()
        data.update({"start": list(self.start_cell), "robot": list(self.cell), "heading": self.heading,
                     "results": [{"cell": list(c), "result": r} for c, r in self.results.items()],
                     "saved_at": time.strftime("%Y-%m-%d %H:%M:%S")})
        try:
            MAP_FILE.write_text(json.dumps(data, indent=2), encoding="utf-8")
            self.log(f"Map saved: {MAP_FILE.name}")
        except OSError as e:
            self.log(f"Cannot save map: {e}")


# ==========================================
# 6. GUI หน้าต่างเดียว: ภาพกล้อง + แผนที่สด + ปุ่ม (วิดเจ็ตจาก gimbalshoot)
# ==========================================
CANVAS_W, CANVAS_H = 1280, 720
CAM_W, CAM_H = 640, 360            # ภาพกล้อง มุมซ้ายบน
MAP_Y0 = CAM_H                     # แผนที่ อยู่ใต้ภาพกล้อง (640 x 360)
PANEL_X = 640                      # แผงปุ่ม ครึ่งขวา
WALL_COLOR = (0, 215, 255)
ROUTE_COLOR = (255, 255, 0)


class App:
    def __init__(self, mission: MazeMission, grabber: FrameGrabber):
        self.m, self.grabber = mission, grabber
        self.ui = gs.Ui()
        self.start_mode = self.end_mode = self.order_mode = 0
        self.color_order = list(gs.COLOR_ORDER)
        self._order_fresh = True
        self.goal: Optional[Cell] = None
        self.running = True
        self._grid = (0, 0, 1)  # (x0, y0, ขนาดช่อง px) ของแผนที่ที่วาดล่าสุด ใช้ตอนคลิก

    # ---------- เมาส์ + คีย์บอร์ด ----------
    def on_mouse(self, event, x, y, flags, param):
        x0, y0, cs = self._grid
        maze = self.m.maze
        if event == cv2.EVENT_LBUTTONDOWN and x0 <= x < x0 + maze.w * cs and y0 <= y < y0 + maze.h * cs:
            cx, cy = (x - x0) // cs, maze.h - 1 - (y - y0) // cs
            self.ui.actions.append(f"goal:{cx},{cy}")
            return
        self.ui.on_mouse(event, x, y, flags, param)

    def key_action(self, key: int) -> Optional[str]:
        ch = gs.key_char(key)
        if ch == "q" or key == 27:
            return "quit"
        if ch == "x" or key == 32:
            return "stop"
        if key in (13, 10):
            return "shoot"
        if ch in gs.KEY_TO_COLOR:
            return "color:" + gs.KEY_TO_COLOR[ch]
        return {"e": "explore", "d": "dry", "c": "center", "1": "opt_start", "2": "opt_end",
                "3": "opt_order", "u": "undo"}.get(ch)

    def do(self, action: str):
        m = self.m
        if action == "quit":
            m.stop()
            self.running = False
        elif action == "stop":
            m.stop()
        elif action.startswith("goal:"):
            self.goal = tuple(int(v) for v in action[5:].split(","))
            m.log(f"GOAL set to {self.goal}")
        elif action == "opt_start":
            self.start_mode = (self.start_mode + 1) % len(START_MODES)
        elif action == "opt_end":
            self.end_mode = (self.end_mode + 1) % len(END_MODES)
        elif action == "opt_order":
            self.order_mode = (self.order_mode + 1) % len(ORDER_MODES)
        elif action.startswith("color:"):
            color = action[6:]
            if self._order_fresh:  # คลิกสีแรก = เริ่มเรียงใหม่ (แบบ gimbalshoot)
                self.color_order, self._order_fresh = [], False
            if color in self.color_order:
                self.color_order.remove(color)
            else:
                self.color_order.append(color)
            self.order_mode = ORDER_MODES.index("COLOR ORDER")
        elif action == "undo":
            self._order_fresh = False
            if self.color_order:
                self.color_order.pop()
        elif m.busy:
            return  # ระหว่างหุ่นทำงาน รับแค่ STOP / QUIT / ตัวเลือก
        elif action == "explore":
            m.start_job("EXPLORE", m.explore)
        elif action in ("shoot", "dry"):
            if not m.maze.explored:
                m.log("Run EXPLORE (round 1) first")
                return
            m.start_job("SHOOT" if action == "shoot" else "DRY RUN", m.shoot_run, action == "shoot",
                        START_MODES[self.start_mode], END_MODES[self.end_mode], ORDER_MODES[self.order_mode],
                        list(self.color_order), self.goal)
        elif action == "center":
            m.start_job("RECENTER", m.recenter)

    # ---------- Loop หลัก ----------
    def run(self):
        cv2.namedWindow(WINDOW, cv2.WINDOW_AUTOSIZE)
        cv2.setMouseCallback(WINDOW, self.on_mouse)
        while self.running:
            self.render()
            action = self.key_action(cv2.waitKey(30) & 0xFF)
            if action is not None:
                self.ui.actions.append(action)
            while self.ui.actions and self.running:
                self.do(self.ui.actions.popleft())

    # ---------- วาดหน้าจอ ----------
    def render(self):
        canvas = np.full((CANVAS_H, CANVAS_W, 3), gs.UI_BG, np.uint8)
        self.draw_camera(canvas)
        self.draw_map(canvas)
        cv2.rectangle(canvas, (PANEL_X, 0), (CANVAS_W, CANVAS_H), gs.UI_PANEL, -1)
        self.ui.begin()
        self.draw_panel(canvas)
        cv2.imshow(WINDOW, canvas)

    def draw_camera(self, canvas):
        frame = self.grabber.frame
        if frame is None:
            gs.put_text(canvas, "Waiting for camera...", (20, 40), gs.UI_DIM, 0.6)
            return
        view = cv2.resize(frame, (CAM_W, CAM_H), interpolation=cv2.INTER_AREA)
        gs.draw_targets(view, list(self.m.overlay), CAM_W / max(1, self.m.overlay_w))
        ax, ay = gs.degrees_to_pixel(gs.AIM_OFFSET_X_DEG, gs.AIM_OFFSET_Y_DEG, CAM_W, CAM_H)
        cv2.drawMarker(view, (ax, ay), (255, 255, 255), markerType=cv2.MARKER_CROSS, markerSize=18, thickness=1)
        canvas[:CAM_H, :CAM_W] = view
        gs.put_text(canvas, self.m.status[:60], (10, CAM_H - 12), (255, 255, 255), 0.5, 1)

    def draw_map(self, canvas):
        m, maze = self.m, self.m.maze
        cs = int(min((CAM_W - 170) / maze.w, (CANVAS_H - MAP_Y0 - 30) / maze.h))
        x0, y0 = 20, MAP_Y0 + 15 + ((CANVAS_H - MAP_Y0 - 30) - maze.h * cs) // 2
        self._grid = (x0, y0, cs)

        def corner(c: Cell) -> Tuple[int, int]:
            return x0 + c[0] * cs, y0 + (maze.h - 1 - c[1]) * cs

        def center(c: Cell) -> Tuple[int, int]:
            px, py = corner(c)
            return px + cs // 2, py + cs // 2

        targets, visited, results = dict(maze.targets), set(maze.visited), dict(m.results)
        for x in range(maze.w):
            for y in range(maze.h):
                c = (x, y)
                px, py = corner(c)
                if c in targets:
                    fill = gs.dim(gs.DISPLAY_COLORS[targets[c]], 0.6)
                elif c in visited:
                    fill = (70, 95, 70)
                elif maze.explored:
                    fill = (22, 22, 22)  # สำรวจครบแล้วไปไม่ถึง = บล็อก/ห้องปิด
                else:
                    fill = (55, 55, 55)
                cv2.rectangle(canvas, (px, py), (px + cs, py + cs), fill, -1)
                cv2.rectangle(canvas, (px, py), (px + cs, py + cs), (85, 85, 85), 1)
                if maze.explored and c not in visited and c not in targets:
                    cv2.line(canvas, (px + 6, py + 6), (px + cs - 6, py + cs - 6), (70, 70, 70), 1)
                    cv2.line(canvas, (px + cs - 6, py + 6), (px + 6, py + cs - 6), (70, 70, 70), 1)
                if c in targets:
                    gs.label(canvas, gs.COLOR_SHORT[targets[c]], px + 6, py + 20, (255, 255, 255), 0.5, 1)
                    if c in results:
                        gs.label(canvas, results[c], px + 6, py + cs - 8, (255, 255, 255), 0.4, 1)
        for (a, b), status in list(maze.edges.items()):
            if status != -1:
                continue
            (ax, ay), (bx, by) = a, b
            if ax == bx:  # กำแพงแนวนอน (ระหว่างช่องบน/ล่าง)
                y_line = y0 + (maze.h - max(ay, by)) * cs
                cv2.line(canvas, (x0 + ax * cs, y_line), (x0 + (ax + 1) * cs, y_line), WALL_COLOR, 4)
            else:        # กำแพงแนวตั้ง
                x_line = x0 + max(ax, bx) * cs
                row = maze.h - 1 - ay
                cv2.line(canvas, (x_line, y0 + row * cs), (x_line, y0 + (row + 1) * cs), WALL_COLOR, 4)
        route = list(m.route)
        if len(route) > 1:
            cv2.polylines(canvas, [np.array([center(c) for c in route], np.int32)], False, ROUTE_COLOR, 2)
        gs.label(canvas, "S", corner(m.start_cell)[0] + cs - 16, corner(m.start_cell)[1] + 18, (80, 255, 80), 0.55, 2)
        if self.goal is not None and maze.inside(self.goal):
            gx, gy = corner(self.goal)
            cv2.rectangle(canvas, (gx + 3, gy + 3), (gx + cs - 3, gy + cs - 3), (255, 80, 255), 2)
            gs.label(canvas, "G", gx + cs - 16, gy + cs - 8, (255, 80, 255), 0.55, 2)
        # หุ่น: สามเหลี่ยมชี้ทิศตัวรถ + เส้นส้มคือทิศที่หัวกำลังมอง
        rx, ry = center(m.cell)
        hx, hy = DIRS[m.heading]
        r = max(8, cs // 4)
        tip = (rx + hx * r, ry - hy * r)
        left = (rx - hy * r * 0.7 - hx * r * 0.6, ry - hx * r * 0.7 + hy * r * 0.6)
        right = (rx + hy * r * 0.7 - hx * r * 0.6, ry + hx * r * 0.7 + hy * r * 0.6)
        cv2.fillPoly(canvas, [np.array([tip, left, right], np.int32)], (255, 170, 60))
        if m.look_dir is not None:
            lx, ly = DIRS[m.look_dir]
            cv2.line(canvas, (rx, ry), (rx + lx * cs // 2, ry - ly * cs // 2), (0, 140, 255), 2)
        legend_x = x0 + maze.w * cs + 20
        for i, (text, color) in enumerate((("S = start", (80, 255, 80)), ("G = goal (click)", (255, 80, 255)),
                                           ("wall", WALL_COLOR), ("dark X = block", (150, 150, 150)),
                                           ("cyan = route", ROUTE_COLOR))):
            gs.label(canvas, text, legend_x, MAP_Y0 + 40 + i * 24, color, 0.45, 1)

    def draw_panel(self, canvas):
        m = self.m
        px, bw = PANEL_X + 16, CANVAS_W - PANEL_X - 32
        half, third = (bw - 8) // 2, (bw - 16) // 3
        gs.label(canvas, "ROBOMASTER MAZE SHOOTER", px, 28, gs.UI_TEXT, 0.6, 2)
        badge_color = {"IDLE": gs.UI_GREEN, "EXPLORE": gs.UI_ORANGE, "SHOOT": gs.UI_RED}.get(m.phase, gs.UI_BLUE)
        cv2.rectangle(canvas, (px, 40), (px + bw, 70), badge_color, -1)
        gs.label(canvas, "READY" if m.phase == "IDLE" else m.phase, px + 10, 62, gs.text_color_for(badge_color), 0.6, 2)
        fire_text, fire_color = ("FIRE: ARMED", (90, 90, 255)) if m.armed else ("FIRE: SAFE", gs.UI_OK)
        gs.label(canvas, f"Robot {m.cell} facing {DIR_NAMES[m.heading]}   "
                         f"Map: {'EXPLORED' if m.maze.explored else 'not explored'}", px, 94, gs.UI_TEXT, 0.48)
        gs.label(canvas, fire_text, px + bw - 120, 94, fire_color, 0.5, 2)
        gs.label(canvas, m.status, px, 118, gs.UI_DIM, 0.45, max_w=bw)

        idle, explored = not m.busy, m.maze.explored
        gs.label(canvas, "ROUND 1", px, 146, gs.UI_DIM, 0.45)
        self.ui.button(canvas, px, 154, half, 42, "EXPLORE MAP (e)", "explore", gs.UI_ORANGE, enabled=idle)
        self.ui.button(canvas, px + half + 8, 154, half, 42, "RECENTER HEAD (c)", "center", enabled=idle)

        gs.label(canvas, "ROUND 2 OPTIONS (click to change)", px, 222, gs.UI_DIM, 0.45)
        self.ui.button(canvas, px, 230, third, 36, f"START: {START_MODES[self.start_mode]} (1)", "opt_start")
        end_text = END_MODES[self.end_mode] + (f" {self.goal}" if END_MODES[self.end_mode] == "GO TO GOAL" and self.goal else "")
        self.ui.button(canvas, px + third + 8, 230, third, 36, f"END: {end_text} (2)", "opt_end")
        self.ui.button(canvas, px + 2 * (third + 8), 230, third, 36, f"ORDER: {ORDER_MODES[self.order_mode]} (3)", "opt_order")
        color_mode = ORDER_MODES[self.order_mode] == "COLOR ORDER"
        chip_w = (bw - 88 - 4 * 8) // 4
        for i, color in enumerate(gs.COLOR_ORDER):
            picked = color in self.color_order
            text = f"{self.color_order.index(color) + 1}:{gs.COLOR_SHORT[color]}" if picked else gs.COLOR_SHORT[color]
            self.ui.button(canvas, px + i * (chip_w + 8), 276, chip_w, 34, text, "color:" + color,
                           gs.DISPLAY_COLORS[color] if picked and color_mode else gs.dim(gs.DISPLAY_COLORS[color]),
                           selected=picked and color_mode)
        self.ui.button(canvas, px + bw - 80, 276, 80, 34, "UNDO (u)", "undo")
        can_shoot = idle and explored
        self.ui.button(canvas, px, 324, half, 56, "SHOOT - LIVE FIRE (ENTER)", "shoot", gs.UI_RED, enabled=can_shoot)
        self.ui.button(canvas, px + half + 8, 324, half, 56, "DRY RUN - NO FIRE (d)", "dry", gs.UI_BLUE,
                       enabled=can_shoot)
        self.ui.button(canvas, px, 392, bw, 50, "STOP NOW (x / SPACE)", "stop", (30, 30, 170), enabled=m.busy)

        gs.label(canvas, "LOG", px, 466, gs.UI_DIM, 0.45)
        for i, line in enumerate(list(m.logs)):
            gs.label(canvas, line, px, 488 + i * 21, gs.UI_TEXT, 0.45, max_w=bw)
        self.ui.button(canvas, px, CANVAS_H - 40, bw, 30, "QUIT (q)", "quit", (70, 60, 110))


# ==========================================
# 7. เริ่มโปรแกรม
# ==========================================
def ask_int(prompt: str, default: int, low: int, high: int) -> int:
    while True:
        text = input(f"{prompt} [Enter = {default}]: ").strip()
        if not text:
            return default
        if text.lstrip("-").isdigit() and low <= int(text) <= high:
            return int(text)
        print(f"  ใส่ตัวเลข {low}-{high}")


def main():
    parser = argparse.ArgumentParser(description="RoboMaster EP: explore a maze (round 1), then shoot targets (round 2)")
    parser.add_argument("--sim", action="store_true", help="จำลองบนคอม ไม่ต่อหุ่น (สนาม 4x4 ตัวอย่าง)")
    args = parser.parse_args()

    print("=" * 56)
    print(" ตั้งค่าสนาม (ช่องละ 60 ซม.)  ทิศ: 0=N(^) 1=E(>) 2=S(v) 3=W(<)")
    print("=" * 56)
    if args.sim:
        print(" [SIM] ใช้สนามตัวอย่าง 4x4")
        width = height = 4
    else:
        width = ask_int("ความกว้างสนาม (จำนวนช่องแกน X)", 4, 1, 12)
        height = ask_int("ความยาวสนาม (จำนวนช่องแกน Y)", 4, 1, 12)
    start = (ask_int("จุดเริ่ม X", 0, 0, width - 1), ask_int("จุดเริ่ม Y", 0, 0, height - 1))
    heading = ask_int("หุ่นหันหน้าไปทิศ (0-3)", 0, 0, 3)

    io = SimRobot(start, heading, width, height) if args.sim else RealRobot()
    grabber = FrameGrabber(io)
    grabber.start()
    mission = MazeMission(io, grabber, MazeMap(width, height), start, heading)
    try:
        App(mission, grabber).run()
    except KeyboardInterrupt:
        print("\nCtrl+C")
    finally:
        mission.stop()
        t0 = time.monotonic()
        while mission.busy and time.monotonic() - t0 < 5:
            time.sleep(0.05)
        grabber.stop()
        io.close()
        cv2.destroyAllWindows()
        print("Closed safely")


if __name__ == "__main__":
    main()
