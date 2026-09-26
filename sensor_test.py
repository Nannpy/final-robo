"""
RoboMaster EP - ทดสอบเซ็นเซอร์ทั้ง 7 ตัวของงาน final แล้วเก็บ log ไว้วิเคราะห์

เซ็นเซอร์ตามแผนติดตั้ง (รวม 7 ตัว: ToF 1, Sharp 2)
  1 กล้องบน gimbal               5 IMU attitude (yaw/pitch/roll)
  2 ToF หน้ารถ ยึดกับตัวรถ        6 chassis position (odometry)
  3-4 Sharp IR ข้างซ้าย/ขวา       7 gimbal angle feedback

วิธีรัน (ต่อ Wi-Fi หุ่นก่อน)
  python sensor_test.py                # เมนู เลือกทดสอบทีละข้อ
  python sensor_test.py --step ports   # รันข้อเดียว
  python sensor_test.py --all          # ไล่ทุกข้อตามลำดับ (ข้อที่รถขยับจะถามก่อนทุกครั้ง)

ลำดับที่แนะนำ: ports -> live -> dist -> cell -> gimbal -> camera -> turn -> drive
  ports   หาว่า ToF อยู่ index ไหน, Sharp ซ้าย/ขวาเสียบ board/port ไหน (บังทีละตัวตามที่โปรแกรมบอก)
  live    วางหุ่นนิ่งๆ เก็บค่าทุกเซ็นเซอร์ ดูความถี่ สัญญาณรบกวน ค่าหลุด
  dist    วางแผ่นกำแพงตามระยะที่ถาม -> สอบเทียบ Sharp (cm = A*adc^B) + เช็คความแม่น ToF
  cell    วางหุ่นกลางช่องในเขาวงกตจริงหลายแบบ บอกว่ามีกำแพงทิศไหนจริง -> หาเกณฑ์ กำแพง/ทางเปิด
  gimbal  สั่งหัวไปหลายมุม เทียบกับมุมที่ gimbal รายงานกลับ
  camera  ก้มกล้องหาเป้าในช่องถัดไป เทียบระยะที่คำนวณกับระยะจริง + เก็บภาพ
  turn    [รถขยับ] หมุน 90 องศา 8 ครั้ง ดูว่า IMU บอกว่าเลี้ยวแม่นแค่ไหน
  drive   [รถขยับ] เดิน 1 ช่องแบบจับเวลา เทียบกับแบบนับ odometry และตลับเมตร

ผลทุกครั้งอยู่ใน logs/sensor_test_YYYYmmdd_HHMMSS/
  raw.csv       ค่าดิบทุกเซ็นเซอร์ 20 Hz (คอลัมน์ step บอกว่ากำลังทดสอบข้อไหน)
  events.log    ขั้นตอน คำตอบที่พิมพ์ และผลสรุป
  summary.json  ผลคำนวณของแต่ละข้อ (เขียนใหม่ทุกครั้งที่จบข้อ โปรแกรมล่มก็ยังเหลือ)
  cam_*.jpg     ภาพจากกล้อง
ผังช่องเซ็นเซอร์ + ค่าสอบเทียบ Sharp บันทึกที่ sensor_map.json (ข้อถัดไปและโปรแกรมหลักใช้ต่อ)

Ctrl+C ระหว่างทดสอบ = หยุดรถ แล้วกลับเมนู   |   q ที่เมนู = ออก
"""
import argparse
import csv
import json
import math
import statistics
import threading
import time
import traceback
from collections import Counter
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import cv2
# pyrefly: ignore [missing-import]
from robomaster import dds, robot

import gimbalshoot as gs   # เชื่อมต่อ/กล้อง/จับสี (ค่า HSV จาก autoaim_config.json)
import maze_shooter as ms  # ค่าที่โปรแกรมหลักใช้จริง: เกณฑ์กำแพง ความเร็ว ความสูงกล้อง ฯลฯ

# ==========================================
# 1. การตั้งค่า
# ==========================================
HERE = Path(__file__).resolve().parent
LOG_ROOT = HERE / "logs"
SENSOR_MAP_FILE = HERE / "sensor_map.json"

SUB_FREQ = 20                   # Hz ของทุก stream (SDK รับ 1/5/10/20/50)
ROW_HZ = 20                     # เขียน raw.csv กี่แถวต่อวินาที

# ผังเริ่มต้น (ตำแหน่งเดียวกับแล็บ 5: ซ้าย = board 1 port 1, ขวา = board 2 port 1) ข้อ ports จะแก้ให้ตามจริง
DEFAULT_MAP = {
    "tof_index": 0,
    "sharp_left": [1, 1],
    "sharp_right": [2, 1],
    "sharp_calib": {"left": [4800.0, -1.05], "right": [4800.0, -1.05]},
    "sharp_calibrated": {"left": False, "right": False},
}
ADC_MIN_VALID = 20              # ADC ต่ำกว่านี้ = ค่าหลุดจากบัส ไม่ใช่ระยะไกล (จากแล็บ 5)
SHARP_RANGE_CM = (8.0, 80.0)    # นอกช่วงนี้ Sharp เชื่อไม่ได้ (ใกล้กว่า ~10 ซม. เส้นโค้งย้อนกลับ)
SIDE_WALL_CM = 40.0             # ค่าตั้งต้น: Sharp ใกล้กว่านี้ = มีกำแพงข้าง (ข้อ cell จะบอกค่าที่ควรใช้จริง)

TOF_RESPONSE_MM = 150           # ข้อ ports: ToF ต้องเปลี่ยนเกินนี้ถึงนับว่าเป็นช่องที่ถูกบัง
ADC_RESPONSE = 30               # ข้อ ports: ADC ต้องเปลี่ยนเกินนี้

DIST_TOF_CM = [10, 15, 20, 25, 30, 40, 60, 80, 100]
DIST_SHARP_CM = [8, 10, 15, 20, 25, 30, 40, 50, 60, 80]
DIST_SAMPLE_S = 1.5

GIMBAL_POSES = [(0, 0), (0, 90), (0, -90), (0, 180), (ms.TARGET_SCAN_PITCH, 0),
                (ms.PITCH_MIN_DEG, 0), (10, 0), (0, 0)]
CAMERA_FRAMES = 30


class StepSkipped(Exception):
    """ ผู้ใช้ไม่ยืนยันให้รถขยับ """


# ==========================================
# 2. ตัวช่วยเล็กๆ
# ==========================================
def median(values: Sequence[Optional[float]]) -> Optional[float]:
    vals = [v for v in values if v is not None]
    return float(statistics.median(vals)) if vals else None


def stats(values: Sequence[Optional[float]]) -> Dict[str, Optional[float]]:
    vals = [float(v) for v in values if v is not None]
    if not vals:
        return {"n": 0, "median": None, "mean": None, "std": None, "min": None, "max": None}
    return {"n": len(vals), "median": round(statistics.median(vals), 3),
            "mean": round(statistics.mean(vals), 3),
            "std": round(statistics.pstdev(vals), 3) if len(vals) > 1 else 0.0,
            "min": round(min(vals), 3), "max": round(max(vals), 3)}


def adapter_index(board_port: Sequence[int]) -> int:
    """ (board 1-6, port 1-2) -> ตำแหน่งใน ad[12] ที่ sub_adapter ส่งมา (แบบแล็บ 5) """
    return (board_port[0] - 1) * 2 + (board_port[1] - 1)


def board_port(index: int) -> List[int]:
    return [index // 2 + 1, index % 2 + 1]


def fit_power_law(points: List[Tuple[float, float]]) -> Optional[Tuple[float, float]]:
    """ points = [(cm, adc)] -> (A, B) ของ cm = A * adc^B ด้วย least squares บนสเกล log """
    usable = [(c, a) for c, a in points if a and a > 0 and c > 0]
    if len(usable) < 2:
        return None
    adcs = [a for _, a in usable]
    if max(adcs) - min(adcs) < ADC_RESPONSE:  # ค่าแทบไม่เปลี่ยนตามระยะ = เซ็นเซอร์ไม่ตอบสนอง ฟิตไม่ได้
        return None
    xs = [math.log(a) for _, a in usable]
    ys = [math.log(c) for c, _ in usable]
    mx, my = statistics.mean(xs), statistics.mean(ys)
    den = sum((x - mx) ** 2 for x in xs)
    if den == 0:
        return None
    b = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / den
    return math.exp(my - b * mx), b


def fit_line(points: List[Tuple[float, float]]) -> Optional[Tuple[float, float]]:
    """ points = [(x, y)] -> (slope, intercept) """
    if len(points) < 2:
        return None
    mx = statistics.mean(p[0] for p in points)
    my = statistics.mean(p[1] for p in points)
    den = sum((p[0] - mx) ** 2 for p in points)
    if den == 0:
        return None
    slope = sum((p[0] - mx) * (p[1] - my) for p in points) / den
    return slope, my - slope * mx


def load_sensor_map() -> dict:
    data = json.loads(json.dumps(DEFAULT_MAP))
    if SENSOR_MAP_FILE.exists():
        try:
            data.update(json.loads(SENSOR_MAP_FILE.read_text(encoding="utf-8")))
        except (OSError, ValueError) as e:
            print(f"[WARN] อ่าน {SENSOR_MAP_FILE.name} ไม่ได้ ({e}) ใช้ผังเริ่มต้น")
    return data


# ==========================================
# 3. รับค่าทุกเซ็นเซอร์
# ==========================================
class PinboardSubject(dds.Subject):
    """ stream ADC ของ Sensor Adaptor ทุกบอร์ด
    robomaster ที่ pip ติดตั้งใน venv (0.1.1.61) ไม่มี sub_adapter / AdapterSubject แต่ DDS รู้จัก subject
    "pinboard" อยู่แล้ว จึงสร้างเองตามแบบ RoboMaster-SDK/src/robomaster/sensor.py (6 บอร์ด x 2 พอร์ต)
    """
    name = dds.DDS_PINBOARD
    uid = dds.SUB_UID_MAP[dds.DDS_PINBOARD]
    type = dds.DDS_SUB_TYPE_PERIOD

    def __init__(self, freq: int):
        super().__init__()
        self.freq = freq
        self._io = [0] * 12
        self._ad = [0] * 12

    def data_info(self):
        return self._io, self._ad

    def decode(self, buf):
        for i in range(min(6, len(buf) // 6)):
            self._io[i * 2] = buf[i * 6]
            self._io[i * 2 + 1] = buf[i * 6 + 1]
            self._ad[i * 2] = buf[i * 6 + 2] + buf[i * 6 + 3] * 256
            self._ad[i * 2 + 1] = buf[i * 6 + 4] + buf[i * 6 + 5] * 256


class Sensors:
    """ subscribe ทุก stream แล้วเก็บค่าล่าสุด + นับจำนวนครั้งที่ได้ค่า (ใช้คำนวณความถี่จริง) """
    STREAMS = ("tof", "adapter", "attitude", "position", "gimbal", "battery")

    def __init__(self, ep):
        self.ep = ep
        self.lock = threading.Lock()
        self.tof = [0] * 4
        self.ad = [0] * 12
        self.att = (0.0, 0.0, 0.0)            # yaw, pitch, roll
        self.pos = (0.0, 0.0, 0.0)            # x, y, z (ม.) นับจากตอน subscribe
        self.gim = (0.0, 0.0, 0.0, 0.0)       # pitch, yaw, pitch_ground, yaw_ground
        self.battery: Optional[int] = None
        self.counts = {s: 0 for s in self.STREAMS}
        self.subscribed: Dict[str, bool] = {}
        self.adapter_mode = "stream"          # "stream" = pinboard DDS, "poll" = get_adc ทีละช่อง, "none"
        self.adapter_boards: List[int] = []
        self._poll_stop = threading.Event()
        self._poll_thread: Optional[threading.Thread] = None

    def _bump(self, name: str):
        self.counts[name] += 1

    def _on_tof(self, info):
        with self.lock:
            self.tof = list(info)
            self._bump("tof")

    def _on_adapter(self, info):
        _io, ad = info
        with self.lock:
            self.ad = list(ad)
            self._bump("adapter")

    def _on_attitude(self, info):
        with self.lock:
            self.att = tuple(info[:3])
            self._bump("attitude")

    def _on_position(self, info):
        with self.lock:
            self.pos = tuple(info[:3])
            self._bump("position")

    def _on_gimbal(self, info):
        with self.lock:
            self.gim = tuple(info[:4])
            self._bump("gimbal")

    def _on_battery(self, info):
        with self.lock:
            self.battery = info if isinstance(info, int) else info[0] if info else None
            self._bump("battery")

    def _subscribe(self, name: str, fn, *args, **kw):
        result = gs.safe_call(f"sub {name}", fn, *args, **kw)
        self.subscribed[name] = result is not None and result is not False

    def start(self):
        ep = self.ep
        self._subscribe("tof", ep.sensor.sub_distance, freq=SUB_FREQ, callback=self._on_tof)
        # add_subject_info(subject, callback, args, kw): SDK อ่าน args/kw แบบ positional เสมอ
        self._subscribe("adapter", ep.dds.add_subject_info, PinboardSubject(SUB_FREQ), self._on_adapter, (), {})
        self._subscribe("attitude", ep.chassis.sub_attitude, freq=SUB_FREQ, callback=self._on_attitude)
        self._subscribe("position", ep.chassis.sub_position, cs=0, freq=SUB_FREQ, callback=self._on_position)
        self._subscribe("gimbal", ep.gimbal.sub_angle, freq=SUB_FREQ, callback=self._on_gimbal)
        self._subscribe("battery", ep.battery.sub_battery_info, freq=1, callback=self._on_battery)

    def ensure_adapter(self, log) -> str:
        """ ถ้า stream pinboard ไม่มีค่าเข้า ถอยไป poll get_adc (แบบแล็บ 5) เฉพาะบอร์ดที่ตอบกลับ """
        if self.counts["adapter"] > 0:
            self.adapter_mode = "stream"
            return self.adapter_mode
        if self.subscribed.get("adapter"):
            gs.safe_call("unsub adapter", self.ep.dds.del_subject_info, dds.DDS_PINBOARD)
            self.subscribed["adapter"] = False
        log("pinboard stream sent no data: probing Sensor Adaptor boards 1-6 with get_adc "
            "(a missing board can take ~3 s to time out)")
        for board in range(1, 7):
            v = gs.safe_call(f"probe board {board}", self.ep.sensor_adaptor.get_adc, id=board, port=1)
            log(f"  board {board}: {'adc ' + str(v) if v is not None else 'no answer'}")
            if v is not None:
                self.adapter_boards.append(board)
        if not self.adapter_boards:
            self.adapter_mode = "none"
            log("!! no Sensor Adaptor board answered: check the cables and the board IDs")
            return self.adapter_mode
        self.adapter_mode = "poll"
        self._poll_thread = threading.Thread(target=self._poll_adapter, name="adc-poll", daemon=True)
        self._poll_thread.start()
        log(f"polling get_adc on boards {self.adapter_boards} (ports 1 and 2)")
        return self.adapter_mode

    def _poll_adapter(self):
        channels = [(b, p) for b in self.adapter_boards for p in (1, 2)]
        while not self._poll_stop.is_set():
            for b, p in channels:
                try:
                    v = self.ep.sensor_adaptor.get_adc(id=b, port=p)
                except Exception:  # SDK จับเองส่วนใหญ่ กันไว้ไม่ให้ thread ตาย
                    v = None
                if v is not None:
                    with self.lock:
                        self.ad[adapter_index((b, p))] = v
            with self.lock:
                self._bump("adapter")
            time.sleep(0.02)

    def stop(self):
        self._poll_stop.set()
        ep = self.ep
        unsubs = {
            "tof": lambda: ep.sensor.unsub_distance(),
            "adapter": lambda: ep.dds.del_subject_info(dds.DDS_PINBOARD),
            "attitude": lambda: ep.chassis.unsub_attitude(),
            "position": lambda: ep.chassis.unsub_position(),
            "gimbal": lambda: ep.gimbal.unsub_angle(),
            "battery": lambda: ep.battery.unsub_battery_info(),
        }
        for name, fn in unsubs.items():
            if self.subscribed.get(name):
                gs.safe_call(f"unsub {name}", fn)
        if self._poll_thread is not None:
            self._poll_thread.join(timeout=2)

    def snapshot(self) -> dict:
        with self.lock:
            return {"tof": list(self.tof), "ad": list(self.ad), "att": self.att, "pos": self.pos,
                    "gim": self.gim, "battery": self.battery, "counts": dict(self.counts)}


# ==========================================
# 4. การบันทึก log
# ==========================================
class Session:
    """ โฟลเดอร์ log หนึ่งครั้งที่รัน: raw.csv + events.log + summary.json + ภาพ """

    def __init__(self):
        self.dir = LOG_ROOT / time.strftime("sensor_test_%Y%m%d_%H%M%S")
        self.dir.mkdir(parents=True, exist_ok=True)
        self._events = (self.dir / "events.log").open("a", encoding="utf-8")
        self.summary: dict = {"started_at": time.strftime("%Y-%m-%d %H:%M:%S"), "steps": {}}

    def event(self, msg: str, echo: bool = True):
        line = f"[{time.strftime('%H:%M:%S')}] {msg}"
        if echo:
            print(line)
        self._events.write(line + "\n")
        self._events.flush()

    def save_summary(self):
        path = self.dir / "summary.json"
        path.write_text(json.dumps(self.summary, indent=2, ensure_ascii=False, default=str), encoding="utf-8")

    def close(self):
        self.save_summary()
        self._events.close()


class Recorder(threading.Thread):
    """ เขียนค่าทุกเซ็นเซอร์ลง raw.csv 20 Hz ตลอดเวลาที่โปรแกรมเปิด """

    def __init__(self, tester: "Tester"):
        super().__init__(daemon=True)
        self.t = tester
        self._halt = threading.Event()

    def run(self):
        path = self.t.session.dir / "raw.csv"
        header = (["t", "clock", "step"] + [f"tof{i}" for i in range(4)] + [f"ad{i}" for i in range(12)]
                  + ["yaw", "pitch", "roll", "pos_x", "pos_y", "pos_z",
                     "g_pitch", "g_yaw", "g_pitch_ground", "g_yaw_ground", "battery",
                     "tof_front_mm", "sharp_l_adc", "sharp_l_cm", "sharp_r_adc", "sharp_r_cm"])
        t0 = time.monotonic()
        last_flush = t0
        with path.open("w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow(header)
            while not self._halt.is_set():
                s = self.t.sensors.snapshot()
                now = time.monotonic()
                l_adc, r_adc = self.t.adc(s, "left"), self.t.adc(s, "right")
                l_cm, r_cm = self.t.sharp_cm(l_adc, "left"), self.t.sharp_cm(r_adc, "right")
                writer.writerow([f"{now - t0:.3f}", time.strftime("%H:%M:%S"), self.t.step]
                                + s["tof"] + s["ad"]
                                + [f"{v:.2f}" for v in s["att"]] + [f"{v:.4f}" for v in s["pos"]]
                                + [f"{v:.2f}" for v in s["gim"]] + [s["battery"]]
                                + [self.t.tof_mm(s), l_adc, "" if l_cm is None else f"{l_cm:.1f}",
                                   r_adc, "" if r_cm is None else f"{r_cm:.1f}"])
                if now - last_flush >= 1.0:
                    f.flush()
                    last_flush = now
                time.sleep(1.0 / ROW_HZ)

    def stop(self):
        self._halt.set()


# ==========================================
# 5. ตัวทดสอบ
# ==========================================
class Tester:
    def __init__(self, ep, session: Session):
        self.ep = ep
        self.session = session
        self.map = load_sensor_map()
        self.sensors = Sensors(ep)
        self.step = "setup"
        self.color_ranges, zones, _, _ = gs.load_config(gs.CONFIG_PATH)
        self.blind_zones = gs.BlindZones(gs.AUTO_BLIND_ZONE, zones)

    # ---------- อ่านค่าตามผังช่อง ----------
    def tof_mm(self, s: dict) -> Optional[int]:
        v = s["tof"][self.map["tof_index"]]
        return v if v >= ms.TOF_MIN_VALID_MM else None

    def adc(self, s: dict, side: str) -> Optional[int]:
        v = s["ad"][adapter_index(self.map[f"sharp_{side}"])]
        return v if v is not None and v >= ADC_MIN_VALID else None

    def sharp_cm(self, adc: Optional[float], side: str) -> Optional[float]:
        if adc is None or adc <= 0:
            return None
        a, b = self.map["sharp_calib"][side]
        cm = a * (adc ** b)
        return cm if SHARP_RANGE_CM[0] <= cm <= SHARP_RANGE_CM[1] else None

    def sample(self, seconds: float) -> List[dict]:
        """ เก็บ snapshot 20 Hz นาน seconds วินาที """
        out = []
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            out.append(self.sensors.snapshot())
            time.sleep(1.0 / ROW_HZ)
        return out

    def reading(self, seconds: float) -> dict:
        """ ค่ามัธยฐานของ ToF หน้า / Sharp ซ้าย-ขวา / IMU / odometry / gimbal ในช่วงเวลาหนึ่ง """
        snaps = self.sample(seconds)
        l_adc = median([self.adc(s, "left") for s in snaps])
        r_adc = median([self.adc(s, "right") for s in snaps])
        return {
            "tof_mm": median([self.tof_mm(s) for s in snaps]),
            "tof_valid": sum(self.tof_mm(s) is not None for s in snaps), "n": len(snaps),
            "l_adc": l_adc, "l_cm": self._round(self.sharp_cm(l_adc, "left")),
            "r_adc": r_adc, "r_cm": self._round(self.sharp_cm(r_adc, "right")),
            "yaw": median([s["att"][0] for s in snaps]),
            "pos": tuple(median([s["pos"][i] for s in snaps]) for i in range(3)),
            "g_pitch": median([s["gim"][0] for s in snaps]),
            "g_yaw": median([s["gim"][1] for s in snaps]),
        }

    @staticmethod
    def _round(v: Optional[float]) -> Optional[float]:
        return None if v is None else round(v, 1)

    def channel_medians(self, seconds: float) -> dict:
        snaps = self.sample(seconds)
        return {"tof": [median([s["tof"][i] for s in snaps]) for i in range(4)],
                "ad": [median([s["ad"][i] for s in snaps]) for i in range(12)]}

    # ---------- คุยกับผู้ใช้ (ทุกคำถาม/คำตอบลง events.log) ----------
    def say(self, msg: str):
        print(msg)
        self.session.event(f"SAY {msg}", echo=False)

    def ask(self, prompt: str, default: str = "") -> str:
        ans = input(f"{prompt} ").strip()
        self.session.event(f"ASK {prompt!r} -> {ans!r}", echo=False)
        return ans if ans else default

    def ask_float(self, prompt: str, default: Optional[float]) -> Optional[float]:
        while True:
            ans = self.ask(prompt)
            if not ans:
                return default
            try:
                return float(ans)
            except ValueError:
                print("  ใส่ตัวเลข")

    def confirm_motion(self, what: str):
        ans = self.ask(f"\n[รถจะขยับ] {what}\n  พร้อมแล้วพิมพ์ y แล้ว Enter (อย่างอื่น = ข้าม):")
        if ans.lower() != "y":
            raise StepSkipped()

    def record(self, key: str, value):
        self.session.summary["steps"].setdefault(self.step, {})[key] = value
        self.session.save_summary()

    def save_map(self):
        SENSOR_MAP_FILE.write_text(json.dumps(self.map, indent=2), encoding="utf-8")
        self.session.event(f"sensor map saved: {json.dumps(self.map)}")

    # ---------- การเคลื่อนที่ ----------
    def stop_motion(self):
        gs.safe_call("wheels stop", self.ep.chassis.drive_wheels, w1=0, w2=0, w3=0, w4=0)
        gs.safe_call("gimbal stop", self.ep.gimbal.drive_speed, pitch_speed=0, yaw_speed=0)

    def gimbal_to(self, pitch: float, yaw: float) -> Tuple[bool, float]:
        """ หันหัวแบบเดียวกับโปรแกรมหลัก คืน (ถึงภายใน timeout ไหม, เวลาที่ใช้) """
        t0 = time.monotonic()
        action = gs.safe_call("gimbal moveto", self.ep.gimbal.moveto, pitch=pitch, yaw=yaw,
                              pitch_speed=ms.GIMBAL_SPEED, yaw_speed=ms.GIMBAL_SPEED)
        done = False
        if action is not None:
            done = bool(gs.safe_call("gimbal wait", action.wait_for_completed, timeout=3))
        return done, time.monotonic() - t0


# ==========================================
# 6. ข้อทดสอบ
# ==========================================
def step_ports(t: Tester):
    t.say("\nหาช่องเซ็นเซอร์: ทำตามทีละขั้น ค้างไว้จนขึ้นว่าอ่านเสร็จ (ขั้นละ 2 วินาที)")
    t.ask("1/4 เอาทุกอย่างออกห่างหุ่นเกิน 50 ซม. ทั้งหน้า ซ้าย ขวา แล้วกด Enter")
    base = t.channel_medians(2.0)
    t.record("baseline", base)
    t.say(f"  ToF 4 ช่อง: {base['tof']}\n  ADC 12 ช่อง: {base['ad']}")

    found: Dict[str, Optional[int]] = {}
    phases = [("tof", "2/4 เอามือ/แผ่นโฟมบังหน้า ToF (หน้ารถ) ห่าง ~10 ซม. แล้วกด Enter"),
              ("left", "3/4 บังหน้า Sharp ซ้าย ห่าง ~10 ซม. (ตัวอื่นปล่อยโล่ง) แล้วกด Enter"),
              ("right", "4/4 บังหน้า Sharp ขวา ห่าง ~10 ซม. แล้วกด Enter")]
    for name, prompt in phases:
        t.ask(prompt)
        blocked = t.channel_medians(2.0)
        d_tof = [None if a is None or b is None else b - a for a, b in zip(base["tof"], blocked["tof"])]
        d_ad = [None if a is None or b is None else b - a for a, b in zip(base["ad"], blocked["ad"])]
        t.record(f"blocked_{name}", {"values": blocked, "delta_tof": d_tof, "delta_ad": d_ad})
        t.say(f"  เปลี่ยนไป  ToF: {d_tof}\n             ADC: {d_ad}")
        if name == "tof":
            cands = [(abs(d), i) for i, d in enumerate(d_tof) if d is not None]
            limit = TOF_RESPONSE_MM
        else:
            taken = {found.get("left")}
            cands = [(abs(d), i) for i, d in enumerate(d_ad) if d is not None and i not in taken]
            limit = ADC_RESPONSE
        best = max(cands) if cands else (0, None)
        found[name] = best[1] if best[0] >= limit else None
        if found[name] is None:
            t.say(f"  !! ไม่เจอช่องที่ตอบสนองต่อ {name} (ต่างสุด {best[0]}) ตรวจสายหรือบังให้ใกล้กว่านี้")

    if found["tof"] is not None:
        t.map["tof_index"] = found["tof"]
    for side in ("left", "right"):
        if found[side] is not None:
            t.map[f"sharp_{side}"] = board_port(found[side])
    t.record("result", {"tof_index": found["tof"],
                        "sharp_left": None if found["left"] is None else board_port(found["left"]),
                        "sharp_right": None if found["right"] is None else board_port(found["right"])})
    t.save_map()
    t.say(f"\nผล: ToF = index {t.map['tof_index']}  |  Sharp ซ้าย = board/port {t.map['sharp_left']}"
          f"  |  Sharp ขวา = board/port {t.map['sharp_right']}")


def step_live(t: Tester):
    secs = t.ask_float("เก็บค่ากี่วินาที? (Enter = 15)", 15.0)
    t.ask("วางหุ่นนิ่งๆ กลางช่อง ไม่ต้องขยับอะไร แล้วกด Enter")
    c0 = t.sensors.snapshot()["counts"]
    snaps = []
    end = time.monotonic() + secs
    while time.monotonic() < end:
        s = t.sensors.snapshot()
        snaps.append(s)
        if len(snaps) % 10 == 0:
            tof = t.tof_mm(s)
            la, ra = t.adc(s, "left"), t.adc(s, "right")
            lc, rc = t.sharp_cm(la, "left"), t.sharp_cm(ra, "right")
            fmt = lambda v, u: "--" if v is None else f"{v:.1f}{u}"
            print(f"\r ToF {fmt(tof, 'mm')} | L {fmt(lc, 'cm')} (adc {la}) | R {fmt(rc, 'cm')} (adc {ra})"
                  f" | yaw {s['att'][0]:.1f} | pos {s['pos'][0]:.3f},{s['pos'][1]:.3f}"
                  f" | gimbal p{s['gim'][0]:.1f} y{s['gim'][1]:.1f} | batt {s['battery']}   ", end="")
        time.sleep(1.0 / ROW_HZ)
    print()
    c1 = t.sensors.snapshot()["counts"]
    rates = {k: round((c1[k] - c0[k]) / secs, 1) for k in c0}
    tof_raw = [s["tof"][t.map["tof_index"]] for s in snaps]
    l_adc = [s["ad"][adapter_index(t.map["sharp_left"])] for s in snaps]
    r_adc = [s["ad"][adapter_index(t.map["sharp_right"])] for s in snaps]
    result = {
        "seconds": secs, "rates_hz": rates,
        "tof_mm": stats([t.tof_mm(s) for s in snaps]),
        "tof_invalid": sum(v < ms.TOF_MIN_VALID_MM for v in tof_raw),
        "sharp_left_adc": stats([t.adc(s, "left") for s in snaps]),
        "sharp_left_invalid": sum(v is None or v < ADC_MIN_VALID for v in l_adc),
        "sharp_right_adc": stats([t.adc(s, "right") for s in snaps]),
        "sharp_right_invalid": sum(v is None or v < ADC_MIN_VALID for v in r_adc),
        "yaw_drift_deg": round(ms.normalize_angle(snaps[-1]["att"][0] - snaps[0]["att"][0]), 3) if snaps else None,
        "pos_drift_m": [round(snaps[-1]["pos"][i] - snaps[0]["pos"][i], 4) for i in range(3)] if snaps else None,
        "gimbal_pitch": stats([s["gim"][0] for s in snaps]),
        "gimbal_yaw": stats([s["gim"][1] for s in snaps]),
        "battery": snaps[-1]["battery"] if snaps else None,
    }
    t.record("result", result)
    t.say(f"ความถี่จริง (Hz): {rates}")
    t.say(f"ToF ค่าหลุด {result['tof_invalid']}/{len(snaps)}  |  Sharp L หลุด {result['sharp_left_invalid']}"
          f"  R หลุด {result['sharp_right_invalid']}  |  yaw drift {result['yaw_drift_deg']} องศา")
    dead = [k for k in ("tof", "adapter", "attitude", "position", "gimbal") if rates[k] == 0]
    if dead:
        t.say(f"!! stream ไม่มีค่าเข้าเลย: {dead}")


def step_dist(t: Tester):
    t.say("\nสอบเทียบระยะ: ใช้แผ่นวัสดุเดียวกับกำแพงสนาม ตั้งฉากกับแนวเซ็นเซอร์ วัดจากหน้าเซ็นเซอร์")
    labels = {"tof": "ToF หน้า", "left": "Sharp ซ้าย", "right": "Sharp ขวา"}
    for which in ("tof", "left", "right"):
        if t.ask(f"\nสอบเทียบ {labels[which]}? (Enter = ทำ, s = ข้าม)").lower() == "s":
            continue
        points: List[Tuple[float, float]] = []
        rows = []
        for cm in (DIST_TOF_CM if which == "tof" else DIST_SHARP_CM):
            ans = t.ask(f"  วางแผ่นห่างหน้า{labels[which]} {cm} ซม. แล้วกด Enter (s = ข้ามจุดนี้, q = พอแล้ว)").lower()
            if ans == "q":
                break
            if ans == "s":
                continue
            snaps = t.sample(DIST_SAMPLE_S)
            if which == "tof":
                vals = [t.tof_mm(s) for s in snaps]
                med = median(vals)
                rows.append({"true_cm": cm, "tof_mm": stats(vals)})
                if med is not None:
                    points.append((cm * 10.0, med))
                t.say(f"    ToF = {med} mm (จริง {cm * 10} mm, ต่าง {None if med is None else round(med - cm * 10)})")
            else:
                vals = [t.adc(s, which) for s in snaps]
                med = median(vals)
                rows.append({"true_cm": cm, "adc": stats(vals)})
                if med is not None:
                    points.append((float(cm), med))
                t.say(f"    ADC = {med}")
        result: dict = {"points": rows}
        if which == "tof":
            line = fit_line(points)
            if line:
                result["fit_measured_mm_vs_true_mm"] = {"slope": round(line[0], 4), "offset_mm": round(line[1], 1)}
                t.say(f"  ToF: วัดได้ = {line[0]:.3f} x จริง + {line[1]:.1f} mm")
        else:
            fit = fit_power_law(points)
            if fit:
                a, b = fit
                errs = [{"true_cm": c, "est_cm": round(a * adc ** b, 2)} for c, adc in points]
                result["fit"] = {"A": round(a, 2), "B": round(b, 5), "errors": errs}
                adcs = [adc for _, adc in sorted(points)]
                result["monotonic"] = all(x > y for x, y in zip(adcs, adcs[1:]))
                t.map["sharp_calib"][which] = [round(a, 2), round(b, 5)]
                if not isinstance(t.map.get("sharp_calibrated"), dict):
                    t.map["sharp_calibrated"] = {"left": False, "right": False}
                t.map["sharp_calibrated"][which] = True
                t.save_map()
                t.say(f"  {labels[which]}: cm = {a:.1f} * adc^{b:.4f}")
                for e in errs:
                    t.say(f"    จริง {e['true_cm']} ซม. -> คำนวณ {e['est_cm']} ซม.")
                if not result["monotonic"]:
                    t.say("  !! ADC ไม่ลดลงตามระยะทุกจุด ช่วงใกล้สุดอาจอยู่ในโซนที่เส้นโค้งย้อนกลับ")
            else:
                adcs = [adc for _, adc in points]
                if len(adcs) >= 2 and max(adcs) - min(adcs) < ADC_RESPONSE:
                    t.say(f"  !! ADC แทบไม่เปลี่ยนตามระยะ ({min(adcs)}-{max(adcs)}) = Sharp ไม่ตอบสนอง ตรวจสาย/ไฟเลี้ยง/ช่องที่เสียบ")
                else:
                    t.say("  !! ข้อมูลไม่พอสำหรับสอบเทียบ")
        t.record(which, result)


def step_cell(t: Tester):
    t.say("\nทดสอบในสนามจริง: วางหุ่นกลางช่อง หันตรงแนวกำแพง แล้วบอกว่าจริงๆ มีกำแพงหน้า/ซ้าย/ขวาไหม"
          "\nทำหลายแบบ (ทุกทิศมีทั้งกำแพงและทางเปิด) + ลองวางเป้าในช่องที่เปิดด้วย ยิ่งหลายยิ่งดี")
    trials = []
    while True:
        truth = t.ask("\nพิมพ์กำแพงจริง หน้า-ซ้าย-ขวา เป็น 1/0 (เช่น 101 = หน้ามี ซ้ายเปิด ขวามี) | q = จบ:").lower()
        if truth == "q":
            break
        if len(truth) != 3 or any(ch not in "01" for ch in truth):
            print("  ต้องเป็นเลข 0/1 สามตัว")
            continue
        note = t.ask("  โน้ต (เช่น 'มีเป้าข้างหน้า', 'เบี้ยวซ้ายนิดหน่อย') Enter = ข้าม:")
        r = t.reading(1.0)
        verdict = {
            "front": r["tof_mm"] is not None and r["tof_mm"] < ms.WALL_TOF_MM,
            "left": r["l_cm"] is not None and r["l_cm"] < SIDE_WALL_CM,
            "right": r["r_cm"] is not None and r["r_cm"] < SIDE_WALL_CM,
        }
        real = {"front": truth[0] == "1", "left": truth[1] == "1", "right": truth[2] == "1"}
        trial = {"truth": real, "verdict": verdict, "note": note, **{k: r[k] for k in
                 ("tof_mm", "tof_valid", "n", "l_adc", "l_cm", "r_adc", "r_cm", "yaw")}}
        trials.append(trial)
        t.record("trials", trials)
        mark = lambda d: "ok" if verdict[d] == real[d] else "WRONG"
        t.say(f"  หน้า ToF {r['tof_mm']} mm -> {mark('front')} | ซ้าย {r['l_cm']} cm (adc {r['l_adc']}) -> "
              f"{mark('left')} | ขวา {r['r_cm']} cm (adc {r['r_adc']}) -> {mark('right')}")

    # สรุปเกณฑ์: ค่าตอนมีกำแพง vs ตอนเปิด แยกกันได้ไหม ควรตั้งเกณฑ์ที่เท่าไหร่
    analysis = {}
    for d, key in (("front", "tof_mm"), ("left", "l_adc"), ("right", "r_adc")):
        wall = [tr[key] for tr in trials if tr["truth"][d] and tr[key] is not None]
        open_ = [tr[key] for tr in trials if not tr["truth"][d] and tr[key] is not None]
        open_none = sum(1 for tr in trials if not tr["truth"][d] and tr[key] is None)
        info = {"wall": wall, "open": open_, "open_out_of_range": open_none,
                "correct": sum(tr["verdict"][d] == tr["truth"][d] for tr in trials), "total": len(trials)}
        if wall and open_:
            if key == "tof_mm":  # ToF: กำแพง = ค่าน้อย
                gap = (max(wall), min(open_))
            else:                # ADC: กำแพง = ค่ามาก
                gap = (min(wall), max(open_))
            info["separable"] = gap[0] < gap[1] if key == "tof_mm" else gap[0] > gap[1]
            info["suggested_threshold"] = round((gap[0] + gap[1]) / 2, 1) if info["separable"] else None
        analysis[d] = info
    t.record("analysis", analysis)
    for d, info in analysis.items():
        t.say(f"  {d}: ถูก {info['correct']}/{info['total']}  เกณฑ์ที่แนะนำ {info.get('suggested_threshold')}"
              f"  (แยกได้: {info.get('separable')})")


def step_gimbal(t: Tester):
    t.ask("\nหัวจะหมุนไปหลายมุม (รวมหันหลัง 180) เอาของออกจากรอบหัว แล้วกด Enter")
    rows = []
    for pitch, yaw in GIMBAL_POSES:
        done, dt = t.gimbal_to(pitch, yaw)
        time.sleep(ms.GIMBAL_SETTLE_S)
        r = t.reading(0.5)
        row = {"cmd_pitch": pitch, "cmd_yaw": yaw, "completed": done, "seconds": round(dt, 2),
               "fb_pitch": r["g_pitch"], "fb_yaw": r["g_yaw"],
               "err_pitch": None if r["g_pitch"] is None else round(r["g_pitch"] - pitch, 2),
               "err_yaw": None if r["g_yaw"] is None else round(ms.normalize_angle(r["g_yaw"] - yaw), 2)}
        rows.append(row)
        t.say(f"  สั่ง p{pitch} y{yaw} -> รายงาน p{r['g_pitch']} y{r['g_yaw']}  "
              f"(ถึง: {done}, {dt:.2f} s)")
    t.gimbal_to(0, 0)
    t.record("poses", rows)


def step_camera(t: Tester):
    pitch = ms.TARGET_SCAN_PITCH
    t.say(f"\nกล้องจะก้ม {pitch} องศาแบบตอนหาเป้าจริง วางเป้าในช่องถัดไป (กลางช่อง) หรือที่ระยะอื่นก็ได้"
          f"\nค่าที่โปรแกรมหลักใช้: กล้องสูง {ms.CAMERA_HEIGHT_M} ม. เป้าสูง {ms.TARGET_HEIGHT_M} ม."
          f" กล้องอยู่หน้ากลางหุ่น {ms.CAMERA_FORWARD_M} ม. นับเป็นช่องติดกันถ้าระยะ {ms.TARGET_CELL_RANGE} ม.")
    trials = []
    n = 0
    while True:
        ans = t.ask("\nระยะจริงจากกลางหุ่นถึงเป้า กี่ซม.? (Enter = 60, n = ไม่มีเป้า, q = จบ):").lower()
        if ans == "q":
            break
        try:
            truth_cm = None if ans == "n" else float(ans) if ans else 60.0
        except ValueError:
            print("  ใส่ตัวเลข, n หรือ q")
            continue
        color_truth = "" if truth_cm is None else t.ask("  เป้าสีอะไร (red/green/blue/yellow) Enter = ข้าม:").lower()
        t.gimbal_to(pitch, 0)
        time.sleep(ms.GIMBAL_SETTLE_S)
        fb_pitch = t.reading(0.3)["g_pitch"]
        est_cmd, est_fb, lateral, colors, accepted = [], [], [], Counter(), Counter()
        last_frame, last_fd, last_found = None, None, []
        got = 0
        t0 = time.monotonic()
        for _ in range(CAMERA_FRAMES):
            frame = gs.read_frame(t.ep.camera)
            if frame is None:
                continue
            got += 1
            fd = gs.prepare_frame(frame)
            found = gs.detect_all_colors(fd, t.color_ranges, t.blind_zones)
            last_frame, last_fd, last_found = frame, fd, found
            for target in found:
                colors[target.color] += 1
            adj = ms.MazeMission.adjacent_target(fd, found)  # ตัดสินแบบเดียวกับโปรแกรมหลักทุกประการ
            if adj is not None:
                accepted[adj] += 1
            if found:
                biggest = max(found, key=lambda x: x.area)
                w, h = fd.size
                bearing, dy = gs.pixel_to_degrees(biggest.cx, biggest.cy, w, h)
                for p, bucket in ((pitch, est_cmd), (fb_pitch, est_fb)):
                    if p is None:
                        continue
                    depression = -(p + dy)
                    if depression > 1.0:
                        ground = (ms.CAMERA_HEIGHT_M - ms.TARGET_HEIGHT_M) / math.tan(math.radians(depression))
                        bucket.append(ground + ms.CAMERA_FORWARD_M)
                        if p == pitch:
                            lateral.append(ground * math.tan(math.radians(bearing)))
        fps = got / max(1e-6, time.monotonic() - t0)
        n += 1
        if last_frame is not None:
            cv2.imwrite(str(t.session.dir / f"cam_{n:02d}_raw.jpg"), last_frame)
            annotated = last_frame.copy()
            gs.draw_targets(annotated, last_found, annotated.shape[1] / last_fd.size[0])
            cv2.imwrite(str(t.session.dir / f"cam_{n:02d}_det.jpg"), annotated)
        trial = {
            "trial": n, "truth_cm": truth_cm, "truth_color": color_truth, "frames": got, "fps": round(fps, 1),
            "cmd_pitch": pitch, "fb_pitch": fb_pitch, "colors_seen": dict(colors),
            "accepted_as_adjacent": dict(accepted),
            "est_cm_cmd_pitch": None if not est_cmd else round(median(est_cmd) * 100, 1),
            "est_cm_fb_pitch": None if not est_fb else round(median(est_fb) * 100, 1),
            "lateral_cm": None if not lateral else round(median(lateral) * 100, 1),
        }
        trials.append(trial)
        t.record("trials", trials)
        t.say(f"  {got} เฟรม ({fps:.1f} fps) | เห็นสี {dict(colors)} | โปรแกรมหลักนับเป็นเป้าช่องติดกัน {dict(accepted)}"
              f"\n  ระยะที่คำนวณ {trial['est_cm_cmd_pitch']} ซม. (ใช้มุมที่สั่ง) / {trial['est_cm_fb_pitch']} ซม."
              f" (ใช้มุมจริงจาก gimbal)  จริง {truth_cm}  | เยื้อง {trial['lateral_cm']} ซม."
              f"\n  ภาพ: cam_{n:02d}_raw.jpg / cam_{n:02d}_det.jpg")
    t.gimbal_to(0, 0)


def step_turn(t: Tester):
    t.confirm_motion("หมุนอยู่กับที่ 90 องศา 8 ครั้ง (ขวา 4 ซ้าย 4) ต้องมีที่ว่างรอบรถ ~30 ซม.")
    t.gimbal_to(0, 0)
    yaw0 = t.reading(0.5)["yaw"]
    rows = []
    for i, (z, expect) in enumerate([(-90, 90.0)] * 4 + [(90, -90.0)] * 4, 1):  # chassis.move: z บวก = หมุนซ้าย
        before = t.reading(0.3)["yaw"]
        t0 = time.monotonic()
        action = gs.safe_call("chassis turn", t.ep.chassis.move, x=0, y=0, z=z, z_speed=ms.TURN_SPEED)
        done = bool(action is not None and gs.safe_call("turn wait", action.wait_for_completed,
                                                         timeout=ms.MOVE_TIMEOUT_S))
        dt = time.monotonic() - t0
        time.sleep(0.5)
        after = t.reading(0.3)["yaw"]
        delta = ms.normalize_angle(after - before)
        rows.append({"turn": i, "cmd_z": z, "expect_yaw_delta": expect, "yaw_before": before, "yaw_after": after,
                     "yaw_delta": round(delta, 2), "error": round(delta - expect, 2),
                     "completed": done, "seconds": round(dt, 2)})
        t.say(f"  ครั้งที่ {i}: yaw เปลี่ยน {delta:+.1f} (ควร {expect:+.0f}) ต่าง {delta - expect:+.1f}  {dt:.2f} s")
    yaw_end = t.reading(0.5)["yaw"]
    drift = ms.normalize_angle(yaw_end - yaw0)
    errs = [r["error"] for r in rows]
    t.record("turns", rows)
    t.record("result", {"mean_abs_error": round(statistics.mean(abs(e) for e in errs), 2),
                        "max_abs_error": round(max(abs(e) for e in errs), 2), "net_drift_after_8": round(drift, 2)})
    t.say(f"  คลาดเฉลี่ย {statistics.mean(abs(e) for e in errs):.2f} องศา | สูงสุด {max(abs(e) for e in errs):.2f}"
          f" | หมุนครบ 8 ครั้งแล้วหัวคลาดจากเดิม {drift:+.2f} องศา")


def drive_one_cell(t: Tester, sign: int, mode: str, distance_m: float = ms.CELL_M) -> dict:
    """ เดินตรงแบบโปรแกรมหลัก (IMU คุมหัวตรง + ToF กันชนเฉพาะตอนเดินหน้า)
    mode = "time": หยุดเมื่อครบเวลา distance_m / BASE_SPEED (แบบที่โปรแกรมหลักใช้ตอนนี้)
    mode = "odom": หยุดเมื่อ odometry นับได้ครบ distance_m
    """
    start = t.reading(0.4)
    target_yaw, pos0 = start["yaw"], start["pos"]
    duration = distance_m / ms.BASE_SPEED
    last_err, max_err, reason = 0.0, 0.0, "time"
    t0 = time.monotonic()
    try:
        while True:
            s = t.sensors.snapshot()
            elapsed = time.monotonic() - t0
            travelled = math.hypot(s["pos"][0] - pos0[0], s["pos"][1] - pos0[1])
            tof = t.tof_mm(s)
            if mode == "time" and elapsed >= duration:
                break
            if mode == "odom" and travelled >= distance_m:
                reason = "odom"
                break
            if elapsed >= duration * 2:
                reason = "timeout"
                break
            if sign > 0 and tof is not None and tof <= ms.BUMPER_MM:
                reason = "bumper"
                break
            err = ms.normalize_angle(target_yaw - s["att"][0])
            max_err = max(max_err, abs(err))
            z = ms.KP_YAW * err + ms.KD_YAW * (err - last_err)
            last_err = err
            t.ep.chassis.drive_speed(x=sign * ms.BASE_SPEED, y=0, z=z)
            time.sleep(0.02)
    finally:
        t.stop_motion()
    time.sleep(0.5)
    end = t.reading(0.4)
    odom = math.hypot(end["pos"][0] - pos0[0], end["pos"][1] - pos0[1])
    tof_delta = None if start["tof_mm"] is None or end["tof_mm"] is None else start["tof_mm"] - end["tof_mm"]
    return {"mode": mode, "direction": "forward" if sign > 0 else "back", "stop_reason": reason,
            "seconds": round(elapsed, 2), "odom_cm": round(odom * 100, 1),
            "tof_before_mm": start["tof_mm"], "tof_after_mm": end["tof_mm"],
            "tof_delta_mm": tof_delta, "max_yaw_err": round(max_err, 2),
            "final_yaw_err": round(ms.normalize_angle(end["yaw"] - target_yaw), 2)}


def step_drive(t: Tester):
    t.confirm_motion(f"เดินหน้า {ms.CELL_M * 100:.0f} ซม. แล้วถอยกลับ ทำ 2 แบบ (จับเวลา / นับ odometry)\n"
                     "  ต้องมีที่ว่างข้างหน้า >= 80 ซม. แนะนำวางกำแพงไว้ข้างหน้า ~100 ซม. ให้ ToF วัดระยะเทียบได้\n"
                     "  ทำเครื่องหมายจุดเริ่มที่พื้นไว้ก่อน จะได้วัดตลับเมตร")
    t.gimbal_to(0, 0)
    rows = []
    for mode in ("time", "odom"):
        r = drive_one_cell(t, 1, mode)
        t.say(f"  [{mode}] หยุดเพราะ {r['stop_reason']} | {r['seconds']} s | odometry {r['odom_cm']} ซม."
              f" | ToF เปลี่ยน {r['tof_delta_mm']} mm | หัวเบี้ยวสูงสุด {r['max_yaw_err']} องศา")
        r["tape_cm"] = t.ask_float("  วัดตลับเมตรจากเครื่องหมายเริ่ม ได้กี่ซม.? (Enter = ข้าม):", None)
        rows.append(r)
        t.record("runs", rows)
        # ถอยกลับเท่าที่ odometry บอกว่าเดินไป (ถ้ากันชนเบรกกลางทาง จะไม่ถอยเลยจุดเริ่ม)
        back = drive_one_cell(t, -1, "odom", r["odom_cm"] / 100.0)
        back["mode"] = "return"
        rows.append(back)
        t.record("runs", rows)
        t.say(f"  ถอยกลับ {back['odom_cm']} ซม. แล้ว")
        time.sleep(0.5)


STEPS = {
    "ports": ("หาช่องเซ็นเซอร์ (ToF index / Sharp board-port)", step_ports),
    "live": ("เก็บค่านิ่ง: ความถี่ สัญญาณรบกวน ค่าหลุด", step_live),
    "dist": ("สอบเทียบระยะ Sharp + ToF", step_dist),
    "cell": ("ทดสอบกำแพง/ทางเปิดในสนามจริง", step_cell),
    "gimbal": ("มุม gimbal สั่ง vs รายงาน", step_gimbal),
    "camera": ("กล้อง: หาเป้า + ระยะเป้า", step_camera),
    "turn": ("[รถขยับ] เลี้ยว 90 องศา x8", step_turn),
    "drive": ("[รถขยับ] เดิน 1 ช่อง จับเวลา vs odometry", step_drive),
}


# ==========================================
# 7. โปรแกรมหลัก
# ==========================================
def run_step(t: Tester, key: str):
    title, fn = STEPS[key]
    t.step = key
    t.session.event(f"=== STEP {key}: {title} ===")
    try:
        fn(t)
        t.session.summary["steps"].setdefault(key, {})["status"] = "done"
    except StepSkipped:
        t.session.event(f"step {key} skipped")
        t.session.summary["steps"].setdefault(key, {})["status"] = "skipped"
    except KeyboardInterrupt:
        t.stop_motion()
        print()
        t.session.event(f"step {key} aborted (Ctrl+C)")
        t.session.summary["steps"].setdefault(key, {})["status"] = "aborted"
    except Exception as e:
        t.stop_motion()
        t.session.event(f"step {key} ERROR {e!r}\n{traceback.format_exc()}")
        t.session.summary["steps"].setdefault(key, {})["status"] = f"error: {e!r}"
    finally:
        t.step = "idle"
        t.session.save_summary()


def menu(t: Tester):
    keys = list(STEPS)
    while True:
        print("\n" + "=" * 60)
        for i, k in enumerate(keys, 1):
            status = t.session.summary["steps"].get(k, {}).get("status", "")
            print(f" {i}. {k:<7} {STEPS[k][0]}  {('[' + status + ']') if status else ''}")
        print(f" a. ทุกข้อตามลำดับ      q. ออก       log: {t.session.dir}")
        try:
            ans = input("เลือก: ").strip().lower()
        except KeyboardInterrupt:
            print()
            return
        if ans == "q":
            return
        if ans == "a":
            for k in keys:
                run_step(t, k)
        elif ans.isdigit() and 1 <= int(ans) <= len(keys):
            run_step(t, keys[int(ans) - 1])
        elif ans in STEPS:
            run_step(t, ans)


def main():
    parser = argparse.ArgumentParser(description="RoboMaster EP sensor test for the final maze run (logs to logs/)")
    parser.add_argument("--step", choices=list(STEPS), help="รันข้อเดียวแล้วออก")
    parser.add_argument("--all", action="store_true", help="ไล่ทุกข้อตามลำดับแล้วออก")
    args = parser.parse_args()

    session = Session()
    session.event(f"log dir: {session.dir}")
    print("[INFO] Connecting to RoboMaster (AP)...")
    ep = robot.Robot()
    if not ep.initialize(conn_type=gs.CONN_TYPE):
        session.event("cannot connect to the robot - check Wi-Fi")
        session.close()
        return
    tester = Tester(ep, session)
    recorder = Recorder(tester)
    try:
        gs.safe_call("set robot mode", ep.set_robot_mode, mode=robot.FREE)  # หันหัวแยกจากตัวรถได้
        tester.sensors.start()
        gs.safe_call("start video", ep.camera.start_video_stream, display=False, resolution=gs.STREAM_RESOLUTION)
        tester.gimbal_to(0, 0)
        time.sleep(1.5)  # รอค่าชุดแรกจากทุก stream
        tester.sensors.ensure_adapter(session.event)
        session.summary.update({"subscribed": tester.sensors.subscribed, "sensor_map_at_start": dict(tester.map),
                                "adapter_mode": tester.sensors.adapter_mode,
                                "adapter_boards": tester.sensors.adapter_boards,
                                "program_constants": {
                                    "WALL_TOF_MM": ms.WALL_TOF_MM, "BUMPER_MM": ms.BUMPER_MM,
                                    "SIDE_WALL_CM": SIDE_WALL_CM, "CELL_M": ms.CELL_M,
                                    "BASE_SPEED": ms.BASE_SPEED, "TURN_SPEED": ms.TURN_SPEED,
                                    "CAMERA_HEIGHT_M": ms.CAMERA_HEIGHT_M, "TARGET_HEIGHT_M": ms.TARGET_HEIGHT_M,
                                    "CAMERA_FORWARD_M": ms.CAMERA_FORWARD_M,
                                    "TARGET_SCAN_PITCH": ms.TARGET_SCAN_PITCH}})
        session.save_summary()
        session.event(f"subscribed: {tester.sensors.subscribed} | adapter: {tester.sensors.adapter_mode} "
                      f"| counts: {tester.sensors.snapshot()['counts']}")
        recorder.start()
        if args.step:
            run_step(tester, args.step)
        elif args.all:
            for key in STEPS:
                run_step(tester, key)
        else:
            menu(tester)
    finally:
        tester.stop_motion()
        recorder.stop()
        tester.gimbal_to(0, 0)
        tester.sensors.stop()
        gs.safe_call("stop video", ep.camera.stop_video_stream)
        gs.safe_call("robot close", ep.close)
        session.event("closed")
        session.close()
        print(f"\nlog อยู่ที่: {session.dir}")


if __name__ == "__main__":
    main()
