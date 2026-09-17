"""
อ่านค่า IR 2 ตัวผ่าน Sensor Adaptor (Class work ข้อ 3)

ทำไมต้องมี thread แยก:
get_adc() เป็นการคุยแบบ request/response กับบอร์ด แต่ละครั้งกินเวลาหลายสิบ
มิลลิวินาที ถ้าเรียกตรง ๆ ใน control loop ที่ต้องวิ่ง 20 Hz จะวิ่งไม่ทัน
จึงให้ thread เบื้องหลัง poll ค่าไว้ตลอด แล้ว control loop มาหยิบค่าล่าสุดไป
"""
import math
import statistics
import threading
import time
from collections import deque
from dataclasses import dataclass

import config


@dataclass
class IRReading:
    """ค่าที่อ่านได้จาก IR คู่หนึ่ง ณ เวลาหนึ่ง"""
    t: float
    adc_front: float
    adc_rear: float
    front_cm: float          # None ถ้าอยู่นอกช่วงที่เชื่อถือได้
    rear_cm: float
    mean_cm: float           # ระยะเฉลี่ยของสองตัว ใช้เป็นระยะห่างกำแพง
    angle_deg: float         # มุมเอียงเทียบกำแพง บวก = หัวหุ่นเบนออกจากกำแพง
    seq: int = 0             # นับรอบที่ poll ได้ค่าใหม่ ใช้แยกว่าค่าซ้ำหรือค่าใหม่

    def ok(self) -> bool:
        return self.front_cm is not None and self.rear_cm is not None


def adc_to_cm(adc, which: str):
    """
    แปลง ADC ของ Sharp IR เป็นระยะเซนติเมตร ตามสมการ cm = A * adc^B

    คืน None เมื่อค่าอยู่นอกช่วงที่เชื่อถือได้ เพราะที่ระยะใกล้กว่าประมาณ 10 ซม.
    เส้นโค้งของ Sharp IR จะย้อนกลับ ทำให้ค่าใกล้มากกับค่าไกลมากแยกกันไม่ออก
    """
    if adc is None or adc <= 0:
        return None
    A, B = config.IR_CALIB[which]
    cm = A * (adc ** B)
    if cm < config.IR_MIN_VALID_CM or cm > config.IR_MAX_VALID_CM:
        return None
    return cm


def adapter_index(board_id: int, port: int) -> int:
    """
    แปลง (board_id, port) เป็นตำแหน่งใน ad_value[12] ที่ sub_adapter ส่งมา

    stream ส่งค่ามาเป็นชุดเดียว 6 บอร์ด บอร์ดละ 2 พอร์ต เรียงติดกัน
    บอร์ดที่ 1 อยู่ index 0-1, บอร์ดที่ 2 อยู่ 2-3 ไปเรื่อย ๆ
    (ดู AdapterSubject.decode ใน src/robomaster/sensor.py)
    """
    return (board_id - 1) * 2 + (port - 1)


class IRPair:
    """IR สองตัวที่ติดด้านข้างฝั่งเดียวกัน ตัวหน้าและตัวหลัง"""

    def __init__(self, ep_robot):
        self._robot = ep_robot
        self._lock = threading.Lock()
        self._running = False
        self._thread = None
        self._streaming = False
        self._buf = {
            "front": deque(maxlen=config.MEDIAN_WINDOW),
            "rear":  deque(maxlen=config.MEDIAN_WINDOW),
        }
        self._seq = 0
        self._ports = {"front": config.IR_FRONT, "rear": config.IR_REAR}
        self._index = {name: adapter_index(*hw) for name, hw in self._ports.items()}

    # -------------------------------------------------- รับค่าจาก stream
    def _on_adapter(self, sub_info):
        """callback ของ sub_adapter — ได้ค่าทุกช่องพร้อมกันในครั้งเดียว"""
        _io, ad = sub_info
        with self._lock:
            for name, idx in self._index.items():
                if idx < len(ad):
                    v = ad[idx]
                    if v is not None and v >= config.ADC_MIN_VALID:
                        self._buf[name].append(v)
            self._seq += 1

    def start(self):
        self._running = True

        if config.USE_ADAPTER_STREAM:
            try:
                self._robot.sensor_adaptor.sub_adapter(
                    freq=config.ADAPTER_FREQ_HZ, callback=self._on_adapter)
                self._streaming = True
            except Exception as e:
                print(f"[ir] เปิด stream ไม่สำเร็จ ({e}) จะถอยไปใช้ get_adc แทน")

        if not self._streaming:
            self._thread = threading.Thread(target=self._poll_loop, daemon=True)
            self._thread.start()

        # รอให้มีค่าอย่างน้อยหนึ่งชุดก่อนคืนการควบคุม ไม่งั้น read() ครั้งแรกได้ None
        for _ in range(50):
            time.sleep(0.05)
            with self._lock:
                if self._buf["front"] and self._buf["rear"]:
                    mode = "stream sub_adapter" if self._streaming else "poll get_adc"
                    print(f"[ir] อ่านค่าได้แล้ว (โหมด {mode})")
                    return
        print("[ir] เตือน: ยังอ่านค่าจาก IR ไม่ได้ ตรวจสายและเลข board id/port")

    def stop(self):
        self._running = False
        if self._streaming:
            try:
                self._robot.sensor_adaptor.unsub_adapter()
            except Exception:
                pass
            self._streaming = False
        if self._thread:
            self._thread.join(timeout=2)

    def _poll_loop(self):
        while self._running:
            got = False
            for name, (board_id, port) in self._ports.items():
                try:
                    v = self._robot.sensor_adaptor.get_adc(id=board_id, port=port)
                except Exception:
                    v = None
                if v is not None and v >= config.ADC_MIN_VALID:
                    with self._lock:
                        self._buf[name].append(v)
                    got = True
            if got:
                # นับเฉพาะรอบที่ได้ค่าใหม่จริง ตัวคุมจะได้รู้ว่าควรคำนวณใหม่เมื่อไร
                with self._lock:
                    self._seq += 1
            time.sleep(config.POLL_INTERVAL)

    def _median(self, name):
        with self._lock:
            buf = list(self._buf[name])
        return statistics.median(buf) if buf else None

    def read(self) -> IRReading:
        adc_f = self._median("front")
        adc_r = self._median("rear")
        with self._lock:
            seq = self._seq
        front_cm = adc_to_cm(adc_f, "front")
        rear_cm = adc_to_cm(adc_r, "rear")

        got = [v for v in (front_cm, rear_cm) if v is not None]
        mean_cm = sum(got) / len(got) if got else None

        if front_cm is not None and rear_cm is not None:
            angle_deg = math.degrees(
                math.atan2(front_cm - rear_cm, config.IR_BASELINE_CM))
        else:
            angle_deg = None

        return IRReading(
            t=time.time(),
            adc_front=adc_f, adc_rear=adc_r,
            front_cm=front_cm, rear_cm=rear_cm,
            mean_cm=mean_cm, angle_deg=angle_deg, seq=seq,
        )
