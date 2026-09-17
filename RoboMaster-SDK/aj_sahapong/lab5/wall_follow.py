"""
Class work ข้อ 5, 6 และ Checkpoint 4: Robot Controlling (IR + ToF)

โหมดการทำงาน
------------
straight : ใช้ IR คู่ด้านข้างคุมทิศทางให้หุ่นเดินตรงขนานกำแพง (ข้อ 5)
           คุมเฉพาะมุม ไม่แก้ระยะห่าง
follow   : เลาะกำแพงโดยรักษาระยะห่าง TARGET_WALL_CM และไม่ชน (ข้อ 6)
           พร้อมหยุดเมื่อ ToF ด้านหน้าเห็นกำแพงใกล้กว่า STOP_FRONT_CM (Checkpoint 4)

รัน:
    python aj_sahapong/lab5/wall_follow.py follow
    python aj_sahapong/lab5/wall_follow.py straight
    python aj_sahapong/lab5/wall_follow.py follow 30      จำกัดเวลา 30 วินาที

หลักการควบคุม
-------------
IR สองตัวติดด้านข้างฝั่งเดียวกัน ให้ข้อมูลสองอย่างพร้อมกัน
  ระยะห่างกำแพง = ค่าเฉลี่ยของ IR ทั้งสอง    -> PID ตัวที่ 1 -> ความเร็วด้านข้าง y
  มุมเอียงเทียบกำแพง = atan2(หน้า - หลัง, baseline) -> PID ตัวที่ 2 -> ความเร็วหมุน z
เพราะหุ่นเป็นล้อ mecanum จึงสไลด์เข้าออกจากกำแพงได้โดยไม่ต้องหมุนตัว
ทำให้แยกการคุมสองเรื่องนี้ออกจากกันได้สะอาด

เครื่องหมายของ y และ z
  z เป็นบวก = หมุนซ้าย (ทวนเข็ม)  ตามที่ใช้ในแล็บก่อนหน้า
  y เป็นบวก = สไลด์ไปทางขวา
ค่า SIDE ด้านล่างกลับเครื่องหมายให้อัตโนมัติเมื่อกำแพงอยู่ฝั่งซ้าย
"""
import csv
import os
import sys
import threading
import time

from robomaster import robot

import config
from ir_sensor import IRPair
from pid import PIDController


class FrontToF:
    """ToF ด้านหน้า — subscribe ไว้แล้วเก็บค่าล่าสุดให้ control loop มาหยิบ"""

    def __init__(self, ep_robot):
        self._robot = ep_robot
        self._lock = threading.Lock()
        self._mm = 0.0

    def _on_distance(self, d):
        with self._lock:
            self._mm = d[config.TOF_INDEX]

    def start(self):
        self._robot.sensor.sub_distance(freq=20, callback=self._on_distance)
        time.sleep(0.5)

    def stop(self):
        try:
            self._robot.sensor.unsub_distance()
        except Exception:
            pass

    def read_cm(self):
        """คืนระยะเป็นเซนติเมตร หรือ None ถ้ายังไม่มีค่าที่ใช้ได้"""
        with self._lock:
            mm = self._mm
        return mm / 10.0 if mm and mm > 0 else None


def run(mode: str, max_seconds: float):
    # กำแพงอยู่ฝั่งขวาให้ SIDE = +1, ฝั่งซ้ายให้ -1 เพื่อกลับเครื่องหมายของ y และ z
    side = 1.0 if config.WALL_SIDE == "right" else -1.0

    os.makedirs(config.LOG_DIR, exist_ok=True)
    log_path = os.path.join(config.LOG_DIR, f"{mode}_run.csv")

    ep_robot = robot.Robot()
    print(f"กำลังเชื่อมต่อหุ่นยนต์ (conn_type={config.CONN_TYPE}) ...")
    ep_robot.initialize(conn_type=config.CONN_TYPE)
    ep_chassis = ep_robot.chassis

    ir = IRPair(ep_robot)
    tof = FrontToF(ep_robot)
    ir.start()
    tof.start()

    if not config.IR_CALIBRATED:
        print("[เตือน] config.IR_CALIBRATED ยังเป็น False — ค่าระยะเป็นค่าประมาณ")
        print("        ควรรัน calibrate_ir.py ก่อน แล้ววางค่า IR_CALIB ที่ได้ลง config.py")

    pid_dist = PIDController(**config.PID_DISTANCE)
    pid_angle = PIDController(**config.PID_ANGLE)

    dt_target = 1.0 / config.CONTROL_HZ
    t0 = time.time()
    t_prev = t0
    stop_reason = "ครบเวลาที่กำหนด"

    print(f"\nโหมด: {mode} | กำแพงอยู่ฝั่ง {config.WALL_SIDE}"
          f" | เป้าหมายระยะห่าง {config.TARGET_WALL_CM} ซม."
          f" | หยุดเมื่อ ToF < {config.STOP_FRONT_CM} ซม.")
    print("กด Ctrl+C เพื่อหยุดฉุกเฉิน\n")

    try:
        with open(log_path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["time_s", "adc_front", "adc_rear", "front_cm", "rear_cm",
                        "mean_cm", "angle_deg", "tof_cm", "vx", "vy", "vz"])

            while True:
                now = time.time()
                dt = now - t_prev
                if dt < dt_target:
                    time.sleep(dt_target - dt)
                    now = time.time()
                    dt = now - t_prev
                t_prev = now
                elapsed = now - t0

                if max_seconds and elapsed > max_seconds:
                    break

                r = ir.read()
                tof_cm = tof.read_cm()

                # เงื่อนไขหยุดของ Checkpoint 4: กำแพงด้านหน้าเข้ามาใกล้ถึงเกณฑ์
                if tof_cm is not None and tof_cm <= config.STOP_FRONT_CM:
                    stop_reason = f"ToF ด้านหน้าวัดได้ {tof_cm:.1f} ซม."
                    break

                vx = config.FORWARD_SPEED
                vy = 0.0
                vz = 0.0

                if r.angle_deg is not None:
                    # อยากให้มุมเอียงเป็นศูนย์ = ขนานกำแพง
                    out_angle = pid_angle.compute(0.0, r.angle_deg, dt)
                    vz = side * out_angle
                else:
                    pid_angle.reset()

                if mode == "follow" and r.mean_cm is not None:
                    out_dist = pid_dist.compute(config.TARGET_WALL_CM, r.mean_cm, dt)
                    # ไกลกว่าเป้าหมาย -> out เป็นลบ -> ต้องสไลด์เข้าหากำแพง
                    vy = -side * out_dist
                elif mode == "follow":
                    # มองไม่เห็นกำแพงชั่วคราว (ช่องว่างหรือค่าหลุดช่วง) วิ่งตรงไว้ก่อน
                    pid_dist.reset()

                ep_chassis.drive_speed(x=vx, y=vy, z=vz, timeout=0.5)

                def fmt(v, w_=6, p=1):
                    return f"{v:{w_}.{p}f}" if v is not None else "   ---"

                print(f"[{elapsed:6.2f}s] ระยะกำแพง={fmt(r.mean_cm)} ซม."
                      f" มุม={fmt(r.angle_deg)} องศา"
                      f" ToF={fmt(tof_cm)} ซม."
                      f" | vy={vy:+.3f} vz={vz:+.1f}")

                w.writerow([f"{elapsed:.3f}", r.adc_front, r.adc_rear,
                            r.front_cm, r.rear_cm, r.mean_cm, r.angle_deg,
                            tof_cm, vx, vy, vz])
                f.flush()

    except KeyboardInterrupt:
        stop_reason = "ผู้ใช้กด Ctrl+C"
    finally:
        # สั่งหยุดก่อนเสมอ แล้วค่อยปิดการ subscribe
        try:
            ep_chassis.drive_speed(x=0, y=0, z=0)
            time.sleep(0.3)
        except Exception:
            pass
        ir.stop()
        tof.stop()
        ep_robot.close()
        print(f"\nหยุดแล้ว เหตุผล: {stop_reason}")
        print(f"บันทึกข้อมูลไว้ที่ {log_path}")


if __name__ == "__main__":
    mode_arg = sys.argv[1] if len(sys.argv) > 1 else "follow"
    if mode_arg not in ("follow", "straight"):
        print("ใช้: python aj_sahapong/lab5/wall_follow.py [follow|straight] [วินาที]")
        sys.exit(1)
    seconds = float(sys.argv[2]) if len(sys.argv) > 2 else 120.0
    run(mode_arg, seconds)
