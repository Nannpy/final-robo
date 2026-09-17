"""
โหมดลงสนามเร็ว — รันได้เลยโดยไม่ต้องสอบเทียบก่อน แล้วค่อยจูนหน้างาน

ต่างจาก wall_follow.py ตรงไหน
-----------------------------
wall_follow.py ต้องมีสมการสอบเทียบ (cm = A * adc^B) ก่อนจึงจะคุมระยะเป็น
เซนติเมตรได้ ส่วนไฟล์นี้ทำงานบนค่า ADC ดิบล้วน ๆ โดยใช้วิธี "สอนจุดสมดุล":
เอาหุ่นไปวางที่ระยะเป้าหมายจริง (10 ซม.) แล้วจำค่า ADC ตอนนั้นไว้เป็นเป้า
การคุมคือดันค่า ADC ให้กลับมาเท่าค่าที่จำไว้ ไม่ต้องรู้ว่าเป็นกี่เซนติเมตร
วิธีนี้หักล้างความต่างระหว่างเซนเซอร์สองตัวไปในตัว และใช้เวลาแค่ครึ่งนาที

นอกจากนี้ยังทดสอบเครื่องหมายของแกนให้อัตโนมัติ (ขยับนิดเดียวแล้วดูว่า ADC
เปลี่ยนไปทางไหน) จึงไม่ต้องเดาว่า y บวกคือซ้ายหรือขวา หรือ z บวกคือหมุนทางไหน

รัน:
    python aj_sahapong/lab5/field_run.py

ระหว่างวิ่ง พิมพ์คำสั่งในเทอร์มินัลแล้วกด Enter เพื่อจูนสด ๆ ได้ทันที
    d 0.003     ตั้ง kp ของการคุมระยะ
    a 0.6       ตั้ง kp ของการคุมมุม
    v 0.15      ตั้งความเร็วเดินหน้า (m/s)
    p           หยุดชั่วคราว / วิ่งต่อ
    w           บันทึกค่าที่จูนไว้ลงไฟล์
    q           จบการทำงาน
"""
import json
import os
import statistics
import sys
import threading
import time

from robomaster import robot

import config
from ir_sensor import IRPair
from pid import FilteredPID
from wall_follow import FrontToF

# ค่าเริ่มต้นของ gain บนสเกล ADC ดิบ
# ที่ระยะราว 10 ซม. Sharp IR เปลี่ยนราว 16 adc ต่อ 1 ซม.
# kp ระยะ 0.002 จึงให้ความเร็วด้านข้างราว 0.03 m/s ต่อความคลาดเคลื่อน 1 ซม.
DEFAULT_GAINS = {
    "kp_dist": 0.002,     # m/s ต่อ 1 adc
    "ki_dist": 0.0,       # แก้ระยะที่ค้างตอนกำแพงเฉียง (เริ่มที่ 0.0008)
    "kd_dist": 0.0002,    # หน่วงการสไลด์เข้าออก
    "kp_angle": 0.30,     # deg/s ต่อ 1 adc ของผลต่างหน้า-หลัง
    "ki_angle": 0.0,
    "kd_angle": 0.03,     # ช่วยหน่วงอาการหัวส่าย แต่ตัวหลักคือลด kp กับลดดีเลย์
    "speed": 0.20,        # m/s เดินหน้า
    "d_alpha": 0.20,      # ตัวกรองพจน์ D ยิ่งน้อยยิ่งกรองหนัก
}

# deadband หน่วย adc — เล็กกว่านี้ถือว่าเข้าเป้าแล้ว ปล่อยนิ่ง ไม่ต้องแก้
DEADBAND_DIST = 6.0
DEADBAND_SKEW = 8.0
# จำกัดอัตราการเปลี่ยนคำสั่งความเร็ว กันกระโดดสุดขั้วในรอบเดียว
SLEW_VY = 0.8      # m/s ต่อวินาที
SLEW_VZ = 180.0    # deg/s ต่อวินาที
GAINS_PATH = os.path.join(config.LOG_DIR, "field_gains.json")

VY_LIMIT = 0.25
VZ_LIMIT = 60.0

# ---------- ค่ากันหุ่นหนี ----------
# ต่ำกว่านี้ = ไม่เห็นกำแพง (Sharp IR ยิ่งไกลค่ายิ่งต่ำ)
ADC_NO_WALL = 80
# สูงกว่านี้ = ใกล้จนอยู่ในช่วงที่เส้นโค้งย้อนกลับ เชื่อค่าไม่ได้
ADC_TOO_CLOSE = 680
# จำกัดขนาด error ที่ยอมป้อนเข้า PID กันค่าหลุดตัวเดียวเหวี่ยงหุ่น
ERR_DIST_LIMIT = 120.0
ERR_SKEW_LIMIT = 150.0
# สั่งความเร็วชนเพดานติดต่อกันนานกว่านี้ = ระบบแกว่งจนคุมไม่อยู่ ให้หยุด
SAT_ABORT_S = 1.2
# ช่อง IR ค่าไม่ขยับเลยนานกว่านี้ทั้งที่หุ่นกำลังเคลื่อนที่ = ช่องนั้นตายแล้ว ให้หยุด
FREEZE_ABORT_S = 2.5
# ตอนทดสอบแกน ค่าต้องเปลี่ยนอย่างน้อยเท่านี้ ไม่งั้นถือว่าช่องนั้นใช้ไม่ได้
MIN_RESPONSE_ADC = 15


def sample_adc(ir, n=15, delay=0.04):
    """เก็บ ADC หลายตัวอย่างแล้วคืนค่ามัธยฐาน (front, rear)"""
    fs, rs = [], []
    for _ in range(n):
        r = ir.read()
        if r.adc_front is not None:
            fs.append(r.adc_front)
        if r.adc_rear is not None:
            rs.append(r.adc_rear)
        time.sleep(delay)
    return (statistics.median(fs) if fs else None,
            statistics.median(rs) if rs else None)


def nudge(chassis, ir, y=0.0, z=0.0, seconds=0.4):
    """ขยับสั้น ๆ แล้วคืนค่า ADC หลังขยับ ใช้ทดสอบเครื่องหมายของแกน"""
    chassis.drive_speed(x=0, y=y, z=z, timeout=1.0)
    time.sleep(seconds)
    chassis.drive_speed(x=0, y=0, z=0)
    time.sleep(0.4)
    return sample_adc(ir, n=10)


def detect_signs(chassis, ir, base_f, base_r):
    """
    หาเครื่องหมายของแกนจากการขยับจริง

    คืน (toward_y, toward_z)
      toward_y = +1 แปลว่าสั่ง y เป็นบวกแล้วหุ่นเข้าใกล้กำแพง
      toward_z = +1 แปลว่าสั่ง z เป็นบวกแล้วหัวหุ่นหันเข้าหากำแพง
    Sharp IR ยิ่งใกล้ค่า ADC ยิ่งสูง จึงดูจากทิศที่ ADC เปลี่ยน
    """
    base_mean = (base_f + base_r) / 2.0
    base_skew = base_f - base_r

    print("\n[ทดสอบแกน] สไลด์ทดสอบ กันคนออกห่างจากหุ่น ...")
    f1, r1 = nudge(chassis, ir, y=+0.12, seconds=0.4)
    nudge(chassis, ir, y=-0.12, seconds=0.4)      # สไลด์กลับที่เดิม
    toward_y = 1.0
    if f1 is None or r1 is None:
        raise RuntimeError("อ่าน IR ไม่ได้ระหว่างทดสอบแกน")

    # สไลด์เข้าออกจากกำแพง ค่าของ IR ทั้งสองตัวต้องเปลี่ยนพอ ๆ กัน
    # ถ้าตัวใดแทบไม่ขยับ แปลว่าตัวนั้นไม่ได้ส่องกำแพง จะคุมมุมไม่ได้เลย
    d_front, d_rear = f1 - base_f, r1 - base_r
    print(f"    ค่าเปลี่ยนหลังสไลด์: front {d_front:+.0f} adc, rear {d_rear:+.0f} adc")
    dead = [n for n, d in (("front", d_front), ("rear", d_rear))
            if abs(d) < MIN_RESPONSE_ADC]
    if dead:
        raise RuntimeError(
            f"IR ช่อง {', '.join(dead)} ไม่ตอบสนองต่อการขยับ (เปลี่ยนน้อยกว่า "
            f"{MIN_RESPONSE_ADC} adc)\n"
            "        แปลว่าช่องนั้นไม่ได้ส่องกำแพงจริง ตรวจสายหรือรัน scan_ports.py "
            "เพื่อหา board id/port ที่ถูกต้อง")

    toward_y = 1.0 if ((f1 + r1) / 2.0 - base_mean) > 0 else -1.0
    print(f"    y เป็นบวก = {'เข้าหา' if toward_y > 0 else 'ออกจาก'}กำแพง")

    print("[ทดสอบแกน] หมุนทดสอบ ...")
    f2, r2 = nudge(chassis, ir, z=+25.0, seconds=0.35)
    nudge(chassis, ir, z=-25.0, seconds=0.35)     # หมุนกลับที่เดิม
    toward_z = 1.0
    if f2 is not None and r2 is not None:
        toward_z = 1.0 if ((f2 - r2) - base_skew) > 0 else -1.0
    print(f"    z เป็นบวก = หัวหุ่น{'เข้าหา' if toward_z > 0 else 'ออกจาก'}กำแพง")

    return toward_y, toward_z


class Console:
    """อ่านคำสั่งจากคีย์บอร์ดใน thread แยก เพื่อไม่ให้ control loop สะดุด"""

    def __init__(self):
        self.cmds = []
        self._lock = threading.Lock()
        self._running = True
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        while self._running:
            try:
                line = sys.stdin.readline()
            except Exception:
                return
            if not line:
                return
            with self._lock:
                self.cmds.append(line.strip())

    def take(self):
        with self._lock:
            out, self.cmds = self.cmds, []
        return out

    def stop(self):
        self._running = False


def load_gains():
    gains = dict(DEFAULT_GAINS)
    if os.path.exists(GAINS_PATH):
        try:
            with open(GAINS_PATH) as f:
                gains.update(json.load(f))
            print(f"โหลดค่าที่จูนไว้เดิมจาก {GAINS_PATH}: {gains}")
        except Exception:
            pass
    return gains


def save_gains(gains):
    os.makedirs(config.LOG_DIR, exist_ok=True)
    with open(GAINS_PATH, "w") as f:
        json.dump(gains, f, indent=2)
    print(f"    บันทึกค่าแล้วที่ {GAINS_PATH}")


def main():
    os.makedirs(config.LOG_DIR, exist_ok=True)
    gains = load_gains()

    ep_robot = robot.Robot()
    print(f"กำลังเชื่อมต่อหุ่นยนต์ (conn_type={config.CONN_TYPE}) ...")
    ep_robot.initialize(conn_type=config.CONN_TYPE)
    chassis = ep_robot.chassis

    ir = IRPair(ep_robot)
    tof = FrontToF(ep_robot)
    ir.start()
    tof.start()

    console = None
    log = None
    stop_reason = "จบก่อนเริ่มวิ่ง"
    try:
        # ---------- ขั้นที่ 1: สอนจุดสมดุล ----------
        print("\n" + "=" * 60)
        print("ขั้นที่ 1: วางหุ่นให้ขนานกำแพง ห่างเท่าระยะเป้าหมาย"
              f" ({config.TARGET_WALL_CM:.0f} ซม.)")
        print("=" * 60)
        input("วางเสร็จแล้วกด Enter")
        base_f, base_r = sample_adc(ir, n=25)
        if base_f is None or base_r is None:
            print("อ่าน IR ไม่ได้ ตรวจสายและเลข board id/port ใน config.py ก่อน")
            return
        print(f"จุดสมดุล: adc front={base_f:.0f} rear={base_r:.0f}")

        # ---------- ขั้นที่ 2: ทดสอบเครื่องหมายของแกน ----------
        ans = input("\nขั้นที่ 2: ให้ทดสอบเครื่องหมายของแกนอัตโนมัติไหม"
                    " (หุ่นจะขยับเล็กน้อย) [Y/n] ").strip().lower()
        if ans in ("", "y", "yes"):
            toward_y, toward_z = detect_signs(chassis, ir, base_f, base_r)
        else:
            toward_y = 1.0 if config.WALL_SIDE == "right" else -1.0
            toward_z = -toward_y
            print(f"    ข้ามการทดสอบ ใช้ค่าจาก WALL_SIDE={config.WALL_SIDE}")

        # ---------- ขั้นที่ 3: วิ่งจริง ----------
        print("\n" + "=" * 60)
        print("ขั้นที่ 3: เริ่มวิ่ง — พิมพ์ d/a/v/p/w/q แล้ว Enter เพื่อจูนสด")
        print(f"          หยุดอัตโนมัติเมื่อ ToF ด้านหน้า < {config.STOP_FRONT_CM} ซม.")
        print("=" * 60)
        input("พร้อมแล้วกด Enter (Ctrl+C หยุดฉุกเฉินได้ตลอด)")

        console = Console()
        log_path = os.path.join(config.LOG_DIR, "field_run.csv")
        log = open(log_path, "w", newline="")
        log.write("time_s,adc_front,adc_rear,err_dist,err_skew,tof_cm,vx,vy,vz\n")

        base_mean = (base_f + base_r) / 2.0
        base_skew = base_f - base_r

        pid_dist = FilteredPID(
            gains["kp_dist"], gains["ki_dist"], gains["kd_dist"],
            out_limit=VY_LIMIT, deadband=DEADBAND_DIST,
            d_alpha=gains["d_alpha"], slew_per_s=SLEW_VY)
        pid_angle = FilteredPID(
            gains["kp_angle"], gains["ki_angle"], gains["kd_angle"],
            out_limit=VZ_LIMIT, deadband=DEADBAND_SKEW,
            d_alpha=gains["d_alpha"], slew_per_s=SLEW_VZ)

        def apply_gains():
            """ดันค่าที่จูนสดเข้าไปในตัวคุม เรียกทุกครั้งที่ผู้ใช้แก้ค่า"""
            pid_dist.kp, pid_dist.ki, pid_dist.kd = (
                gains["kp_dist"], gains["ki_dist"], gains["kd_dist"])
            pid_angle.kp, pid_angle.ki, pid_angle.kd = (
                gains["kp_angle"], gains["ki_angle"], gains["kd_angle"])
            pid_dist.d_alpha = pid_angle.d_alpha = gains["d_alpha"]
        dt_target = 1.0 / config.CONTROL_HZ
        t0 = time.time()
        t_prev = t0
        paused = False
        stop_reason = "ผู้ใช้สั่งจบ"
        sat_since = None          # เวลาที่เริ่มสั่งความเร็วชนเพดานติดต่อกัน
        last_seq = -1             # ลำดับตัวอย่าง IR ล่าสุดที่คำนวณไปแล้ว
        t_sample = time.time()    # เวลาที่ได้ตัวอย่างใหม่ครั้งก่อน
        vy_hold = vz_hold = 0.0   # คำสั่งล่าสุด ใช้ค้างไว้ระหว่างรอตัวอย่างใหม่
        last_adc = (None, None)   # ค่าล่าสุด ใช้จับช่องที่ค้าง
        still_since = time.time()

        while True:
            now = time.time()
            dt = now - t_prev
            if dt < dt_target:
                time.sleep(dt_target - dt)
                now = time.time()
                dt = now - t_prev
            t_prev = now
            elapsed = now - t0

            # ----- รับคำสั่งจูน -----
            quit_now = False
            for cmd in console.take():
                if not cmd:
                    continue
                key, _, val = cmd.partition(" ")
                # ตัวพิมพ์ใหญ่คือพจน์ D ของแกนเดียวกัน จึงไม่แปลงเป็นตัวพิมพ์เล็ก
                keymap = {
                    "d": "kp_dist", "D": "kd_dist", "I": "ki_dist",
                    "a": "kp_angle", "A": "kd_angle", "i": "ki_angle",
                    "f": "d_alpha", "v": "speed",
                }
                try:
                    if key in keymap:
                        gains[keymap[key]] = float(val)
                        apply_gains()
                    elif key == "p":
                        paused = not paused
                        chassis.drive_speed(x=0, y=0, z=0)
                        pid_dist.reset()
                        pid_angle.reset()
                        vy_hold = vz_hold = 0.0
                    elif key == "r":
                        pid_dist.reset()
                        pid_angle.reset()
                        vy_hold = vz_hold = 0.0
                        print("    ล้างค่าสะสมของ PID แล้ว")
                        continue
                    elif key == "w":
                        save_gains(gains)
                        continue
                    elif key == "q":
                        quit_now = True
                    else:
                        print("    คำสั่ง: d/D/I <ค่า> ระยะ (kp/kd/ki) | a/A/i <ค่า> มุม"
                              " | f <ค่า> ตัวกรอง | v <ค่า> ความเร็ว | p | r | w | q")
                        continue
                    print(f"    ค่าปัจจุบัน: {gains} {'(หยุดชั่วคราว)' if paused else ''}")
                except ValueError:
                    print("    ใส่ตัวเลขไม่ถูกต้อง")
            if quit_now:
                break

            r = ir.read()
            tof_cm = tof.read_cm()

            if tof_cm is not None and tof_cm <= config.STOP_FRONT_CM:
                stop_reason = f"ToF ด้านหน้าวัดได้ {tof_cm:.1f} ซม."
                break

            if paused:
                continue

            vx, vy, vz = gains["speed"], 0.0, 0.0
            err_dist = err_skew = None

            # เชื่อค่าเฉพาะเมื่ออยู่ในช่วงที่ Sharp IR ให้ค่าที่ตีความได้
            def trusted(v):
                return v is not None and ADC_NO_WALL <= v <= ADC_TOO_CLOSE

            ok_front, ok_rear = trusted(r.adc_front), trusted(r.adc_rear)

            # ช่องที่ค่าไม่ขยับเลยระหว่างหุ่นกำลังวิ่ง = ช่องตาย ต้องหยุด
            # ไม่งั้นตัวคุมมุมจะเอาระยะไปคุมมุม กลายเป็น positive feedback หมุนคว้าง
            if (r.adc_front, r.adc_rear) != last_adc:
                last_adc = (r.adc_front, r.adc_rear)
                still_since = now
            elif now - still_since > FREEZE_ABORT_S:
                stop_reason = (f"ค่า IR ไม่เปลี่ยนเลยนานเกิน {FREEZE_ABORT_S} วินาที"
                               " — สงสัยสายหลุดหรือ board id/port ผิด")
                break

            if ok_front and ok_rear:
                # ค่าบวก = ใกล้กำแพงกว่าจุดสมดุล (Sharp IR ยิ่งใกล้ค่ายิ่งสูง)
                err_dist = (r.adc_front + r.adc_rear) / 2.0 - base_mean
                err_skew = (r.adc_front - r.adc_rear) - base_skew
                err_dist = max(-ERR_DIST_LIMIT, min(err_dist, ERR_DIST_LIMIT))
                err_skew = max(-ERR_SKEW_LIMIT, min(err_skew, ERR_SKEW_LIMIT))

                # IR อัปเดตราว 8-10 Hz แต่ loop วิ่ง 20 Hz
                # ถ้าคำนวณ PID ทุกรอบ พจน์ D จะได้ศูนย์สลับกับค่ากระโดด เพราะหาร
                # ผลต่างของค่าเดิมซ้ำ ๆ จึงคำนวณเฉพาะรอบที่มีตัวอย่างใหม่จริง
                # แล้วค้างคำสั่งเดิมไว้ระหว่างรอ
                if r.seq != last_seq:
                    dt_s = now - t_sample
                    last_seq, t_sample = r.seq, now
                    # ป้อน setpoint=0 โดยใช้ error เป็นค่าที่วัด ตัวคุมจึงคืน -k*err มาแล้ว
                    # เหลือแค่คูณทิศของแกนที่ทดสอบได้ในขั้นที่ 2 ไม่ต้องใส่ลบซ้ำ
                    vy_hold = toward_y * pid_dist.compute(0.0, err_dist, dt_s)
                    vz_hold = toward_z * pid_angle.compute(0.0, err_skew, dt_s)
                vy, vz = vy_hold, vz_hold
            else:
                # อ่านค่าไม่ได้หรือค่าอยู่นอกช่วงที่เชื่อได้ เดินตรงไว้ก่อน ดีกว่าเลี้ยวมั่ว
                vy = vz = vy_hold = vz_hold = 0.0
                pid_dist.reset()
                pid_angle.reset()
                t_sample = now

            # ชนเพดานความเร็วค้างนาน = ตัวคุมสู้ไม่ไหวหรือสัญญาณผิด ให้หยุดก่อนหุ่นหนี
            if abs(vz) >= VZ_LIMIT - 0.01 or abs(vy) >= VY_LIMIT - 1e-6:
                if sat_since is None:
                    sat_since = now
                elif now - sat_since > SAT_ABORT_S:
                    stop_reason = (f"สั่งความเร็วชนเพดานติดต่อกันเกิน {SAT_ABORT_S} วินาที"
                                   " — ระบบแกว่งคุมไม่อยู่ ให้ลด kp แล้วรันใหม่")
                    break
            else:
                sat_since = None

            chassis.drive_speed(x=vx, y=vy, z=vz, timeout=0.5)

            def fmt(v, p=1):
                return f"{v:7.{p}f}" if v is not None else "    ---"

            print(f"[{elapsed:6.1f}s] err ระยะ={fmt(err_dist, 0)} adc"
                  f" err มุม={fmt(err_skew, 0)} adc"
                  f" ToF={fmt(tof_cm)} ซม. | vy={vy:+.3f} vz={vz:+.1f}")

            log.write(f"{elapsed:.3f},{r.adc_front},{r.adc_rear},"
                      f"{err_dist},{err_skew},{tof_cm},{vx},{vy},{vz}\n")
            log.flush()

    except KeyboardInterrupt:
        stop_reason = "ผู้ใช้กด Ctrl+C"
    except RuntimeError as e:
        stop_reason = f"ตรวจพบปัญหาเซนเซอร์: {e}"
    finally:
        try:
            chassis.drive_speed(x=0, y=0, z=0)
            time.sleep(0.3)
        except Exception:
            pass
        if console:
            console.stop()
        if log:
            log.close()
        ir.stop()
        tof.stop()
        ep_robot.close()
        print(f"\nหยุดแล้ว เหตุผล: {stop_reason}")
        print(f"ค่าที่ใช้ล่าสุด: {gains}")
        print("ถ้าพอใจกับค่านี้ ครั้งหน้าโปรแกรมจะโหลดให้เองเมื่อกด w บันทึกไว้")


if __name__ == "__main__":
    main()
