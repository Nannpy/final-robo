"""
Class work ข้อ 3 — อ่านค่าสัญญาณจาก IR Sensor ทั้งสองตัวด้วย RoboMaster SDK

รัน:
    python aj_sahapong/lab5/read_ir.py            อ่าน 30 วินาที
    python aj_sahapong/lab5/read_ir.py 60         อ่าน 60 วินาที

พิมพ์ค่า ADC ดิบและระยะที่แปลงแล้วออกหน้าจอ พร้อมบันทึกลง logs/ir_raw.csv
เอาไฟล์ CSV นี้ไปใช้ต่อในการสอบเทียบและตรวจสัญญาณรบกวน
"""
import csv
import os
import statistics
import sys
import time

from robomaster import robot

import config
from ir_sensor import IRPair


def main(duration_s: float):
    os.makedirs(config.LOG_DIR, exist_ok=True)
    log_path = os.path.join(config.LOG_DIR, "ir_raw.csv")

    ep_robot = robot.Robot()
    print(f"กำลังเชื่อมต่อหุ่นยนต์ (conn_type={config.CONN_TYPE}) ...")
    ep_robot.initialize(conn_type=config.CONN_TYPE)

    ir = IRPair(ep_robot)
    ir.start()

    print(f"เริ่มอ่านค่า {duration_s:.0f} วินาที — บันทึกที่ {log_path}")
    print("กด Ctrl+C เพื่อหยุดก่อนเวลา\n")

    t0 = time.time()
    samples = {"front": [], "rear": []}
    try:
        with open(log_path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["time_s", "adc_front", "adc_rear",
                        "front_cm", "rear_cm", "mean_cm", "angle_deg"])
            while time.time() - t0 < duration_s:
                r = ir.read()
                t = r.t - t0

                def fmt(v, unit=""):
                    return f"{v:6.1f}{unit}" if v is not None else "   --- "

                print(f"[{t:6.2f}s] ADC front={fmt(r.adc_front)} rear={fmt(r.adc_rear)}"
                      f" | front={fmt(r.front_cm)} cm rear={fmt(r.rear_cm)} cm"
                      f" | มุมเอียง={fmt(r.angle_deg)} องศา")

                if r.adc_front is not None:
                    samples["front"].append(r.adc_front)
                if r.adc_rear is not None:
                    samples["rear"].append(r.adc_rear)

                w.writerow([f"{t:.3f}", r.adc_front, r.adc_rear,
                            r.front_cm, r.rear_cm, r.mean_cm, r.angle_deg])
                f.flush()
                time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nหยุดโดยผู้ใช้")
    finally:
        ir.stop()
        ep_robot.close()
        summarize(samples)
        print(f"\nข้อมูลอยู่ที่ {log_path}")


def summarize(samples):
    """
    สรุปว่าสัญญาณของแต่ละตัวสะอาดพอจะเอาไปคุมหุ่นหรือยัง

    เกณฑ์: ตอนหุ่นจอดนิ่งและกำแพงอยู่กับที่ ค่าควรกระเพื่อมไม่เกินไม่กี่สิบ adc
    ที่ระยะราว 10 ซม. ค่า 16 adc คิดเป็น 1 ซม. และ baseline ระหว่าง IR สองตัว
    ยาวเพียง 13.5 ซม. ผลต่างที่เพี้ยน 16 adc จึงกลายเป็นมุมที่เพี้ยนราว 4 องศา
    สัญญาณที่กระเพื่อมเกิน 50 adc จึงคุมมุมไม่ได้เลย
    """
    print("\n" + "=" * 56)
    print(f"{'ช่อง':<10}{'ต่ำสุด':>8}{'สูงสุด':>8}{'ค่ากลาง':>9}{'sd':>8}   สรุป")
    print("=" * 56)
    verdicts = {}
    for name in ("front", "rear"):
        vals = samples[name]
        if len(vals) < 10:
            print(f"{name:<10}   ข้อมูลน้อยเกินไป")
            continue
        sd = statistics.pstdev(vals)
        if sd < 5:
            v = "นิ่งผิดปกติ อาจไม่ได้ส่องกำแพง"
        elif sd <= 25:
            v = "สัญญาณดี"
        elif sd <= 50:
            v = "กระเพื่อมพอควร คุมได้แต่ต้องกรอง"
        else:
            v = "กระเพื่อมแรงเกินไป << ต้องแก้ก่อนจูน"
        verdicts[name] = sd
        print(f"{name:<10}{min(vals):8.0f}{max(vals):8.0f}"
              f"{statistics.median(vals):9.0f}{sd:8.1f}   {v}")

    if len(verdicts) == 2:
        hi, lo = max(verdicts.values()), min(verdicts.values())
        if lo > 0 and hi / lo > 3:
            worse = max(verdicts, key=verdicts.get)
            print(f"\nสองตัวกระเพื่อมต่างกันเกิน 3 เท่า ตัว {worse} ผิดปกติ")
            print("ไล่ตรวจตามลำดับ: ระดับความสูงตรงกับอีกตัวไหม / ตั้งฉากกับกำแพงไหม /")
            print("โฟมยาวคลุมจุดที่มันส่องไหม / สายแน่นไหม / ลองสลับไปเสียบช่องของ")
            print("IR ฝั่งซ้ายที่ไม่ได้ใช้ ถ้าอาการตามไปด้วยแปลว่าตัวเซนเซอร์เอง")


if __name__ == "__main__":
    seconds = float(sys.argv[1]) if len(sys.argv) > 1 else 30.0
    main(seconds)
