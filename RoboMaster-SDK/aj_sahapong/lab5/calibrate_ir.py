"""
สอบเทียบ Sharp IR ทั้งสองตัว: หาค่า A, B ในสมการ cm = A * adc^B

วิธีใช้
-------
1. รัน:  python aj_sahapong/lab5/calibrate_ir.py
2. โปรแกรมจะไล่ถามระยะทีละค่า ให้วางแผ่นกำแพง (วัสดุเดียวกับที่ใช้จริง)
   ห่างจาก IR ตามระยะที่ถาม แล้วกด Enter
3. จบแล้วโปรแกรมพิมพ์ค่า IR_CALIB ที่ได้ ให้คัดลอกไปวางทับใน config.py
   และตั้ง IR_CALIBRATED = True

ที่มาของสมการ: Sharp IR ให้ output ไม่เป็นเชิงเส้น แต่ log(cm) กับ log(adc)
เกือบเป็นเส้นตรง จึงหา A, B ด้วย least squares บนสเกล log
"""
import math
import statistics
import time

from robomaster import robot

import config
from ir_sensor import IRPair

# ระยะที่ใช้เก็บข้อมูล (ซม.) ครอบช่วงใช้งานจริงรอบ ๆ 10 ซม.
DISTANCES_CM = [5, 8, 10, 15, 20, 30, 40, 60, 80]
SAMPLES_PER_POINT = 20


def fit_power_law(points):
    """
    points = [(cm, adc), ...]  คืน (A, B) ของ cm = A * adc^B
    ทำ least squares บน ln(cm) = ln(A) + B * ln(adc)
    """
    usable = [(c, a) for c, a in points if a and a > 0 and c > 0]
    if len(usable) < 2:
        return None
    xs = [math.log(a) for _, a in usable]
    ys = [math.log(c) for c, _ in usable]
    mx, my = statistics.mean(xs), statistics.mean(ys)
    num = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
    den = sum((x - mx) ** 2 for x in xs)
    if den == 0:
        return None
    B = num / den
    A = math.exp(my - B * mx)
    return A, B


def collect(ir, label):
    """เก็บ ADC หลายตัวอย่างแล้วคืนค่ามัธยฐานของ IR ทั้งสองตัว"""
    fronts, rears = [], []
    for _ in range(SAMPLES_PER_POINT):
        r = ir.read()
        if r.adc_front is not None:
            fronts.append(r.adc_front)
        if r.adc_rear is not None:
            rears.append(r.adc_rear)
        time.sleep(0.05)
    f = statistics.median(fronts) if fronts else None
    b = statistics.median(rears) if rears else None
    print(f"    {label}: adc front={f} rear={b}")
    return f, b


def main():
    ep_robot = robot.Robot()
    print(f"กำลังเชื่อมต่อหุ่นยนต์ (conn_type={config.CONN_TYPE}) ...")
    ep_robot.initialize(conn_type=config.CONN_TYPE)

    ir = IRPair(ep_robot)
    ir.start()

    front_points, rear_points = [], []
    try:
        for cm in DISTANCES_CM:
            input(f"\nวางกำแพงให้ห่าง IR {cm} ซม. แล้วกด Enter")
            f, b = collect(ir, f"{cm} ซม.")
            if f:
                front_points.append((cm, f))
            if b:
                rear_points.append((cm, b))
    except KeyboardInterrupt:
        print("\nหยุดเก็บข้อมูลก่อนครบ จะคำนวณจากเท่าที่เก็บได้")
    finally:
        ir.stop()
        ep_robot.close()

    print("\n" + "=" * 60)
    print("ผลการสอบเทียบ — คัดลอกไปวางทับใน config.py")
    print("=" * 60)
    print("IR_CALIB = {")
    for name, pts in (("front", front_points), ("rear", rear_points)):
        fit = fit_power_law(pts)
        if fit:
            A, B = fit
            print(f'    "{name}": ({A:.1f}, {B:.4f}),')
        else:
            print(f'    "{name}": (4800.0, -1.05),   # ข้อมูลไม่พอ ใช้ค่าเดิม')
    print("}")
    print("IR_CALIBRATED = True")

    # แสดงความคลาดเคลื่อนของแต่ละจุด เพื่อดูว่าสมการใช้ได้จริงหรือไม่
    for name, pts in (("front", front_points), ("rear", rear_points)):
        fit = fit_power_law(pts)
        if not fit:
            continue
        A, B = fit
        print(f"\nความคลาดเคลื่อน {name}:")
        for cm, adc in pts:
            est = A * (adc ** B)
            print(f"  จริง {cm:5.1f} ซม. -> คำนวณได้ {est:6.2f} ซม. (ต่าง {est - cm:+.2f})")


if __name__ == "__main__":
    main()
