"""
อ่านล็อกจากการรันครั้งล่าสุด แล้วบอกว่าควรปรับ gain ตัวไหนเป็นเท่าไร

รัน:
    python aj_sahapong/lab5/tune_help.py
    python aj_sahapong/lab5/tune_help.py logs/field_run.csv

วิธีใช้เป็นวงรอบ
    1. รัน field_run.py ราว 20 วินาที แล้วกด q
    2. รันไฟล์นี้ อ่านคำแนะนำ
    3. แก้ค่าตามที่บอก แล้ววนกลับข้อ 1 จนได้ผลที่รับได้

สิ่งที่ดู
    แอมพลิจูดของ error   บอกว่าแกว่งแรงแค่ไหน
    ความถี่ของการแกว่ง   บอกว่าเป็นการแกว่งจาก kp สูงเกินหรือเป็นแค่ noise
    เปอร์เซ็นต์ที่ชนเพดาน บอกว่าตัวคุมสู้ไม่ไหวหรือสัญญาณผิด
    ค่าเฉลี่ยของ error   บอกว่ามีระยะค้างที่ต้องใช้ I มาแก้หรือไม่
"""
import csv
import json
import os
import statistics
import sys

import config

# จากสเปค Sharp IR ที่ระยะราว 10 ซม. ค่าเปลี่ยนราว 16 adc ต่อ 1 ซม.
ADC_PER_CM = 16.0
GAINS_PATH = os.path.join(config.LOG_DIR, "field_gains.json")

# เกณฑ์ตัดสิน (หน่วย adc)
SWING_OK = 25.0        # แกว่งน้อยกว่านี้ถือว่านิ่งพอ (~1.5 ซม.)
SWING_BAD = 80.0       # มากกว่านี้ถือว่าแกว่งแรง
OFFSET_OK = 12.0       # ค่าค้างน้อยกว่านี้ถือว่าเข้าเป้าแล้ว (~0.75 ซม.)
FAST_HZ = 1.2          # แกว่งเร็วกว่านี้ = อาการ gain สูงเกิน ไม่ใช่การไล่ตามกำแพง


def load_rows(path):
    with open(path) as f:
        return list(csv.DictReader(f))


def col(rows, key):
    out = []
    for r in rows:
        v = r.get(key)
        if v not in (None, "", "None"):
            try:
                out.append(float(v))
            except ValueError:
                pass
    return out


def swing(vals):
    """ช่วงการแกว่งแบบตัดหางทิ้ง กันค่าหลุดตัวเดียวทำให้ตัวเลขบวม"""
    if len(vals) < 10:
        return 0.0
    s = sorted(vals)
    lo = s[int(len(s) * 0.05)]
    hi = s[int(len(s) * 0.95)]
    return hi - lo


def cross_rate_hz(vals, duration_s):
    """นับจำนวนครั้งที่ค่าตัดผ่านค่าเฉลี่ย แล้วแปลงเป็นความถี่ของการแกว่ง"""
    if len(vals) < 10 or duration_s <= 0:
        return 0.0
    m = statistics.mean(vals)
    crossings = sum(1 for a, b in zip(vals, vals[1:]) if (a - m) * (b - m) < 0)
    return crossings / 2.0 / duration_s


def pct_saturated(vals, limit):
    if not vals:
        return 0.0
    return 100.0 * sum(1 for v in vals if abs(v) >= limit * 0.999) / len(vals)


def report_axis(name, err, dur, gains, kp_key, kd_key, ki_key, sat_pct):
    sw = swing(err)
    hz = cross_rate_hz(err, dur)
    off = abs(statistics.mean(err)) if err else 0.0
    kp = gains.get(kp_key)

    print(f"\n--- {name} ---")
    print(f"  แกว่ง (ช่วง 5-95%) : {sw:7.1f} adc  ({sw / ADC_PER_CM:.2f} ซม.)")
    print(f"  ความถี่การแกว่ง    : {hz:7.2f} Hz")
    print(f"  ค่าค้างเฉลี่ย       : {off:7.1f} adc  ({off / ADC_PER_CM:.2f} ซม.)")
    print(f"  ชนเพดานความเร็ว    : {sat_pct:7.1f} %")

    advice = []
    if sat_pct > 20:
        advice.append(f"ชนเพดานบ่อยมาก ลด {kp_key} ครึ่งหนึ่งก่อนเป็นอันดับแรก")
    if sw > SWING_BAD and hz > FAST_HZ:
        advice.append(f"แกว่งแรงและเร็ว = gain สูงเกิน ตั้ง {kp_key} = {kp * 0.5:.5g}")
    elif sw > SWING_OK and hz > FAST_HZ:
        advice.append(f"ยังส่ายอยู่ ตั้ง {kp_key} = {kp * 0.7:.5g}"
                      f" ถ้ายังไม่หายค่อยเพิ่ม {kd_key} ทีละ 50 เปอร์เซ็นต์")
    elif sw > SWING_OK:
        advice.append("แกว่งช้า น่าจะเป็นการไล่ตามรูปร่างกำแพงมากกว่าอาการ gain สูง"
                      " ลองลดความเร็วด้วย v ก่อน")
    if off > OFFSET_OK and sw <= SWING_BAD:
        if gains.get(ki_key, 0.0) == 0.0:
            advice.append(f"มีค่าค้างไม่เข้าเป้า เปิด {ki_key} = 0.0008")
        else:
            advice.append(f"ยังค้างอยู่ เพิ่ม {ki_key} เป็น {gains[ki_key] * 2:.5g}")
    if sw <= SWING_OK and off <= OFFSET_OK:
        advice.append(f"นิ่งและเข้าเป้าแล้ว ถ้าอยากเร็วขึ้นค่อยเพิ่ม {kp_key}"
                      " ทีละ 20 เปอร์เซ็นต์ หรือเพิ่มความเร็วด้วย v")

    for a in advice:
        print(f"  -> {a}")


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else os.path.join(config.LOG_DIR,
                                                              "field_run.csv")
    if not os.path.exists(path):
        print(f"ไม่พบไฟล์ {path} — รัน field_run.py ก่อน")
        return

    rows = load_rows(path)
    if len(rows) < 20:
        print("ข้อมูลน้อยเกินไป ให้วิ่งอย่างน้อย 15-20 วินาทีแล้วค่อยวิเคราะห์")
        return

    times = col(rows, "time_s")
    span = (times[-1] - times[0]) if len(times) > 1 else 0.0
    if span < 8.0:
        print(f"เตือน: รันสั้นแค่ {span:.1f} วินาที ตัวเลขข้างล่างเชื่อไม่ได้")
        print("       ช่วงต้นเป็นการเข้าเป้าไม่ใช่การแกว่ง ต้องวิ่งอย่างน้อย 15 วินาที")
        print("       ถ้าหุ่นถึงกำแพงหน้าก่อน ให้เอากำแพงหน้าออกตอนจูน"
              " เหลือแต่กำแพงข้างยาว ๆ\n")

    gains = {}
    if os.path.exists(GAINS_PATH):
        with open(GAINS_PATH) as f:
            gains = json.load(f)
    # เติมค่าที่ยังไม่มีจากค่าเริ่มต้น เผื่อยังไม่เคยกด w บันทึก
    try:
        from field_run import DEFAULT_GAINS
    except Exception:
        # เครื่องที่ไม่ได้ลง SDK ก็ยังวิเคราะห์ล็อกได้ ใช้ค่าเริ่มต้นสำรอง
        DEFAULT_GAINS = {"kp_dist": 0.002, "ki_dist": 0.0, "kd_dist": 0.0002,
                         "kp_angle": 0.30, "ki_angle": 0.0, "kd_angle": 0.03,
                         "speed": 0.20, "d_alpha": 0.20}
    merged = dict(DEFAULT_GAINS)
    merged.update(gains)
    gains = merged

    t = col(rows, "time_s")
    dur = (t[-1] - t[0]) if len(t) > 1 else 0.0
    err_d = col(rows, "err_dist")
    err_s = col(rows, "err_skew")
    vy = col(rows, "vy")
    vz = col(rows, "vz")
    adc_f = col(rows, "adc_front")
    adc_r = col(rows, "adc_rear")

    print("=" * 62)
    print(f"ไฟล์: {path}")
    print(f"ระยะเวลา {dur:.1f} วินาที  {len(rows)} รอบ  "
          f"({len(rows) / dur:.1f} รอบต่อวินาที)" if dur else "")
    print(f"gain ที่ใช้: {gains}")
    print("=" * 62)

    # ตรวจสุขภาพเซนเซอร์ก่อน ถ้าช่องไหนนิ่งผิดปกติ การจูนไม่มีความหมาย
    for label, vals in (("front", adc_f), ("rear", adc_r)):
        if not vals:
            continue
        sd = statistics.pstdev(vals)
        note = ""
        if sd < 10:
            note = "  << ค่านิ่งผิดปกติ สงสัยช่องนี้ไม่ได้ส่องกำแพง รัน scan_ports.py"
        print(f"IR {label}: ช่วง {min(vals):.0f}-{max(vals):.0f} sd={sd:.1f}{note}")

    report_axis("การคุมมุม (หัวส่าย)", err_s, dur, gains,
                "kp_angle", "kd_angle", "ki_angle", pct_saturated(vz, 60.0))
    report_axis("การคุมระยะห่างกำแพง", err_d, dur, gains,
                "kp_dist", "kd_dist", "ki_dist", pct_saturated(vy, 0.25))

    print("\nแก้ค่าได้สองทาง: พิมพ์คำสั่งสดระหว่างวิ่ง หรือแก้ DEFAULT_GAINS"
          " ใน field_run.py")


if __name__ == "__main__":
    main()
