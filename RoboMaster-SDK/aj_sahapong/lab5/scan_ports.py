"""
ไล่อ่านทุกช่องของ Sensor Adaptor เพื่อหาว่า IR เสียบอยู่ที่ board id / port ไหนจริง

ใช้เมื่อสงสัยว่าค่าที่อ่านได้ไม่ตอบสนองต่อโลกจริง เช่นค่าค้างนิ่งทั้งที่ขยับหุ่นแล้ว
พอร์ตที่ไม่มีอะไรเสียบมักคืนค่าคงที่ค่าหนึ่ง ซึ่งดูเผิน ๆ เหมือนค่าที่ใช้ได้

รัน:
    python aj_sahapong/lab5/scan_ports.py

โปรแกรมจะอ่านทุกช่องสองรอบ รอบแรกตอนไม่มีอะไรบัง รอบสองตอนเอามือบังห่างราว 10 ซม.
ช่องที่ค่าเปลี่ยนมากคือช่องที่มี IR เสียบอยู่จริง
"""
import statistics
import time

from robomaster import robot

import config

BOARD_IDS = range(1, 7)      # stream ส่งค่ามา 6 บอร์ด
PORTS = (1, 2)
# ค่าต้องเปลี่ยนมากกว่านี้จึงนับว่าช่องนั้นตอบสนองจริง
RESPONSE_THRESHOLD = 30


def read_all(ep_robot, seconds=2.0):
    """
    อ่านทุกช่องพร้อมกันผ่าน stream sub_adapter แล้วคืน {(id, port): ค่ามัธยฐาน}

    ใช้ stream แทนการไล่ get_adc ทีละช่อง เพราะการยิงคำสั่งถี่ ๆ สลับหลายบอร์ด
    ทำให้คำตอบชนกันจนได้ค่าหลุดเป็นเลขต่ำ ๆ ปนมา ซึ่งทำให้อ่านผลสแกนผิด
    """
    frames = []

    def on_adapter(sub_info):
        _io, ad = sub_info
        frames.append(list(ad))

    adaptor = ep_robot.sensor_adaptor
    adaptor.sub_adapter(freq=20, callback=on_adapter)
    time.sleep(seconds)
    adaptor.unsub_adapter()

    out = {}
    if not frames:
        return out
    width = min(len(f) for f in frames)
    for idx in range(min(width, len(BOARD_IDS) * len(PORTS))):
        vals = [f[idx] for f in frames if f[idx] is not None]
        if vals:
            board_id = idx // 2 + 1
            port = idx % 2 + 1
            out[(board_id, port)] = statistics.median(vals)
    return out


def main():
    ep_robot = robot.Robot()
    print(f"กำลังเชื่อมต่อหุ่นยนต์ (conn_type={config.CONN_TYPE}) ...")
    ep_robot.initialize(conn_type=config.CONN_TYPE)
    try:
        input("\nรอบที่ 1: เปิดหน้าเซนเซอร์ให้โล่ง (ไม่มีอะไรใกล้กว่า 50 ซม.) แล้วกด Enter")
        far = read_all(ep_robot)
        print(f"อ่านได้ {len(far)} ช่อง")

        input("รอบที่ 2: เอามือหรือแผ่นโฟมบังหน้า IR ทุกตัว ห่างราว 10 ซม. แล้วกด Enter")
        near = read_all(ep_robot)

        print("\n" + "=" * 62)
        print(f"{'board/port':<12}{'ไกล':>8}{'ใกล้':>8}{'ต่าง':>8}   สรุป")
        print("=" * 62)
        live = []
        for key in sorted(set(far) | set(near)):
            a = far.get(key)
            b = near.get(key)
            if a is None or b is None:
                continue
            d = b - a
            verdict = "<< มี IR เสียบอยู่" if abs(d) >= RESPONSE_THRESHOLD else "ค่านิ่ง ไม่มี IR"
            if abs(d) >= RESPONSE_THRESHOLD:
                live.append(key)
            print(f"id{key[0]} port{key[1]:<5}{a:8.0f}{b:8.0f}{d:+8.0f}   {verdict}")

        print("\nช่องที่ตอบสนองจริง:", live if live else "ไม่พบเลย ตรวจสายและไฟเลี้ยงบอร์ด")
        if len(live) >= 2:
            print("\nเอาไปใส่ใน config.py:")
            print(f"IR_FRONT = {live[0]}")
            print(f"IR_REAR  = {live[1]}")
            print("(สลับสองบรรทัดนี้ถ้าตัวแรกคือตัวที่ติดค่อนไปทางท้ายหุ่น)")
    finally:
        ep_robot.close()


if __name__ == "__main__":
    main()
