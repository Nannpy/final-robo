import time
import csv
import os
from robomaster import robot

# 1. กำหนดโครงสร้างโฟลเดอร์สำหรับเก็บข้อมูลดิบ
DATA_DIR = "data/raw"
os.makedirs(DATA_DIR, exist_ok=True)

# 2. ตั้งชื่อไฟล์ CSV ทั้ง 4 ไฟล์แยกตามชนิดเซนเซอร์
FILE_ATTITUDE = os.path.join(DATA_DIR, "lab2_attitude_logs.csv")
FILE_POSITION = os.path.join(DATA_DIR, "lab2_position_logs.csv")
FILE_IMU      = os.path.join(DATA_DIR, "lab2_imu_logs.csv")
FILE_ESC      = os.path.join(DATA_DIR, "lab2_esc_logs.csv")

# 3. สร้างและเขียน Header ลงในแต่ละไฟล์ CSV
with open(FILE_ATTITUDE, mode='w', newline='') as f:
    writer = csv.writer(f)
    writer.writerow(["timestamp", "yaw", "pitch", "roll"])

with open(FILE_POSITION, mode='w', newline='') as f:
    writer = csv.writer(f)
    writer.writerow(["timestamp", "position_x", "position_y", "position_z"])

with open(FILE_IMU, mode='w', newline='') as f:
    writer = csv.writer(f)
    writer.writerow(["timestamp", "acc_x", "acc_y", "acc_z", "gyro_x", "gyro_y", "gyro_z"])

with open(FILE_ESC, mode='w', newline='') as f:
    writer = csv.writer(f)
    writer.writerow(["timestamp", "speed_m1", "speed_m2", "speed_m3", "speed_m4"])

# 4. ฟังก์ชันสำหรับดึงเวลาเป็น Unix Timestamp (มิลลิวินาที พร้อมทศนิยม)
def get_unix_timestamp():
    # คูณ 1000 เพื่อแปลงวินาทีให้เป็นมิลลิวินาทีตามฟอร์แมตโจทย์
    return time.time() * 1000

# 5. Callback Functions สำหรับบันทึกข้อมูลเซนเซอร์ในขณะเคลื่อนที่
def sub_attitude_handler(attitude_info):
    yaw, pitch, roll = attitude_info
    timestamp = get_unix_timestamp()
    with open(FILE_ATTITUDE, mode='a', newline='') as f:
        csv.writer(f).writerow([timestamp, yaw, pitch, roll])

def sub_position_handler(position_info):
    pos_x, pos_y, pos_z = position_info
    timestamp = get_unix_timestamp()
    with open(FILE_POSITION, mode='a', newline='') as f:
        csv.writer(f).writerow([timestamp, pos_x, pos_y, pos_z])

def sub_imu_handler(imu_info):
    acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z = imu_info
    timestamp = get_unix_timestamp()
    with open(FILE_IMU, mode='a', newline='') as f:
        csv.writer(f).writerow([timestamp, acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z])

def sub_esc_handler(esc_info):
    # ดึงค่าความเร็วรอบ (Speed) ของมอเตอร์ล้อทั้ง 4 ตัว (m1, m2, m3, m4)
    speed_m1, speed_m2, speed_m3, speed_m4 = esc_info
    timestamp = get_unix_timestamp()
    with open(FILE_ESC, mode='a', newline='') as f:
        csv.writer(f).writerow([timestamp, speed_m1, speed_m2, speed_m3, speed_m4])

# 6. ส่วนควบคุมการเชื่อมต่อและการสั่งการเคลื่อนที่ของหุ่นยนต์
if __name__ == '__main__':
    ep_robot = robot.Robot()
    print("Connecting to RoboMaster...")
    ep_robot.initialize(conn_type="ap") # หรือเปลี่ยนเป็น "sta" ตามเน็ตเวิร์กแล็บ

    ep_chassis = ep_robot.chassis

    # เปิดระบบสมัครรับข้อมูล (Subscription) จากเซนเซอร์ทุกตัวด้วยความถี่ 5 Hz
    print("Starting sensor subscriptions...")
    ep_chassis.sub_attitude(freq=5, callback=sub_attitude_handler)
    ep_chassis.sub_position(freq=5, callback=sub_position_handler)
    ep_chassis.sub_imu(freq=5, callback=sub_imu_handler)
    ep_chassis.sub_esc(freq=5, callback=sub_esc_handler)

    time.sleep(1) # รอระบบเซนเซอร์ Set up แป๊บหนึ่งก่อนเริ่มเคลื่อนที่

    try:
        # สเต็ปที่ 1: เคลื่อนที่ไปด้านหน้า +x ระยะ 30 cm (0.3 m) ด้วยความเร็ว 0.1 m/s
        print("Moving Forward: +30 cm at 0.1 m/s")
        ep_chassis.move(x=0.3, y=0, z=0, xy_speed=0.1).wait_for_completed()

        # หน่วงเวลาสั้นๆ ป้องกันจังหวะกระชากสะดุด
        time.sleep(0.5)

        # สเต็ปที่ 2: เคลื่อนที่ถอยหลัง -x ระยะ 30 cm (-0.3 m) ด้วยความเร็ว 0.1 m/s
        print("Moving Backward: -30 cm at 0.1 m/s")
        ep_chassis.move(x=-0.3, y=0, z=0, xy_speed=0.1).wait_for_completed()

        print("Movement completed successfully.")

    except KeyboardInterrupt:
        print("Program interrupted by user.")

    finally:
        # ยกเลิกการสมัครรับข้อมูลเซนเซอร์เพื่อคืนหน่วยความจำและปิดหุ่นยนต์
        print("Unsubscribing and closing connection...")
        ep_chassis.unsub_attitude()
        ep_chassis.unsub_position()
        ep_chassis.unsub_imu()
        ep_chassis.unsub_esc()
        ep_robot.close()
        print("Done! Check your 4 CSV files in 'data/raw/' directory.")