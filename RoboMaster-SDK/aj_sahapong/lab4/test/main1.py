# test code

import time
import csv
import os
from robomaster import robot

# 1. ปรับโฟลเดอร์หลักให้ชี้ไปที่ aj_sahapong/lab4 โดยตรงตามโครงสร้างในรูป
DATA_DIR = os.path.join("aj_sahapong", "lab4")
os.makedirs(DATA_DIR, exist_ok=True)

# 2. กำหนดชื่อไฟล์ให้งอกออกมาในโฟลเดอร์ lab4 เคียงข้าง main.py[cite: 1]
FILE_ACC      = os.path.join(DATA_DIR, "ACC.csv")
FILE_GYRO     = os.path.join(DATA_DIR, "GYRO.csv")
FILE_DISTANCE = os.path.join(DATA_DIR, "Distance.csv")

# 3. สร้างและเขียน Header ลงไฟล์ CSV แต่ละตัว
with open(FILE_ACC, mode='w', newline='') as f:
    csv.writer(f).writerow(["Time", "acc_x", "acc_y", "acc_z"])

with open(FILE_GYRO, mode='w', newline='') as f:
    csv.writer(f).writerow(["Time", "gyro_x", "gyro_y", "gyro_z"])

with open(FILE_DISTANCE, mode='w', newline='') as f:
    csv.writer(f).writerow(["Time", "Distance(mm.)"])

# 4. ฟังก์ชันสำหรับดึง Unix Timestamp สัมพัทธ์ (หน่วยมิลลิวินาที)
def get_timestamp():
    return time.time() * 1000

# 5. Callback Functions สำหรับ Subscription ข้อมูลเซนเซอร์
def sub_imu_handler(imu_info):
    acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z = imu_info
    timestamp = get_timestamp()
    
    # แยกบันทึกเข้า ACC.csv
    with open(FILE_ACC, mode='a', newline='') as f:
        csv.writer(f).writerow([timestamp, acc_x, acc_y, acc_z])
        
    # แยกบันทึกเข้า GYRO.csv
    with open(FILE_GYRO, mode='a', newline='') as f:
        csv.writer(f).writerow([timestamp, gyro_x, gyro_y, gyro_z])
        
    # แสดงค่าตรวจสอบความเรียบร้อยแบบ Real-time บนหน้าจอ
    print(f"[IMU] AccX: {acc_x:.2f} | GyroZ: {gyro_z:.2f}")

def sub_tof_handler(tof_info):
    # ดึงค่าระยะทางจากเซนเซอร์ ToF (ตัวแปรแรกของข้อมูลที่ส่งกลับมา)
    distance_mm = tof_info[0]
    timestamp = get_timestamp()
    
    # บันทึกเข้า Distance.csv
    with open(FILE_DISTANCE, mode='a', newline='') as f:
        csv.writer(f).writerow([timestamp, distance_mm])
        
    print(f"[ToF] Distance: {distance_mm} mm.")

# 6. ส่วนควบคุมการสั่งเคลื่อนที่ตามแต่ละสถานการณ์ (Motion Scenarios)
if __name__ == '__main__':
    ep_robot = robot.Robot()
    print("Connecting to RoboMaster...")
    ep_robot.initialize(conn_type="ap") # ปรับเป็น "sta" ตามโหมดเครือข่ายของแล็บ

    ep_chassis = ep_robot.chassis
    ep_sensor = ep_robot.sensor # สำหรับใช้งาน Distance Sensor (ToF)

    # เปิดระบบสมัครรับข้อมูล (Subscription) ความถี่ 10 Hz
    print("Starting sensor subscriptions...")
    ep_chassis.sub_imu(freq=10, callback=sub_imu_handler)
    ep_sensor.sub_distance(freq=10, callback=sub_tof_handler)

    time.sleep(1) # รอระบบเซนเซอร์เสถียรก่อนเริ่มเคลื่อนที่

    try:
        # 🚨 หมายเหตุ: เลือกเปิดรันทีละสถานการณ์ (Scenario) โดยการเอา '#' ออกด้านหน้า
        
        # --- สถานการณ์ที่ 1: หุ่นยนต์หยุดนิ่ง (Stationary) ---
        print("Running Scenario 1: Stationary for 30 seconds")
        # time.sleep(30)
        ep_chassis.move(x=2.5, y=0, z=0, xy_speed=1).wait_for_completed()
        ep_chassis.move(x=0, y=0, z=90, z_speed=50).wait_for_completed()
        ep_chassis.move(x=2.5, y=0, z=0, xy_speed=1).wait_for_completed()

        
        # --- สถานการณ์ที่ 2: เคลื่อนที่ตรงด้วยความเร็วคงที่ ---
        # print("Running Scenario 2: Constant Speed for 3 meters")
        # ep_chassis.move(x=3.0, y=0, z=0, xy_speed=0.5).wait_for_completed()
        
        # --- สถานการณ์ที่ 3: เร่งความเร็วและหยุดทันที ---
        # print("Running Scenario 3: Acceleration and Sudden Brake")
        # ep_chassis.drive_speed(x=0.8, y=0, z=0) # เร่งความเร็วสูง
        # time.sleep(3)                           # วิ่งตรง 3 วินาที
        # ep_chassis.drive_speed(x=0, y=0, z=0)   # เบรกทันที
        # time.sleep(1)
        
        # --- สถานการณ์ที่ 4: เลี้ยวซ้าย 90° ---
        # print("Running Scenario 4: Straight 3s -> Left 90°")
        # ep_chassis.move(x=0.5, y=0, z=0, xy_speed=0.2).wait_for_completed()
        # ep_chassis.move(x=0, y=0, z=90, z_speed=50).wait_for_completed()
        
        # --- สถานการณ์ที่ 5: เลี้ยวขวา 90° ---
        # print("Running Scenario 5: Straight 3s -> Right 90°")
        # ep_chassis.move(x=0.5, y=0, z=0, xy_speed=0.2).wait_for_completed()
        # ep_chassis.move(x=0, y=0, z=-90, z_speed=50).wait_for_completed()

        # --- สถานการณ์ที่ 6: เข้าใกล้วัตถุระยะ 30 cm ---
        # print("Running Scenario 6: Approach object until 30cm")
        # # ในสถานการณ์นี้ แนะนำให้ใช้ Loop ตรวจสอบระยะทางจาก ToF (เขียนเสริมแยกในแล็บจริงได้)
        # ep_chassis.move(x=1.0, y=0, z=0, xy_speed=0.2).wait_for_completed()

        print("Scenario executed successfully.")

    except KeyboardInterrupt:
        print("Program interrupted by user.")
        
    finally:
        # ยกเลิกการรับข้อมูลและปิดการเชื่อมต่อ
        print("Unsubscribing and closing connection...")
        ep_chassis.unsub_imu()
        ep_sensor.unsub_distance()
        ep_robot.close()
        print("Data acquisition complete. Files saved to aj_sahapong/lab4/")