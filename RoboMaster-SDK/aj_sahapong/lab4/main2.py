# 6 situation

import time
import csv
import os
from robomaster import robot

# 1. กำหนด Root Directory หลักของแล็บ
BASE_DIR = os.path.join("aj_sahapong", "lab4")

# ตัวแปรควบคุมสถานการณ์ปัจจุบัน
chosen_scenario = 1

def get_timestamp():
    return time.time() * 1000

# ฟังก์ชันดึง Path ไฟล์มาตรฐาน โดยแยกตามโฟลเดอร์ย่อย sit1 - sit6
def get_file_paths(scenario_num):
    sit_dir = os.path.join(BASE_DIR, f"sit{scenario_num}")
    os.makedirs(sit_dir, exist_ok=True)
    
    return {
        "acc": os.path.join(sit_dir, "ACC.csv"),
        "gyro": os.path.join(sit_dir, "GYRO.csv"),
        "dist": os.path.join(sit_dir, "Distance.csv")
    }

# Callback สำหรับดักจับข้อมูลและบันทึกเข้าโฟลเดอร์ย่อยของสถานการณ์นั้น ๆ
def sub_imu_handler(imu_info):
    acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z = imu_info
    timestamp = get_timestamp()
    
    paths = get_file_paths(chosen_scenario)
    
    with open(paths["acc"], mode='a', newline='') as f:
        csv.writer(f).writerow([timestamp, acc_x, acc_y, acc_z])
    with open(paths["gyro"], mode='a', newline='') as f:
        csv.writer(f).writerow([timestamp, gyro_x, gyro_y, gyro_z])

def sub_tof_handler(tof_info):
    distance_mm = tof_info[0]
    timestamp = get_timestamp()
    
    paths = get_file_paths(chosen_scenario)
    
    with open(paths["dist"], mode='a', newline='') as f:
        csv.writer(f).writerow([timestamp, distance_mm])

def init_csv_headers(scenario_num):
    paths = get_file_paths(scenario_num)
    
    with open(paths["acc"], mode='w', newline='') as f:
        csv.writer(f).writerow(["Time", "acc_x", "acc_y", "acc_z"])
    with open(paths["gyro"], mode='w', newline='') as f:
        csv.writer(f).writerow(["Time", "gyro_x", "gyro_y", "gyro_z"])
    with open(paths["dist"], mode='w', newline='') as f:
        csv.writer(f).writerow(["Time", "Distance(mm.)"])

if __name__ == '__main__':
    print("="*50)
    print(" RoboMaster Lab 04: Select Motion Scenario to Run")
    print("="*50)
    print("1 : หุ่นยนต์หยุดนิ่ง (Stationary) 30 วินาที")
    print("2 : เคลื่อนที่ตรงด้วยความเร็วคงที่ (3 เมตร)")
    print("3 : เร่งความเร็วและหยุดทันที (วิ่ง 3 วินาทีแล้วเบรก)")
    print("4 : เลี้ยวซ้าย 90° (วิ่งตรง -> หมุนซ้าย)")
    print("5 : เลี้ยวขวา 90° (วิ่งตรง -> หมุนขวา)")
    print("6 : เข้าใกล้วัตถุ (วิ่งเข้าหาจนระยะ 30 cm แล้วหยุด)")
    print("="*50)
    
    try:
        user_input = input("ป้อนหมายเลขสถานการณ์ที่ต้องการรัน (1-6): ")
        chosen_scenario = int(user_input)
        if chosen_scenario < 1 or chosen_scenario > 6:
            raise ValueError
    except ValueError:
        print("❌ ป้อนข้อมูลไม่ถูกต้อง! กรุณากรอกเฉพาะตัวเลข 1 ถึง 6 เท่านั้น")
        exit()

    init_csv_headers(chosen_scenario)

    ep_robot = robot.Robot()
    print(f"\nConnecting to RoboMaster for Scenario {chosen_scenario}...")
    ep_robot.initialize(conn_type="ap")

    ep_chassis = ep_robot.chassis
    ep_sensor = ep_robot.sensor

    ep_chassis.sub_imu(freq=10, callback=sub_imu_handler)
    ep_sensor.sub_distance(freq=10, callback=sub_tof_handler)
    time.sleep(1)

    try:
        if chosen_scenario == 1:
            print(f">>> [Running Sit 1]: เก็บข้อมูลสถานการณ์หยุดนิ่ง 30 วินาที...")
            print(f"📁 ข้อมูลจะจัดเก็บที่: {os.path.join(BASE_DIR, 'sit1')}")
            time.sleep(30)
            
        elif chosen_scenario == 2:
            print(f">>> [Running Sit 2]: วิ่งตรงยาวความเร็วคงที่ 3 เมตร...")
            print(f"📁 ข้อมูลจะจัดเก็บที่: {os.path.join(BASE_DIR, 'sit2')}")
            ep_chassis.move(x=3.0, y=0, z=0, xy_speed=0.685).wait_for_completed()
            
        elif chosen_scenario == 3:
            print(f">>> [Running Sit 3]: เร่งความเร็วเต็มพิกัด 3 วินาทีแล้วดึงเบรก...")
            print(f"📁 ข้อมูลจะจัดเก็บที่: {os.path.join(BASE_DIR, 'sit3')}")
            ep_chassis.drive_speed(x=0.8, y=0, z=0)
            time.sleep(3)
            ep_chassis.drive_speed(x=0, y=0, z=0)
            time.sleep(1)
            
        elif chosen_scenario == 4:
            print(f">>> [Running Sit 4]: วิ่งตรงและหมุนตัวเลี้ยวซ้าย 90 องศา...")
            print(f"📁 ข้อมูลจะจัดเก็บที่: {os.path.join(BASE_DIR, 'sit4')}")
            ep_chassis.move(x=0.8, y=0, z=0, xy_speed=0.685).wait_for_completed()
            ep_chassis.move(x=0, y=0, z=90, z_speed=45).wait_for_completed()
            
        elif chosen_scenario == 5:
            print(f">>> [Running Sit 5]: วิ่งตรงและหมุนตัวเลี้ยวขวา 90 องศา...")
            print(f"📁 ข้อมูลจะจัดเก็บที่: {os.path.join(BASE_DIR, 'sit5')}")
            ep_chassis.move(x=0.8, y=0, z=0, xy_speed=0.685).wait_for_completed()
            ep_chassis.move(x=0, y=0, z=-90, z_speed=45).wait_for_completed()
            
        elif chosen_scenario == 6:
            print(f">>> [Running Sit 6]: วิ่งเข้าหากล่องกำแพง...")
            print(f"📁 ข้อมูลจะจัดเก็บที่: {os.path.join(BASE_DIR, 'sit6')}")
            ep_chassis.move(x=0.65, y=0, z=0, xy_speed=0.685).wait_for_completed()

        print(f"\n✅ สถานการณ์ที่ {chosen_scenario} ทำงานเสร็จสิ้นและบันทึกไฟล์เรียบร้อย!")

    except KeyboardInterrupt:
        print("\n🛑 หยุดโปรแกรมกะทันหันโดยผู้ใช้")
    finally:
        ep_chassis.unsub_imu()
        ep_sensor.unsub_distance()
        ep_robot.close()
        print("ปิดระบบเรียบร้อย แยกโฟลเดอร์แยกไฟล์เรียบร้อยครับ!")