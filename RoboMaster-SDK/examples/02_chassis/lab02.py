# -*-coding:utf-8-*-
import time
from robomaster import robot
from robomaster import config

# ฟังก์ชันดึง Unix Timestamp หน่วยมิลลิวินาที (13 หลัก)
def get_ms_timestamp():
    return time.time() * 1000

# 1. บันทึกข้อมูล Attitude
def sub_attitude_handler(attitude_info):
    yaw, pitch, roll = attitude_info
    with open('lab02_attitude_data.csv', 'a') as f:
        f.write('{0:.3f},{1},{2},{3}\n'.format(get_ms_timestamp(), yaw, pitch, roll))

# 2. บันทึกข้อมูล Position
def sub_position_handler(position_info):
    x, y, z = position_info
    with open('lab02_position_data.csv', 'a') as f:
        f.write('{0:.3f},{1},{2},{3}\n'.format(get_ms_timestamp(), x, y, z))

# 3. บันทึกข้อมูล IMU
def sub_imu_handler(imu_info):
    acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z = imu_info
    with open('lab02_imu_data.csv', 'a') as f:
        f.write('{0:.3f},{1},{2},{3},{4},{5},{6}\n'.format(
            get_ms_timestamp(), acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z
        ))

# 4. บันทึกข้อมูล ESC
def sub_esc_handler(esc_info):
    speed, angle, timestamp, state = esc_info
    with open('lab02_esc_data.csv', 'a') as f:
        f.write('{0:.3f},{1},{2},{3},{4}\n'.format(
            get_ms_timestamp(), speed, angle, timestamp, state
        ))

if __name__ == '__main__':
    # เขียน Header ให้แต่ละไฟล์ CSV ใหม่แบบเติม prefix lab02_
    with open('lab02_attitude_data.csv', 'w') as f: f.write('timestamp,yaw,pitch,roll\n')
    with open('lab02_position_data.csv', 'w') as f: f.write('timestamp,x,y,z\n')
    with open('lab02_imu_data.csv', 'w') as f: f.write('timestamp,acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z\n')
    with open('lab02_esc_data.csv', 'w') as f: f.write('timestamp,speed,angle,motor_timestamp,state\n')

    ep_robot = robot.Robot()
    
    # ดัดแปลงแก้บั๊กเน็ตเวิร์กของ SDK เพื่อคุยผ่านเครือข่าย Mac Direct (AP Mode)
    config.ROBOT_IP_STR = '192.168.2.1'
    ep_robot.initialize(conn_type='ap', proto_type='tcp')
    
    ep_chassis = ep_robot.chassis

    # เริ่มรับสัญญาณเซนเซอร์พร้อมกันที่ความถี่ 10Hz (10 ข้อมูลต่อวินาที)
    ep_chassis.sub_attitude(freq=10, callback=sub_attitude_handler)
    ep_chassis.sub_position(freq=10, callback=sub_position_handler)
    ep_chassis.sub_imu(freq=10, callback=sub_imu_handler)
    ep_chassis.sub_esc(freq=10, callback=sub_esc_handler)

    print('--- เริ่มเก็บข้อมูลเซนเซอร์ และขยับหุ่นยนต์ ---')
    
    # โจทย์: เดินหน้า +x เป็นระยะ 30 cm (0.3m) ด้วยความเร็ว 0.1 m/s
    
    ep_chassis.move(x=4, y=0, z=0, xy_speed=1).wait_for_completed(timeout=4)

    ep_chassis.move(x=0, y=5, z=0, xy_speed=1).wait_for_completed(timeout=4)

    ep_chassis.move(x=-3.25, y=0, z=0, xy_speed=1).wait_for_completed(timeout=4)

    ep_chassis.move(x=0, y=0, z=90, xy_speed=1).wait_for_completed(timeout=4)

    # ep_chassis.move(x=-4, y=0, z=0, xy_speed=3).wait_for_completed(timeout=2.5)

    
    # # พักให้หุ่นนิ่งสั้นๆ ก่อนถอยกลับ
    # time.sleep(0.5)
    
    # # โจทย์: ถอยหลัง -x เป็นระยะ 30 cm (-0.3m) ด้วยความเร็ว 0.1 m/s
    # ep_chassis.move(x=-2, y=0, z=0, xy_speed=5).wait_for_completed()
    
    # ep_chassis.move(x=0, y=-2, z=0, xy_speed=5).wait_for_completed()

    print('--- เคลื่อนที่เสร็จสิ้น กำลังปิดระบบและบันทึกข้อมูล ---')

    # ยกเลิกสับสคริปต์เซนเซอร์เพื่อปิดสตรีม
    ep_chassis.unsub_esc()
    ep_chassis.unsub_imu()
    ep_chassis.unsub_attitude()
    ep_chassis.unsub_position()

    ep_robot.close()
    print('--- เสร็จเรียบร้อย! ไฟล์ CSV ทั้ง 4 (มีแล็บนำหน้าชื่อ) พร้อมใช้งานแล้ว ---')