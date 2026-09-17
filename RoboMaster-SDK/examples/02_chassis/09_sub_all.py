# -*-coding:utf-8-*-
# Copyright (c) 2020 DJI.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License in the file LICENSE.txt or at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import time
import robomaster
from robomaster import robot
from robomaster import config  # 1. นำเข้า config เพิ่มเข้ามาเพื่อระบุ IP


def sub_info_handler(sub_info):
    print("sub info: {0}".format(sub_info))


if __name__ == '__main__':
    ep_robot = robot.Robot()
    
    # 2. ล็อก IP ของหุ่นยนต์สำหรับโหมดต่อตรง (รูปโทรศัพท์)
    config.ROBOT_IP_STR = "192.168.2.1"
    
    # 3. เปลี่ยนจาก "sta" เป็น "ap" และเพิ่ม proto_type="tcp" แก้อาการ NoneType พังตัวแรกสุด
    ep_robot.initialize(conn_type="ap", proto_type="tcp")

    ep_chassis = ep_robot.chassis

    # 订阅底盘位置信息 (รับข้อมูลตำแหน่ง - ความถี่ 1Hz)
    ep_chassis.sub_position(freq=1, callback=sub_info_handler)

    # 订阅底盘姿态信息 (รับข้อมูลองศาการเอียง - ความถี่ 5Hz)
    ep_chassis.sub_attitude(freq=5, callback=sub_info_handler)

    # 订阅底盘IMU信息 (รับข้อมูลเซนเซอร์ความเร่ง IMU - ความถี่ 10Hz)
    ep_chassis.sub_imu(freq=10, callback=sub_info_handler)

    # 订阅底盘电调信息 (รับข้อมูลกล่องควบคุมความเร็วมอเตอร์ ESC - ความถี่ 20Hz)
    ep_chassis.sub_esc(freq=20, callback=sub_info_handler)

    # 订阅底盘状态信息：(รับข้อมูลสถานะช่วงล่าง - ความถี่ 50Hz)
    ep_chassis.sub_status(freq=50, callback=sub_info_handler)

    # ปล่อยให้รับข้อมูลต่อเนื่องกันยาวๆ 10 วินาที
    time.sleep(10)

    # ยกเลิกการรับข้อมูลทั้งหมดเมื่อครบเวลา
    ep_chassis.unsub_status()
    ep_chassis.unsub_esc()
    ep_chassis.unsub_imu()
    ep_chassis.unsub_attitude()
    ep_chassis.unsub_position()
    
    ep_robot.close()