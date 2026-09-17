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


from robomaster import robot
from robomaster import config  # นำเข้า config เพื่อกำหนดค่า IP โดยตรง


if __name__ == '__main__':
    ep_robot = robot.Robot()
    
    # 1. กำหนด IP ของหุ่นยนต์ผ่าน config เพื่อไม่ให้เกิด AttributeError
    config.ROBOT_IP_STR = "192.168.2.1"
    
    # 2. บังคับใช้ proto_type="tcp" เพื่อแก้ปัญหาการเชื่อมต่อหลุดตัวแรกสุด
    # หมายเหตุ: ถ้า Mac ต่อ Wi-Fi ของหุ่นยนต์โดยตรง ให้เปลี่ยน "sta" เป็น "ap"
    ep_robot.initialize(conn_type="ap", proto_type="tcp")

    ep_chassis = ep_robot.chassis

    x_val = 0.1
    y_val = 0.1
    z_val = 90

    # 前进 0.5米 (เดินหน้า 0.5 เมตร)
    ep_chassis.move(x=x_val, y=0, z=0, xy_speed=0.7).wait_for_completed()

    # 后退 0.5米 (ถอยหลัง 0.5 เมตร)
    ep_chassis.move(x=-x_val, y=0, z=0, xy_speed=0.7).wait_for_completed()

    # 左移 0.6米 (สไลด์ซ้าย 0.6 เมตร)
    ep_chassis.move(x=0, y=-y_val, z=0, xy_speed=0.7).wait_for_completed()

    # 右移 0.6米 (สไลด์ขวา 0.6 เมตร)
    ep_chassis.move(x=0, y=y_val, z=0, xy_speed=0.7).wait_for_completed()

    # 左转 90度 (หมุนซ้าย 90 องศา)
    ep_chassis.move(x=0, y=0, z=z_val, z_speed=45).wait_for_completed()

    # 右转 90度 (หมุนขวา 90 องศา)
    ep_chassis.move(x=0, y=0, z=-z_val, z_speed=45).wait_for_completed()
    print("Done")
    ep_robot.close()