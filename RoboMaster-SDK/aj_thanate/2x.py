import robomaster
from robomaster import robot

if __name__ == '__main__':
    # ตั้งค่า IP ของ Mac 
    robomaster.config.LOCAL_IP_STR = "192.168.2.23"
    ep_robot = robot.Robot()
    # ตั้งโหมด ap
    ep_robot.initialize(conn_type="ap")

    # เรียกใช้งานโมดูลแขนกล (Robotic Arm)
    ep_arm = ep_robot.robotic_arm

    # สั่งให้แขนกลเคลื่อนที่ไปทางขวาตามแนวแกน X เป็นระยะ 150 mm (15 cm)
    ep_arm.move(x=150, y=0).wait_for_completed()        # x เป็นบวก ขยับไปหน้า 150 mm
    
    ep_arm.move(x=-150, y=0).wait_for_completed()       # x เป็นลบ ขยับถอยหลังกลับมาที่เดิม 150 mm

    print('done')
    # 4. ปิดการเชื่อมต่อเมื่อทำงานเสร็จ
    ep_robot.close()