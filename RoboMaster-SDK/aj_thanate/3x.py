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

    # สั่งให้แขนกลเคลื่อนที่ไปทางขวาตามแนวแกน Y เป็นระยะ 150 mm (15 cm)
    ep_arm.move(x=0, y=150).wait_for_completed()        # y เป็นบวก ขยับขึ้น 150 mm
    
    ep_arm.move(x=0, y=-150).wait_for_completed()       # y เป็นลบ ขยับกลับลงมาที่เดิม 150 mm
    
    print('done')
    # ปิดการเชื่อมต่อเมื่อทำงานเสร็จ
    ep_robot.close()