# -*-coding:utf-8-*-
import time, sys
from robomaster import robot, config
def sub_attitude_handler(attitude_info):
    yaw, pitch, roll = attitude_info
    print('{0},{1},{2}'.format(yaw, pitch, roll))
    sys.stdout.flush()
if __name__ == '__main__':
    ep_robot = robot.Robot()
    config.ROBOT_IP_STR = '192.168.2.1'
    # เปลี่ยนตรงนี้เป็น udp
    ep_robot.initialize(conn_type='ap', proto_type='udp')
    ep_chassis = ep_robot.chassis
    ep_chassis.sub_attitude(freq=10, callback=sub_attitude_handler)
    time.sleep(10)
    ep_chassis.unsub_attitude()
    ep_robot.close()
