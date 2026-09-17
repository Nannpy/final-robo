# -*-coding:utf-8-*-
import time, sys
from robomaster import robot, config
def sub_position_handler(position_info):
    x, y, z = position_info
    print('{0},{1},{2}'.format(x, y, z))
    sys.stdout.flush()
if __name__ == '__main__':
    ep_robot = robot.Robot()
    config.ROBOT_IP_STR = '192.168.2.1'
    ep_robot.initialize(conn_type='ap', proto_type='tcp')
    ep_chassis = ep_robot.chassis
    ep_chassis.sub_position(freq=10, callback=sub_position_handler)
    time.sleep(10)
    ep_chassis.unsub_position()
    ep_robot.close()
