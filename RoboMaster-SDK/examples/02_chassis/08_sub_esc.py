# -*-coding:utf-8-*-
import time, sys
from robomaster import robot, config
def sub_esc_handler(esc_info):
    speed, angle, timestamp, state = esc_info
    print('{0},{1},{2},{3}'.format(speed, angle, timestamp, state))
    sys.stdout.flush()
if __name__ == '__main__':
    ep_robot = robot.Robot()
    config.ROBOT_IP_STR = '192.168.2.1'
    ep_robot.initialize(conn_type='ap', proto_type='tcp')
    ep_chassis = ep_robot.chassis
    ep_chassis.sub_esc(freq=10, callback=sub_esc_handler)
    time.sleep(10)
    ep_chassis.unsub_esc()
    ep_robot.close()
