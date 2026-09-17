# -*-coding:utf-8-*-
import time, sys
from robomaster import robot, config
def sub_imu_handler(imu_info):
    acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z = imu_info
    print('{0},{1},{2},{3},{4},{5}'.format(acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z))
    sys.stdout.flush()
if __name__ == '__main__':
    ep_robot = robot.Robot()
    config.ROBOT_IP_STR = '192.168.2.1'
    ep_robot.initialize(conn_type='ap', proto_type='tcp')
    ep_chassis = ep_robot.chassis
    ep_chassis.sub_imu(freq=10, callback=sub_imu_handler)
    time.sleep(10)
    ep_chassis.unsub_imu()
    ep_robot.close()
