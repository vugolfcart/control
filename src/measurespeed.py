#!/usr/bin/env python

import math
import rospy
import argparse
from control.msg import drive_param
from sensor_msgs.msg import LaserScan
import time

pub = rospy.Publisher('control_drive_parameters', drive_param, queue_size=1)
parser = argparse.ArgumentParser(description='Measuring speed using known dist')
parser.add_argument("measured_dist", type=float, help="Distance to travel")
parser.add_argument("speed", type=int, help="Speed at which to travel")
args = parser.parse_args()

start_time = time.time()
start_dist = 0
started = False
ended = False
elapsed_time = 0

measured_dist = args.measured_dist
speed = args.speed

msg = drive_param()
msg.angle = 0


def offhook():
    msg = drive_param()
    msg.velocity = 0
    msg.angle = 0
    pub.publish(msg)


def getrange(data, theta):
    step = [int(i/data.angle_increment) for i in theta]
    print('step', step)
    distance = [data.ranges[i] for i in step]
    print('distance', distance)
    return distance


def ready(meas_dist):
    global start_time
    global start_dist
    global started

    start_time = time.time()
    start_dist = meas_dist[0]
    started = True


def finished():
    global ended
    global elapsed_time

    elapsed_time = time.time() - start_time
    ended = True


def callback(data):
    global msg
    rospy.on_shutdown(offhook)
    distance = getrange(data, 0.5*(data.angle_max-data.angle_min))

    if not started:
        ready(distance[0])

    dif = distance[0] - start_dist

    if (started and not finished) and dif < measured_dist:
        msg.velocity = speed
        pub.publish(msg)
    elif (started and not finished) and dif >= measured_dist:
        msg.velocity = 0
        pub.publish(msg)
        finished()
    elif started and ended:
        msg.velocity = 0
        pub.publish(msg)
        print ('Velocity:', (measured_dist/elapsed_time))


if __name__ == '__main__':
    print("Measuring Speed")
    rospy.init_node('measure_speed', anonymous=True)
    rospy.Subscriber("scan", LaserScan, callback)
    rospy.spin()
