#!/usr/bin/env python

import math
import rospy
import argparse
from control.msg import drive_param
from sensor_msgs.msg import LaserScan

parser = argparse.ArgumentParser(description='PID control choose dist, speed, angle, and mode')
parser.add_argument("max_dist", type=float, help="Maximum Distance")
parser.add_argument("max_speed", type=int, help="Maximum Speed from 0 to 100")
parser.add_argument("max_angle", type=int, help="Maximum Steering Angle from 0 to 100")
parser.add_argument("mode", type=str, choices=['centering', 'following'], help="self centering or wall following")
args = parser.parse_args()

fwd_target_dist = args.max_dist
side_target_dist = args.max_dist

max_speed = args.max_speed
max_angle = args.max_angle

speed_pid = [25, 0, 0.09]
angle_pid = [30, 0, 0.25]

prev_dist_error = 0
sum_dist_error = 0
prev_angle_error = 0
sum_angle_error = 0

mode = args.mode

pub = rospy.Publisher('control_drive_parameters', drive_param, queue_size=1)


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


def calculateresponse(cur_error=0, sum_error=0, dif_error=0, param='none'):
    if param == 'speed':
        pid = speed_pid
    elif param == 'angle':
        pid = angle_pid
    else:
        pid = [0, 0, 0]

    p_value = pid[0]*cur_error
    i_value = pid[1]*sum_error
    d_value = pid[2]*dif_error

    correction = p_value + i_value + d_value

    return correction


def calculateerror(cur_error=0, prev_error=0, sum_error=0):
    d_time = 1
    sum_error += cur_error * d_time
    dif_error = ((prev_error - cur_error) / d_time)
    return sum_error, dif_error


def callback(data):
    global prev_angle_error
    global prev_dist_error
    global sum_angle_error
    global sum_dist_error

    rospy.on_shutdown(offhook)

    #angle_range = math.degrees(data.angle_max - data.angle_min)
    angle_range = data.angle_max - data.angle_min
    print('angle ranges', angle_range)
    cur_dist_error = 0
    cur_angle_error = 0

    forward = 0.5 * angle_range
    actual_dists = [0, 0, 0]

    if mode == 'centering':
        right = 0.4 * angle_range
        left = 0.6 * angle_range
        angles = [forward, left, right]
        print('angles', angles)
        actual_dists = getrange(data, angles)

        cur_angle_error = actual_dists[2] - actual_dists[1]

    else:
        angle1 = math.radians(45)
        angle2 = math.radians(60)
        angles = [forward, angle1, angle2]

        actual_dists = getrange(data, angles)

        swing = math.radians(angle2 - angle1)

        alpha = math.atan2((actual_dists[2] * math.cos(swing)) - actual_dists[1], actual_dists[2] * math.sin(swing))
        AB = actual_dists[1] * math.cos(alpha)
        AC = 1
        CD = AB + (AC * math.sin(alpha))

        cur_angle_error = -1*(CD - side_target_dist)

    cur_dist_error = actual_dists[0] - fwd_target_dist
    print ('distances: [%s]' % ', '.join(map(str, actual_dists)))

    sum_dist_error, dif_dist_error = calculateerror(cur_dist_error, prev_dist_error, sum_dist_error)
    sum_angle_error, dif_angle_error = calculateerror(cur_angle_error, prev_angle_error, sum_angle_error)

    speed = calculateresponse(cur_dist_error, sum_dist_error, dif_dist_error, 'speed')
    angle = calculateresponse(cur_angle_error, sum_angle_error, dif_dist_error, 'angle')

    prev_dist_error = cur_dist_error
    prev_angle_error = cur_angle_error

    speed = max(min(speed, max_speed), -1*max_speed)
    if speed < 0:
        speed *= 10
    #min max
    angle = max(min(angle, max_angle), -1*max_angle)

    msg = drive_param()
    msg.velocity = speed
    msg.angle = angle
    print('speed: %f' % speed)
    print('angle: %f' % angle)
    pub.publish(msg)


if __name__ == '__main__':
    print("PID control started")
    rospy.init_node('pid_control', anonymous=True)
    rospy.Subscriber("scan", LaserScan, callback)
    rospy.spin()
