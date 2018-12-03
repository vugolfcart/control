#!/usr/bin/python

import rospy
from control.msg import drive_param

minimum_velocity = 30
miscalibration_angle = 9.2


def offhook():
    pub = rospy.Publisher('control_drive_parameters', drive_param, queue_size=10)
    msg = drive_param()
    msg.velocity = 0
    msg.angle = 0
    pub.publish(msg)


def circle():
    rospy.init_node('control_straight', anonymous=True)
    pub = rospy.Publisher('control_drive_parameters', drive_param, queue_size=10)
    rospy.on_shutdown(offhook)

    rate = rospy.Rate(25)
    while not rospy.is_shutdown():
        msg = drive_param()
        msg.velocity = minimum_velocity
        msg.angle = 0 + miscalibration_angle
        pub.publish(msg)
        rate.sleep()


if __name__ == '__main__':
    try:
        circle()
    except rospy.ROSInterruptException:
        pass
