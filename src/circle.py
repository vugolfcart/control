#!/usr/bin/python

import rospy
from f1tenths_controller.msg import drive_param
rospy.init_node("circle_driver", anonymous=True)
pub = rospy.Publisher('drive_parameters', drive_param, queue_size=10)
while True:
    msg = drive_param()
    msg.velocity = 20
    msg.angle = 90
    pub.publish(msg)
