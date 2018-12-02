#!/usr/bin/python

import rospy
from f1tenths_controller.msg import drive_param

def offhook():
	pub = rospy.Publisher('drive_parameters', drive_param, queue_size=10)
	msg = drive_param()
	msg.velocity = 0
	msg.angle = 0
	pub.publish(msg)

def circle():
	rospy.init_node('circle_driver', anonymous=True)
	pub = rospy.Publisher('drive_parameters', drive_param, queue_size=10)
	rospy.on_shutdown(offhook)

	msg = drive_param()

	r = rospy.Rate(25)
	while not rospy.is_shutdown():

	    msg.velocity = 30
	    msg.angle = 9.2
	    pub.publish(msg)

	    r.sleep()


if __name__ == '__main__':
	try:
		circle()
	except rospy.ROSInterruptException:
		pass
