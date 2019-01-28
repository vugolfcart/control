#!/usr/bin/env python

import rospy
import numpy as np
import cv2
from std_msgs.msg import Image
from keras.models import load_model
from control.msg import drive_param

model = load_model('model.h5')
control_drive_parameters = rospy.Publisher('control_drive_parameters', drive_param, queue_size=10)


def offhook():
    control_drive_parameters = rospy.Publisher('control_drive_parameters', drive_param, queue_size=10)
    message = drive_param()
    message.velocity = 0
    message.angle = 0
    control_drive_parameters.publish(message)


def callback(message):
    print "[#callback]: received message"
    data = np.array(map(ord, message.data), dtype=np.uint8)
    image = data.reshape(message.height, message.width, 3)

    # BGR -> RGB
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    image = image[188:, 0:672, 0:3]
    image = cv2.resize(image, (320, 160))
    
    angle = float(model.predict(image, batch_size=1))
    message = drive_param()
    message.velocity = 27
    message.angle = angle
    control_drive_parameters.publish(message)

    
def main():
    rospy.on_shutdown(offhook)
    rospy.init_node('control_behavioral_cloning', anonymous=True)
    rospy.Subscriber("zed/rgb/image_rect_color", Image, callback)
    rospy.spin()


if __name__ == '__main__':
    main()
