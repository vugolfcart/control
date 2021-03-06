import csv
import cv2
import numpy as np
from sklearn.utils import shuffle
from sklearn.model_selection import train_test_split
from keras.models import Sequential
from keras.layers import Conv2D, Cropping2D, Dense, Dropout, Flatten, Lambda, MaxPooling2D
import rosbag
import argparse
from cv_bridge import CvBridge, CvBridgeError


def get_data(path):
    bridge = CvBridge()
    bag = rosbag.Bag(path)
    topics = [
        '/zed/rgb/image_rect_color',
        '/control_drive_parameters'
    ]

    angle = 0.0
    count = 0

    images = []
    labels = []

    for topic, message, timestamp in bag.read_messages(topics=topics):
        print('{}: [{}]: {}'.format(timestamp, topic, ''))

        if topic == '/zed/rgb/image_rect_color':
            count += 1

            try:
                image = bridge.imgmsg_to_cv2(message)

                # crop and resize
                image = image[188:, 0:672, 0:3]
                image = cv2.resize(image, (320, 160))

                # convert to grayscale
                weights = [1, 0, 0] # BGR (standard luminescence: [0.114, 0.587, 0.299])
                weights = np.array([weights]).reshape((1,3))
                image = cv2.transform(image, weights)

                cv2.imshow('image', image)
                cv2.waitKey(0)

                print('[{}] message=({}, {}), shape=({})'.format(count, message.height, message.width, image.shape))

                images.append(image)
                labels.append(angle)
            except CvBridgeError as e:
                print(e)

        elif topic == '/control_drive_parameters':
            angle = message.angle

    bag.close()

    return np.array(images), np.array(labels)


def get_model(activation_type='elu', dropout=0.3):
    model = Sequential()
    model.add(Cropping2D(cropping=((60, 25), (0, 0)), input_shape=(160, 320, 3)))
    model.add(Lambda(lambda x: (x / 255) - .5))
    model.add(Conv2D(24, (5, 5), strides=(2, 2), activation=activation_type))
    # model.add(MaxPooling2D())
    model.add(Dropout(dropout))
    model.add(Conv2D(36, (5, 5), strides=(2, 2), activation=activation_type))
    # model.add(MaxPooling2D())
    model.add(Dropout(dropout))
    model.add(Conv2D(48, (5, 5), strides=(2, 2), activation=activation_type))
    # model.add(MaxPooling2D())
    model.add(Dropout(dropout))
    model.add(Conv2D(64, (3, 3), activation=activation_type))
    # model.add(MaxPooling2D())
    model.add(Dropout(dropout))
    model.add(Conv2D(64, (3, 3), activation=activation_type))
    # model.add(MaxPooling2D())
    model.add(Dropout(dropout))
    model.add(Flatten())
    model.add(Dense(1162, activation=activation_type))
    model.add(Dense(100, activation=activation_type))
    model.add(Dense(50, activation=activation_type))
    model.add(Dense(10, activation=activation_type))
    model.add(Dense(1))

    return model
  

def main():
    parser = argparse.ArgumentParser(description='[behavioral cloning model] choose bag')
    parser.add_argument("path", type=str, help="path to bag")
    args = parser.parse_args()

    model = get_model()
    data, labels = get_data(args.path)

    print('[#main]: training the model')
    model.compile(optimizer='adam', loss='mse')
    model.fit(data, labels, batch_size=128, epochs=7, validation_split=.2)

    print('[#main]: saving to model.h5')
    model.save('model.h5')


if __name__ == '__main__':
    main()
