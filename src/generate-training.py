#!/usr/bin/env python

import rospy
from control.msg import drive_param
import curses
from time import time

rospy.init_node('control_keyboard', anonymous=True)
control_drive_parameters = rospy.Publisher('control_drive_parameters', drive_param, queue_size=10)

stdscr = curses.initscr()
curses.cbreak()
stdscr.keypad(1)
stdscr.refresh()

angle = 0
increment = 4


def offhook():
    control_drive_parameters = rospy.Publisher('control_drive_parameters', drive_param, queue_size=10)
    message = drive_param()
    message.velocity = 0
    message.angle = 0
    control_drive_parameters.publish(message)


# def on_frame(data):
#     global angle
    # print(data)

    # message = drive_param()
    # message.velocity = 30
    # message.angle = angle
    # control_drive_parameters.publish(message)

    # unique_timestamp = (''.join(str(time()).split('.')))[:15]
    # filename = '{}.json'.format(unique_timestamp, angle)
    # stdscr.addstr(1, 0, 'outputting to: {}'.format(filename))


def main():
    rospy.on_shutdown(offhook)
    global angle
    # rospy.Subscriber('control_drive_parameters', drive_param, on_frame)

    key = ''
    while key != ord('q'):
        key = stdscr.getch()
        stdscr.refresh()
        if key == curses.KEY_LEFT and angle > -100 + increment:
            angle -= increment
            stdscr.addstr(0, 0, "angle")
            stdscr.addstr(0, 6, '%3.2f' % angle)
        elif key == curses.KEY_RIGHT and angle < 100 - increment:
            angle += increment
            stdscr.addstr(0, 0, "angle")
            stdscr.addstr(0, 6, '%3.2f' % angle)

        message = drive_param()
        message.velocity = 27
        message.angle = angle
        control_drive_parameters.publish(message)

    curses.endwin()

if __name__ == '__main__':
    main()
