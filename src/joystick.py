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
velocity = 0
increment = 15


def offhook():
    control_drive_parameters = rospy.Publisher('control_drive_parameters', drive_param, queue_size=10)
    message = drive_param()
    message.velocity = 0
    message.angle = 0
    control_drive_parameters.publish(message)


def main():
    rospy.on_shutdown(offhook)
    global angle
    global velocity

    key = ''
    while key != ord('q'):
        key = stdscr.getch()
        stdscr.refresh()
        if key == curses.KEY_LEFT and angle > -100 + increment:
            angle -= increment
        elif key == curses.KEY_RIGHT and angle < 100 - increment:
            angle += increment
        elif key == curses.KEY_UP and velocity < 100 + increment:
            velocity += increment
        elif key == curses.KEY_DOWN and velocity > -100 + increment:
            velocity -= increment

        stdscr.addstr(0, 0, "angle: ")
        stdscr.addstr(0, 7, '%3.2f' % angle)
        stdscr.addstr(1, 0, "velocity: ")
        stdscr.addstr(1, 10, '%3.2f' % velocity)

        message = drive_param()
        message.velocity = velocity
        message.angle = angle
        control_drive_parameters.publish(message)

    curses.endwin()

if __name__ == '__main__':
    main()
