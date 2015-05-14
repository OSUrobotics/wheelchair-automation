#!/usr/bin/env python

import rospy
from rqt_topic.topic_info import TopicInfo
from rqt_top.node_info import NodeInfo
import curses
import numpy as np
from geometry_msgs.msg import PoseArray

def find_laser_drivers(node_info):
    return [n for n in node_info.get_all_node_info() if n[1].name == 'hokuyo_node']

def get_laser_status(laser_rate):
    if laser_rate == 0:
        status_text = 'NO DATA     '
        color = curses.color_pair(curses.COLOR_RED)
    elif 0 < laser_rate < 7:
        status_text = 'SLOW (%0.2fHz)   ' % laser_rate
        color = curses.color_pair(curses.COLOR_YELLOW)
    elif laser_rate >= 7:
        status_text = 'OK (%0.2fHz)     ' % laser_rate
        color = curses.color_pair(curses.COLOR_GREEN)
    else:
        status_text = 'UNKNOWN     '
        color = curses.color_pair(curses.COLOR_RED)
    return status_text, color

def get_laser_driver_status(node_name, node_info):
    try:
        proc = node_info.get_node_info(node_name)
        if proc.is_running():
            return 'RUNNING', curses.color_pair(curses.COLOR_GREEN)
        else:
            return 'NOT RUNNING', curses.color_pair(curses.COLOR_RED)

    except IOError:
        return 'NOT RUNNING', curses.color_pair(curses.COLOR_RED)

posearray = None

def posearray_cb(msg):
    global posearray
    posearray = np.array([(p.position.x, p.position.y, p.position.z) for p in msg.poses])

if __name__ == '__main__':
    rospy.init_node('wheelchair_monitor')
    ti_left_laser   = TopicInfo('/wheelchair_lasers/left', 'sensor_msgs/LaserScan')
    ti_right_laser  = TopicInfo('/wheelchair_lasers/right', 'sensor_msgs/LaserScan')
    ti_merged_laser = TopicInfo('/scan_multi', 'sensor_msgs/LaserScan')

    rospy.Subscriber('/particlecloud', PoseArray, posearray_cb)

    node_info = NodeInfo()

    ti_left_laser.start_monitoring()
    ti_right_laser.start_monitoring()
    ti_merged_laser.start_monitoring()

    ti_left_laser.window_size = 10
    ti_right_laser.window_size = 10
    ti_merged_laser.window_size = 10

    rospy.sleep(1)

    try:
        screen = curses.initscr()
        curses.start_color()
        curses.use_default_colors()
        curses.curs_set(0)

        curses.init_pair(curses.COLOR_RED, curses.COLOR_RED, -1)
        curses.init_pair(curses.COLOR_YELLOW, curses.COLOR_YELLOW, -1)
        curses.init_pair(curses.COLOR_GREEN, curses.COLOR_GREEN, -1)

        rate = rospy.Rate(0.5)
        while not rospy.is_shutdown():
            left_laser_rate  = ti_left_laser.get_hz()[0]
            right_laser_rate = ti_right_laser.get_hz()[0]

            screen.addstr(0, 0, 'Left Laser', curses.A_NORMAL)
            screen.addstr(4, 0, 'Right Laser', curses.A_NORMAL)

            driver_txt = '  Driver: '
            data_txt = '  Data  : '

            # set the left driver and data lines
            screen.addstr(1, 0, driver_txt, curses.A_NORMAL)
            screen.addstr(2, 0, data_txt, curses.A_NORMAL)

            # update the left laser driver status
            status_text, color = get_laser_driver_status('/left_laser_driver', node_info)
            screen.addstr(1, len(driver_txt), status_text, color)

            # update the left laser topic status
            status_text, color = get_laser_status(left_laser_rate)
            screen.addstr(2, len(data_txt), status_text, color)

            # set the right driver and data lines
            screen.addstr(5, 0, driver_txt, curses.A_NORMAL)
            screen.addstr(6, 0, data_txt, curses.A_NORMAL)

            # update the left laser driver status
            status_text, color = get_laser_driver_status('/right_laser_driver', node_info)
            screen.addstr(5, len(driver_txt), status_text, color)

            # update the right laser topic status
            status_text, color = get_laser_status(right_laser_rate)
            screen.addstr(6, len(data_txt), status_text, color)

            screen.addstr(8, 0, 'Localization Status:')

            if posearray is None:
                # screen.addstr(8, 21, 'UNKNOWN', curses.color_pair(curses.COLOR_YELLOW))
                text = 'UNKNOWN'
                color = curses.color_pair(curses.COLOR_YELLOW)
            else:
                std = posearray.std(0).sum()
                if std <= 1:
                    text = 'GOOD'
                    color = curses.color_pair(curses.COLOR_GREEN)
                elif 1 < std < 2:
                    text = 'FAIR'
                    color = curses.color_pair(curses.COLOR_YELLOW)
                else:
                    text = 'POOR'
                    color = curses.color_pair(curses.COLOR_RED)
            screen.addstr(8, 21, text, color)


            screen.refresh()
            rate.sleep()

        curses.endwin()
    except Exception, e:
        curses.endwin()
        print e
