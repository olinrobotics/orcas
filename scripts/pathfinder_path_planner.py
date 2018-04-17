#!/usr/bin/env python
import sys
import rospy
from sensor_msgs.msg import LaserScan

# path plan in python
import math


class PathPlanner(object):
    """ ROS node to plan a path for the Pathfinder Boat """
    def __init__(self):
        super(PathPlanner, self).__init__()
        rospy.init_node('path_planner')
        self.scan_sub = rospy.Subscriber("structured_light_scan", LaserScan, self.on_scan)

    def on_scan(self, laser_scan):
        gap_center, gap_width = path_location(laser_scan.ranges)
        sys.stderr.write("{} {}\n".format(gap_center, gap_width))


def path_location(range_array):
    count = 0
    longest_pos = -1
    threshold = 100.0
    location_range = [0, 0]  # indices
    length = len(range_array)
    for i in range(length):
        if range_array[i] <= threshold or math.isnan(range_array[i]):
            count = 0
        else:
            count += 1
            if longest_pos < count:
                longest_pos = count
                location_range[0] = i - longest_pos + 1
                location_range[1] = i
        i += 1
    center_point = (location_range[0] + location_range[1]) / 2.0

    return center_point, length


def rudder_pos(gap_center, range_length):
    '''
    normalized values:
    r = 30, l = -30, s = 0
    not normalized values:
    r = 120, l = 60, s = 90
    '''
    slope = 30 / (range_length / 2.0)
    intercept = -30.0
    position = ((slope * gap_center) + intercept) + 90.0

    return int(position)

if __name__ == '__main__':
    # test = [70, 60, 50, 10, 70, 60, 50, 50, 50, 70]
    # gap_center, range_length = path_location(test)
    # print(rudder_pos(gap_center, range_length))
    path_planner = PathPlanner()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
