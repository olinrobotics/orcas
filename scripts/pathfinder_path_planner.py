#!/usr/bin/env python
import sys
import rospy
import roslib
roslib.load_manifest('orcas')
from std_msgs.msg import Header, ColorRGBA
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Vector3, Pose, Point, Quaternion
from visualization_msgs.msg import Marker
from orcas.msg import BoatMotorCommand
from tf.transformations import quaternion_from_euler
# path plan in python
import math

PLAN_USING_EMPTY = False

class PathPlanner(object):
    """ ROS node to plan a path for the Pathfinder Boat """
    def __init__(self):
        super(PathPlanner, self).__init__()
        rospy.init_node('path_planner')
        self.scan_sub = rospy.Subscriber("structured_light_scan", LaserScan, self.on_scan)
        self.path_pub = rospy.Publisher("path_plan", Marker, queue_size=10)
        self.motor_pub = rospy.Publisher("motor_plan", BoatMotorCommand, queue_size=10)

    def on_scan(self, laser_scan):
        gap_left, gap_right, left_is_empty, right_is_empty = path_location(laser_scan.ranges)
        sys.stderr.write("{} [{} {}] {}\n".format(
            "<   " if left_is_empty else "<!!!",
            gap_left,
            gap_right,
            "   >" if right_is_empty else "!!!>"
        ))

        path_header = Header(
            stamp=rospy.Time.now(),
            frame_id='webcam_frame'
        )

        gap_left_angle = gap_left * laser_scan.angle_increment + laser_scan.angle_min
        # check to see if there's an obstacle left of the found gap
        if left_is_empty and PLAN_USING_EMPTY:
            gap_left_angle = laser_scan.angle_min

        gap_right_angle = gap_right * laser_scan.angle_increment + laser_scan.angle_min
        # check to see if there's an obstacle right of the found gap
        if right_is_empty and PLAN_USING_EMPTY:
            gap_right_angle = laser_scan.angle_max

        desired_angle = (
            gap_left_angle + (gap_right_angle - gap_left_angle) * 0.5
        ) * 3.0  # arbitrary

        motor_power = 1.0
        if gap_right_angle - gap_left_angle < math.radians(5.0):
            motor_power = 0.1
            desired_angle = 0.0

        self.path_pub.publish(
            header=path_header,
            type=Marker.ARROW,
            pose=self.get_pose(desired_angle),
            scale=Vector3(0.5 * motor_power, 0.03, 0.03),
            color=ColorRGBA(0.0, 0.0, 1.0, 0.4)
        )

        self.motor_pub.publish(
            header=path_header,
            propeller_angle=126 if motor_power > .5 else 95,
            rudder_angle=int(-math.degrees(desired_angle) + 90)
        )

    def get_pose(self, angle):
        orientation_tuple = quaternion_from_euler(
            0,
            0,
            angle
        )

        return Pose(
            position=Point(x=0, y=0, z=0),
            orientation=Quaternion(
                x=orientation_tuple[0],
                y=orientation_tuple[1],
                z=orientation_tuple[2],
                w=orientation_tuple[3]
            )
        )


def path_location(range_array):
    count = 0
    longest_pos = -1
    threshold = 0.5
    location_range = [0, 0]  # indices
    length = len(range_array)
    for i in range(length):
        if range_array[i] >= threshold and not math.isnan(range_array[i]):
            count += 1
            if longest_pos < count:
                longest_pos = count
                location_range[0] = i - longest_pos + 1
                location_range[1] = i
        else:
            count = 0
        i += 1
    center_point = (location_range[0] + location_range[1]) / 2.0

    left_is_empty = all(math.isnan(v) for v in range_array[0:location_range[0]])
    right_is_empty = all(math.isnan(v) for v in range_array[location_range[1] + 1:])


    return location_range[0], location_range[1], left_is_empty, right_is_empty


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
