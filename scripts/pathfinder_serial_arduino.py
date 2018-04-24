#!/usr/bin/env python
import sys
import roslib
roslib.load_manifest('orcas')
import rospy
import serial
from orcas.msg import BoatMotorCommand


SERIAL_DEV = "/dev/ttyACM0"
SERIAL_BAUD = 9600


class MotorCommander(object):
    """ ROS to send motor commands to arduino """
    def __init__(self):
        super(MotorCommander, self).__init__()
        rospy.init_node('motor_commander')
        self.motor_plan_sub = rospy.Subscriber("motor_plan", BoatMotorCommand, self.on_plan_msg)
        self.sent_messages = 0

        try:
            self.serial_conn = serial.Serial(SERIAL_DEV, SERIAL_BAUD, timeout=.1)
            sys.stderr.write("{}\n".format(self.serial_conn))
            sys.stderr.flush()
        except serial.serialutil.SerialException:
            sys.stderr.write("{}\n".format("Couldn't connect to arduino!"))
            sys.stderr.flush()
            self.serial_conn = None


    def on_plan_msg(self, motor_command):
        self.sent_messages += 1
        serial_message = "{},{} ".format(
            motor_command.propeller_angle,
            motor_command.rudder_angle
        )
        if self.serial_conn is not None and self.serial_conn.isOpen():
            self.serial_conn.write(serial_message.encode('ascii', 'replace'))
            sys.stderr.write("from arduino: {}\n".format(self.serial_conn.readline()))
        sys.stderr.write("{} motor: {}\n".format(self.sent_messages, serial_message))
        sys.stderr.flush()


if __name__ == '__main__':
    motor_commander = MotorCommander()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
