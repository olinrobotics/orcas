#include "servo.h"

/******************************************************************************
 * @file        servo.cpp
 * Servo class for OAK (Olin Autonomous Kore)
 * @author      Connor Novak
 * @email       connor@students.olin.edu
 * @version     1.0
 * @date        17/07/17
 ******************************************************************************/

// Function & variable declaractions needed because class is static
ros::Publisher *signalIn;
std_msgs::Byte Servo::servo_signal;

void Servo::setup(ros::NodeHandle *nh){
  signalIn = new ros::Subscriber("/servo", &servo_signal)
  nh->subscribe(*signalIn); // tells ROS about the subscriber
}

void Servo::servoCB(){

}
