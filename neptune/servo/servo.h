#ifndef SERVO_H // include guard
#define SERVO_H

/******************************************************************************
 * @file        servo.h
 * Header file for OAK (Olin Autonomous Kore) Servo class
 * @author      Connor Novak
 * @email       connor@students.olin.edu
 * @version     1.0
 * @date        17/07/17
 ******************************************************************************/

 #include "ros.h"
 #include "std_msgs/Byte.h"
 #include "config.h"

#ifndef SERVO_PIN
  #error SERVO_PIN NEEDS TO BE DEFINED
#endif

class Servo{
public:
  void setup(ros::NodeHandle *nh);
private:
  ros::Subscriber *signalIn;
  std_msgs::Byte servo_signal;
  void servoCB(std_msgs::Byte sig);

};

#endif //SERVO_H
