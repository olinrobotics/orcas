#ifndef ROS_SWITCH_H  // include guard
#define ROS_SWITCH_H

/******************************************************************************
 * Switch class for OAK (Olin Autonomous Kore)
 * @author: Connor Novak
 * @email: connor@students.olin.edu
 ******************************************************************************/

#include "ros.h"
#include "std_msgs/Bool.h"
#include "config.h"

#ifndef SWITCH_PIN
  #error SWITCH PIN NEEDS TO BE DEFINED
#endif

#ifndef SWITCH_DEBOUNCE_TIME
  #pragma SWITCH DEBOUNCE TIME NOT DEFINED - DEFAULTING TO 1
  #define SWITCH_DEBOUNCE_TIME 1 //half milliseconds
#endif

class RosSwitch{
public:
  static void setup(ros::NodeHandle *nh);
  static void onSwitch(void (*func)());
  static void offSwitch(void (*func)());

private:
  static ros::Publisher *switchState;
  static std_msgs::Bool state;
  static long last_mill;
  static void (*offfunc)();
  static void (*onfunc)();
  static void onChange();
  static void defaultOnSwitch();
  static void defaultOffSwitch();
  
};

#endif //ROS_SWITCH_H
