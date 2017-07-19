#include <Arduino.h>

//ROS library and messages
#include "ros.h"

//User classes
#include "config.h"
#include "rosswitch.h"

// ROS variables
ros::NodeHandle nh;
RosSwitch *sw;

void setup() {
  nh.getHardware()->setBaud(ROSSERIAL_BAUD);
  nh.initNode(); // Initialize ROS nodehandle
  sw->setup(&nh);
  sw->onSwitch(onPos);
  sw->offSwitch(offPos);
  pinMode(13, OUTPUT);
}

void loop() {
  delay(100);
  nh.spinOnce();
}

void onPos(){
  digitalWrite(13, HIGH);
}

void offPos(){
  digitalWrite(13, LOW);
}
