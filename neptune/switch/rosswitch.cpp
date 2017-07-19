#include "rosswitch.h"

/******************************************************************************
 * Switch class for OAK (Olin Autonomous Kore)
 * @author: Connor Novak
 * @email: connor@students.olin.edu
 *
 * This is meant to be a modular class for any robot within the lab
 * it automatically creates a subscriber and publisher for switches and buttons
 * true is stop
 *
 * TODO: add support for multiple estop pins
 * TODO: check the state of the pin(s) on change
 *
 * TO USE:
 *  1- in the config header #define SWITCH_PIN and SWITCH_DEBOUNCE_TIME
 *  2- include the header in the .ino file (#include "switch.h")
 *  3- declare a pointer to the class (Switch *s)
 *  4- within void setup do the following:
 *    a- call the setup function with the memory address of the nodehandle
 *       (s->setup(&nh))
 *    b- set the function you want to run on an activation (e->onActive(on))
 *       where on is a function
 *    c- set the function you want to run after an activation
 *       (e->offActive(off)) where off is a function
 *  5- that is it...the class will take care of the rest
 ******************************************************************************/

// Function & variable declarations needed bc class is static
ros::Publisher *RosSwitchswitchState;
std_msgs::Bool RosSwitch::state;
long RosSwitch::last_mill;
void RosSwitch::defaultOnSwitch(){};
void RosSwitch::defaultOffSwitch(){};
void (*RosSwitch::offfunc)() = defaultOffSwitch;
void (*RosSwitch::onfunc)() = defaultOnSwitch;

void RosSwitch::setup(ros::NodeHandle *nh){
  switchState = new ros::Publisher("/rosswitch", &state);
  nh->advertise(*switchState);
  last_mill = millis();
  pinMode(SWITCH_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(SWITCH_PIN), onChange, CHANGE);
  state.data = !digitalRead(SWITCH_PIN);
}

/*
 * Function that runs on switch activation
 * If activating - calls the onfunc and publishes true
 * If disactivating - calls the offfunc and publishes false
 */
void RosSwitch::onChange(){
  if(millis()-last_mill >= SWITCH_DEBOUNCE_TIME*50){
    state.data = !state.data;
    switchState->publish(&state);
    if(state.data){
      (*RosSwitch::onfunc)();
    }
    else{
      (*RosSwitch::offfunc)();
    }
    last_mill = millis();
  }
}

/*
 * Set on function pointer to function pointer
 */
void RosSwitch::onSwitch(void (*func)()){
  onfunc = func;
}

/*
 * Set off function pointer to function pointer
 */
 void RosSwitch::offSwitch(void (*func)()){
   offfunc = func;
 }
