/* Tugboat Code
 *  
*/

#include <Servo.h>

Servo myservo;
Servo rudder;// create servo object to control a servo
// twelve servo objects can be created on most boards

int pos = 0;
boolean has_run = false;

void setup() {
  myservo.attach(9);
  rudder.attach(5);
  //rudder.write(90);
  // attaches the servo on pin 9 to the servo object
}

void loop() {
  delay(100);
  rudder.write(90);
    //go forward
    for (int i = 0; i <= 8; i++) {
      for (pos = 90; pos <= 180; pos += 1) {
        myservo.write(pos);
        delay(5);
      }
    }

    delay(30);

    //turn rudder
    rudder.write(70);

    //continue moving in the direction of the turn
    for (int i = 0; i <= 8; i++) {
      for (pos = 90; pos <= 180; pos += 1) {
        myservo.write(pos);
        delay(5);
      }
    }
    delay(30);

    //stop moving to the side, move straight
    rudder.write(90);
    for (int i = 0; i <= 8; i++) {
      for (pos = 90; pos <= 180; pos += 1) {
        myservo.write(pos);
        delay(5);
      }
    }
    delay(30);

    //turn to be back on track
    rudder.write(120);

    //move back towards original track
    for (int i = 0; i <= 8; i++) {
      for (pos = 90; pos <= 180; pos += 1) {
        myservo.write(pos);
        delay(5);
      }
    }
    delay(30);

    //continue straight
    rudder.write(90);
    for (int i = 0; i <= 8; i++) {
      for (pos = 90; pos <= 180; pos += 1) {
        myservo.write(pos);
        delay(5);
        myservo.detach();
      }
    }
}



