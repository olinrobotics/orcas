#include <Servo.h>
// Tugboat Code

Servo propellers;
Servo rudder;

int pos = 0;
boolean has_run = false;

int prop_pos = 95;  // approximately off
int rudd_pos = 90;  // middle

bool propellers_on = false;

#define RUDDER_MIDDLE 90
#define RUDDER_RANGE 40

void setup() {
  Serial.begin(9600);
  Serial.println("Ready to accept input: prop angle, rudder angle");

  propellers.attach(9);
  propellers.write(prop_pos);
  rudder.attach(10);
  rudder.write(rudd_pos);

  Serial.println("command.");
}

void loop() {
  while (Serial.available() > 0) {

    prop_pos = Serial.parseInt();
    rudd_pos = Serial.parseInt();
  }

  propellers.write(prop_pos);
  rudder.write(rudd_pos);
}
