#include <Servo.h>

byte servoPin1 = 8;
byte servoPin2 = 9;
byte servoPin3 = 10;
Servo servo1;
Servo servo2;
Servo servo3;
int stop = 1500; // stop value, which is 1500 (center of values for rotation servo)
int signal = 1100; // Set signal value, which should be between 1100 and 1900

void setup() {
  servo1.attach(servoPin1);
  servo2.attach(servoPin2);
  // servo3.attach(servoPin3);


  servo1.writeMicroseconds(1500); // send "stop" signal to ESC.
  delay(1000);
  servo2.writeMicroseconds(1500);
  // servo3.writeMicroseconds(1500);
  delay(1000); // delay to allow the ESC to recognize the stopped signal

  Serial.begin(9600); // opens the serial port
}

void loop() {

  // check for incoming serial data:
  if (Serial.available() > 0) {
    // read incoming serial data:
    char inChar = Serial.read();
    if (inChar - '0' > 1100 && inChar - '0' < 1900){
      signal = inChar - '0';
    }
    Serial.print(inChar- '0');
    if (inChar == 'a'){
        servo1.writeMicroseconds(signal); // Send signal to ESC.
        delay(1000);
    }
    else if (inChar == 'b'){
        servo2.writeMicroseconds(signal);
        delay(1000);
    }
    /*else if (inChar == 'c'){
        servo3.writeMicroseconds(signal);
        delay(1000);
    }*/
    else if (inChar == 's'){
        servo1.writeMicroseconds(stop);
        delay(1000);
        servo2.writeMicroseconds(stop);
        delay(1000);
        // servo3.writeMicroseconds(stop);
        // delay(1000);
    }
}
}
