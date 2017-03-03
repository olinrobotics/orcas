#include <Servo.h>

byte servoPin1 = 8;
byte servoPin2 = 9;
byte servoPin3 = 10;
Servo servo1;
Servo servo2;
Servo servo3;
int stop = 1500; // stop value, which is 1500 (center of values for rotation servo)
int signal = 1600; // Set signal value, which should be between 1100 and 1900
int corresponding_signal = 1900;

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

  if (signal > 1500){
    corresponding_signal = 1500 - (signal-1500);
  }
  else if (signal < 1500){
    corresponding_signal = 1500 + (1500-signal);
  }
  else{
    corresponding_signal = 1500;
  }
  // check for incoming serial data:
  if (Serial.available() > 0) {
    // read incoming serial data:
    char inChar = Serial.read();
    if (inChar == 'w'){
        Serial.println("Going forward");
        servo1.writeMicroseconds(signal); // Send signal to ESC.
        servo2.writeMicroseconds(signal);
        servo3.writeMicroseconds(stop);
        delay(1000);
    }
    else if (inChar == 'a'){
        Serial.println("Going left");
        servo1.writeMicroseconds(signal); // Send signal to ESC.
        servo2.writeMicroseconds(corresponding_signal);
        servo3.writeMicroseconds(stop);
        delay(1000);
    }
    else if (inChar == 'd'){
        Serial.println("Going right");
        servo1.writeMicroseconds(corresponding_signal); // Send signal to ESC.
        servo2.writeMicroseconds(signal);
        servo3.writeMicroseconds(stop);
        delay(1000);
    }
    else if (inChar == 's'){
        Serial.println("Going backwards");
        servo1.writeMicroseconds(corresponding_signal);
        servo2.writeMicroseconds(corresponding_signal);
        servo3.writeMicroseconds(stop);
        delay(1000);
    }
    else if (inChar == 'k' && signal < 1900){
        signal = signal + 100;
        Serial.println("Increasing the signal to: ");
        Serial.println(signal);
    }
    else if (inChar == 'j' && signal > 1100){
        signal = signal - 100;
        Serial.println("Decreasing the signal to: ");
        Serial.println(signal);
    }
    else if (inChar == 'u'){
        Serial.println("Going up");
        servo1.writeMicroseconds(stop);
        servo2.writeMicroseconds(stop);
        servo3.writeMicroseconds(signal);
        delay(1000);
    }
    else if (inChar == 'i'){
        Serial.println("Going down");
        servo1.writeMicroseconds(stop);
        servo2.writeMicroseconds(stop);
        servo3.writeMicroseconds(corresponding_signal);
        delay(1000);
    }
    else if (inChar == ' '){
      Serial.println("Stopping all motors!");
      servo1.writeMicroseconds(stop);
      servo2.writeMicroseconds(stop);
      servo3.writeMicroseconds(stop);
      delay(1000);
    }
}
}
