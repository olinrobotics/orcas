#include <Servo.h>
#include <SoftwareSerial.h>

// XBee's DOUT (TX) is connected to pin 2 (Arduino's Software RX)
// XBee's DIN (RX) is connected to pin 3 (Arduino's Software TX)
SoftwareSerial XBee(2, 3); // RX, TX

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
   servo3.attach(servoPin3);


// For each servo, send 'stop' signal, then wait for ESC to recognize
  servo1.writeMicroseconds(1500);
  delay(1000);
  servo2.writeMicroseconds(1500);
  delay(1000);
  servo3.writeMicroseconds(1500);
  delay(1000);

  XBee.begin(9600); // Init XBee radio
  Serial.begin(9600); // opens the serial port
  
  XBee.write("TST: Comms to XBee");
}

void loop() {
  
  // If data comes in from XBee, send it out to serial monitor
  if (XBee.available()){
    char inChar = XBee.read();

//    if (inChar - '0' > 1100 && inChar - '0' < 1900){
//      signal = inChar - '0';
//    }
    
    if (inChar == 'a'){
    servo1.writeMicroseconds(signal); // Send signal to ESC.
      Serial.print("MSG: Working . . .");
      delay(1000);
      Serial.print("Set servo 1 to " + String(signal) + "\n");
    }
    else if (inChar == 'b'){
      servo2.writeMicroseconds(signal);
      Serial.print("MSG: Working . . .");
      delay(1000);
      Serial.print("Set servo 2 to " + String(signal) + "\n");
    }
    else if (inChar == 'c'){
      servo3.writeMicroseconds(signal);
      Serial.print("MSG: Working . . .");
      delay(1000);
      Serial.print("Set servo 3 to " + String(signal) + "\n");
    }
    else if (inChar == 's'){
      Serial.print("MSG: Working .");
      servo1.writeMicroseconds(stop);
      delay(1000);
      Serial.print(" .");
      servo2.writeMicroseconds(stop);
      delay(1000);
      Serial.print(" .");
      servo3.writeMicroseconds(stop);
      delay(1000);
      Serial.print("Stopped all servos\n");
    }
    else{
      Serial.print("MSG: Unknown Input\n");
    }
  }
}
