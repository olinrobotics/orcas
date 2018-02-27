#include <Servo.h>
#include <SoftwareSerial.h>

/* DOCSTRING
Control scheme for Not-The-Remus to be controlled remotely through an XBee radio.
packet_fwd - forward at [signal value]
packet_bck - backward at [signal value]
packet_lft - rotate left at [signal value]
packet_rgt - rotate right at [signal value]
packet_fst - increase [signal value] (speed up)
packet_slw - decrease [signal value] (slow down)
packet_flt - up at [signal value]
packet_snk - down at [signal value]
packet_stp -
*/

// XBee's DOUT (TX) is connected to pin 2 (Arduino's Software RX)
// XBee's DIN (RX) is connected to pin 3 (Arduino's Software TX)
SoftwareSerial XBee(2, 3); // RX, TX

byte servoPin1 = 8;
byte servoPin2 = 9;
byte servoPin3 = 10;
Servo servo1;
Servo servo2;
Servo servo3;
int stop = 1500; // stop value: 1500 (center of values for rotation servo)
int signal = 1700; // Set signal value (between 1100 and 1900)
int corresponding_signal = 1900;
int pause = 10; // ms to pause btw messages
int counter = 0; // counter to determine if it has been too long since we have received signals
boolean state = true; // Setting for printing or running

void setup() {


  servo1.attach(servoPin1);
  servo2.attach(servoPin2);
  servo3.attach(servoPin3);
  pinMode(12, OUTPUT);


  servo1.writeMicroseconds(1500); // send "stop" signal to ESC.
  delay(pause); // delay to allow the ESC to recognize the stopped signal
  servo2.writeMicroseconds(1500);
  delay(pause);
  servo3.writeMicroseconds(1500);
  delay(pause);

  XBee.begin(9600); // Init XBee radio
  Serial.begin(9600); // opens the serial port

  Serial.println("Doing the setup");

  XBee.write("TST: Comms to XBee");
}

void loop() {

  counter = counter + 1; // update the counter each time we enter the loop

  // Updates corresponding signal (forward vs reverse, etc.)
  if (signal > 1500){
    corresponding_signal = 1500 - (signal-1500);
  }
  else if (signal < 1500){
    corresponding_signal = 1500 + (1500-signal);
  }
  else{
    corresponding_signal = 1500;
  }

  // check for incoming data through XBee:
  if (XBee.available()) {
    counter = 0;
    // read incoming data:
    char inChar = XBee.read();
    Serial.println(inChar);
    digitalWrite(12, HIGH);
    delay(pause);
    digitalWrite(12, LOW);

    if (inChar == 'f'){
      if (state == true) Serial.println("Going forward");
        XBee.write("Going forward");
        servo1.writeMicroseconds(signal); // Send signal to ESC.
        delay(pause);
        servo2.writeMicroseconds(signal);
        delay(pause);
        servo3.writeMicroseconds(stop);
        delay(pause);
    }

    else if (inChar == 'l'){
      if (state == true) Serial.println("Going left");
        XBee.write("Going left");
        servo1.writeMicroseconds(signal); // Send signal to ESC.
        delay(pause);
        servo2.writeMicroseconds(corresponding_signal);
        delay(pause);
        servo3.writeMicroseconds(stop);
        delay(pause);
    }

    else if (inChar == 'r'){
      if (state == true) Serial.println("Going right");
        XBee.write("Going right");
        servo1.writeMicroseconds(corresponding_signal); // Send signal to ESC.
        delay(pause);
        servo2.writeMicroseconds(signal);
        delay(pause);
        servo3.writeMicroseconds(stop);
        delay(pause);
    }

    else if (inChar == 'b'){
      if (state == true) Serial.println("Going backwards");
        XBee.write("Going backwards");
        servo1.writeMicroseconds(corresponding_signal);
        delay(pause);
        servo2.writeMicroseconds(corresponding_signal);
        delay(pause);
        servo3.writeMicroseconds(stop);
        delay(pause);
    }

    else if (inChar == 't' && signal < 1900){
      signal = signal + 10;
      String strSig = String(signal);
      String message = "Increasing the signal to " + strSig;
      char const* msg = message.c_str();
      if (state == true){Serial.println(msg);
      XBee.write(msg);
    }}

    else if (inChar == 's' && signal > 1100){
      signal = signal - 10;
      String strSig = String(signal);
      String message = "Decreasing the signal to " + strSig;
      char const* msg = message.c_str();
      if (state == true){Serial.println(msg);
      XBee.write(msg);
    }}

    else if (inChar == 'o'){
      if (state == true) Serial.println("Going up");
        XBee.write("Going up");
        servo1.writeMicroseconds(stop);
        delay(pause);
        servo2.writeMicroseconds(stop);
        delay(pause);
        servo3.writeMicroseconds(signal);
        delay(pause);
    }

    else if (inChar == 'n'){
      if (state == true) Serial.println("Going down");
        XBee.write("Going down");
        servo1.writeMicroseconds(stop);
        delay(pause);
        servo2.writeMicroseconds(stop);
        delay(pause);
        servo3.writeMicroseconds(corresponding_signal);
        delay(pause);
    }

    else if (inChar == 'x'){
      if (state == true) Serial.println("Stopping all motors!");
        XBee.write("Stopping all motors");
        servo1.writeMicroseconds(stop);
        delay(pause);
        servo2.writeMicroseconds(stop);
        delay(pause);
        servo3.writeMicroseconds(stop);
        delay(pause);
    }
  }
}
