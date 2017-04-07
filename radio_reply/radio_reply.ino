#include <SoftwareSerial.h>

// XBee's DOUT (TX) is connected to pin 2 (Arduino's Software RX)
// XBee's DIN (RX) is connected to pin 3 (Arduino's Software TX)
SoftwareSerial XBee(2, 3); // RX, TX

void setup() {

  XBee.begin(9600); // Init XBee radio
  Serial.begin(9600); // opens the serial port
  
  XBee.write("TST: Comms to XBee");
}

void loop() {
  
  // If data comes in from XBee, send it out to serial monitor
  if (XBee.available()){
    char inChar = XBee.read();
    Serial.print("Ok! I recieved: " + String(inChar));
  }
}
