#include <SoftwareSerial.h>

// XBee's DOUT (TX) is connected to pin 2 (Arduino's Software RX)
// XBee's DIN (RX) is connected to pin 3 (Arduino's Software TX)
SoftwareSerial XBee(2, 3); // RX, TX

int time_buffer = 0; // Keeps track of length of time since last ping
boolean comms = 0;

void setup() {
  
    XBee.begin(9600); // Init XBee radio
  Serial.begin(9600); // opens the serial port
  
  XBee.write("TST: Comms to XBee");
  
}

void loop() {
  if (XBee.available()) {
    char msg = XBee.read();
    time_buffer = 0;
    if (msg == 'p') {
      Serial.println("MSG: Recieved ping!");
      comms = true;
    }
  }
  else {
    time_buffer++;
    if (time_buffer >= 200) {
      Serial.println("ERR: Did not recieve ping!");
      comms = false;
    }
  }
}
