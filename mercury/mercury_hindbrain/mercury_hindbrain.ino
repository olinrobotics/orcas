// Mercury Hind Brain Code
// Version 1.0 17/07/07
// Used: FastLED Blink Ex.

#include "FastLED.h"

// Variable Declarations/Initializations
static int ledPin = 13;                 // runtime LED pin
unsigned long intervalLED = 1000;       // LED time delay
unsigned long prevMillis = millis();    // Time-tracking variable
boolean debug = true;                   // Serial output switch
int ledState = HIGH;                    // Binary LED state indicator
char message = 'g';                     // I/O storage variable from MidBrain 

// Define array of 1 rgb led
#define DATA_PIN 0
#define NUM_LEDS 1
CRGB leds[NUM_LEDS];

/* FUNCTION: Main setup
 *  ARG: none
 *  RTN: none
 */
void setup() {
  // Sets up runtime LED, toggles on
  pinMode (ledPin, OUTPUT);
  digitalWrite(ledPin, ledState);

  // Sets up status LED
  FastLED.addLeds<WS2812B, DATA_PIN, GRB>(leds, NUM_LEDS);
  
  if (debug) {Serial.begin(9600);}
  leds[0] = CRGB::Green;
  FastLED.show();
}
//------------------------------------------------------------\\

/* FUNCTION: Main loop
 * ARG: none
 * RTN: none
 */
void loop() {

  // Read Midbrain commands
  if (Serial.available()) {
    message = Serial.read();
    if (debug) {
      Serial.print("MSG: Recieved: ");
      Serial.println(message);
    } 
  }
  
  if (message == 's') {
    Serial.println("ERR: E-Stop command sent");
    leds[0] = CRGB::Red;
    FastLED.show();
    while(true) {}
    
  }
  // Sense: Read Robot Sensors
  // Think: Run low level cognition and safety code
  unsigned long currMillis = millis();
  if (currMillis - prevMillis > intervalLED) {
    prevMillis = currMillis;
    ledState = toggleLED(ledState);
  }

  // Act: Run actuators and behavior lights

  // Write status data up to MidBrain

}
//---------------------------------------------------------------\\

/* FUNCTION: Given current state of LED, toggle LED to other state
 * ARG: integer representing LED state (LOW/HIGH)
 * RTN: integer representing updated LED state (HIGH/LOW)
 */
int toggleLED(int state) {
  if (state == LOW) {
    state = HIGH;
  }
  else if (state == HIGH) {
    state = LOW;
  }
  else {
    if (debug) {Serial.println("ERR: LED state value non-binary!");}
  }
  digitalWrite(ledPin, state);
  return state;
}
