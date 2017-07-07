// Mercury Hind Brain Code
// Version 1.0 17/07/07

// Variable Declarations/Initializations
static int ledPin = 13;
int delayPeriod = 50;

void setup() {
  pinMode (ledPin, OUTPUT); // Sets up main light

}

void loop() {

  // Read Midbrain commands

  // Sense: Read Robot Sensors

  // Think: Run low level cognition and safety code
  digitalWrite(ledPin, HIGH);
  delay(delayPeriod);
  digitalWrite(ledPin, LOW);
  delay(delayPeriod);

  // Act: Run actuators and behavior lights

  // Write status data up to MidBrain

}
