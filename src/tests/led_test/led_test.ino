void setup() {
  // initialize digital pins 12 & 13 for LEDS
  pinMode(12, OUTPUT);
  pinMode(13, OUTPUT);
}

void loop() {
  digitalWrite(12, OUTPUT);   // turn the LED on (HIGH is the voltage level)
  digitalWrite(13, OUTPUT);                  // wait for a second
}
