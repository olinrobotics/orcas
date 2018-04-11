void setup() {
  Serial.begin(115200); // use the same baud-rate as the python side
}
void loop() {
    // Python to Arduino
    if(Serial.available() > 0) {
    char data = Serial.read();
    char str[2];
    str[0] = data;
    str[1] = '\0';
    Serial.print(str);
    }
  // Arduino to Python
  //Serial.println("Hello world from Arduino!"); // write a string
  //delay(1000);
}

