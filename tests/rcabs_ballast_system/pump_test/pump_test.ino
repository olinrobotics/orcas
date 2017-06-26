static int pumpPin = 2;
static int valvePin = 3;

void setup() {
  pinMode(pumpPin, OUTPUT);
  pinMode(valvePin, OUTPUT);
}

void loop() {
  digitalWrite(valvePin, LOW);
  digitalWrite(pumpPin, HIGH);
  delay(3000);
  digitalWrite(pumpPin, LOW);
  digitalWrite(valvePin, HIGH);
  delay(8000);
}

