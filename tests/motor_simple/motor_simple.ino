#include <Servo.h>

byte servoPin = 9;
Servo servo;

void setup() {
	servo.attach(servoPin);

	servo.writeMicroseconds(1500); // send "stop" signal to ESC.
	delay(1000); // delay to allow the ESC to recognize the stopped signal
        Serial.begin(9600);
}

void loop() {
	int signal = 1300; // Set signal value, which should be between 1100 and 1900

	servo.writeMicroseconds(signal); // Send signal to ESC.
        Serial.write("Signal sent!");
        delay(1000);
}
