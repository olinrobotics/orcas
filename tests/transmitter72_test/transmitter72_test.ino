int ch1;
int ch2;
int ch3;
int ch4;
int ch5;
static int pin1 = 2;
static int pin2 = 3; //Right stick left-right
static int pin3 = 4; //Right stick up-down
static int pin4 = 5; //Left stick left-right
static int pin5 = 6;

void setup() {
  
  // Set receiver pins to input
  pinMode(pin1, INPUT);
  pinMode(pin2, INPUT);
  pinMode(pin3, INPUT);
  pinMode(pin4, INPUT);
  pinMode(pin5, INPUT);

  Serial.begin(9600);
}

void loop() {
  
  //Stores read values from receiver
  ch1 = pulseIn(pin1, HIGH, 25000);
  ch2 = pulseIn(pin2, HIGH, 25000);
  ch3 = pulseIn(pin3, HIGH, 25000);
  ch4 = pulseIn(pin4, HIGH, 25000);
  ch5 = pulseIn(pin5, HIGH, 25000);
  
    Serial.print("Channel 2:");
    Serial.println(ch1);
  
    Serial.print("Channel 3:");
    Serial.println(ch2);
  
    Serial.print("Channel 4:");
    Serial.println(ch3);
    
    Serial.print("Channel 5:");
    Serial.println(ch4);
    
    Serial.print("Channel 6:");
    Serial.println(ch5);
    
    Serial.println("--------------------");
    delay(1000);
    
}
