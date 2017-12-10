#include <Wire.h>                                 // I2C Communication with MPU
#include <Adafruit_MotorShield.h>                 // Motorshield library
#include "utility/Adafruit_MS_PWMServoDriver.h"   // Lets motor shield talk to motor
#include "I2Cdev.h"
#include "MPU6050.h"


#define LED_PIN 13
bool blinkState = false;

// Create the motor shield object with the default I2C address
const int SHIELD_addr = 0x61;
Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61);
Adafruit_DCMotor *myMotor = AFMS.getMotor(1);


//MPU constants/variables
MPU6050 accelgyro;
const int MPU_addr = 0x68; // I2C address of the MPU-6050
int16_t ax, ay, az;
int16_t gx, gy, gz;

// accelerometer values
int accel_reading;
int accel_corrected;
int accel_offset = 200;
float accel_angle;
float accel_scale = 1; // set to 0.01
float x;

// gyro values
int gyro_offset = 151; // 151
int gyro_corrected;
int gyro_reading;
float gyro_rate;
float gyro_scale = 0.02; // 0.02 by default - tweak as required
float gyro_angle;
float loop_time = 0.05; // 50ms loop
float angle = 0.00; // value to hold final calculated gyro angle

// time stamp variables
int last_update;
int cycle_time;
long last_cycle = 0;


void setup() { // ----------S----------S----------S----------S----------S----------S----------S
  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();

  // initialize serial communication
  Serial.begin(9600);

  // initialize device
  Serial.println("Initializing I2C devices...");
  accelgyro.initialize();

  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  // configure Arduino LED for
  pinMode(LED_PIN, OUTPUT);
  Wire.begin();//opens communication

  Wire.beginTransmission(MPU_addr);//sets variables with the given address
  Wire.write(0x6B);  // PWR_MGMT_1 register or (data, length)
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);//does the actual transmitting

  Serial.begin(9600);//sets serial monitor
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("Adafruit Motorshield v2 - DC Motor test!");

  AFMS.begin();  // create with the default frequency 1.6KHz
  //AFMS.begin(1000);  // OR with a different frequency, say 1KHz

  // Set the speed to start, from 0 (off) to 255 (max speed)
  myMotor->setSpeed(150);
  myMotor->run(FORWARD);
  // turn on motor
  myMotor->run(RELEASE);
}


void loop() { // ----------L----------L----------L----------L----------L----------L----------L----------L
  //SENSE
  // read raw accel/gyro measurements from device
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // accelerometer_X_Axis angle calc
  accel_reading = ax;
  accel_corrected = accel_reading - accel_offset;
  accel_corrected = map(accel_corrected, -16800, 16800, -90, 90);
  accel_corrected = constrain(accel_corrected, -90, 90);
  accel_angle = (float)(accel_corrected * accel_scale);
  Serial.print(accel_angle);
  Serial.print("\t");

  // gyro_Y_Axis angle calc
  gyro_reading = gy;
  gyro_corrected = (float)((gyro_reading / 131) - gyro_offset); // 131 is sensivity of gyro from data sheet
  gyro_rate = (gyro_corrected * gyro_scale) * -loop_time;      // loop_time = 0.05 ie 50ms
  gyro_angle = angle + gyro_rate;

  Serial.print(gyro_angle);

  Serial.println(" ");

  // blink LED to indicate activity
  blinkState = !blinkState;
  digitalWrite(LED_PIN, blinkState);

  //timestamp
  time_stamp();
  translate();
}

// ----------F----------F----------F----------F----------F----------F----------F----------F

void time_stamp() {
  while ((millis() - last_cycle) < 50) {
    delay(1);
  }
  // once loop cycle reaches 50ms, reset timer value and continue
  cycle_time = millis() - last_cycle;
  last_cycle = millis();
}
//THINK

//ACT
void translate() {
  x = gyro_angle;
  Serial.println(gyro_angle);
  if (x > 0)
    {myMotor->setSpeed(x);
    Serial.println("vle");}
  if (x < 0)
    { myMotor->setSpeed(-x);
    Serial.println("gwe");

}}
