/* Pluto RC Control
 * DATE: 17/10/06
 * VERS: 1.1
 * DESC: RC control of a submarine through 72-MHz Radio
 */

// Include Libraries
#include <Servo.h>

//#include <rc_control_func.ino>
const byte WATER_SENSOR_PIN = A4;

// Debug Settings
boolean DEBUG_MOTORS = false;
boolean DEBUG = false;

// Declare Variables
byte LED_BLINK_DELAY;
SoftwareSerial XBee(2, 3); // RX, TX

// maybe delete if it doesn't work
int waterSensorReading;
boolean waterSensorState;

void setup() {

  // Set pin modes
  pinMode(WATER_SENSOR_PIN, INPUT);
  
  // Initialize Variables
 // LED_BLINK_DELAY = 500;        // ms

  // Set actuators to initial positions

  // Turn on LED positional system

  // Start Serial connections
  Serial.begin(9600);
  
}

void loop() {

  // ---------- SENSE ----------

  // Read E-Stop
  //Read Water Sensor
  waterSensorReading = analogRead(WATER_SENSOR_PIN);
  
  // Read Depth Sensor
  // Read Accelerometer
  // Read Gyroscope
  // Read Reciever
  // Read Switch
  // Read Radio
  
  // Read Battery Voltage
  
  // ---------- THINK ----------

    // Water Sensor Logic
    if (waterSensorReading == 0) {
      waterSensorState = LOW;
      }
    }
    
    else {
      waterSensorState= HIGH;
      }
      
  /* Should sub E-Stop?
   * Check: E-Stop, Water Sensor, Battery Sensor, Receiver E-Stop, Receiver Connectivity
   */

  /* Convert receiver signals into motor commands
   * Throttle for speed, left stick for yaw
   */

  /* Convert receiver signals into pump commands
   *  Right stick for raise/lower
   */

  /* Update & check LED timer
   *  See if current time since last blink is greater than blink delay
   */
  
  // ---------- ACT ----------

  // Estop if water sensor reads water
  if (waterSensorState==HIGH) {
    eStop();
  }
  
   // Write settings to thrusters
  // Write LED settings
  // Write pump settings

