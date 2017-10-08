/* Pluto RC Control
 * DATE: 17/10/06
 * VERS: 1.1
 * DESC: RC control of a submarine through 72-MHz Radio
 */

// Include Libraries

// Debug Settings
boolean DEBUG_MOTORS = false;

// Declare Variables
byte LED_BLINK_DELAY;

void setup() {

  // Set pin modes
  
  // Initialize Variables
  LED_BLINK_DELAY = 500;        // ms

  // Set actuators to initial positions

  // Start Serial connections
  
}

void loop() {

  // ---------- SENSE ----------

  // Read E-Stop
  // Read Water Sensor
    int waterSenseRead;
    int Grove_Water_Sensor = LOW;  
  // Read Depth Sensor
  // Read Accelerometer
  // Read Gyroscope
  // Read Reciever
  // Read Switch
  // Read Radio
  // Read Battery Voltage
  
  // ---------- THINK ----------
  //Read Water Sensor
   boolean waterSensorState= analogRead(Grove_Water_Sensor);
    if LOW
  /* Should sub E-Stop?
   * Check: E-Stop, Water Sensor, Battery Sensor, Receiver E-Stop, Receiver Connectivity
   */

  /* 
   * 
   */
  
  // ---------- ACT ----------
  //Read Water Sensor
  // Write settings to thrusters
  // Write LED settings
  // Write pump settings
  
}
