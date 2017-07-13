/* Pluto RC-Control
 *  AUTH: Connor Novak
 *  MAIL: connor@students.olin.edu
 *  DATE: 17/07/12
 *  VERS: 1.3
 *  DESC: Through the use of a 72 MHz transmitter-receiver pair, remote-controls a small submersible
 */

#include <Servo.h>

// Uncomment line below for output through serial monitor
#define DEBUG true

// Pin to Control Mapping
#define THROTTLE_V_PIN 2            // Left stick up-down
#define THROTTLE_H_PIN 5            // Left stick left-right
#define ELEVATOR_V_PIN 4            // Right stick up-down
#define ELEVATOR_H_PIN 3            // Right stick left-right
#define SAFETY_PIN 6                // RTS switch
#define RUNNING_LED_PIN 13           // Off/On/Active LED
#define STATUS_LED_PIN 1            // FLight Mode/Error LED
#define ACTIVATE_SWITCH 7           // Activation Reed Switch
#define THRUSTER_1_PIN 9            // Left Thruster
#define THRUSTER_2_PIN 10           // Right Thruster
#define THRUSTER_3_PIN 11           // Top Thruster

// Other constants
#define RUNNING_LED_INTERVAL 500   // LED time delay

int ch1;
int ch2;
int ch3;
int ch4;
int ch5;

unsigned long prevMillis = millis();                          // Time-tracking variable
static int minRange = 1400;                                   // Minimum transmitter signal
static int maxRange = 2400;                                   // Maximum transmitter signal
static int midRange = minRange + ((maxRange - minRange)/2);   // Mid transmitter signal
static float servoConvert = 180.0/float(maxRange - minRange); // Convert from transmitter signal to servo signal

static int deadRangeT = 30; // Dead zone band on throttle
static int deadRangeE = 30; // Dead zone band on elevator

int flightMode = 0; //Stores flight mode

//Servo Init
Servo thruster1;
Servo thruster2;
Servo thruster3;

// Declare vars for servo values, led states
float lThrust;
float rThrust;
float tThrust;
int ledState = HIGH;


/* FUNCTION: Main setup function
 ARGS: none
 RTRN: none*/
 
void setup() {
  
  // Set pin modes
  pinMode(THROTTLE_V_PIN, INPUT);
  pinMode(THROTTLE_H_PIN, INPUT);
  pinMode(ELEVATOR_V_PIN, INPUT);
  pinMode(ELEVATOR_H_PIN, INPUT);
  pinMode(SAFETY_PIN, INPUT);
  pinMode(RUNNING_LED_PIN, OUTPUT);
  thruster1.attach(THRUSTER_1_PIN);
  thruster2.attach(THRUSTER_2_PIN);
  thruster3.attach(THRUSTER_3_PIN);
  
  // Turn all props off, running LED on for safety
  lThrust = 0;
  rThrust = 0;
  tThrust = 0;
  digitalWrite(RUNNING_LED_PIN, ledState);

  #ifdef DEBUG //Starts Serial if debugging
  Serial.begin(9600);
  #endif
}

/* FUNCTION: Main loop function
 ARG: none
 RTN: none*/
 
void loop() {
  
  #ifdef DEBUG //Slows things down for debugging
  delay(500);
  #endif

  ledState = updateRunningLED(ledState);
  
  //Stores read values from receiver 
  ch1 = pulseIn(THROTTLE_H_PIN, HIGH, 25000);
  ch2 = pulseIn(THROTTLE_V_PIN, HIGH, 25000);
  ch3 = pulseIn(ELEVATOR_H_PIN, HIGH, 25000);
  ch4 = pulseIn(ELEVATOR_V_PIN, HIGH, 25000);
  ch5 = pulseIn(SAFETY_PIN, HIGH, 25000);
  
  #ifdef DEBUG // Prints receiver values if debugging
  Serial.print("DBUG: THsig: ");
  Serial.print(ch1);
  Serial.print(" | TVsig: ");
  Serial.print(ch2);
  Serial.print(" | EHSig: ");
  Serial.print(ch3);
  Serial.print(" | EVSig: ");
  Serial.print(ch4);
  Serial.print(" | SSig: ");
  Serial.println(ch5);
  #endif
      
  // Checks for RTS switch flipped
  if (ch5 > midRange) {
    
    returnToSurface(&lThrust, &rThrust, &tThrust); // Calls RTS function
  }
    
  else {
    
    // Call respective flightmode function for mixing
    if (flightMode == 0) {
      flightMode0(ch1, ch2, ch3, ch4, &lThrust, &rThrust, &tThrust);
    }

    else if (flightMode == 1) {
      //flightMode1(ch1, ch2, ch3, ch4, lThrust, rThrust, tThrust);
    }
  
    else if (flightMode == 2){
      //flightMode2(ch1, ch2, ch3, ch4, lThrust, rThrust, tThrust);
    }
    else {
      returnToSurface(&lThrust, &rThrust, &tThrust);
      #ifdef DEBUG // Print Error message if debugging
      Serial.println("EROR: Flight mode not registered");
      #endif
      
    }
  }
  
  // Write calculated values to thrusters
  thruster1.write(lThrust);
  thruster2.write(rThrust);
  thruster3.write(tThrust);
  
  
  #ifdef DEBUG // Print out thrust values if debugging
  Serial.print("DBUG: Left: ");
  Serial.print(int(lThrust));
  Serial.print(" | Right: ");
  Serial.print(int(rThrust));
  Serial.print(" | Vertical: ");
  Serial.println(int(tThrust));
  #endif
}

/* FUNCTION: Update running LED on a given interval
 *  ARGS: none
 *  RTNS: none
 */
int updateRunningLED(int state) {
  unsigned long currMillis = millis();
  if (currMillis - prevMillis > RUNNING_LED_INTERVAL) {
    prevMillis = currMillis;
    if (state == LOW) {
      state = HIGH;
      Serial.println("True");
    }
    else if (state == HIGH) {
      state = LOW;
      Serial.println("False");
    }
    else {
      #ifdef DEBUG
      Serial.println("EROR: LED state value non-binary!");
      #endif
    }

    digitalWrite(RUNNING_LED_PIN, state);
  }
  return state;
 }
 
/* FUNCTION: Give thruster inputs for flight mode 0 (stabilize)
 ARG: throttle vert. @ horiz. signals, elevator vert. @ horiz. signals
 RTN: none*/
 
void flightMode0(int THsig, int TVsig, int EHsig, int EVsig, float *lThrust, float *rThrust, float *tThrust) {
  
  // Clean signals
  float power = getPower(TVsig);
  THsig = midRange - cleanSignal(THsig);
  
  #ifdef DEBUG // Print out throttle signal if debugging
  Serial.print("DBG: cleaned THsig: ");
  Serial.println(THsig);
  #endif
  
  EVsig = midRange = cleanSignal(EVsig);
  /*if (debug) {
    Serial.print("DBG: cleaned EVsig: ");
    Serial.println(EVsig);
  }*/
  
  // Turning right
  if (THsig > deadRangeT) {
    *lThrust = power * 180;
    *rThrust = *lThrust - power * (servoConvert) * THsig;
  }

  // Turning left
  else if (THsig < -deadRangeT) {
    *rThrust = power * 180;
    *lThrust = *rThrust - power * (servoConvert) * -THsig;
  }
  
  // Going straight
  else {
    *rThrust = power * 180;
    *lThrust = power * 180;
  }
  
  // Going up
  if (EVsig > deadRangeT) {
    *tThrust = power * servoConvert *  EVsig;
  }
  
  // Going down
  else {
    *tThrust = power * servoConvert * -EVsig;
  }
}

/* FUNCTION: Return sub to surface
 ARG: none
 RTN: none*/
 
void returnToSurface(float *lThrust, float *rThrust, float *tThrust) {
  *lThrust = 0;
  *rThrust = 0;
  *tThrust = 180;
}

/* FUNCTION: Calculate 0-1000 power scale
 ARG: integer signal from throttle stick vertical
 RTN: float power, cleaned and scaled*/
 
float getPower(int THsig) {
  
    float power = THsig - minRange;
    if (power < 0) {power = 0;} // Zeros out negative power
    if (power > 1000) {power = 1000;} // Max power @ 1000
    
    // Prints power modifier
    /*if(debug) {
      Serial.print("DBG: Power modifier: ");
      Serial.println(power/1000);
    }*/
    
    return (power/1000);
}

/* FUNCTION: Chop signal to between minRange & maxRange
 ARG: integer signal to clean
 RTN: cleaned signal*/

int cleanSignal(int sig) {
  if (sig < minRange) {sig = minRange;};
  if (sig > maxRange) {sig = maxRange;};
  return sig;
}
