#include <Servo.h>

int ch1;
int ch2;
int ch3;
int ch4;
int ch5;

//Pin to Control Mapping
static int throttleVertPin = 2; //Left stick up-down
static int throttleHorizPin = 5; //Left stick left-right
static int elevatorVertPin = 4; //Right stick up-down
static int elevatorHorizPin = 3; //Right stick left-right
static int safetyPin = 6; //RTS switch

static int minRange = 1400; // Minimum transmitter signal
static int maxRange = 2500; // Maximum transmitter signal
static int midRange = minRange + ((maxRange - minRange)/2); // Mid transmitter signal
static float servoConvert = 180/(maxRange - minRange); // Convert from transmitter signal to servo signal

static int deadRange = 30; // Dead zone band on transmitter

static boolean debug = true; //Set True if Serial debugging
int flightMode = 0; //Stores flight mode

//Pin to Motor Mapping
static int t1Pin = 9;
Servo thruster1;
static int t2Pin = 10;
Servo thruster2;
static int t3Pin = 11;
Servo thruster3;

// Declare vars for servo values
float lThrust;
float rThrust;
float tThrust;


/* FUNCTION: Sets up various things for program
 ARG: none
 RTN: none*/
 
void setup() {
  
  // Set receiver pins to input; thruster pins to output
  pinMode(throttleVertPin, INPUT);
  pinMode(throttleHorizPin, INPUT);
  pinMode(elevatorVertPin, INPUT);
  pinMode(elevatorHorizPin, INPUT);
  pinMode(safetyPin, INPUT);
  thruster1.attach(t1Pin);
  thruster2.attach(t2Pin);
  thruster3.attach(t3Pin);
  
  // TODO: Turn all props off for safety
  lThrust = 0;
  rThrust = 0;
  tThrust = 0;

  if (debug) {Serial.begin(9600);} //Starts Serial if debugging
}

/* FUNCTION: Main loop body for program
 ARG: none
 RTN: none*/
 
void loop() {
  
  if (debug) {delay(500);} // Slow things down for debugging
  
  //Stores read values from receiver 
  ch1 = pulseIn(throttleHorizPin, HIGH, 25000);
  ch2 = pulseIn(throttleVertPin, HIGH, 25000);
  ch3 = pulseIn(elevatorHorizPin, HIGH, 25000);
  ch4 = pulseIn(elevatorVertPin, HIGH, 25000);
  ch5 = pulseIn(safetyPin, HIGH, 25000);
  
/*  //Prints receiver values if debugging
  if(debug) {
    Serial.print("DBG: THsig: ");
    Serial.print(ch1);
    Serial.print(" | TVsig: ");
    Serial.print(ch2);
    Serial.print(" | EHSig: ");
    Serial.print(ch3);
    Serial.print(" | EVSig: ");
    Serial.print(ch4);
    Serial.print(" | SSig: ");
    Serial.println(ch5);
  }*/
      
  // Checks for RTS switch flipped
  if (ch5 > midRange) {
    
    //Calls RTS function
    returnToSurface(&lThrust, &rThrust, &tThrust);
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
      if(debug) {Serial.println("ERR: Flight mode not registered");}
      
    }
  }
  
  // Write calculated values to thrusters
  thruster1.write(lThrust);
  thruster2.write(rThrust);
  thruster3.write(tThrust);
  
  // Print out thrust values
  if (debug){
    Serial.print("DBG: Left: ");
    Serial.print(int(lThrust));
    Serial.print(" | Right: ");
    Serial.print(int(rThrust));
    Serial.print(" | Vertical: ");
    Serial.println(int(tThrust));
  }
}

/* FUNCTION: Give thruster inputs for flight mode 0 (stabilize)
 ARG: throttle vert. @ horiz. signals, elevator vert. @ horiz. signals
 RTN: none*/
 
void flightMode0(int THsig, int TVsig, int EHsig, int EVsig, float *lThrust, float *rThrust, float *tThrust) {
  
  float power = getPower(TVsig);
  THsig = midRange - cleanSignal(THsig);
  if  (debug) {
    Serial.print("DBG: cleaned THsig: ");
    Serial.println(THsig);
  }
  
  // Turning right
  if (THsig > deadRange) {
    *lThrust = power * 180;
    *rThrust = power * servoConvert * THsig;
  }
  
  // Turning left
  else if (THsig < -deadRange) {
    *rThrust = power * 180;
    *lThrust = power * servoConvert * (float(midRange) + 2 * THsig);
  }
  
  // Going straight
  else {
    *rThrust = power * 180;
    *lThrust = power * 180;
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

/* FUNCTION: Slam signal between minRange & maxRange
 ARG: integer signal to clean
 RTN: cleaned signal*/

int cleanSignal(int sig) {
  if (sig < minRange) {sig = minRange;};
  if (sig > maxRange) {sig = maxRange;};
  return sig;
}
