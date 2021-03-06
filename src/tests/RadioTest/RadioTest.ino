#include <EnableInterrupt.h>

#define SERIAL_PORT_SPEED 9600
#define RC_NUM_CHANNELS  4

#define RC_CH1  0
#define RC_CH2  1
#define RC_CH3  2
#define RC_CH4  3

#define RC_CH1_INPUT  A0
#define RC_CH2_INPUT  A1
#define RC_CH3_INPUT  A2
#define RC_CH4_INPUT  A3


uint16_t rc_values[RC_NUM_CHANNELS];
uint32_t rc_start[RC_NUM_CHANNELS];
volatile uint16_t rc_shared[RC_NUM_CHANNELS];

const int motorPin = 9;//attaches the motor to pin 9
int inputValue = 0;//variable to store the value coming from transmitter
int outputValue = 0;//variable to store the remapped output value

void rc_read_values() {
  noInterrupts();
  memcpy(rc_values, (const void *)rc_shared, sizeof(rc_shared));
  interrupts();
}

void calc_input(uint8_t channel, uint8_t input_pin) {
  if (digitalRead(input_pin) == HIGH) {
    rc_start[channel] = micros();
  } else {
    uint16_t rc_compare = (uint16_t)(micros() - rc_start[channel]);
    rc_shared[channel] = rc_compare;
  }
}

void calc_ch1() { calc_input(RC_CH1, RC_CH1_INPUT); }
void calc_ch2() { calc_input(RC_CH2, RC_CH2_INPUT); }
void calc_ch3() { calc_input(RC_CH3, RC_CH3_INPUT); }
void calc_ch4() { calc_input(RC_CH4, RC_CH4_INPUT); }

void setup() {
  Serial.begin(SERIAL_PORT_SPEED);

  pinMode(RC_CH1_INPUT, INPUT);
  pinMode(RC_CH2_INPUT, INPUT);
  pinMode(RC_CH3_INPUT, INPUT);
  pinMode(RC_CH4_INPUT, INPUT);

  enableInterrupt(RC_CH1_INPUT, calc_ch1, CHANGE);
  enableInterrupt(RC_CH2_INPUT, calc_ch2, CHANGE);
  enableInterrupt(RC_CH3_INPUT, calc_ch3, CHANGE);
  enableInterrupt(RC_CH4_INPUT, calc_ch4, CHANGE);
}

void loop() {
  rc_read_values();
<<<<<<< HEAD
  
  inputValue = rc_values[RC_CH1];//read the value from the transmitter
  
  if (map(inputValue,1090,1900,0,255) < 0)
  {
    outputValue = 0;
  }
  else if (map(inputValue,1090,1900,0,255) > 255)
  {
    outputValue = 255;
  }
  else
  {
    outputValue = map(inputValue,1090,1900,0,255);
  }                                                 ;//Convert from 0-1023 proportional to the number of a number of from 0 to 255
  
  analogWrite(motorPin,outputValue);//turn the motor on depending on the output value
  
  //Serial.print("CH1:"); Serial.print(rc_values[RC_CH1]); Serial.print("\t");
  //Serial.print("CH2:"); Serial.print(rc_values[RC_CH2]); Serial.print("\t");
  //Serial.print("CH3:"); Serial.print(rc_values[RC_CH3]); Serial.print("\t");
  //Serial.print("CH4:"); Serial.println(rc_values[RC_CH4]);
  
  Serial.println(outputValue);
  
=======

  Serial.println("Test");
  Serial.print("CH1:"); Serial.print(rc_values[RC_CH1]); Serial.print("\t");
  Serial.print("CH2:"); Serial.print(rc_values[RC_CH2]); Serial.print("\t");
  Serial.print("CH3:"); Serial.print(rc_values[RC_CH3]); Serial.print("\t");
  Serial.print("CH4:"); Serial.println(rc_values[RC_CH4]);

>>>>>>> 7c82698343770e36b64833b0d389666c635e97c0
  delay(200);
}
