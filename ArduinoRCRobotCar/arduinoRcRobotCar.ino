/*
4Wheel RC Car Drive using RC Receiver Transmitter with Arduino
Syed Razwanul Haque(Nabil), https://www.nabilbd.com
************** Motor Driver & Arduino Pin **************
----------------------------------------------------
Motor                 | In1 | In2 | PWM Pin
----------------------------------------------------
Forward Right Motor   | A3  | A2  | 9        
Forward Left Motor    | A0  | A1  | 6 
Aft Right Motor       | 13  | 12  | 11
Aft Left Motor        | A4  | A5  | 10
----------------------------------------------------
 ************* RC Receiver Pin and Arduino *************
 ---------------------------------------------------
  RC Receiver Channel | Arduino Pin
 ---------------------------------------------------
        CH1           |   2
        CH2           |   3
        CH6           |   4
 ---------------------------------------------------
 Tested Remote Control Model: Fly Sky FS-CT6B
 Arduino: Arduino Uno
 Motor Driver: L298N Red
 Controlling: RC Tx in Mode 2(Throttle Left), Robot Forward-Backward by Channel 2 and Turn by Channel 1. If Channel 2 moves without channel 1 value then robot will rotate.
*/
#include "Motor.h"
//********* Motor Object - Start**********
Motor motorForwardRight(A3, A2, 9);  //Change Pin Position if Direction Reverse
Motor motorForwardLeft(A0, A1, 6);
Motor motorAftRight(13, 12, 11);
Motor motorAftLeft(A4, A5, 10);
//********* Motor Object - End**********

//********* Serial output on/off based on debugFlag ****
#define debugFlag true //If true show output in Serial

#if debugFlag == true
#define debug(x) Serial.print(x)
#define debugln(x) Serial.println(x)
#else
#define debug(x)
#define debugln(x)
#endif
//*****************************************************

// ******** Start RC Input Pin ***********
uint8_t rcCh1_input = 2;  // Right-Left
uint8_t rcCh2_input = 3;  // Up-Down
uint8_t rcCh6_input = 4;  // Auxulary
// ******** End RC Input Pin ***********

unsigned long ch1PulseDuration;
unsigned long ch2PulseDuration;
unsigned long ch6PulseDuration;

//******** Start Tuning Parameter *****
int ch1DeadZoneValue = 30; // Adjust according to your RC Tx
int ch2DeadZoneValue = 30; // Adjust according to your RC Tx
int ch2SharpTurnOffsetValue = 20; // Adjust according to your RC Tx

int ch1HighestValue = 1830; // Adjust according to your RC Tx
int ch1LowestValue = 1075; //Adjust according to your RC Tx
int ch2HighestValue = 1865;//Adjust according to your RC Tx
int ch2LowestValue = 1185;//Adjust according to your RC Tx

int ch1MidValue = 1485;
int ch2MidValue = 1505;
float steeringConstant = 0.7; // If this value increases steering sensitivity will increase. Adjust accordingly
//******** End Tuning Parameter *******

int rightMotorAppliedPwm = 0;
int leftMotorAppliedPwm = 0;

char leftMotorDirection;
char rightMotorDirection;

int linearPwm = 0;
int steeringPwm = 0;

bool safetyStop = true;

// Function to Stop All Motor during safety implementation.
void allWheelStop() {
  motorForwardRight.Stop();
  motorForwardLeft.Stop();
  motorAftLeft.Stop();
  motorAftRight.Stop();
  rightMotorAppliedPwm = 0;
  leftMotorAppliedPwm = 0;
  debugln("Stopppppeeedddd");
}


void setup() {
  Serial.begin(9600);
  pinMode(rcCh1_input, INPUT);
  pinMode(rcCh2_input, INPUT);
  pinMode(rcCh6_input, INPUT);
}

void loop() {
  ch1PulseDuration = pulseIn(rcCh1_input, HIGH, 30000);  //pulseIn(pin, value, timeout),timeout (optional): the number of microseconds to wait for the pulse to start; default is one second (unsigned long)
  ch2PulseDuration = pulseIn(rcCh2_input, HIGH, 30000);
  ch6PulseDuration = pulseIn(rcCh6_input, HIGH, 30000);

  safetyStop = ((ch1PulseDuration < 5) || (ch2PulseDuration < 5) || (ch2PulseDuration < 5)) ? true : false;  // If RC Transmitter is off then pulse duration become zero. For safety margin we put less than 5

  debug("RC Input:-- ");debug("CH1:");debug(ch1PulseDuration);debug("  CH2:");debug(ch2PulseDuration);debug("  CH6:");debug(ch6PulseDuration);debugln(); // To see RC values in Serial Monitor

// ************************ Start- Channel 2 (Forward-Backward) considering deadzone *****************************************************************************
  // Channel 2(Forward) Map value considering deadzone. 
  if (ch2PulseDuration > (ch2MidValue + ch2DeadZoneValue)) {  // channel 2 forward backward
    linearPwm = map(ch2PulseDuration, ch2MidValue, ch2HighestValue, 0, 255);
  }
 // Channel 2(Backward) Map value considering deadzone. 
  if (ch2PulseDuration < (ch2MidValue - ch2DeadZoneValue)) {
    linearPwm = - map(ch2PulseDuration, ch2MidValue, ch2LowestValue,  0, 255); // - minus sign to reverse the robot in next step. It can be implemented diffrently
  }
  
  // Dead Zone Implementaion. If Channel 2 stick position within Deadzone then it will consider as zero and will stop motors. 
  if ((ch2PulseDuration < (ch2MidValue + ch2DeadZoneValue)) && (ch2PulseDuration > (ch2MidValue - ch2DeadZoneValue))) {
    linearPwm = 0;
    //rightMotorAppliedPwm = 0;
    //leftMotorAppliedPwm = 0;
  }
// ************************ End- Channel 2 (Forward-Backward) con *****************************************************************************

// ************************ Start- Channel 1 (Forward-Backward) considering deadzone *****************************************************************************
  // Channel 1(Right) Map value considering deadzone. 
  if (ch1PulseDuration > (ch1MidValue + ch1DeadZoneValue)) {  // channel 1 right left
    steeringPwm = map(ch1PulseDuration, ch1MidValue, ch1HighestValue, 0, 255);
  }
  // Channel 1(Left) Map value considering deadzone.
  if (ch1PulseDuration < (ch1MidValue - ch1DeadZoneValue)) {  // channel 1 right left
    steeringPwm = - map(ch1PulseDuration, ch1MidValue, ch1LowestValue, 0, 255); // - minus sign to reverse the steering in the next step. It can be implemented diffrently
  }

  // Dead Zone Implementaion. If Channel 1 stick position within Deadzone then it will consider as zero and will stop motors. 
  if ((ch1PulseDuration < (ch1MidValue + ch1DeadZoneValue)) && (ch1PulseDuration > (ch1MidValue - ch1DeadZoneValue))) {
    steeringPwm = 0;
  }
// ************************ End- Channel 1 (Forward-Backward) considering deadzone *****************************************************************************  
  //Forward Going with steering 
  if (linearPwm >= 0) {  // >= is important. Otherwise if linearPwm is 0 then it will go to undesired state.
    rightMotorAppliedPwm = constrain((linearPwm - (steeringPwm * steeringConstant)), 0, 255);
    leftMotorAppliedPwm = constrain((linearPwm + (steeringPwm * steeringConstant)), 0, 255);
    leftMotorDirection = 'F';
    rightMotorDirection = 'F';
  }
  //Backward Going with steering
  if (linearPwm < 0) {
    linearPwm = abs(linearPwm);
    rightMotorAppliedPwm = constrain((linearPwm - (steeringPwm * steeringConstant)), 0, 255);
    leftMotorAppliedPwm = constrain((linearPwm + (steeringPwm * steeringConstant)), 0, 255);
    leftMotorDirection = 'R';
    rightMotorDirection = 'R';
  }

  // ********************** Sharp Turn Start - Considering Sharp Turn Offset Value********************
  if ((ch2PulseDuration < (ch2MidValue + ch2DeadZoneValue + ch2SharpTurnOffsetValue)) && (ch2PulseDuration > (ch2MidValue - (ch2DeadZoneValue + ch2SharpTurnOffsetValue)))) {
    if (steeringPwm >= 0) {  //right sharp turn
      rightMotorAppliedPwm = constrain(steeringPwm, 0, 255);
      leftMotorAppliedPwm = constrain(steeringPwm, 0, 255);
      leftMotorDirection = 'F';
      rightMotorDirection = 'R';
    }
    if (steeringPwm < 0) {  // left sharp turn
      rightMotorAppliedPwm = constrain(abs(steeringPwm), 0, 255);
      leftMotorAppliedPwm = constrain(abs(steeringPwm), 0, 255);
      leftMotorDirection = 'R';
      rightMotorDirection = 'F';
    }
  }
  // ******************** Sharp Turn End ************************************************************

// *************** Actual Motor Drive Start *****************************
  if (safetyStop == true) {
    allWheelStop();
  } else {
    motorForwardLeft.Drive(leftMotorDirection, leftMotorAppliedPwm);
    motorForwardRight.Drive(rightMotorDirection, rightMotorAppliedPwm);
    motorAftLeft.Drive(leftMotorDirection, leftMotorAppliedPwm);
    motorAftRight.Drive(rightMotorDirection, rightMotorAppliedPwm);
  }
// *************** Actual Motor Drive End *****************************
  debug(linearPwm); debug("  SteeringPwm: "); debugln(steeringPwm);
  debug("Left Motor Direction: **");debug(leftMotorDirection); debug("** Left Motor PWM: **");debug(leftMotorAppliedPwm); debug("** Right Motor Direction: **");debug(rightMotorDirection); debug("**Right Motor PWM: **");debugln(rightMotorAppliedPwm);

  //delay(1000); //Delay to see debug value
}
