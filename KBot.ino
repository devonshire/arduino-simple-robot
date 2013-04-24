/*
  ---------------------------------------------------------------------
  ROBOT OPERATING CODE
    Author:  Kevin Peat - 2012/2013
    License: All original project code is public domain
             Licenses for libraries are shown below
  ---------------------------------------------------------------------
  HARDWARE ENVIRONMENT
    DFRobot Pirate Chassis (powered by 5 x AA 3800mAh)
    Arduino Mega 2560 V3 (powered by 9V PP3 300mAh)
    L298N Motor Shield
    HC-SR04 Ultrasonic Sensor
    Infra-red Sensors
  ---------------------------------------------------------------------
  LCD WIRING
    LCD pin name    RS  EN  DB4 DB5 DB6 DB7
    Arduino pin #   32  30  28  26  24  22
  ---------------------------------------------------------------------
  L298N MOTOR SHIELD WIRING
    L298N pin name  IN1 IN2 IN3 IN4 ENA ENB
    Arduino pin #   2   3   4   5   n/a n/a
    ENA/ENB disabled as motors not powerful enough on lower voltage
  ---------------------------------------------------------------------  
  HC-SR04 Ultrasonic Sensor
    HC-SR04 pin name  TRIG ECHO
    Arduino pin #     13   12

    Using the NewPing library which is available from:
      http://code.google.com/p/arduino-new-ping/
    This library is licensed "GNU GPL v3"
  ---------------------------------------------------------------------  
  MMA7361 3-axis MEMS Accelerometer
    MMA7361 pin name  SLEEP TEST 0G GSELECT x  y  z  
    Arduino pin #     11    10   9  8       A1 A2 A3

    Using the AcceleroMMA7361 library which is available from:
      http://code.google.com/p/mma7361-library/
    This library is licensed "GNU Lesser GPL"
  ---------------------------------------------------------------------  
  Infra-red Sensors
    IR pins         LEFT RIGHT CENTRE
    Arduino pin #   7    6     1
  ---------------------------------------------------------------------  
*/

// LCD
#include <LiquidCrystal.h>
LiquidCrystal lcd(32, 30, 28, 26, 24, 22);

// HC-SR04 Ultrasonic Sensor
#include <NewPing.h>
const int usTRIGGER = 13;
const int usECHO = 12;
const int usMAXDIST = 100;
NewPing sonar(usTRIGGER, usECHO, usMAXDIST);

// MMA7361 Accelerometer
#include <AcceleroMMA7361.h>
AcceleroMMA7361 accelero;

// Motor controller pin definitions
#define MR_one 4
#define MR_two 5
#define ML_one 3
#define ML_two 2
// #define MR_enable 6
// #define ML_enable 7

// Motor states
const int mSTOPPED = 0;
const int mFORWARD = 1;
const int mREVERSE = -1;
const int mLEFT = 0;
const int mRIGHT = 1;
int leftMotorState = 0;
int rightMotorState = 0;

// Infra-red Sensors
const int irLEFT = 7;
const int irRIGHT = 6;
const int irCENTRE = 1;

//---------------------------------------------------------------------  
// Set-up
//---------------------------------------------------------------------  
void setup() {
  
  // Seed random function (relies on Arduino pin 0 being unused!)
  randomSeed(analogRead(0));
  
  // Initialise lcd and sensors
  lcdInit();
  irInit();
  accInit();
}

//---------------------------------------------------------------------  
// Main Loop
//---------------------------------------------------------------------  
void loop() {

  // Is bot upright?  
  if (getUpright()) {
  
    // Move forward unless obstacle sensed
    if ((getUSDistance() > 20 || getUSDistance() == 0) && !getIRLeft() && !getIRRight() && !getIRCentre()) {

      motorControl(mLEFT, mSTOPPED, 0);    
      motorControl(mRIGHT, mSTOPPED, 0);    
      motorControl(mLEFT, mFORWARD, 255);    
      motorControl(mRIGHT, mFORWARD, 255);    
    
    // Else, turn a random direction until clear
    } else {

      // If not already turning then choose a direction
      if (leftMotorState == rightMotorState) {

        // Turn left      
        if (random(0, 2) == 1) {
          motorControl(mLEFT, mREVERSE, 255);    
          motorControl(mRIGHT, mFORWARD, 255);    

        // Or right
        } else {
          motorControl(mLEFT, mFORWARD, 255);    
          motorControl(mRIGHT, mREVERSE, 255);    
        }      
      }
    }

  // Has tipped over so stop motors
  } else {

    motorControl(mLEFT, mSTOPPED, 0);    
    motorControl(mRIGHT, mSTOPPED, 0);    
  }

  // Update the lcd with current status
  lcdRefresh();

  // Wait a bit
  delay(250);
}

//---------------------------------------------------------------------  
// Subroutines
//---------------------------------------------------------------------  

// Initialise LCD
void lcdInit() {
  lcd.begin(16, 2);  
  lcd.clear();
  lcd.print("Initialising...");
}

// Initialise IR Sensors
void irInit() {
  pinMode(irLEFT, INPUT);
  pinMode(irRIGHT, INPUT);
  pinMode(irCENTRE, INPUT);
} 

// Initialise Accelerometer
void accInit() {
  accelero.begin(11, 10, 9, 8, A1, A2, A3);
  // accelero.setARefVoltage(3.3); // Doesn't work if set
  accelero.setSensitivity(LOW); // Sets the sensitivity to +/-6G
  accelero.calibrate();
}

// Read left IR sensor
boolean getIRLeft() {
  return !digitalRead(irLEFT);
}

// Read right IR sensor
boolean getIRRight() {
  return !digitalRead(irRIGHT);
}

// Read centre IR sensor
boolean getIRCentre() {
  return !digitalRead(irCENTRE);
}

// Get Accelerometer X value
int getAccX() {
  return accelero.getXAccel();
}

// Get Accelerometer Y value
int getAccY() {
  return accelero.getYAccel();
}

// Get Accelerometer Z value
int getAccZ() {
  return accelero.getZAccel();
}

// Get Accelerometer orientation
// getOrientation returns which axis is perpendicular with the earths 
// surface x=1, y=2, z=3 is positive or negative depending on which 
// side of the axis is pointing downwards
int getAccO() {
  return accelero.getOrientation();
}

// Returns true if bot is upright false if tipped over (or is close to doing so)
// Value may need changing depending on sensor mounting orientation
boolean getUpright() {
  if (getAccO() == 3) {
    return 1;
  } else {
    return 0;
  }       
}

// Get distance US sensor (29ms should be the shortest delay between pings)
int getUSDistance() {
//  return sonar.ping() / US_ROUNDTRIP_CM;
  return sonar.ping_median(5) / US_ROUNDTRIP_CM;
}

// Refresh status information on LCD
void lcdRefresh() {
//  String top = "L" + String(leftMotorState) + " U" + String(getUSDistance());
  String top = "L" + String(leftMotorState) + " U" + String(getUSDistance()) + " O" + String(getAccO())  + " X" + String(getAccX());
  String bot = "R" + String(rightMotorState) + " I" + String(getIRLeft()) + String(getIRCentre()) + String(getIRRight())  + " Y" + String(getAccY());
  lcd.clear();
  lcd.print(top);
  lcd.setCursor(0, 1);
  lcd.print(bot);
}

// Control motors
void motorControl(int motor, int direction, int speed) {

  if (motor == mLEFT) {     
    if (direction == mSTOPPED) {
      digitalWrite(ML_one, LOW);
      digitalWrite(ML_two, LOW);
      leftMotorState = mSTOPPED;
    }
      
    if (direction == mFORWARD) {
      // analogWrite(ML_enable, speed);
      digitalWrite(ML_one, LOW);
      digitalWrite(ML_two, HIGH);
      leftMotorState = mFORWARD;
    } 
      
    if (direction == mREVERSE) {
      // analogWrite(ML_enable, speed);
      digitalWrite(ML_one, HIGH);
      digitalWrite(ML_two, LOW);
      leftMotorState = mREVERSE;
    }
  } 

  if (motor == mRIGHT) {     
    if (direction == mSTOPPED) {
      digitalWrite(MR_one, LOW);
      digitalWrite(MR_two, LOW);
      rightMotorState = mSTOPPED;
    }
      
    if (direction == mFORWARD) {
      // analogWrite(MR_enable, speed);
      digitalWrite(MR_one, LOW);
      digitalWrite(MR_two, HIGH);
      rightMotorState = mFORWARD;
    } 
      
    if (direction == mREVERSE) {
      // analogWrite(MR_enable, speed);
      digitalWrite(MR_one, HIGH);
      digitalWrite(MR_two, LOW);
      rightMotorState = mREVERSE;
    }
  } 
}

