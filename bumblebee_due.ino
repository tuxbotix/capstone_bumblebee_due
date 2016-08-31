/**
Copyright (c) 2015, A.A. Darshana Sanjeewan Adikari
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the <organization> nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLU
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.DING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

Additional notes:
01.06.2015 - Version 1.0 (stable)
06.06.2015 - Version 1.2 (stable)
    Use 4 PID loops (2 PI - velocity), 2 PID for position & heading
06.06.2015 - Version 1.2 (stable)
    Added DUEPWm library - changing frequency*
**/

#include "includes.h"
//#include <DueTimer.h>
#include <PID_v1.h>
#include <DuePWM.h>
#include <Wire.h>
#include <Encoder.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>


/*
timing stuff
*/
int watchdogCounter = 100;// This calls watchdogPC for each 100 systick executions. (for 10ms systick, 100 * 10 = 1 second)
/*
  Robot Motion Group
*/
//volatile int_fast64_t currentPositionR = 0; //position accumiliator, NO buffer overflow
//volatile int_fast64_t currentPositionL = 0; //position accumiliator
unsigned long timeSpent = 0;

int_fast64_t newpositionR;    //position variables 32 bits per side
int_fast64_t oldpositionR = 0;

int_fast64_t newpositionL;
int_fast64_t oldpositionL = 0;

double theta = 0;
double thetaOffset = 0;

double thetaXOffset = -7;
double thetaYOffset = 1;

double thetaYGain = 1.1286;


//double declinationAngle = -0.0107628637 ;
double xMM ;  //absolute position with respect to world coordinates. Y axis is Aligned with magnetic north***
double yMM;

double xVelocity ;  //velocity in each axis W.R.T. world
double yVelocity;
double theta_dot;  //rotational velocity. Can be taken from gyro! because encoders alone can be error prone.

/*
Details on robot's specifications.

The encoder used is 50 lines per revolution attached to motor, and two optical switches (optical interruptor) used in Quadrature format.
cunting change on both channels (called "tick") so in Quad Format, 4

50 * 4 = 200
Gear ratio = 1:6.25 = wheel : motor

therefore, 200*6.25 = 1250 -> Number of "ticks" per one wheel revolution.
Wheel diameter is 3", -> 239~ mm circumference
*/
const double R = 38.1;  //Wheel Diameter in millimeters -> 1.5" = 38.1mm
const double L = 202.5;  //track width/2 = 405mm/2 = 202.5
const double tickToMM = R * TWO_PI / 1250;  //wheel radius * pi / 1250 (1250 ticks per rev)
const double tickToMMDiv2 = R * PI / 1250;  //define divide by 2 to reduce computational load (on loopy loops ;) )
const double Ldiv2 = L / 2;
double deltaLeft = 0;    //position difference of each wheel within sample time (10ms @ 100Hz)
double deltaRight = 0;

/*
Trajectory Variables
*/
double targetsX[5];  //Store targets. Use a ringbuffer form***, make sure not to request points more than nessesary :P
double targetsY[5];
boolean activeValue[5];        //  bit set to indicate active point or disabled
short ringBufIndex = 0;        //  Currently active target point
short lastUpdatedIndex = -1;   //  last updated index -> to check which cell was filled most recently, so can increment from this upon new addition
int addedPoints = 0;           //  total number of points added
int completedPoints = 0;      //  total number of points navigated

double distance = 0;          //  distance from current to goalpoint (or target if small enough)
double normalDistance = 0;    //  perpendicular distance from robot to the specified path
double goalPointX = 0;        //  goal point, a point placed on the path from a certain distance from the robot
double goalPointY = 0;
/*double lastNearestPointX;
double lastNearestPointY;
*/
double nearestPointX = 0;  //  Nearest point to the robot on the path -> place where perpendicular from robot intersect the path.
double nearestPointY = 0;
double pathHeading = 0;    //  slope of the path segment - used to get required heading and other info
double totalDistance = 0;  //  total distance from robot to main target point
double targetHeading = 0;
boolean useGoalPoint = false;  //  use a goalpoint or not.

/*
Begin Robot Sensors
*/

// 11.06.2015 changed pins to allow soldered pin headers.
Encoder wheelLeft(29, 31);  // encoder objects. Attach interrupt capable pins (all in case of arduino due)
Encoder wheelRight(32, 34);  // encoder objects. Attach interrupt capable pins (all in case of arduino due)

/* Assign a unique ID to this sensor at the same time */
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);


/*
 * Serial and other state variables;
 */

String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete

String inputStringDebug = "";         // a string to hold incoming data   (wifi/ debug)
boolean stringCompleteDebug = false;  // whether the string is complete For Serial 1 (wifi/ debug)

boolean isPCAlive = false;      // Used to make sure the bot is communicating with PC.

boolean isMotionAllowed = false; //used to shut down motors immidiately
boolean isCalibration = false;
uint_fast64_t ticks = 0;

char buff[50];
/*
PID controller and related variables
*/

double leftMotorSetpoint, leftMotorOut;  //Target set in ticks per sample time.
double rightMotorSetpoint, rightMotorOut;
double velocitySetpoint, velocityOut; //linear velocity target for the robot.
double headingOut, headingInput;


//Specify the links and initial tuning parameters

PID leftMotorPID(&deltaLeft, &leftMotorOut, &leftMotorSetpoint, 0.3, 0.02, 0, DIRECT);//0.5, 0.03
PID rightMotorPID(&deltaRight, &rightMotorOut, &rightMotorSetpoint, 0.3, 0.02, 0, DIRECT);

PID positionPID(&velocitySetpoint, &velocityOut, &distance, 0.1, 0.008, 1, DIRECT);

PID headingPID(&headingInput, &headingOut, &targetHeading, 0.25, 0, 0, DIRECT);

/**
New PWM lobrary**
*/
boolean calibrate = false;

sensors_event_t event;
int xMin = 0;
int xMax = 0;
int yMin = 0;
int yMax = 0;
/*
 * setup and other function declarations
 */
void setup()
{
  pinMode(CALIB_ON_SW, INPUT_PULLUP);
  pinMode(CALIB_ACCL_COMPASS_SELECT, INPUT_PULLUP);
  pinMode(CALIB_ACCL_GYRO_LED, OUTPUT);
  pinMode(CALIB_COMPASS_LED, OUTPUT);
  digitalWrite(CALIB_COMPASS_LED, LOW);
  digitalWrite(CALIB_ACCL_GYRO_LED, LOW);

  // Use Serial 0 (USB) as main comms with onboard PC
  // Use Serial 1 (thru WiFi) as Debug/ remote control interface
  Serial.begin(115200);
  Serial1.begin(115200);
  Serial3.begin(115200);

  Serial.println("$Start*");
  Serial1.println("$Start*");

  /* Initialise the sensor */
  if (!mag.begin())
  {
    /* There was a problem detecting the HMC5883 ... check your connections */
    Serial1.println("Ooops, no HMC5883 detected ... Check your wiring!");

  }
  mag.setMagGain(HMC5883_MAGGAIN_4_0);
  mag.setUpdateRate();  //added function to library code to increase sample rate to 75Hz Refer HMC5883 datasheet


  leftMotorSetpoint = 0;
  rightMotorSetpoint = 0;


  //turn the PID on & config
  leftMotorPID.SetMode(AUTOMATIC);    //initiate PID controller
  rightMotorPID.SetMode(AUTOMATIC);

  headingPID.SetMode(AUTOMATIC);

  leftMotorPID.SetSampleTime(20);   //Set Sample time to 10ms **Important as default is 200ms
  rightMotorPID.SetSampleTime(20);  //Set Sample time to 10ms **Important as default is 200ms

  headingPID.SetSampleTime(20);    //Set Sample time to 10ms **Important as default is 200ms

  leftMotorPID.SetOutputLimits(-255, 255);   //just to limit running motors at max(255), also to enable reversing
  rightMotorPID.SetOutputLimits(-255, 255);

  headingPID.SetOutputLimits(-0.2, 0.2);     //0.3 rad/s ~= 17 deg/sec - max rotational velocity the robot can attain

  positionPID.SetMode(AUTOMATIC);
  positionPID.SetOutputLimits(0, V_MAX); //max velocity in mm per each DT -> 20mm per 20 ms
  positionPID.SetSampleTime(10);

  //  velocitySetpoint = 30;  //  Using ON/OFF controller for convenience :P PID or distance & physical model based trajectory control should work better
  velocitySetpoint = 0;  //  Using ON/OFF controller for convenience :P PID or distance & physical model based trajectory control should work better

  setupMotors(); //  initiate motor pins and keep all to zero

  avgTheta();
  /*
    leftMotorSetpoint = 30;
    rightMotorSetpoint = 30;*/
  // Timer3.attachInterrupt(sysTick).start(20000);  //10 ms - setup a timer to act like Systick (Arduino Due don't have systick - optional for cortex M3)
  if (!digitalRead(CALIB_ON_SW)) { //using internal pullups

    calibrate = true;
  }
}

void loop() {
  if (calibrate) {

    if (!digitalRead(CALIB_ACCL_COMPASS_SELECT)) { //ON = compass
      digitalWrite(CALIB_COMPASS_LED, HIGH);
      Serial1.println("Rotate the robot around!!! You got around 5 seconds :P");
      int samples = 300;
      for (int i = 0; i < samples; i++) {
        mag.getEvent(&event);
        if (event.magnetic.x < xMin) xMin = event.magnetic.x;
        if (event.magnetic.x > xMax) xMax = event.magnetic.x;
        if (event.magnetic.y < yMin) yMin = event.magnetic.y;
        if (event.magnetic.y > yMax) yMax = event.magnetic.y;
        thetaXOffset = (xMax + xMin) / 2;
        thetaYOffset = (yMax + yMin) / 2;
        thetaYGain = abs(xMin - xMax) / abs(yMin - yMax);
        delay(20);
      }
    } else {//off = accel/gyro
      digitalWrite(CALIB_ACCL_GYRO_LED, HIGH);
      Serial1.println("Keep the robot still");
    }
    calibrate = !digitalRead(CALIB_ON_SW); //using internal pullups
    digitalWrite(CALIB_COMPASS_LED, LOW);
    digitalWrite(CALIB_ACCL_GYRO_LED, LOW);
    if (!calibrate) {
      Serial1.print(xMin);
      Serial1.print(":");
      Serial1.print(xMax);
      Serial1.print(":");
      Serial1.print(yMin);
      Serial1.print(":");
      Serial1.print(yMax);
      Serial1.print(":");
      Serial1.print(thetaXOffset, 3);
      Serial1.print(":");
      Serial1.print(thetaYOffset, 3);
      Serial1.println("Calibration Finished");
    }
    return;

  }

  if ((micros() - timeSpent) > DT) {

    sysTick();
    timeSpent = micros();
  }

  processSerialInputs();  //process serial input. Serial 0 is connected to onboard PC, Serial 1 connected to WiFi module for data logging, debugging
  /**
    TODO Add Serial 1 commands**
  **/
}

void sysTick() {

  updateThings();  //update velocities, headings,etc

  /**
  EXPERIMNENTAL
  **/



  /**
  if and only if trajectory system is active, motors will recieve any power.
  This behavior will replace all testing that directly addressed motors or motor PID values (Remove all direct low level control)

  In addition, Motors will immidiately halt if the PC is not alive. -> TEMPORARILY DISABLED
  **/

  if (isTrajRunning()) {
    planner();  //  Call the planner, the core of going thru the set of points.
    ticks++;
    if (distance < DEADBAND) { //simple on-off controller
      velocityOut = 0;
      headingOut = 0;
      //stopAllMotion();
    } else {

      headingInput = theta;
      if ((targetHeading - headingInput) < -PI) {
        headingInput -= TWO_PI;
      } else if ((targetHeading - headingInput) > PI) {//if heading input is negative and the total diff > pi -> when robot is near -180 deg. and setpoint is near 180
        headingInput += TWO_PI;
      }

      /**
      Do High level PID computations -> Position and Heading to linear velocity and rotational speed;
      Ideally the final two params can be defined through "curvature" where normal distance and robot orientation makes up the main parameters
      */

      positionPID.timerCompute();
      headingPID.timerCompute();  //Call timer based compute, for heading

      /**
      Now check if the heading error is really small, if so ignore it.
      WARNING - Only use to filter very small values, else the heading PID output will be quite drastic
      */
      setHighLevelInputs(&velocityOut, &targetHeading, &headingInput);
      /*
            if (abs(headingInput - targetHeading) < 0.1) {
              headingOut = 0;
            }*/
      //    headingOut = 0;

      lowLevelControl(&velocityOut, &headingOut, &leftMotorSetpoint, &rightMotorSetpoint);
      sendPosition();

      leftMotorPID.timerCompute();  //compute PID for motors. Maybe need further tuning
      rightMotorPID.timerCompute();

    }


    updateMotors();

  } else {
    velocityOut = 0;
    headingOut = 0;
    stopAllMotion();
  }
  watchDogPC();  // called to check status of PC app. If not, immidiately stop motion.

  //  Serial.println(deltaLeft, 4);
}

void beginCSVData() {
  sendLogData("time,x,y,heading,leftmotorOut,rightmotorOut,leftPID,rightPID,velocityPID,headingPID\r");
}
void sendCSVData() {
  //sprintf(buff, "%3\r", fileNumber);
  sendLogPiece(ticks);
  sendLogPiece(xMM);
  sendLogPiece(yMM);
  sendLogPiece(theta);
  sendLogPiece(leftMotorOut);
  sendLogPiece(rightMotorOut);
  sendLogPiece(leftMotorSetpoint);
  sendLogPiece(rightMotorSetpoint);
  sendLogPiece(velocityOut);
  sendLogData(headingOut);
}

void updateThings() { //update variables that say current state of the robot (position, velocity,etc)

  Theta_dot();
  mag.getEvent(&event);

  theta = -atan2((event.magnetic.y - thetaYOffset), (event.magnetic.x - thetaXOffset) * thetaYGain); //output between -pi and +pi
  //  heading += declinationAngle;
  // theta += theta_dot;

  theta += thetaOffset;
  if (theta > PI) {
    theta - TWO_PI;
  } else if (theta < -(PI)) {
    theta + TWO_PI;
  }

  checkDelta();  //read encoder values and compute the delta.
  //  updateLinearVelocity();
  X_Velocity();
  Y_Velocity();
  if (xVelocity != 0) {
    xMM += xVelocity;
  }
  if (yVelocity != 0) {
    yMM += yVelocity;
  }

}

/**

Used to compute velocities of each wheel

TODO devise a method to avoid overflow for newPosition and oldPosition

*/
void checkDelta() {
  newpositionL = wheelLeft.read();
  newpositionR = wheelRight.read();

  deltaLeft = newpositionL - oldpositionL;
  deltaRight = newpositionR - oldpositionR;
  oldpositionL = newpositionL;
  oldpositionR = newpositionR;

}


void sendPosition() {  //just send position! lol

  Serial1.print("$pos,");
  Serial1.print(xMM, 2);
  Serial1.print(',');
  Serial1.print(yMM, 2);
  Serial1.print(',');
  Serial1.print(theta * 180 / PI, 6);
  Serial1.println('*');
}

void sendPositionPC() {  //just send position! lol

  Serial.print("$pos,");
  Serial.print(xMM, 2);
  Serial.print(',');
  Serial.print(yMM, 2);
  Serial.print(',');
  Serial.print(theta, 6);
  Serial.println('*');
}

/**
  Watchdog-ish function - Used to verify comms with the PC. else sieze operations immidiately.
*/

void watchDogPC() {
  if (isCalibration) {
    return;
  }
  if (watchdogCounter > 50) {//50 cycles -> @ 50Hz = 1 second
    Serial1.print(useGoalPoint);
    Serial.println("$alive?*");
    sendPosition();
    watchdogCounter = 0;
  }
  // if (watchdogCounter % 5 == 0) { // approx 10Hz updates
  if (isLoggingEnabled()) {
    sendCSVData();
  }
  // }
  watchdogCounter++;
  startMotion();
  /*  if (!isPCAlive) {
      stopAllMotion();
      isMotionAllowed = false;
    } else {
      isMotionAllowed = true;
    }*/
}

/*===================END OF PROGRAM======================*/
