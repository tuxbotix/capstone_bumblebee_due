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
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
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
07.06.2015 - Version 1.2
**/
#define leftMotorA 3
#define leftMotorB 2
#define leftMotorSpeed 7
#define rightMotorA 4
#define rightMotorB 5
#define rightMotorSpeed 6

DuePWM pwm( 22000, 22000 );

void setupMotors() {
  pinMode(leftMotorA, OUTPUT);
  pinMode(leftMotorB, OUTPUT);

  pinMode(rightMotorA, OUTPUT);
  pinMode(rightMotorB, OUTPUT);

  pwm.pinFreq1( 6 );  // Pin 6 freq set to "pwm_freq1" on clock A
  pwm.pinFreq2( 7 );  // Pin 7 freq set to "pwm_freq2" on clock B

  digitalWrite(leftMotorA, 0);
  digitalWrite(leftMotorB, 0);
  digitalWrite(rightMotorA, 0);
  digitalWrite(rightMotorB, 0);
}

/*
 * Direction 0 = coast, 1 = forward, 2= reverse, 3 = brake
 * motorSide = false -> left, else right
 * PWM - max <2500
 */
void runMotor(bool motorSide, char direction, int motorPWM) {

  if (!motorSide) {
    switch (direction) {
      case 'f':
      case 'F':
        digitalWrite(leftMotorA, 1);
        digitalWrite(leftMotorB, 0);
        //        analogWrite(leftMotorSpeed, motorPWM);
        pwm.pinDuty( leftMotorSpeed, motorPWM );  // v 1.3 addition


        break;
      case 'r':
      case 'R':
        digitalWrite(leftMotorA, 0);
        digitalWrite(leftMotorB, 1);
        pwm.pinDuty( leftMotorSpeed, motorPWM );  // v 1.3 addition
        break;
      case 'c':
      case 'C':
        digitalWrite(leftMotorA, 0);
        digitalWrite(leftMotorB, 0);
        pwm.pinDuty( leftMotorSpeed, 0 );  // v 1.3 addition
        break;
      case 'b':
      case 'B':
        digitalWrite(leftMotorA, 1);
        digitalWrite(leftMotorB, 1);
        pwm.pinDuty( leftMotorSpeed, motorPWM );  // v 1.3 addition
        break;

    }

  } else {
    switch (direction) {
      case 'f':
      case 'F':
        digitalWrite(rightMotorA, 1);
        digitalWrite(rightMotorB, 0);
        pwm.pinDuty( rightMotorSpeed, motorPWM );  // v 1.3 addition


        break;
      case 'r':
      case 'R':
        digitalWrite(rightMotorA, 0);
        digitalWrite(rightMotorB, 1);
        pwm.pinDuty( rightMotorSpeed, motorPWM );  // v 1.3 addition
        break;
      case 'c':
      case 'C':
        digitalWrite(rightMotorA, 0);
        digitalWrite(rightMotorB, 0);
        pwm.pinDuty( rightMotorSpeed, 0 );  // v 1.3 addition
        break;
      case 'b':
      case 'B':
        digitalWrite(rightMotorA, 1);
        digitalWrite(rightMotorB, 1);
        pwm.pinDuty( rightMotorSpeed, motorPWM );  // v 1.3 addition
        break;

    }


  }
}

void stopAllMotion() { //used to immidiately stop both motors Maybe add another variable to reset motors
  isMotionAllowed = false;
  runMotor(0, 'c', 0);
  runMotor(1, 'c', 0);
}
void startMotion() {
  isMotionAllowed = true;
}
void updateMotors() {

  if (!isMotionAllowed) {
    return;
  }
  if (leftMotorOut > 0) {
    runMotor(0, 'f', ((int)leftMotorOut));
    //		runMotor(0, 'f', 4000);
  } else if (leftMotorOut < 0) {
    runMotor(0, 'r', (-(int)leftMotorOut));
  } else {
    runMotor(0, 'c', 0);
  }

  if (rightMotorOut > 0) {
    runMotor(1, 'f', ((int)rightMotorOut));
    //runMotor(1, 'f', 4000);
  } else if (rightMotorOut < 0) {
    runMotor(1, 'r', (-(int)rightMotorOut));
  } else {
    runMotor(1, 'c', 0);
  }

}
