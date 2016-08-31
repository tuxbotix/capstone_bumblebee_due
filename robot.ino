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
**/
void X_Velocity()
{
  xVelocity =  cos(theta) * (double)(deltaRight + deltaLeft) * tickToMMDiv2;
}

void Y_Velocity()
{
  yVelocity =  sin(theta) * (double)(deltaRight + deltaLeft) * tickToMMDiv2;
}

void Theta_dot()
{
  theta_dot = (double)(deltaRight - deltaLeft) * tickToMMDiv2 / L;
}

void avgTheta() {
  Serial1.println("Heading calibration");
  double thetaTemp;
  short samples = 20;
  for (int i = 0; i < samples; i++) {
    sensors_event_t event;
    mag.getEvent(&event);


    thetaTemp += -atan2(event.magnetic.y, event.magnetic.x); //output between -pi and +pi
    delay(20);
  }
  theta = thetaTemp / samples;

}

void calibrateCompass() {

  double minX = 0;
  double maxX = 0;
  double minY = 0;
  double maxY = 0;

  Serial1.println("Heading calibration");
  isCalibration = true;
  double thetaTemp;
  short k = 0;
  int samples = 300;
  sensors_event_t event;

  for (int i = 0; i < samples; i++) {
    mag.getEvent(&event);
    thetaTemp = -atan2(event.magnetic.y, event.magnetic.x);
    if (event.magnetic.x < minX) minX = event.magnetic.x;
    if (event.magnetic.x > maxX) maxX = event.magnetic.x;
    if (event.magnetic.y < minY) minY = event.magnetic.y;
    if (event.magnetic.y > maxY) maxY = event.magnetic.y;

    // Calculate offsets
    thetaXOffset = (maxX + minX) / 2;
    thetaYOffset = (maxY + minY) / 2;


    switch (k) {
      case 0:
        Serial1.print('-');
        break;
      case 1:
        Serial1.print('\\');
        break;
      case 2:
        Serial1.print(' | ');
        break;
      case 3:
        Serial1.print(' \/ ');
        break;
    }
    /*k++;
    if (k > 3) {
      k = 0;
    }
    */
    delay(50);
    // Serial1.print('\b');
  }
  Serial1.println();
  Serial1.print(event.magnetic.x);
  Serial1.print(":");
  Serial1.print(event.magnetic.y);
  Serial1.print(":");
  Serial1.print(minX);
  Serial1.print(":");
  Serial1.print(maxX);
  Serial1.print(":");
  Serial1.print(minY);
  Serial1.print(":");
  Serial1.print(maxY);
  Serial1.print(":");
  Serial1.print(thetaXOffset);
  Serial1.print(":");
  Serial1.print(thetaYOffset);

  Serial1.print(" Heading (degrees): ");
  Serial1.println((thetaTemp * 180 / PI));
  isCalibration = false;

}
/**
void updateLinearVelocity() { // give velocity in mm/s
velocityInput = (deltaRight + deltaLeft) / 2 * tickToMM * FREQ / 1000;
}
*/
/*
* Trajectory gen gives desired heading, distance to go.
* Then decideMotionType decide pure rotation vs rot+translate
* that input fed into higher level PID controllers
* Finally setMotorOutputs called to calculate low level PID values
* that fed to low level PID controllers
*
* Risky part is is crashing on things sideways on hard rotations!!! - So remove pure rotation or add sensors :P
*/

void setHighLevelInputs(double* robotInput, double* headingSetpoint, double* headingInput) {

  if (abs(*headingSetpoint - *headingInput) > 1.0472) { //if desired angle (difference) less than 30deg
    *robotInput = 0;
  }
}

/**
@params
@\velocityIn -  Linear Velocity in mm/s
@\headingIn  -  Rotational Velocity in rad/s

*/
void lowLevelControl(double* linearVelocity,
                     double* rotationalVelocity, double* leftPIDSetpoint,
                     double* rightPIDSetpoint) {

  *leftPIDSetpoint = (double) (*linearVelocity
                               - (*rotationalVelocity) * Ldiv2) / tickToMM;
  *rightPIDSetpoint = (double) (*linearVelocity
                                + (*rotationalVelocity) * Ldiv2) / tickToMM;
  /*
  if (abs(*rightPIDSetpoint) > V_MAX) {
  *rightPIDSetpoint = V_MAX * (*rightPIDSetpoint) / abs(*rightPIDSetpoint);
  } else if (abs(*leftPIDSetpoint) > V_MAX) {
  *leftPIDSetpoint = V_MAX * (*leftPIDSetpoint) / abs(*leftPIDSetpoint);
  }
  */
}




void wheelSync(double * leftWheel, double * rightWheel, double * leftMotorVelocity, double * rightMotorVelocity) {
  double ratio = 0;
  if (*leftWheel != 0 && *rightWheel != 0) { // if none of the wheels got 0 velocity
    if (abs(*leftWheel) > abs(*rightWheel)) { //this is to avoid fractional ratios or tiny decimals :P
      ratio = *leftWheel / *rightWheel;
      if (abs((*leftMotorVelocity / *rightMotorVelocity) - ratio) > 0.05) { //0.05 is deadband

      }
    } if (abs(*leftWheel) < abs(*rightWheel)) {
      ratio = *rightWheel / *leftWheel;
      if (abs((*leftMotorVelocity / *rightMotorVelocity) - ratio) > 0.05) { //0.05 is deadband

      }
    }
  } else { // if any got zero, no need to synchronize as one motor will be stopped neverthless :P
    return;
  }
}
