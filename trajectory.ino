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

boolean isTrajectoryRunning = false; //only after this the planner will work.
double initialPositionX, initialPositionY, pathLength;

void startTraj() {
  if (addedPoints <= 0) {
    Serial1.println("No Points Added");
    return;
  }
  //  if (addedPoints == 1) {
  initialPositionX = xMM;
  initialPositionY = yMM;
  //  }
  updateNewTrajectoryStuff();
  isTrajectoryRunning = true;
}

void stopTraj() {
  isTrajectoryRunning = false;
  useGoalPoint = false;
  stopAllMotion();    headingOut = 0;
  stopLogging();
}

boolean isTrajRunning() {
  return isTrajectoryRunning;
}
void planner() { //run in systick, No idea about performance with all the calculations


  if (!isTrajectoryRunning) {
    return;
  }
  /**
  Keep Running the planner only if these conditions satisfied, else trajectory system would be halted altogether
  1. Nopoints added to the system! Then no point of continuing
  2. All added points have been successfully navigated
  3. Current active value is NOT active - to handle buggy behaviors
  **/

  if (addedPoints == 0 || (addedPoints - completedPoints) <= 0  || !activeValue[ringBufIndex] ) {
    stopTraj();
    Serial1.print("Finished");
    return;
  } else {
    startMotion();
  }

  updateGoalPoint();
  totalDistance = getDistance((goalPointX - xMM ), (goalPointY - yMM ));

  if (useGoalPoint) { //determined by call for "updateGoalPoint()"
    distance = getDistance((goalPointX - xMM), (goalPointY - yMM)); //distance to lookahead target point
  } else {
    distance = totalDistance;
    if (distance < DEADBAND) { //12cm inaccuracy is quite fine ;)
    //  sendPosition();
      disablePoint(ringBufIndex); //disable current point as it's done with navigation
      incrementIndex();  //increment to the new one.
      completedPoints++;
      requestNewPoint();  //at the same time, ask for a new point
      updateNewTrajectoryStuff();
      //      updateGoalPoint();  // add goalpoint or not,etc.
      Serial1.print("New mainPoint ; Points remain: ");
      Serial1.println((addedPoints - completedPoints));
    }
  }

}

void updateNewTrajectoryStuff() { //need only for starting New paths
  if (addedPoints >= 2 && completedPoints > 0) { //to make the bot start from current position to first target.
    pathHeading = getPathHeading(targetsX[getPrevIndex()], targetsY[getPrevIndex()], targetsX[ringBufIndex], targetsY[ringBufIndex]);
  } else {
    pathHeading = getPathHeading(initialPositionX, initialPositionY, targetsX[ringBufIndex], targetsY[ringBufIndex]);
  }
}

void updateTargetPath() {
  targetHeading = getPathHeading(xMM, yMM, goalPointX, goalPointY);
}

/**
Determines goal points in distance of lookahead. If the robot and target point situated closer than lookahead, goalpoint = target point
else goalpoint = point between target point & robot with distance of lookahead from robot

      UPDATE 03.06.2015 - will keep updating goal point on each loop.
**/

void updateGoalPoint() {

  if (addedPoints >= 2 && completedPoints > 0) { //to make the bot start from current position to first target.
    normalDistance = getNormalDistance(getPrevIndex(), ringBufIndex, &xMM, &yMM); //since the starting and robot's position points automatically lie on same place, normal distance =0
  } else {
    normalDistance = getNormalDistanceInit(&initialPositionX, &initialPositionY, ringBufIndex, &xMM, &yMM); //since the starting and robot's position points automatically lie on same place, normal distance =0

  }

  int_fast64_t checkLookAhead = (int_fast64_t)getDistance((nearestPointX - targetsX[ringBufIndex]), (nearestPointY - targetsY[ringBufIndex])); //check if lookahead is needed

  if (checkLookAhead > (LOOK_AHEAD)) {

    goalPointX =  nearestPointX + cos(pathHeading) * LOOK_AHEAD;  //compute goalpoint. the above check ensure the goalpoint lies on the line*
    goalPointY =  nearestPointY + sin(pathHeading) * LOOK_AHEAD;
    useGoalPoint = true;

  } else if (activeValue[getNextIndex()]) { // if closer to the nearest end of point, check if next target point exist. If so, find a point on that place.
    int_fast64_t diff = (int_fast64_t)(LOOK_AHEAD - checkLookAhead);
    double tempPathHeading = getPathHeading(targetsX[ringBufIndex], targetsY[ringBufIndex], targetsX[getNextIndex()], targetsY[getNextIndex()]);

    goalPointX =  targetsX[getNextIndex()] + cos(tempPathHeading) * diff;  //compute goalpoint. the above check ensure the goalpoint lies on the line*
    goalPointY =  targetsY[getNextIndex()] + sin(tempPathHeading) * diff;

    if (diff < DEADBAND) { //if nearestPoint is closer than deadband to the target, its time to switch to the next point
   //   sendPosition();
      disablePoint(ringBufIndex); //disable current point as it's done with navigation
      incrementIndex();  //increment to the new one.
      completedPoints++;
      requestNewPoint();  //at the same time, ask for a new point
      //      updateNewTrajectoryStuff(); // Replaced by pathHeading = tempPathHeading; -> same result**
      pathHeading = tempPathHeading;
      Serial1.print("New mainPoint ; Points remain: ");
      Serial1.println((addedPoints - completedPoints));
    }
    useGoalPoint = true;

  } else {// When its the last point for the whole path set and near end of the path segment.
    goalPointX = targetsX[ringBufIndex];
    goalPointY = targetsY[ringBufIndex];
    useGoalPoint = false;

    /*   Serial.print("point : ");
       Serial.print(completedPoints);
       Serial.println("NO goalpoint");
       */
  }

  if (useGoalPoint) {
    double goalPointHead = getPathHeading(goalPointX, goalPointY, targetsX[ringBufIndex], targetsY[ringBufIndex]);
    if (abs(pathHeading - goalPointHead) > 0.3) { //Test for overshoot - to avoid generating goalpoints even further from actual place...
      //      pathHeading = -pathHeading;
      goalPointX = targetsX[ringBufIndex];
      goalPointY = targetsY[ringBufIndex];
    }

  }

  updateTargetPath();

  Serial1.print("Points to go");
  Serial1.print(completedPoints);
  Serial1.print("/");

  Serial1.print(addedPoints);

  /*
  Serial1.print(" nX :");
  Serial1.print(nearestPointX);
  Serial1.print(" nY : ");
  Serial1.print(nearestPointY);
  Serial1.print(" X :");
  Serial1.print(goalPointX);
  Serial1.print(" Y : ");
  Serial1.print(goalPointY);
  Serial1.print("targetHead");
  Serial1.println(targetHeading);
  */
}


void addPoint(double x, double y) {
  lastUpdatedIndex++;
  if (!(lastUpdatedIndex < TRAJ_INDEX_MAX)) {
    lastUpdatedIndex = 0;
  }
  targetsX[lastUpdatedIndex] = x;
  targetsY[lastUpdatedIndex] = y;
  activeValue[lastUpdatedIndex] = true;
  addedPoints++;
  Serial1.print("Point");
  Serial1.print(addedPoints);
  Serial1.print(" : ");
  Serial1.print(x);
  Serial1.print(" : ");
  Serial1.println(y);

}

void incrementIndex() {
  if (ringBufIndex < TRAJ_INDEX_MAX) {
    ringBufIndex++;
  } else {
    ringBufIndex = 0;
  }
}
short getNextIndex() {
  if (ringBufIndex < TRAJ_INDEX_MAX) {
    return ringBufIndex + 1;
  } else {
    return 0;
  }
}

short getPrevIndex() {
  if (ringBufIndex > 0) {
    return ringBufIndex - 1;
  } else {
    return TRAJ_INDEX_MAX - 1;
  }
}

void disablePoint(short index) {
  activeValue[index] = false;
}

/**
TODO send a number - to sync actual number of sent and recieved points
**/

void requestNewPoint() {
  if ((addedPoints - completedPoints) > 2) {//at least two cells should be empty prior to asking a new one
    Serial.println("$point*");
  }

}

double getDistance(double xIn, double yIn) {

  return (double)sqrt(pow(xIn, 2) + pow(yIn, 2));
}

double getPathHeading(double x1, double y1, double x2, double y2) {
  return atan2((y2 - y1), (x2 - x1));
}

/**

normal distance;

distance(P1, P2, (X0, Y0)) = abs( (y2 - y1)*x0   - (x2 - x1)*y0 + x2*y1 -y2*x1)
                             --------------------------------------------------
                              getDistance( (p2[x] - p1[x]), (p2[y] - p1[y]))
*/
double getNormalDistance(short startPointIndex, short endPointIndex, double* curPositionX, double* curPositionY) {
  double y1, y2, x1, x2;
  x1 = targetsX[startPointIndex];
  y1 = targetsY[startPointIndex];
  x2 = targetsX[endPointIndex];
  y2 = targetsY[endPointIndex];

  setNormalInteceptingPoint(&x1, &y1, &x2, &y2, curPositionX, curPositionY, &nearestPointX, &nearestPointY);
  return abs( (y2 - y1) * (*curPositionX)   - (x2 - x1) * (*curPositionY) + x2 * y1 - y2 * x1) / getDistance((x2 - x1), (y2 - y1));

}

double getNormalDistanceInit(double* initStartPointX, double* initStartPointY, short endPointIndex, double* curPositionX, double* curPositionY) {
  double y1, y2, x1, x2;
  x1 = *initStartPointX;
  y1 = *initStartPointY;
  x2 = targetsX[endPointIndex];
  y2 = targetsY[endPointIndex];

  setNormalInteceptingPoint(initStartPointX, initStartPointY, &x2, &y2, curPositionX, curPositionY, &nearestPointX, &nearestPointY);
  return abs( (y2 - y1) * (*curPositionX)   - (x2 - x1) * (*curPositionY) + x2 * y1 - y2 * x1) / getDistance((x2 - x1), (y2 - y1));

}

void setNormalInteceptingPoint(double* x1, double* y1, double* x2, double* y2, double* curPositionX, double* curPositionY, double* nearestPointXptr, double* nearestPointYptr) {
  double dx = *x2 - *x1;
  double dy = *y2 - *y1;
  double mag = sqrt(dx * dx + dy * dy);
  dx /= mag;
  dy /= mag;
  // translate the point and get the dot product
  double lambda = (dx * (*curPositionX - *x1)) + (dy * (*curPositionY - *y1));
  *nearestPointXptr = (dx * lambda) + *x1;
  *nearestPointYptr = (dy * lambda) + *y1;
}
