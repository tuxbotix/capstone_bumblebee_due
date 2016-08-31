
void processSerialInputs() { //called in the loop
  /**
  Serial 0
  List of commands
  $bb*                            PC asking for current location
  $pt,[double x],[double y]*      PC sending point to store
  $alive*                         PC replies its alive
  $trajStart*                     PC ask to start trajectory system
  $trajStop*                      PC ask to stop trajectory system
  $clearPoints*                   PC ask to clear the buffer and reset stored coordinates

  TODO

  Set position
  Set zero/ reset
  **/
  if (stringComplete) {
    if (inputString == "bb") {// message from PC asking position & heading
      sendPositionPC();

    } else if (inputString == "alive") {
      // Serial.println("YEESSSSSSSSSSSSSSSS");

      isPCAlive = true;

    } else if (inputString.length() > 4 && inputString.substring(0, 2) == "pt") {
      int commaIndex = inputString.indexOf(',');
      double xIn, yIn;
      if (commaIndex != -1) {
        xIn = strtod(inputString.substring(commaIndex + 1).c_str(), NULL);
      }
      commaIndex = inputString.indexOf(',', commaIndex + 1);
      if (commaIndex != -1) {
        yIn = strtod(inputString.substring(commaIndex + 1).c_str(), NULL);
        addPoint(xIn, yIn);
      }

    } else if (inputString == "trajStart") {
      Serial1.println("trajstart");
      startTraj();
    } else if (inputString == "stopMotion") {
      Serial1.println("trajstart");
      stopTraj();
    } else if (inputString == "trajStop") {
      Serial1.println("trajstart");
      stopTraj();
    } else if (inputString == "clearPoints") {
      for (short i = 0; i < TRAJ_INDEX_MAX; i++) {
        disablePoint(i);

      }
      ringBufIndex = 0;
      lastUpdatedIndex = -1;
      addedPoints = 0;
      Serial1.println("removed Points");
    }
    inputString = "";
    stringComplete = false;
  }

  /**
  Serial 1 - WIFI and Debug
  List of commands
  $bb*                            PC asking for current location
  $pt,[double x],[double y]*      PC sending point to store
  $alive*                         PC replies its alive
  $trajStart*                     PC ask to start trajectory system
  $trajStop*                      PC ask to stop trajectory system
  $clearPoints*                   PC ask to clear the buffer and reset stored coordinates

  TODO

  Set position
  Set zero/ reset
  **/
  if (stringCompleteDebug) {
    if (inputStringDebug == "bb") {// message from PC asking position & heading
      sendPositionPC();

    } else if (inputStringDebug == "alive") {
      // Serial.println("YEESSSSSSSSSSSSSSSS");

      isPCAlive = true;

    } else if (inputStringDebug.length() > 4 && inputStringDebug.substring(0, 2) == "pt") {
      int commaIndex = inputStringDebug.indexOf(',');
      double xIn, yIn;
      if (commaIndex != -1) {
        xIn = strtod(inputStringDebug.substring(commaIndex + 1).c_str(), NULL);
      }
      commaIndex = inputStringDebug.indexOf(',', commaIndex + 1);
      if (commaIndex != -1) {
        yIn = strtod(inputStringDebug.substring(commaIndex + 1).c_str(), NULL);
        addPoint(xIn, yIn);
      }

    } else if (inputStringDebug == "trajStart") {
      Serial1.println("trajstart");
      startLogging();

      ticks = 0;
      if (isLoggingEnabled()) {
        beginCSVData();
        delay(20);
      }


      startTraj();

    } else if (inputStringDebug == "trajStop") {
      Serial1.println("trajstart");
      void stopLogging();
      stopTraj();
    } else if (inputStringDebug == "zero") {
      xMM = 0;
      yMM = 0;
      Serial1.println("zero");

    } else if (inputStringDebug == "clearPoints") {
      for (short i = 0; i < TRAJ_INDEX_MAX; i++) {
        disablePoint(i);

      }
      ringBufIndex = 0;
      lastUpdatedIndex = -1;
      addedPoints = 0;
      Serial1.println("removed Points");
    }
    else if (inputStringDebug == "calib") {
      calibrateCompass();

    }
    inputStringDebug = "";
    stringCompleteDebug = false;
  }

}

/*
  SerialEvent occurs whenever a new data comes in the
 hardware serial RX.  This routine is run between each
 time loop() runs, so using delay inside loop can delay
 response.  Multiple bytes of data may be available.
 */
void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    if (inChar == '$') { //beginning of message;
      //ideally two buffers should be used to make sure nothing is missed.
      inputString = "";
    } else if (inChar == '*') {
      stringComplete = true;
    } else {
      inputString += inChar;
      // if the incoming character is a newline, set a flag
      // so the main loop can do something about it:
    }

  }
}

/*
  For Serial 1 - WIFI
 */
void serialEvent1() {
  while (Serial1.available()) {
    // get the new byte:
    char inChar = (char)Serial1.read();
    // add it to the inputString:
    if (inChar == '$') { //beginning of message;
      //ideally two buffers should be used to make sure nothing is missed.
      inputStringDebug = "";
    } else if (inChar == '*') {
      stringCompleteDebug = true;
    } else {
      inputStringDebug += inChar;
      // if the incoming character is a newline, set a flag
      // so the main loop can do something about it:
    }

  }
}
