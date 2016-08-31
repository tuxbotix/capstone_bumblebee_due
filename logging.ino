boolean isLoggingOn = false;

/**

Start by resetting openLogger and other start up stuff

*/
int resetOpenLog = 11;


void startLogging() {

  pinMode(resetOpenLog, OUTPUT);
  digitalWrite(resetOpenLog, LOW);
  delay(100);
  digitalWrite(resetOpenLog, HIGH);
  unsigned  long temp = 0;
  temp = millis();
  while (1) {
    if (Serial3.available()) {
      if (Serial3.read() == '<') {
        isLoggingOn = true;
        break;

      }
      if ((millis() - temp) > 500) {//timeout of 30ms~
        isLoggingOn = false;
          Serial1.println("logging failed");
          return;

      }
    }
  }
  Serial1.println("logging Started");

}
/**
Set boolean flag to start logging
*/
void stopLogging() {
  Serial3.write(26);
  Serial3.write(26);
  Serial3.write(26);

  //Wait for OpenLog to respond with '>' to indicate we are in command mode
  unsigned  long temp = 0;
  temp = millis();
  while (1) {
    if (Serial3.available()) {
      if (Serial3.read() == '<') {
        break;
      }
    }
    if ((millis() - temp) > 500) {//timeout of 30ms~
      isLoggingOn = false;
      break;
    }
  }
  Serial3.print("sync\r");
  while (1) {
    if (Serial3.available())
      if (Serial3.read() == '<') break;
    if ((millis() - temp) > 500) {//timeout of 30ms~
      isLoggingOn = false;
      break;
    }
  }
  isLoggingOn = false;
}

boolean isLoggingEnabled() {
  return isLoggingOn;
}

boolean sendLogData(char* message) {
  Serial3.print(message); //\r in string + regular print works with older v2.5 Openlogs
}
boolean sendLogPiece(char* message) {//
  Serial3.print(message); //\r in string + regular print works with older v2.5 Openlogs
  Serial3.print(',');
}
boolean sendLogData(double message) {
  Serial3.print(message); //\r in string + regular print works with older v2.5 Openlogs
  Serial3.print("\r");
}
boolean sendLogPiece(double message) {//
  Serial3.print(message); //\r in string + regular print works with older v2.5 Openlogs
  Serial3.print(',');
}

