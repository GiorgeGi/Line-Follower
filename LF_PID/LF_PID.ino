#include "LF_PID.h"

void setup() {
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(STBY, OUTPUT);
  manual_calibration();
  digitalWrite(STBY, HIGH);
}

void loop() {
  unsigned int sensors[NUM_SENSORS];
  int motorSpeed = computePID(sensors);
  setMotorSpeeds(motorSpeed);
}
