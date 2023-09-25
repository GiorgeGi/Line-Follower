#include <QTRSensors.h>

#define KP 0.095                // Proportional gain
#define KI 0.001                // Integral gain
#define KD 0.7                  // Derivative gain
#define MAX_SPEED 140
#define DEFAULT_SPEED 100
#define MIDDLE_SENSOR 4, 5
#define NUM_SENSORS 8           // Number of sensors used
#define TIMEOUT 2500            // Waits for 2500 us for sensor outputs to go low
#define EMITTER_PIN 2           // Emitter is controlled by digital pin 2
#define DEBUG 1                 // Set to 1 if serial debug output needed

QTRSensorsRC qtrrc((unsigned char[]) {54, 55, 56, 57, 58, 59, 60, 61}, NUM_SENSORS, TIMEOUT, EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS];

// pins for TB6612FNG
#define PWMA 3
#define AIN1 5
#define AIN2 4
#define PWMB 9
#define BIN1 7
#define BIN2 8
#define STBY 6


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

void manual_calibration() {
  int i;
  for (i = 0; i < 250; i++) {                           // The calibration will take a few seconds
    qtrrc.calibrate(QTR_EMITTERS_ON);
    delay(20);
  }
  if (DEBUG) {                                          // If true, generate sensor dats via serial output
    Serial.begin(9600);
    for (int i = 0; i < NUM_SENSORS; i++) {
      Serial.print(qtrrc.calibratedMinimumOn[i]);
      Serial.print(' ');
    }
    Serial.println();
    for (int i = 0; i < NUM_SENSORS; i++) {
      Serial.print(qtrrc.calibratedMaximumOn[i]);
      Serial.print(' ');
    }
    Serial.println();
    Serial.println();
  }
}

int computePID(unsigned int sensors[]) {
  int position = qtrrc.readLine(sensors);           // Get the distance
  int error = position - 3500;                      // Get the error value - proportional term
  int lastError = 0;                                // Variables for PID operations
  int totalError = 0;
  int deltaError = error - lastError;               // Predicts the error through difference - derivative term
  int Pvalue = KP * error;                          // Compute the PID components P term, I term, D term
  int Ivalue = KI * totalError;
  int Dvalue = KD * deltaError;
  int motorSpeed = Pvalue + Ivalue + Dvalue;        // PID algorithm for the PID value
                                                    // The sum of present errors (P), accumulation of past errors (I) and prediction of future errors based on current rate of change (D)
                                                    // Necessary operations for past and future errors
  lastError = error;                                // Operation for error prediction - derivative term
  totalError += error;                              // Accumalates the error - integral term
  return motorSpeed;
}

void setMotorSpeeds(int motorSpeed) {
  int rightMotorSpeed = DEFAULT_SPEED + motorSpeed;
  int leftMotorSpeed = DEFAULT_SPEED - motorSpeed;

  if (rightMotorSpeed > MAX_SPEED) rightMotorSpeed = MAX_SPEED;         // Limit top speed
  if (leftMotorSpeed > MAX_SPEED) leftMotorSpeed = MAX_SPEED;

  if (rightMotorSpeed < 0) {                                            // Keep motors speed above 0
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    analogWrite(PWMB, abs(leftMotorSpeed));
    Serial.println("Left motor going backwards");
  }

  if (leftMotorSpeed < 0) {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    analogWrite(PWMA, abs(rightMotorSpeed));
    Serial.println("Right motor going backwards");
  }

  digitalWrite(AIN1, HIGH);                                             // Set the motors
  digitalWrite(AIN2, LOW);
  analogWrite(PWMA, rightMotorSpeed);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  analogWrite(PWMB, leftMotorSpeed);
}

