#ifndef LF_PID_H
#define LF_PID_H

#include <Arduino.h>
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

extern QTRSensorsRC qtrrc;
extern unsigned int sensorValues[NUM_SENSORS];

// pins for TB6612FNG
#define PWMA 3
#define AIN1 5
#define AIN2 4
#define PWMB 9
#define BIN1 7
#define BIN2 8
#define STBY 6

void manual_calibration();
int computePID(unsigned int sensors[]);
void setMotorSpeeds(int motorSpeed);

#endif
