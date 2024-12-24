#include <Arduino.h>
#include <Servo.h>
#include "Adafruit_VL53L1X.h"

#define TRIG_PIN 2
#define ECHO_PIN 3
#define IRQ_PIN 2
#define XSHUT_PIN 3

// create servo object to control a servo
Servo servo;        
// create a VL53L1X object
Adafruit_VL53L1X vl53 = Adafruit_VL53L1X(XSHUT_PIN, IRQ_PIN);

// declare variables
float input = 0, error = 0, kp = 0.3, ki = 0.01, kd = 0.15, setpoint = 200, output = 0;
float maxOutput = 70, minOutput = -60;
float distance;

// PID controller
void PID() {
  // declare variables
  double derror;
  static float ierror = 0;
  static float prvError;
  double dt = 0;
  static unsigned long prvMillis = 0;

  // wait till the data is ready
  if (vl53.dataReady()) {
    // new measurement for the taking!
    distance = vl53.distance();
    // check if the distance is valid
    if (distance >= 0 && distance < 500) {
      input = distance;
    }
    // data is read out, time for another reading!
    vl53.clearInterrupt();
    // Calculate the delta time
    dt = (millis() - prvMillis)/1000.0 ; 
    prvMillis = millis();
    // Calculate the error
    error = setpoint - input;
    // Calculate the derivative of the error
    derror = (error - prvError)/dt;
    // Update the previous error
    prvError = error;
    // Calculate the integral of the error
    // Check if the output is within the limits
    if (output < maxOutput && output > minOutput) {
      ierror += error * dt;
    }

    // Calculate the output

    output = kp * error + ki * ierror + kd * derror;

    // constarin the output
    if (output > maxOutput) output = maxOutput;
    if (output < minOutput) output = minOutput;

    Serial.print("Distance is   =  ");
    Serial.print(input);
    Serial.print(" Output of the PID is   =  ");
    Serial.println(output);
  }
}


void setup() {
  // initialize serial communication
  Serial.begin(115200);
  // initialize servo
  servo.attach(9);
  // initialize the VL53L1X sensor
  while (!Serial) delay(10);

  Serial.println(F("Adafruit VL53L1X sensor demo"));

  Wire.begin();
  if (!vl53.begin(0x29, &Wire)) {
    Serial.print(F("Error on init of VL sensor: "));
    Serial.println(vl53.vl_status);
    while (1) delay(10);
  }
  Serial.println(F("VL53L1X sensor OK!"));

  Serial.print(F("Sensor ID: 0x"));
  Serial.println(vl53.sensorID(), HEX);

  if (!vl53.startRanging()) {
    Serial.print(F("Couldn't start ranging: "));
    Serial.println(vl53.vl_status);
    while (1) delay(10);
  }
  Serial.println(F("Ranging started"));

  // Valid timing budgets: 15, 20, 33, 50, 100, 200 and 500ms!
  vl53.setTimingBudget(15);
  Serial.print(F("Timing budget (ms): "));
  Serial.println(vl53.getTimingBudget());
  // wait for serial port to open on native usb devices
}

void loop() {
  PID();
  servo.write(110 + output);
}