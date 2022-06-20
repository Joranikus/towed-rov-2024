/* Author: TowedROV NTNU 2022

   Demo for WingStepper library

   This code is written for Teensy but should work with arduino aswell.
   SENSOR is a magnetic sensor. Two magnets are mounted on the stepper shaft.
   SENSOR is HIGH when a magnet is close.

    Write a number in Serial Monitor and watch the stepper move.
    IMPORTANT: Select No line ending in Serial Monitor

*/

#include "WingStepper.h"
                           // SB   PORT
const int STEP_PIN   = 3;  // 6     3
const int DIR_PIN    = 2;  // 5     2
const int SENSOR_PIN = 4;  // 7     4

WingStepper stepper_port(STEP_PIN, DIR_PIN, SENSOR_PIN);

#define CALIBRATING 0
#define MOVING      1
#define STANDBY     2
int state = CALIBRATING;  //state at start. use STANDBY if you do not want to calibration

float desired_angle = 0;

void setup() {
  stepper_port.begin();
  stepper_port.flip_ref_direction();
  //stepper_port.set_offset_angle(-90);
  
  // Serial.begin(9600); not necessary with Teensy

}

void loop() {

  // get user number input
  if (Serial.available() > 0) {
    desired_angle = Serial.parseInt();
    Serial.print("desired_angle: "); Serial.println(desired_angle);
    delay(500);
    state = MOVING;
  }

  switch (state) {
    case CALIBRATING: {
        // pratically, it places the sensor in the middle of the two magnets
        
        bool is_calibrated = false;
        is_calibrated = stepper_port.calibrate(); // calibrate returns true when finished
        
        // write something to serial, and the program will stop
        while (Serial.available());

        //change state when finished
        if (is_calibrated) {
          Serial.println("Finished calibrating!");
          state = MOVING;
        }
      }
      break;

    case MOVING: {
        bool finished = stepper_port.rotate(desired_angle);
        if (finished) {
          state = STANDBY;
        }
      }
      break;

    case STANDBY: {
        //wait for command
        Serial.println("standby: enter number to move stepper");
        delay(1000);
      }
      break;
  }
}
