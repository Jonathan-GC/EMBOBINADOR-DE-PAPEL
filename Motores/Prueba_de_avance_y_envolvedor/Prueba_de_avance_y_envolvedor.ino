/*
 * Multi-motor control (experimental)
 *
 * Move two or three motors at the same time.
 * This module is still work in progress and may not work well or at all.
 *
 * Copyright (C)2017 Laurentiu Badea
 *
 * This file may be redistributed under the terms of the MIT license.
 * A copy of this license has been included with this distribution in the file LICENSE.
 */
#include <Arduino.h>
#include "BasicStepperDriver.h"
#include "MultiDriver.h"
#include "SyncDriver.h"

// Motor steps per revolution. Most steppers are 200 steps or 1.8 degrees/step
#define MOTOR_STEPS 200
// Target RPM for X axis motor
#define MOTOR_X_RPM 70
// Target RPM for Y axis motor
#define MOTOR_Y_RPM 70
// Target RPM for Z axis motor
#define MOTOR_Z_RPM 70


// X motor
#define DIR_X 2
#define STEP_X 5

// Y motor
#define DIR_Y 3
#define STEP_Y 6


// Z motor
#define DIR_Z 4
#define STEP_Z 7

// If microstepping is set externally, make sure this matches the selected mode
// 1=full step, 2=half step etc.
#define MICROSTEPS 1

// 2-wire basic config, microstepping is hardwired on the driver
// Other drivers can be mixed and matched but must be configured individually
BasicStepperDriver stepperX(MOTOR_STEPS, DIR_X, STEP_X);
BasicStepperDriver stepperY(MOTOR_STEPS, DIR_Y, STEP_Y);
BasicStepperDriver stepperZ(MOTOR_STEPS, DIR_Z, STEP_Z);
// Pick one of the two controllers below
// each motor moves independently, trajectory is a hockey stick
// MultiDriver controller(stepperX, stepperY);
// OR
// synchronized move, trajectory is a straight line
SyncDriver controller(stepperX, stepperY,stepperZ);

void setup() {

    Serial.begin(9600);
    /*
     * Set target motors RPM.
     */
    stepperX.begin(MOTOR_X_RPM, MICROSTEPS);
    stepperY.begin(MOTOR_Y_RPM, MICROSTEPS);
    stepperZ.begin(MOTOR_Z_RPM, MICROSTEPS);
    // if using enable/disable on ENABLE pin (active LOW) instead of SLEEP uncomment next two lines
    // stepperX.setEnableActiveState(LOW);
    // stepperY.setEnableActiveState(LOW);
}

void loop() {
    if(Serial.available()>0){
      char data = Serial.read();

      if(data == 'a'){
        pinMode(8,1); digitalWrite(8,0);
        controller.rotate(287*4, 0,0);
        delay(1000);
        controller.rotate(287*6,360*7,0);
       delay(1500);
       //templar
        controller.rotate(-90,90,0);
       //voy a cortar
        controller.rotate(0,360*5,0);
        
      }
       if(data == 'b'){
     controller.rotate(-287*4, 0,0);   
       }
    }


    /*
    delay(1000);
    controller.rotate(90*5, -30*15,0);
    delay(1000);
    controller.rotate(0, -30*15, -60);
    delay(1000);
    controller.rotate(0, 0, 60);
    */
    pinMode(8,1); digitalWrite(8,1);
    delay(15000);

}
