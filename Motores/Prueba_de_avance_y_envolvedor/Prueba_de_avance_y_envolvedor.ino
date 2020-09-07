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
#define MOTOR_X_RPM 150
// Target RPM for Y axis motor
#define MOTOR_Y_RPM 150
// Target RPM for Z axis motor
#define MOTOR_Z_RPM 150


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

    Serial.begin(115200);
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

char data;
void loop() {
    if(Serial.available()>0){
        
        data = Serial.read();

      
    }

    if(data == 'a'){
        pinMode(8,1); digitalWrite(8,0);
        controller.rotate(255*3, 0,0);
        delay(6000);
        controller.rotate(1021*3, 440*3,0);
        delay(2000);
        controller.rotate(1021*5, 365*5,0);
        delay(2000);
        controller.rotate(255*5, 150*2,0);
        delay(1000);
        controller.rotate(-200*1,0,0);
        /*
        controller.rotate(1012*1,360*0,0);
        controller.rotate(1012*1,360*1,0);
        controller.rotate(1012*1,360*0,0);
        controller.rotate(1012*1,360*4,0);
        */
        
        //controller.rotate(100,90,0);
        //controller.rotate(-100,0,0);
        //controller.rotate(0,360*2,0);
       // controller.rotate(0,0,10);
       // controller.rotate(0,0,0);
        //controller.rotate(-180*1,360*1,0);

        //controller.rotate(0,360*5,0);
       // delay(1500);
        //templar
        //controller.rotate(200*1,360*1,0);
       // delay(1500);
       //voy a cortar
       //controller.rotate(0,360*5,0);
       data = 0;
      }
      
       if(data == 'b'){
          pinMode(8,1); digitalWrite(8,0);
          controller.rotate(1003, 0,0);   
       }

       if(data == 'c'){
          pinMode(8,1); digitalWrite(8,0);
          controller.rotate(0, 360 * 10,0);   
       }

       if(data == 'r'){
          pinMode(8,1); digitalWrite(8,0);
          controller.rotate(-1003,0,0);   
       }

        if(data == 'q'){
          pinMode(8,1); digitalWrite(8,0);
          controller.rotate(1012 * 10,0,0);
          //delay(2000);
          //controller.rotate(-1003 * 2,0,0);
       }

       
        if(data == 'w'){
          pinMode(8,1); digitalWrite(8,0);
          controller.rotate(-1012 * 10,0,0);   
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
    data = 0;
    delay(1000);

}
