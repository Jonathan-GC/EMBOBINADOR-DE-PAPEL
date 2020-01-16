/*
 * Using accelerated motion ("linear speed") in nonblocking mode
 *
 * Copyright (C)2015-2017 Laurentiu Badea
 *
 * This file may be redistributed under the terms of the MIT license.
 * A copy of this license has been included with this distribution in the file LICENSE.
 */
#include <Arduino.h>

// Motor steps per revolution. Most steppers are 200 steps or 1.8 degrees/step
#define MOTOR_STEPS 200
// Target RPM for cruise speed
#define RPM 70
// Acceleration and deceleration values are always in FULL steps / s^2
#define MOTOR_ACCEL 2000
#define MOTOR_DECEL 2000

// Microstepping mode. If you hardwired it to save pins, set to the same value here.
#define MICROSTEPS 1

#define DIR 3
#define STEP 6
//#define SLEEP 13 // optional (just delete SLEEP from everywhere if not used)
 

#include "BasicStepperDriver.h" // generic
BasicStepperDriver stepper(MOTOR_STEPS, DIR, STEP);

void setup() {
    Serial.begin(115200);

    stepper.begin(RPM, MICROSTEPS);
    // if using enable/disable on ENABLE pin (active LOW) instead of SLEEP uncomment next line
    // stepper.setEnableActiveState(LOW);
    stepper.enable();
   

    /*
     * Set LINEAR_SPEED (accelerated) profile.
     * 
     * FUNCION PARA ACELERAR
     */
    stepper.setSpeedProfile(stepper.LINEAR_SPEED, MOTOR_ACCEL, MOTOR_DECEL);

    Serial.println("START");
    /*
     * Using non-blocking mode to print out the step intervals.
     * We could have just as easily replace everything below this line with 
     * stepper.rotate(360);
     */
     //stepper.startRotate(360);
     stepper.rotate(720);
}

void loop() {
    static int step = 0;
    boolean wait_time = stepper.nextAction();
    if (wait_time){
          //Serial.println(wait_time);
        Serial.print("  step="); Serial.print(step++);
        Serial.print("  dt="); Serial.print(wait_time);
        Serial.print("  rpm="); Serial.print(stepper.getCurrentRPM());
        Serial.println();
    } else {
        stepper.disable();
        Serial.println("END");
        delay(2000);
    }
}
