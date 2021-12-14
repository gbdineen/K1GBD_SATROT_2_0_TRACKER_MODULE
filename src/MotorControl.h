#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>
#include <Adafruit_MotorShield.h>
#include <iostream>
#include "Adafruit_PWMServoDriver.h"



class MotorControl
{
    private:
        #define SERVO_FREQ 50
        #define DEGREE_OFFSET 1 // This is the amount +/- the servo can go past 
                                // the targeted Az/El so the antenna doesn't blow
                                // past the target and cause an infite bounce

        // *******************************************************
        // setPWM() IN THE ADAFRUIT PWM LIBRARY
        // ****  NOTE! CW NUMBERS ARE OPPOSITES. FASTER IS A LOWER NUMBER, SLOWER IS HIGHER. JUST FYI. 
        #define SERVO_CW_MAX 370 // @ 100HZ
        #define SERVO_CW_MIN 614 // @ 100HZ
        #define SERVO_CCW_MIN 656 // @ 100HZ
        #define SERVO_CCW_MAX 880 // @ 100HZ
        #define SERVO_STOP 635 // @ 100Hz

        Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();  // Init PWM Driver
        Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
        Adafruit_DCMotor *myMotor = AFMS.getMotor(1);

       // WS_Client * wscMc;
        

    public:
        MotorControl();
        //MotorControl(WS_Client * wsClient);
        void initMotors();
        void moveMotor(int pos);
};

#endif

