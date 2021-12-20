#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>
#include <Adafruit_MotorShield.h>
#include <iostream>
#include <istream>
#include "Adafruit_PWMServoDriver.h"

#define CLOCKWISE 0
#define COUNTER_CLOCKWISE 1
#define FULL_STOP 2

class MotorControl
{
    private:
        
        // LET'S DECLARE SOME CONSTANCES FOR INTERACTING WITH THE SERVOS
        const int MIN_PULSE_WIDTH = 210;
        const int MAX_PULSE_WIDTH = 1044;
        const float PWM_FREQUENCY = 100;



        /************************************************************************************
        The Adafruit PWM library breaks the PWM signal into 64-bit resolution (0 - 4096)
        but we don't need nearly that much granularity, so we'll scale it down to 4-bit
        and use a scale of 0 - 15 like so:
                    -------- 0 --------------- 7 -------------- 15 ----------
                    -- FULL SPEED CW ------ FULL STOP ------ FULL SPEED CCW --
        ************************************************************************************/
        
        const double SERVO_LOW_RANGE = 0;
        const double SERVO_HIGH_RANGE = 128;
        
        Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();  // Init PWM Driver
        Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
        Adafruit_DCMotor *myMotor = AFMS.getMotor(1);
        
        int calcPWM(int spd);

    public:
        MotorControl();
        void begin();
        void moveServo(int spd);
        void moveServo(int svo, int spd, int dir);
        void moveDCMotor(int pos);
};

#endif

