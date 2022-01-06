#ifndef POSITION_CONTROL_H
#define POSITION_CONTROL_H

#include <Arduino.h>
#include <functional>
#include <iostream>
#include <memory>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "GuyTimer.h"
#include "MotorControl.h"

#define MANUAL 0
#define AUTO 1

class PositionControl
{
    private:
        
        bool calibrationActive;
        bool systemCalibrated;
        uint8_t controlMethod = AUTO;
        uint8_t systemCalibrationScore = 0;
        Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
        GuyTimer * gt;
        GuyTimer azTimer;
        GuyTimer * elTimer;
        MotorControl * mc;
        uint16_t currAz;
        uint16_t currEl;
        uint16_t currRoll;
        uint16_t targAz;
        uint16_t targEl;
        uint16_t prevAz;
        uint16_t prevEl;

        bool trackingAz;


    public:
        
        void initBNO();
        void autoCalibration();
        uint8_t getCalStatus();
        void checkPosition();
        void trackAz();
        void trackEl();
        void updateKeps(uint16_t az, uint16_t el);
        void setControlMethod(uint8_t cm);
        uint8_t direction(uint16_t targ, uint16_t prev);
        void loop();
        
        PositionControl(/* args */);
        PositionControl(GuyTimer * guyTmr);
        PositionControl(GuyTimer * guyTmr, MotorControl * mtrCtrl);
        ~PositionControl();

};

#endif
