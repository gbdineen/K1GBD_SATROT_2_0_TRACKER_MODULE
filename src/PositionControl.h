#ifndef POSITION_CONTROL_H
#define POSITION_CONTROL_H

#include <Arduino.h>
#include <functional>
#include <iostream>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "GuyTimer.h"

class PositionControl
{
    private:
        
        bool calibrationActive;
        bool systemCalibrated;
        uint8_t systemCalibrationScore = 0;
        Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
        GuyTimer * gt;

    public:
        
        PositionControl(/* args */);
        PositionControl(GuyTimer * guyTmr);
        void initBNO();
        void getCalStatus();
        void checkPosition();
        ~PositionControl();

};

#endif
