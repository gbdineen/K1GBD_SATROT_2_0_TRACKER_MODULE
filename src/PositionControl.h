#ifndef POSITION_CONTROL_H
#define POSITION_CONTROL_H

#include <Arduino.h>
#include <EEPROM.h>
#include <functional>
#include <iostream>
#include <memory>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "GuyTimer.h"
#include "MotorControl.h"

// SERVO CONTROL TYPES
#define MANUAL 0 // speed control via rotary knobs
#define AUTO 1 // az/el changes updated via rotary knobs
#define UDP 2 // az/el updated via UDP packets
#define EEPROM_SIZE 64
#define AZIMUTH_SERVO 0
#define ELEVATION_SERVO 1
#define ROLL_MOTOR 2

// struct CalibrationData
// {

//     signed long accll;
//     signed long accllR;
//     signed long gyro;
//     signed long mag;
//     signed long magR;


// };

class PositionControl
{
    private:
        
        int tempTimerCount =0;
        
        int bnoInterval = 50;
        bool calibrationActive;
        bool systemCalibrated;
        bool prevPosSet = false;
        uint8_t controlMethod = AUTO;
        uint8_t systemCalibrationScore;
        uint8_t magScore;
        Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
        GuyTimer * gt;
        GuyTimer azTimer;
        GuyTimer elTimer;
        GuyTimer rollTimer;
        MotorControl * mc;
        std::function<void()> calibrationCallback;
        std::function<void(int az, int el, int roll)> targetsCallback;

        uint16_t currAz;
        int currEl;
        int currRoll;
        uint16_t targAz;
        int targEl;
        int targRoll;
        uint16_t prevAz;
        int prevEl;
        int prevRoll;
        int addr = 0;

        bool trackingAz;
        bool trackingEl;
        bool trackingRoll;

        template <typename T>
        adafruit_bno055_offsets_t getEEPROM(int adr, T tp);
        template <typename T>
        void putEEPROM(int adr, T val);
        adafruit_bno055_offsets_t getSensorOffsets(adafruit_bno055_offsets_t &calibData);
        void setSensorOffsets(adafruit_bno055_offsets_t &calibData);
        bool checkCalibrationEEPROM();
       
    public:
        
        void initBNO();
        void autoCalibration();
        uint8_t calibrateSystem();
        void parkAntenna(int azPos=355, int elPos=0);
        void checkPosition();
        void trackAz();
        void trackEl();
        void trackRoll();
        void antennaStationaryCheck();
        void updateKeps(int az, int el);
        void getCurrentTargets();
        void setControlMethod(uint8_t cm);
        uint8_t servoDirection(int targ, int prev, int servo);
        uint8_t motorDirection(uint16_t targ, uint16_t prev);
        void setCalibrationCallback(std::function<void()> cb);
        void setTargetsCallback(std::function<void(int az, int el, int roll)> cb);
        void setTargets();
        void loop();
        
        PositionControl(/* args */);
        PositionControl(GuyTimer * guyTmr);
        PositionControl(GuyTimer * guyTmr, MotorControl * mtrCtrl);
        ~PositionControl();

};

#endif
