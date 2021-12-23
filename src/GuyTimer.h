#ifndef GUY_TIMER_H
#define GUY_TIMER_H

#include <Arduino.h>
#include <functional>
#include <iostream>
//#include "PositionControl.h"

class GuyTimer
{
    private:
        unsigned long millSecOne = 1000;
        unsigned long prevTime = 0;
    public:
        GuyTimer(/* args */);
        
        void guyTimer(unsigned long timerMills);
        void timerCheck(std::function<void(const char*)> fn, unsigned long prevTime);
        void timerCheck(std::function<void(bool)> fn);
        void loop();
        void loop(std::function<void(bool)> fn);
        ~GuyTimer();
};

#endif

