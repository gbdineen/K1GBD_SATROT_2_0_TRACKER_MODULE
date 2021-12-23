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
        unsigned long timerMillis;
        
    public:
        GuyTimer(/* args */);
        GuyTimer(std::function<void(bool)> cb, unsigned long timerMills);
        
        void guyTimer();
        void timerCheck();
        void timerCheck(std::function<void(const char*)> fn, unsigned long prevTime);
        template <typename T>
        void timerCheck(std::function<void(T)> fn);
        void loop();
        void loop(std::function<void(bool)> fn);
        void setTimer(unsigned long timerMillis);

        ~GuyTimer();
};

#endif

