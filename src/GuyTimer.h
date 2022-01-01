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
        bool timerEnabled;
        std::function<void()> cb;
        void timerCheck();
        
    public:
        GuyTimer(/* args */);
        GuyTimer(std::function<void(bool)> cb, unsigned long timerMills);
        
        void guyTimer(std::function<void()> fn, unsigned long pt);
        void setMillis(unsigned long timerMillis);
        void start();
        void stop();
        void loop();

        ~GuyTimer();
};

#endif

