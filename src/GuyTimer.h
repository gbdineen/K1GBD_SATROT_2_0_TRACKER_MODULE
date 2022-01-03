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
        GuyTimer(std::function<void()> cb, unsigned long pt);
        
        //bool guyTimer(std::function<void()> cb);
        bool guyTimer(std::function<void()> cb, bool start);
        bool guyTimer(std::function<void()> cb, unsigned long ms=50, bool start=true);

        void setMillis(unsigned long timerMillis);
        void setCallback(std::function<void()> cb);
        
        void start(unsigned long ms=50);
        void stop();
        void loop();

        ~GuyTimer();
};

#endif

