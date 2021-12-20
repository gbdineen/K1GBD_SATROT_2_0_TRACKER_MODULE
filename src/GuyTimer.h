#ifndef GUY_TIMER_H
#define GUY_TIMER_H

#include <Arduino.h>
#include <functional>
//#include "PositionControl.h"

class GuyTimer
{
    //
    private:
        /* data */
    public:
        GuyTimer(/* args */);
        //template <typename T, void(T::*func)()>
        // template<class T>
        // void timerCheck(T* instance, void (T::*fn)());
        
        void timerCheck(std::function<void(const char*)> fn);

       // void timerCheck(T * obj);
        void forFun();
        ~GuyTimer();
};

#endif

