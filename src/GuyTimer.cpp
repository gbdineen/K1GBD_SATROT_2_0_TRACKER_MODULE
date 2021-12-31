#include "GuyTimer.h"


GuyTimer::GuyTimer(/* args */)
{
}

GuyTimer::GuyTimer(std::function<void(bool)> cb, unsigned long timerMills)
{
    std::cout << "GuyTime init" << std::endl;
}

void GuyTimer::guyTimer(std::function<void()> fn, unsigned long ms)
{
    this->cb = fn;
    setTimer(ms);
    timerEnabled=true; 
}
void GuyTimer::setTimer(unsigned long timerMillis)
{
    this->timerMillis = timerMillis;
}

void GuyTimer::timerCheck()
{
    unsigned long ct = millis();
    if (ct - this->prevTime >= this->timerMillis) {
        //std::cout << "GuyTimer::timerCheck: prevTime " << this->prevTime << "\t|\t" << "GuyTimer::timerCheck: ct " << ct << std::endl;
        cb();
        this->prevTime=ct;
    }
}

void GuyTimer::start()
{
    if (!timerEnabled)
    {
        timerEnabled=true;
    }
}

void GuyTimer::stop()
{
    if (timerEnabled)
    {
        timerEnabled=false;
    }
}

void GuyTimer::loop()
{
    if (timerEnabled)
    {
        timerCheck();
    }
}

GuyTimer::~GuyTimer()
{
}
