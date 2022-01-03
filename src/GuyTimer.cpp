#include "GuyTimer.h"


GuyTimer::GuyTimer(/* args */)
{
    std::cout << "GuyTimer init" << std::endl;
}

GuyTimer::GuyTimer(std::function<void()> cb, unsigned long pt)
{
    std::cout << "GuyTimer init" << std::endl;
}

// bool GuyTimer::guyTimer(std::function<void()> cb)
// {
//     this->cb = cb;
    
//     return true;
// }


bool GuyTimer::guyTimer(std::function<void()> cb, bool start)
{
    this->cb = cb;
    timerEnabled=start; 

    return true;
}

bool GuyTimer::guyTimer(std::function<void()> cb, unsigned long ms, bool start)
{
    this->cb = cb;
    setMillis(ms);
    timerEnabled=start; 

    return true;
}

void GuyTimer::setMillis(unsigned long timerMillis)
{
    this->timerMillis = timerMillis;
}

void GuyTimer::setCallback(std::function<void()> cb) 
{
    
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

void GuyTimer::start(unsigned long ms)
{
    timerMillis=ms;
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
