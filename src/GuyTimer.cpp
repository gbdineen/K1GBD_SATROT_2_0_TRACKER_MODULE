#include "GuyTimer.h"


GuyTimer::GuyTimer(/* args */)
{
    //std::cout << "GuyTimer init" << std::endl;
}

GuyTimer::GuyTimer(std::function<void()> cb, unsigned long pt)
{
    //std::cout << "GuyTimer init" << std::endl;
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

bool GuyTimer::setMillis(unsigned long timerMillis)
{
    

    
    if (this->timerMillis==50)
    {
        // Serial.println("\n0000000000000000000000000000000000000000000");
        // Serial.print("guyTimer->timerMillis: "); Serial.print(this->timerMillis); Serial.println("\tTRUE");
        // Serial.println("0000000000000000000000000000000000000000000\n");
        return true;
    }
    else
    {
        // Serial.println("\n0000000000000000000000000000000000000000000");
        // Serial.print("guyTimer->timerMillis: "); Serial.print(this->timerMillis); Serial.println("\FALSE");
        // Serial.println("0000000000000000000000000000000000000000000\n");
        this->timerMillis = timerMillis;
        return false; 
    }
    // this->timerMillis = timerMillis;     
    // return false; 
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
