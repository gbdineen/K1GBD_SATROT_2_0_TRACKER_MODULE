#include "GuyTimer.h"


GuyTimer::GuyTimer(/* args */)
{
    std::cout << "GuyTime init" << std::endl;
}

GuyTimer::GuyTimer(std::function<void(bool)> cb, unsigned long timerMills)
{
    std::cout << "GuyTime init" << std::endl;
}


/*!
 *  @brief  Set length in milliseconds of timer callback
 *  @param  timerMillis milliseconds to count between each timer callback
 */
void GuyTimer::setTimer(unsigned long timerMillis)
{
    this->timerMillis = timerMillis;
}

void GuyTimer::guyTimer(std::function<void()> fn, unsigned long ms)
{
    
    this->cb = cb;
    setTimer(ms);
    timerEnabled=true;    
    // unsigned long ct = millis();
    // std::cout << "GuyTimer::timerCheck: prevTime " << this->prevTime << "\t|\t" << "GuyTimer::timerCheck: ct " << ct << std::endl;
    // if (ct - this->prevTime >= this->millSecOne) {
    
    //     //fn("yay we  did it!");  
    // }
    // this->prevTime=ct;
}

void GuyTimer::timerCheck()
{
    unsigned long ct = millis();
    std::cout << "GuyTimer::timerCheck: prevTime " << this->prevTime << "\t|\t" << "GuyTimer::timerCheck: ct " << ct << std::endl;
    if (ct - this->prevTime >= this->millSecOne) {
    
        cb(); 
    }
    this->prevTime=ct;
}

//template <typename T>
void GuyTimer::timerCheck(std::function<void()> fn, unsigned long pt)
{
    unsigned long ct = millis();
    
    std::cout << "GuyTimer::timerCheck: prevTime " << this->prevTime << "\t|\t" << "GuyTimer::timerCheck: ct " << ct << std::endl;
    if (ct - this->prevTime >= this->millSecOne) {
    
        //fn("yay we  did it!");  
    }
    this->prevTime=ct;
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
