#include "GuyTimer.h"


GuyTimer::GuyTimer(/* args */)
{
    std::cout << "GuyTime init" << std::endl;
}

GuyTimer::GuyTimer(std::function<void(bool)> cb, unsigned long timerMills)
{
    std::cout << "GuyTime init" << std::endl;
}


void GuyTimer::setup()
{

}

void GuyTimer::loop()
{

    std::cout << "GuyTimer loop" << std::endl;

}

/*!
 *  @brief  Set length in milliseconds of timer callback
 *  @param  timerMillis milliseconds to count between each timer callback
 */
void GuyTimer::setTimer(unsigned long timerMillis)
{
    this->timerMillis = timerMillis;
}

void GuyTimer::guyTimer()
{

}

void GuyTimer::timerCheck(std::function<void(const char*)> fn, unsigned long pt)
{
    unsigned long ct = millis();
    
    std::cout << "GuyTimer::timerCheck: prevTime " << this->prevTime << "\t|\t" << "GuyTimer::timerCheck: ct " << ct << std::endl;
    if (ct - this->prevTime >= this->millSecOne) {
    
        fn("yay we  did it!");  
    }
    this->prevTime=ct;
}

template <typename T>
void GuyTimer::timerCheck(std::function<void(T)> fn)
{
    unsigned long ct = millis();

    if (ct - this->prevTime >= this->millSecOne) {
        
    }

    //bool sw;
    //std::cout << "GuyTimer::timerCheck: prevTime " << this->prevTime << "\t|\t" << "GuyTimer::timerCheck: ct " << ct << std::endl;
    // if (ct - this->prevTime >= this->millSecOne) {
    //     sw = true; 
    // } else {
    //     sw = false;
    // }
    // this->prevTime=ct;
    // fn(sw);
}

void GuyTimer::timerCheck()
{

}

// void GuyTimer::loop() 
// {
//     std::cout << "GuyTimer::loop()" << std::endl;
//     //GuyTimer * CbPtr = timerCheck;
//     //timerCheck(CbPtr);
// }

// void GuyTimer::loop(std::function<void(bool)> fn) 
// {
//     std::cout << "GuyTimer::loop(withstuff)" << std::endl;

// }

GuyTimer::~GuyTimer()
{
}
