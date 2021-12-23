#include "GuyTimer.h"


GuyTimer::GuyTimer(/* args */)
{
}

void GuyTimer::guyTimer(unsigned long timerMills) 
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

void GuyTimer::timerCheck(std::function<void(bool)> fn)
{
    unsigned long ct = millis();
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

void GuyTimer::loop() 
{
    std::cout << "GuyTimer::loop()" << std::endl;
}

void GuyTimer::loop(std::function<void(bool)> fn) 
{
    std::cout << "GuyTimer::loop(withstuff)" << std::endl;
    
}

GuyTimer::~GuyTimer()
{
}
