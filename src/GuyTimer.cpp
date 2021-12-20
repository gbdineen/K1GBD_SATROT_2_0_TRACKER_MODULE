#include "GuyTimer.h"


GuyTimer::GuyTimer(/* args */)
{
}


void GuyTimer::timerCheck(std::function<void(const char*)> fn)
{
    fn("yay we  did it!");
    //
}
// template <class T>void GuyTimer::timerCheck(T* instance, void (T::*fn)(void))
// {
//     instance.fn();
//     //return obj;
//     //obj();

// }

void GuyTimer::forFun() 
{

}

GuyTimer::~GuyTimer()
{
}
