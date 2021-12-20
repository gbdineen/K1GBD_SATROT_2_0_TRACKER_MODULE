#include "PositionControl.h"

PositionControl::PositionControl(/* args */)
{
}

PositionControl::PositionControl(GuyTimer * guyTmr)
{
    this->gt = guyTmr;
}


void PositionControl::initBNO()
{ 
    // if(!bno.begin()) {
    //     /* There was a problem detecting the BNO055 ... check your connections */
    //     Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    //     while(1);
    // } else {
    //     Serial.println("beginBNO");
    //     bno.setExtCrystalUse(true);
        
    // }
  sendCalStatus();
}

void PositionControl::myCalStatus(const char* s)
{
      std::cout << "PositionControl::myCalStatus: " << s << std::endl;
}

bool PositionControl::sendCalStatus()
{

    //CbPtr mydef = &PositionControl::myCalStatus;
    //CbPtr = &PositionControl::myCalStatus;
    //using namespace std::placeholders;
    auto CbPtr = std::bind(&PositionControl::myCalStatus, this, std::placeholders::_1);


    //gt->template timerCheck<CbPtr>(mydef);
    gt->timerCheck(CbPtr);
    //gt->forFun();
    return 0; 

}

// template <typename F> F PositionControl::sendCalStatus()
// {

//     CbPtr mydef = &PositionControl::myCalStatus;

//     gt->timerCheck(mydef);
//     //gt->forFun();
//     return 0; 

// }

PositionControl::~PositionControl()
{
}


