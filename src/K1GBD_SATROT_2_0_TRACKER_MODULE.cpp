#include <Arduino.h>
#include <WiFi.h>
#include <WebSocketsClient.h>
#include <Adafruit_MotorShield.h>
#include "Adafruit_PWMServoDriver.h"
#include <iostream>
#include <memory>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <functional>


// Custom Classes   
#include "WS_Client.h"
#include "MotorControl.h"
#include "PositionControl.h"
#include "GuyTimer.h"

MotorControl mtrCtrl; 
MotorControl * mtrCtrlPtr = &mtrCtrl;
GuyTimer gTmrPC;
GuyTimer * gTmrPtr = &gTmrPC;
PositionControl posCtrl(gTmrPtr,mtrCtrlPtr);//PositionControl posCtrl;
PositionControl * posCtrlPtr = &posCtrl;
WS_Client wsClient(mtrCtrlPtr, posCtrlPtr);
WS_Client * wsPtr = &wsClient;


void setup()
{
    Serial.begin(115200);
    while (!Serial) {
        ; // wait for serial port to connect. Needed for native USB
    }
    wsClient.begin();
    mtrCtrl.begin();

}

void loop()
{
    wsClient.loop();
    gTmrPC.loop();
    posCtrl.loop();
}