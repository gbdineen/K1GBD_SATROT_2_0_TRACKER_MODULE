#include <Arduino.h>
#include <WiFi.h>
#include <WebSocketsClient.h>
#include <Adafruit_MotorShield.h>
#include "Adafruit_PWMServoDriver.h"
#include <iostream>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <functional>


// Custom Classes   
#include "WS_Client.h"
#include "MotorControl.h"
#include "PositionControl.h"
#include "GuyTimer.h"

MotorControl motorCtrl; 
MotorControl * motorCtrlPtr = &motorCtrl;
GuyTimer gTmr;
GuyTimer * gTmrPtr = &gTmr;
//PositionControl posCtrl(gTmrPtr);
PositionControl posCtrl;
PositionControl * posCtrlPtr = &posCtrl;
WS_Client wsClient(motorCtrlPtr, posCtrlPtr);

void setup()
{
    Serial.begin(115200);
    while (!Serial) {
        ; // wait for serial port to connect. Needed for native USB
    }
    wsClient.begin();
    motorCtrl.begin();
}

void loop()
{
    wsClient.webSocketLoop();
    gTmr.loop();
}