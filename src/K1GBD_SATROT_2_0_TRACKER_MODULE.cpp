#include <Arduino.h>
#include <WiFi.h>
#include <WebSocketsClient.h>
#include <Adafruit_MotorShield.h>
#include "Adafruit_PWMServoDriver.h"
#include <iostream>

// Custom Classes   
#include "WS_Client.h"
#include "MotorControl.h"

MotorControl motorCtrl; 
MotorControl * motorCtrlPtr = &motorCtrl;
WS_Client wsClient(motorCtrlPtr);

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
}