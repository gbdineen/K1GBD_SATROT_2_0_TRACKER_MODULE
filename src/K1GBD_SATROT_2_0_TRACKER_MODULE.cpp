#include <Arduino.h>
#include <WiFi.h>
#include <WebSocketsClient.h>
#include <Adafruit_MotorShield.h>
#include "Adafruit_PWMServoDriver.h"
#include <iostream>

// Custom Classes   
#include "WS_Client.h"
#include "MotorControl.h"

//WS_Client wsClient;
MotorControl motorCtrl; 
MotorControl * motorCtrlPtr = &motorCtrl;
WS_Client wsClient(motorCtrlPtr);

//WS_Client * wsClientPtr = &wsClient;



void setup()
{
    Serial.begin(115200);
    while (!Serial) {
        ; // wait for serial port to connect. Needed for native USB
    }
    wsClient.initWebSocketClient();
    // wsClient.setMotorControl(motorCtrlPtr);
    motorCtrl.initMotors();
}

void loop()
{
    wsClient.webSocketLoop();
}