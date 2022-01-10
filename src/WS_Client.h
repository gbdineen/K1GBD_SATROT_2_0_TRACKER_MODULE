#ifndef WS_CLIENT_H
#define WS_CLIENT_H

#include <Arduino.h>
#include <WiFi.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>
#include <iostream>
#include <functional>
#include "MotorControl.h"
#include "PositionControl.h"
#include "GuyTimer.h"



class WS_Client
{
    private:
        
        const char* ssid = "Radiotron_Hub";
        const char* password = "YourPASS";
        WebSocketsClient wsc;
        void webSocketEvent(WStype_t type, uint8_t * payload, size_t length);
        MotorControl * mc;
        PositionControl * pc;

    public:
        
        WS_Client();
        WS_Client(MotorControl * mcPtr, PositionControl * pcPtr);
        bool begin();
        void sendTextToServer(String s);
        void confirmCalibration();      
        void loop();
        ~WS_Client();

};

#endif