#ifndef WS_CLIENT_H
#define WS_CLIENT_H

#include <Arduino.h>
#include <WiFi.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>
#include <iostream>
#include "MotorControl.h"

class WS_Client
{
    private:
        
        const char* ssid = "Radiotron_Hub";
        const char* password = "YourPASS";
        WebSocketsClient wsc;
        void webSocketEvent(WStype_t type, uint8_t * payload, size_t length);
        MotorControl * mc;

    public:
        
        WS_Client();
        WS_Client(MotorControl * mcPtr);
        ~WS_Client();
        void initWebSocketClient();
        void webSocketLoop();
        void sendTextToServer(String s);
        //void setMotorControl(MotorControl * mcObj);
};

#endif