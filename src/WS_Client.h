#ifndef WS_CLIENT_H
#define WS_CLIENT_H

#include <Arduino.h>
#include <WiFi.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>
#include <iostream>
#include "MotorControl.h"
#include "PositionControl.h"


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
        ~WS_Client();
        bool begin();
        void loop();
        void sendTextToServer(String s);

};

#endif