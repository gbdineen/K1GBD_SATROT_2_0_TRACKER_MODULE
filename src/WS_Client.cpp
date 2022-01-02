#include "WS_Client.h"



WS_Client::WS_Client()
{
    std::cout << "WS_Client init\n";
    //Serial.print("WS_Client init");
}


WS_Client::WS_Client(MotorControl * mcPtr, PositionControl * pcPtr) 
{
    std::cout << "WS_Client init\n";
		this->mc = mcPtr;
    	this->pc = pcPtr;
}

void WS_Client::loop()
{
    wsc.loop();
}

void WS_Client::sendTextToServer(String s)
{
    //std::cout << "WS_Client::sendTextToServer";
    //Serial.print("WS_Client::sendTextToServer "); Serial.println(s);
    wsc.sendTXT(s);
}


bool WS_Client::begin()
{
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        //Serial.print(".");
        delay(500);
    }
    //Serial.println();
    //Serial.print("IP Address: ");
    Serial.println(WiFi.localIP()); // Print local IP address
    wsc.begin("192.168.4.1", 81, "/");
    wsc.onEvent([&](WStype_t t, uint8_t * p, size_t l) {
      webSocketEvent(t, p, l);
    });
	// wsc.onEvent(webSocketEvent) {
    // });
    // And now we initialize the web socket event listener
    wsc.setReconnectInterval(3000); // If the connect fails, rety the connection every 5 seconds
    wsc.enableHeartbeat(5000,3000,100);
    ///sendTextToServer("init I'M CONNECTED"); 
    return true;
}

void WS_Client::webSocketEvent(WStype_t type, uint8_t * payload, size_t length)
{

 //Serial.print("WS_Client::webSocketEvent: "); Serial.println(type);

 switch (type) {
	case WStype_DISCONNECTED: // enum that read status this is used for debugging.
		// Serial.print("WS Type ");
		// Serial.print(type);
		//Serial.println(": DISCONNECTED");
		wsc.sendPing();
		break;
    case WStype_CONNECTED:  // Check if a WebSocket client is connected or not
		//Serial.print("WS Type "); Serial.println(type); 
		//Serial.println(": CONNECTED");
		sendTextToServer("WS_Client -> CONNECTED");
		//wsc.sendTXT("wsc I'M CONNECTED"); 
		break;
    case WStype_TEXT: // check responce from client
		// Serial.print("WS Type "); Serial.println(type);
		String pay = String((char*) payload);
		//Serial.println(pay);
		StaticJsonDocument<200> obj;
		DeserializationError error = deserializeJson(obj, payload);
		String subject = obj["Subject"];
		String rollcontrol = obj["RollControl"];
      
		if (subject == "clientconnected")
		{
			pc->initBNO();
		}
		else if (subject == "manualcontrol")
		{
			pc->setControlMethod(MANUAL);
			if (rollcontrol=="false")
			{
				mc->moveServo(obj["Servo"],obj["Position"],obj["Direction"]);	
			}
			else
			{
				mc->moveDCMotor(obj["Direction"]);
			}
		}
		else if (subject == "autocontrol")
		{
			pc->setControlMethod(AUTO);
			if (obj["Azimuth"] || obj["Elevation"])
			{
				pc->updateKeps(obj["Azimuth"],obj["Elevation"]);
			}
		}
		
		break;
  }  
}

WS_Client::~WS_Client()
{
}
