#include "WS_Client.h"



WS_Client::WS_Client()
{
    //std::cout << "WS_Client init\n";
    //Serial.print("WS_Client init");
}


WS_Client::WS_Client(MotorControl * mcPtr, PositionControl * pcPtr) 
{
    //std::cout << "WS_Client init\n";
	this->mc = mcPtr;
    this->pc = pcPtr;
	auto CbPtr = std::bind(&WS_Client::confirmCalibration, this);
	pc->setCalibrationCallback(CbPtr);

	auto TgPtr = std::bind(&WS_Client::setTargets, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
	pc->setTargetsCallback(TgPtr);

	auto PoPtr = std::bind(&WS_Client::getCurrPosition, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
	pc->setPositionCallback(PoPtr);

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
    wsc.begin("192.168.4.1", 81, "/");
    wsc.onEvent([&](WStype_t t, uint8_t * p, size_t l) {
      webSocketEvent(t, p, l);
    });
	 // And now we initialize the web socket event listener
    wsc.setReconnectInterval(3000); // If the connect fails, rety the connection every 5 seconds
    wsc.enableHeartbeat(5000,3000,100);
    return true;
}

void WS_Client::sendTextToServer(String s)
{
    //std::cout << "WS_Client::sendTextToServer";
    //Serial.print("WS_Client::sendTextToServer "); Serial.println(s);
    wsc.sendTXT(s);
}

void WS_Client::confirmCalibration()
{
	StaticJsonDocument<200> obj;
	String str;
	obj["Subject"] = "system-calibration-status";
	obj["Calibration-Status"] = 1;
	serializeJson(obj, str);
	sendTextToServer(str);
}

void WS_Client::setTargets(int az, int el, int roll)
{
	//Serial.print("\n----- [Setting Targets 2] -----]\n");
	StaticJsonDocument<200> tobj;
	String tstr;
	tobj["Subject"] = "previous-targets";
	tobj["Azimuth"] = az;
	tobj["Elevation"] = el;
	tobj["Roll"] = roll;
	serializeJson(tobj, tstr);
	sendTextToServer(tstr);
}

void WS_Client::getCurrPosition(int az, int el, int roll)
{
	//Serial.print("\n----- [Setting Targets 2] -----]\n");
	StaticJsonDocument<200> obj;
	String str;
	obj["Subject"] = "current-position";
	obj["Azimuth"] = az;
	obj["Elevation"] = el;
	obj["Roll"] = roll;
	serializeJson(obj, str);
	sendTextToServer(str);
}


void WS_Client::webSocketEvent(WStype_t type, uint8_t * payload, size_t length)
{

	//Serial.print("WS_Client::webSocketEvent: "); Serial.println(type);

	StaticJsonDocument<200> obj;
	String str;
	switch (type)
	{
		case WStype_DISCONNECTED: // enum that read status this is used for debugging.
			// Serial.print("WS Type ");
			// Serial.print(type);
			//Serial.println(": DISCONNECTED");
			wsc.sendPing();
			break;
		case WStype_CONNECTED:  // Check if a WebSocket client is connected or not
			//Serial.print("WS Type "); Serial.println(type); 
			//Serial.println(": CONNECTED");
			//Serial.print("====> Connection Step 2: Connected to WebSocket Server on IP "); Serial.println(WiFi.localIP()); // Print local IP address
			Serial.println("===> WS Init Step 3 : WS Client connected to WS Server. Sending confirmation via JSON broadcast");
			obj["Subject"] = "client-connection-confirmed";
			serializeJson(obj, str);
			sendTextToServer(str);
			//sendTextToServer("WS_Client -> CONNECTED");
			//wsc.sendTXT("wsc I'M CONNECTED"); 
			break;
		case WStype_TEXT: // check responce from client
			
			// String pay = (char*) payload;
			// Serial.println(pay);
			
			DeserializationError error = deserializeJson(obj, payload);
			String subject = obj["Subject"];
			String rollcontrol = obj["RollControl"];

			if (subject == "server-connection-confirmed")
			{
				Serial.println("=====> WS Init Step 5 : WS Server JSON broadcast recevied. Connection sequence complete. Beginning sensor calibration.");
				//pc->setWS<WS_Client>(this);
				pc->initBNO();

			}
			else if (subject == "MANUAL_SPEED")
			{
				//Serial.print("Manual control");
				pc->setControlMethod(MANUAL_SPEED);
				// if (pc->getCalibrationStatus())
				// {
					if (rollcontrol=="false")
					{
						mc->moveServo(obj["Servo"],obj["Speed"],obj["Direction"]);	
					}
					else
					{
						mc->moveDCMotor(obj["Direction"]);
					}
				//}
			}
			else if (subject == "MANUAL_POSITION")
			{
				pc->setControlMethod(MANUAL_POSITION);
				if (obj["Azimuth"] || obj["Elevation"])
				{
					pc->updateKeps(obj["Azimuth"],obj["Elevation"]);
				}
			}
			else if (subject == "udpcontrol")
			{
				String pay = (char*) payload;
				Serial.println(pay);
				pc->setControlMethod(UDP);
				// if (obj["Azimuth"] || obj["Elevation"])
				// {
					pc->updateKeps(obj["Azimuth"],obj["Elevation"]);
				// }
			}
			else if (subject=="controlmethod")
			{
				pc->setControlMethod(obj["ControlMethod"]);
			}
			else if (subject=="get-position")
			{
				pc->getCurrPosition();
			}

			break;
	}  
}

void WS_Client::loop()
{
    wsc.loop();
}

WS_Client::~WS_Client()
{
}
