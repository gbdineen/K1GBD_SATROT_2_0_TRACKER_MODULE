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
    if(!bno.begin())
    {
        /* There was a problem detecting the BNO055 ... check your connections */
        Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
        while(1);
    }
    else
    {
        Serial.println("beginBNO");
        bno.setExtCrystalUse(true);
        auto CbPtr = std::bind(&PositionControl::getCalStatus, this);
        gt->guyTimer(CbPtr,50);
    }
    
}

void PositionControl::getCalStatus()
{
    if (!systemCalibrated)
    {
        uint8_t system, gyro, accel, mag;
        system = gyro = accel = mag = 0;
        bno.getCalibration(&system, &gyro, &accel, &mag);
        systemCalibrationScore = system;

        if (systemCalibrationScore<3 && !bno.isFullyCalibrated())
        {
            Serial.print("Sys:");
            Serial.print(system);
            Serial.print(" Gx:");
            Serial.print(gyro);
            Serial.print(" A:");
            Serial.print(accel);
            Serial.print(" M:");
            Serial.print(mag);
            Serial.println("");
        }
        else 
        {
            calibrationActive=false;
            systemCalibrated=true;
            gt->stop();
            auto CbPtr = std::bind(&PositionControl::checkPosition, this);
            gt->guyTimer(CbPtr,50);
            //checkPosition();
        }
    }
    else 
    {
        
    }
}

void PositionControl::checkPosition()
{
    sensors_event_t event;
    bno.getEvent(&event);
    int posX = event.orientation.x;
    int posY = event.orientation.y;
    int posZ = -event.orientation.z;

    uint8_t system, gyro, accel, mag;
    system = gyro = accel = mag = 0;
    bno.getCalibration(&system, &gyro, &accel, &mag);
    systemCalibrationScore = system;

    Serial.print("\t");
    Serial.print("X: ");
    Serial.print(posX);
    Serial.print("\tY: ");
    Serial.print(posY);
    Serial.print("\tZ: ");
    Serial.print(posZ);
    Serial.print("");
    Serial.print("\t|\t");
    Serial.print("Systemm: "); 
    Serial.println(system);
}

/* OLD VERSION OF void PositionControl::getCalStatus(void)
void PositionControl::getCalStatus(void)
{
    unsigned long currentTime = millis();
    if (!systemCalibrated) {
        //if (currentTime - previousCaliCheckTime >= caliCheckInterval) {
        uint8_t system, gyro, accel, mag;
        system = gyro = accel = mag = 0;
        bno.getCalibration(&system, &gyro, &accel, &mag);
        systemCalibrationScore = system;
        //   gyroCalibrationScore = gyro;
        //   accelCalibrationScore = accel;
        //   magCalibrationScore = mag;

        if (systemCalibrationScore<3 && !bno.isFullyCalibrated()) {

            Serial.print("Sys:");
            Serial.print(system);
            Serial.print(" G:");
            Serial.print(gyro);
            Serial.print(" A:");
            Serial.print(accel);
            Serial.print(" M:");
            Serial.print(mag);
            Serial.println("");

            #if USE_OLED
            display.clearDisplay(); // Clear the display
            display.setCursor(0, 0); // Set the cursor position to (0,0)
            display.print("Sys:");
            display.print(system);
            display.print(" G:");
            display.print(gyro);
            display.print(" A:");
            display.print(accel);
            display.print(" M:");
            display.print(mag);
            display.println("");
            display.display();
            #endif

            //   sensorCaliObject["Subject"] = "Calibration";
            //     sensorCaliObject["Calibration"] = "active";
            //     sensorCaliObject["Sys"] = system;
            //     sensorCaliObject["G"] = gyro;
            //     sensorCaliObject["A"] = accel;
            //     sensorCaliObject["M"] = mag;
            //     serializeJson(sensorJSON, sensorString); // serialize the object and save teh result to teh string variable.
            //     webSocket.sendTXT(sensorString); // send the JSON object through the websocket
            //    // Serial.print("Still scoring : "); Serial.println(sensorString);  ! USE
            //     sensorString = ""; // clear the String.


        } else {
            // sensorString = ""; // clear the String.
            // //Serial.println("! CALIBTRATION COMPLETE");
            // #if USE_OLED
            // display.clearDisplay(); // Clear the display
            // display.setCursor(0, 0); // Set the cursor position to (0,0)
            // display.print("Calibration complete");
            // display.display();
            // #endif

            // sensorCaliObject["Subject"] = "Calibration";
            // sensorCaliObject["Calibration"] = "complete";
            // serializeJson(sensorJSON, sensorString); // serialize the object and save teh result to teh string variable.
            // webSocket.sendTXT(sensorString); // send the JSON object through the websocket  ! USE
            // //Serial.print("Scoring complete : "); Serial.println(sensorString);
            // sensorString = ""; // clear the String.
            // //delay(10000);
            // calibrationActive=false;
            // systemCalibrated=true;

        }
    //previousCaliCheckTime=currentTime;
    //}
    } else {
        display.clearDisplay(); // Clear the display
        display.setCursor(0, 0); // Set the cursor position to (0,0)
        display.print("Calibration complete");
        display.display();

        sensorCaliObject["Subject"] = "Calibration";
        sensorCaliObject["Calibration"] = "complete";
        serializeJson(sensorJSON, jsonString); // serialize the object and save teh result to teh string variable.
        webSocket.sendTXT(jsonString); // send the JSON object through the websocket
        Serial.println(jsonString);
        jsonString = ""; // clear the String.
        //delay(5000);
    }
    }
}
*/

PositionControl::~PositionControl()
{
}


