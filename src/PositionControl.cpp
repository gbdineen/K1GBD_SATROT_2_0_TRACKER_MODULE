#include "PositionControl.h"

PositionControl::PositionControl(/* args */)
{
}

PositionControl::PositionControl(GuyTimer * guyTmr)
{
    this->gt = guyTmr;
}

PositionControl::PositionControl(GuyTimer * guyTmr, MotorControl * mtrCtrl)
{
    this->gt = guyTmr;
    this->mc = mtrCtrl;
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

        if (systemCalibrationScore<3/* && !bno.isFullyCalibrated()*/)
        // if (systemCalibrationScore<3 && mag<2)
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
            Serial.println("SYSTEM CALIBRATED");
            calibrationActive=false;
            systemCalibrated=true;
            gt->stop();
            auto CbPtr = std::bind(&PositionControl::checkPosition, this);
            gt->guyTimer(CbPtr,1000);
        }
    }
}

void PositionControl::checkPosition()
{
    sensors_event_t event;
    bno.getEvent(&event);
    currAz = event.orientation.x;
    currEl = -event.orientation.z;
    currRoll = event.orientation.y;  

    if (controlMethod == AUTO)
    {
        if (currAz!=newAz)
        {
            auto CbPtr = std::bind(&PositionControl::trackAz, this);
            GuyTimer * azTimer = new GuyTimer(CbPtr,50);
            trackAz();
        } else {
            azTimer->stop();
            
        }

        if (currEl!=newEl)
        {
            trackEl();
        }
    }
    
    // Serial.print("\t");
    // Serial.print("currAz: ");
    // Serial.print(currAz);
    // Serial.print("\tcurrEl: ");
    // Serial.print(currEl);
    // Serial.print("\tcurrRoll: ");
    // Serial.print(currRoll);
    // Serial.print("");
    // Serial.print("\t|\t");
    // Serial.print("Systemm: "); 
    // Serial.println(bno.isFullyCalibrated());   
}

void PositionControl::trackAz()
{
    Serial.println("Now tracking Azimuth");
}

void PositionControl::trackEl()
{

}

void PositionControl::updateKeps(uint16_t az, uint16_t el)
{
    this->newAz = az;
    this->newEl = el;
}

void PositionControl::setControlMethod(uint8_t cm)
{
    this->controlMethod = cm;
}

void PositionControl::loop() {

}

/* OLD VERSION OF void PositionControl::getCalStatus(void)
void PositionControl::getCalStatus(void)x
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


