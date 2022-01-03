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
    auto tazPtr = std::bind(&PositionControl::trackAz, this);
    azTimer.guyTimer(tazPtr,false);
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

        //systemCalibrationScore=2;

        auto CbPtr = std::bind(&PositionControl::getCalStatus, this);
        gt->guyTimer(CbPtr);



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

        if (systemCalibrationScore<1/* && !bno.isFullyCalibrated()*/)
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
            // if (!systemCalibrated)
            // {
            Serial.println("SYSTEM CALIBRATED");

            setControlMethod(AUTO); // temp for testing. should happen explicitly from user input

            calibrationActive=false;
            systemCalibrated=true;

            gt->stop();
            mc->moveDCMotor(FULL_STOP);
            mc->moveServo(0,0,FULL_STOP);
            mc->moveServo(1,0,FULL_STOP);
            targAz = 10;
            delay(2000);  // give the antenna a moment to settle
            
            auto CbPtr = std::bind(&PositionControl::checkPosition, this);
            gt->guyTimer(CbPtr,1000,true);
                /* code */
            //}
            
        }
    }
}

void PositionControl::checkPosition()
{
    
    // Serial.print("currAz: "); Serial.print(currAz); Serial.print("\t|\ttargAz: "); Serial.print(targAz);
    //     Serial.print("\t|\tprevAz: "); Serial.println(prevAz); 
    
    //Serial.print("controlMethod: "); Serial.println(controlMethod);
    sensors_event_t event;
    bno.getEvent(&event);
    currAz = event.orientation.x;
    currEl = -event.orientation.z;
    currRoll = event.orientation.y;

    Serial.print("currAz:");
    Serial.print(currAz);
    Serial.print("\t|\tcurrEl:");
    Serial.print(currEl);
    Serial.print("\t|\tcurrRoll:");
    Serial.println(currRoll);

    if (controlMethod == AUTO)
     {
        if (targAz!=prevAz  && !trackingAz)
        {
            trackAz();
            gt->setMillis(50);
        }
    }
}


void PositionControl::trackAz()
{
    Serial.println("Now tracking Azimuth");
    if (!trackingAz)
    {
        uint8_t dir = direction(targAz,prevAz);
        trackingAz=true;
        azTimer.start();
        mc->moveServo(0,8,dir);
    }

    //if (currAz>(targAz-1) && currAz<(targAz+1))
    if (currAz == targAz)
    {
        //gt->setMillis(1000);
        mc->moveServo(0,0,FULL_STOP);
        //gt->stop();
        //delay(3000);
        gt->start(1000);
        trackingAz=false;
        azTimer.stop();
     }
    prevAz=currAz;
}

void PositionControl::trackEl()
{

}

void PositionControl::updateKeps(uint16_t az, uint16_t el)
{
    this->targAz = az;
    this->targEl = el;
   // Serial.print("updating keps");
}

void PositionControl::setControlMethod(uint8_t cm)
{
    this->controlMethod = cm;
    //Serial.print("Control Method set to "); Serial.println(controlMethod);
}

uint8_t PositionControl::direction(uint16_t targ, uint16_t prev)
{
    if (targ>prev)
    {
        return 0; // CLOCKWISE
    }
    else if (targ<prev)
    {  
        return 1; // COUNTER_CLOCKWISE
    }
    else
    {
        return 2; // FULL_STOP
    }
}

void PositionControl::loop() {
    azTimer.loop();
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


