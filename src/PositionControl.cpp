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
    //this->ws = ws;
    
    auto tazPtr = std::bind(&PositionControl::trackAz, this);
    azTimer.guyTimer(tazPtr,false);

    auto telPtr = std::bind(&PositionControl::trackEl, this);
    elTimer.guyTimer(telPtr,false);

    auto rollPtr = std::bind(&PositionControl::trackRoll, this);
    rollTimer.guyTimer(rollPtr,false);

}


void PositionControl::initBNO()
{ 
    if(!bno.begin())
    {
        /* There was a problem detecting the BNO055 ... check your connections */
        Serial.println("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
        while(1);
    }
    else
    {
        Serial.println("BNO sensor object initialized.");
        bno.setExtCrystalUse(true);
        
        /*
        if (checkCalibrationEEPROM())
        {
            systemCalibrated=true;
            //  --- CODE FOR MOVING ANTENNA INTO READY POSITION --- 
        }
        else
        {
            auto CbPtr = std::bind(&PositionControl::calibrateSystem, this);
            gt->guyTimer(CbPtr);
        }
        */

        auto CbPtr = std::bind(&PositionControl::calibrateSystem, this);
        gt->guyTimer(CbPtr);

    }
}

bool PositionControl::checkCalibrationEEPROM() 
{

    sensor_t mySensor;
    long bnoID = mySensor.sensor_id;  // Get the unique ID of the BNO sensor
    int adr = 400;  // 400 is where the ID will have been saved if it has indeed been saved for this sensor previously

    if (EEPROM.get(adr,bnoID)==bnoID)
    {
        return true;
    } else {
        return false;
    }

}

void PositionControl::autoCalibration()
{


}

uint8_t PositionControl::calibrateSystem()
{
    
    if (!systemCalibrated) {
    
        uint8_t system, gyro, accel, mag;
        system = gyro = accel = mag = 0;
        bno.getCalibration(&system, &gyro, &accel, &mag);
        
        systemCalibrationScore = system;

        /* Display system callibration scores while we move the antenna this way and that
        to reach a score of 3 for all but mag which we will accept as 2 
        */
        //if (system<3 || gyro<3 || accel<3 || mag<2)
        if (accel<3 || mag<1)
        {
            Serial.print("Sys:");
            Serial.print(system);
            Serial.print(" G:");
            Serial.print(gyro);
            Serial.print(" A:");
            Serial.print(accel);
            Serial.print(" M:");
            Serial.print(mag);
            Serial.println("");
        }
        else 
        {
            
            Serial.println("=========> SYSTEM CALIBRATED");
            Serial.print("Sys:");
            Serial.print(system);
            Serial.print(" Gx:");
            Serial.print(gyro);
            Serial.print(" A:");
            Serial.print(accel);
            Serial.print(" M:");
            Serial.print(mag);
            Serial.println("");
            
            this->calibrationCallback(); // Send notification to WS_Client that calibration has been completed and we're ready to start taking UDP packets

            gt->stop();  // Stop the timer from checking for any further calibration
            
            /*
            adafruit_bno055_offsets_t offsets;
            putEEPROM(400,offsets);  // Store the calibration data we just got into EEPROM
            setSensorOffsets(offsets); // Set the calibration data into the BNO sensor to have and to hold until a power cut dues us part
            */

            //setControlMethod(AUTO); // temp for testing. should happen explicitly from user input

            calibrationActive=false;
            systemCalibrated=true;

            mc->moveDCMotor(FULL_STOP);
            mc->moveServo(0,0,FULL_STOP);
            mc->moveServo(1,0,FULL_STOP);
            delay(2000);  // give the antenna a moment to settle

            
            
            
            /* 
            Pass checkPosition as the new callback function to guyTimer
            and start polling for position via sensor @ 1 second (1000 miscroseconds)increments
            */
            if (controlMethod!=UDP)
            {
                parkAntenna();
            }

            auto CbPtr = std::bind(&PositionControl::checkPosition, this);
            gt->guyTimer(CbPtr,bnoInterval,true);

            // elTimer.start(bnoInterval);
            // azTimer.start(bnoInterval);

            

        }
    }
    //return systemCalibrationScore;
}


void PositionControl::checkPosition()
{
    
    // Serial.print("currAz: "); Serial.print(currAz); Serial.print("\t|\ttargAz: "); Serial.print(targAz);
    //     Serial.print("\t|\tprevAz: "); Serial.println(prevAz); 

    // Serial.println("+++++++++++++++ [Checking Position] +++++++++++++++");
    // Serial.print("currEl: "); Serial.print(currEl); Serial.print("\t|\ttargEl: "); Serial.print(targEl); Serial.print("\t|\tprevEl: "); Serial.println(prevEl); 
    // Serial.println("---------------------------------------------------");
    // Serial.print("currAz: "); Serial.print(currAz); Serial.print("\t|\ttargAz: "); Serial.print(targAz); Serial.print("\t|\tprevAz: "); Serial.println(prevAz);
    // Serial.println("---------------------------------------------------");
    // Serial.print("currRoll: "); Serial.print(currRoll); Serial.print("\t|\ttargRoll: "); Serial.print(targRoll); Serial.print("\t|\tprevRoll: "); Serial.println(prevRoll); 
    // Serial.println(" ");
    
    //Serial.print("controlMethod: "); Serial.println(controlMethod);
    sensors_event_t event;
    bno.getEvent(&event);
    currAz = event.orientation.x;
    currEl = -event.orientation.z;
    currRoll = event.orientation.y;

    uint8_t system, gyro, accel, mag;
    system = gyro = accel = mag = 0;
    bno.getCalibration(&system, &gyro, &accel, &mag);

    Serial.println("+++++++++++++++ [Checking Calibration] +++++++++++++++"); 
     Serial.print("Sys:"); Serial.print(system); Serial.print(" G:"); Serial.print(gyro);
        Serial.print(" A:"); Serial.print(accel); Serial.print(" M:");  Serial.print(mag); Serial.println("");

    //Serial.print

    // Serial.print("currAz:");
    // Serial.print(currAz);
    // Serial.print("\t|\tcurrEl:");
    // Serial.print(currEl);
    // Serial.print("\t|\tcurrRoll:");
    // Serial.println(currRoll);

    //if (controlMethod == AUTO)
     ///{

        /*
        if this is the first time through the system, the previous positions
        haven't yet been set, so set them = to the curr position, which is what will
        happen automatically every other time */
        if (!prevPosSet && controlMethod!=UDP) 
        {
            prevPosSet=true;
            prevAz=currAz;
            prevEl=currEl;
            prevRoll=currRoll;
            //setTargets();
        }

        if (!initialServoStart) // starting the servo timers at this juncture gives the sensor a moment to collect its first set up data
        {
            initialServoStart=true;
            elTimer.start(bnoInterval);
            azTimer.start(bnoInterval);
        }



        
        if (targRoll!=prevRoll  && !trackingRoll)
        {
            trackRoll();
            gt->setMillis(bnoInterval);
           
        }
}

std::vector<int> PositionControl::getCurrPosition()
{
    sensors_event_t event;
    bno.getEvent(&event);
    int currAz = event.orientation.x;
    int currEl = -event.orientation.z;
    int currRoll = event.orientation.y;
    
    std::vector<int> v = {prevAz,prevEl,prevRoll};

    positionCallback(currAz,currEl,currRoll);

    return v;
}

void PositionControl::trackEl()
{
    if (systemCalibrated)
    {
        //Serial.println("========= Tracking Elevation ===========================");
        if (targEl!=prevEl)
        {
            uint8_t dir = servoDirection(targEl,prevEl,ELEVATION_SERVO);
            trackingEl=true;
            if (!gt->setMillis(bnoInterval))
            {
                gt->setMillis(bnoInterval);
            }
            elTimer.start(bnoInterval);
            mc->moveServo(EL_SERVO,8,dir);
            Serial.print("El direction: "); Serial.println(dir);
            
            if (currEl == targEl)
            {
                elTimer.start(1000);
                trackingEl=false;
                mc->moveServo(EL_SERVO,0,FULL_STOP);
                
                antennaStationaryCheck();
                
            }
            prevEl=currEl;
            Serial.print("currEl:");
            Serial.print(currEl);
            Serial.print("\t|\ttargEl:");
            Serial.print(targEl);
            Serial.print("\t|\tprevEl:");
            Serial.println(prevEl);
        }
    }
}

void PositionControl::trackAz()
{
    if (systemCalibrated)
    {
        Serial.println("========= Tracking Azimuth ===========================");
        if (targAz!=prevAz)
        {
            if (!trackingAz)
            {
                // if (targAz>prevAz+3 || targAz<prevAz-3)
                // {
                    uint8_t dir = servoDirection(targAz,prevAz,AZIMUTH_SERVO);
                    trackingAz=true;
                    // bool gtt = gt->setMillis(bnoInterval);
                    // Serial.print("---------------------Guy timer? ---------------> "); Serial.println(gtt);
                    if (!gt->setMillis(bnoInterval))
                    {
                        gt->setMillis(bnoInterval);
                        tempTimerCount++;
                        Serial.print("---------------------------------------------------tempTimerCount: "); Serial.println(tempTimerCount);
                    }
                    azTimer.start();
                    mc->moveServo(AZ_SERVO,9,dir);
                    Serial.print("Az Direction: "); Serial.println(dir);
                }

                if (currAz == targAz)
                {
                    
                    azTimer.start(1000);
                    trackingAz=false;
                    mc->moveServo(AZ_SERVO,0,FULL_STOP);
                    
                    antennaStationaryCheck();
                    
                }
                prevAz=currAz;
                Serial.print("currAz:");
                Serial.print(currAz);
                Serial.print("\t|\ttargAz:");
                Serial.print(targAz);
                Serial.print("\t|\tprevAz:");
                Serial.println(prevAz);         
           // }
        }
    }
}

void PositionControl::trackRoll()
{
    if (systemCalibrated)
    {    Serial.println("========= Tracking Roll ===========================");
        if (!trackingRoll)
        {
            uint8_t dir = servoDirection(targRoll,prevRoll, ROLL_MOTOR);
            trackingRoll=true;
            rollTimer.start();
            mc->moveDCMotor(dir);
            Serial.print("Roll direction: "); Serial.println(dir);
        } 
        Serial.print("currRoll:");
        Serial.print(currRoll);
        Serial.print("\t|\ttargRoll:");
        Serial.print(targRoll);
        Serial.print("\t|\tprevRoll:");
        Serial.println(prevRoll);
        if (currRoll == targRoll)
        {
            
            //gt->stop();
            rollTimer.stop();
            trackingRoll=false;
            mc->moveDCMotor(FULL_STOP);
            
            antennaStationaryCheck();
        }
        prevRoll=currRoll;
    }
}

void PositionControl::parkAntenna(int azPos, int elPos)
{
    
    Serial.println("********************************************************> PARKING ANTENNA");
    
    targAz=350;
    targEl=5;
    targRoll=45;
    
    // int dirAz = servoDirection(355,cAz);
    // int dirEl = servoDirection(0,cEl);
    // int dirRoll = servoDirection(0,cRoll);
    
    // mc->moveServo(0,9,dirAz);
    // mc->moveServo(1,7,dirEl);
    // mc->moveDCMotor(dirRoll);
    
}

void PositionControl::antennaStationaryCheck()
{
    Serial.println("--------------------------------------------------> antennaStationaryCheck");
    if (!trackingAz && !trackingEl && !trackingRoll)
    {
        setTargets();

        if (controlMethod!=UDP)
        {
           gt->start(1000);
            //return true;
        }
        else
        {
            gt->start(50);
        }
    }
}

void PositionControl::updateKeps(int az, int el)
{
    this->targAz = az;
    this->targEl = el;
   // Serial.print("updating keps");
}

void PositionControl::getCurrentTargets(){

}

void PositionControl::setTargets()
{

    //Serial.print("\n----- [Setting Targets 1] -----]\n");
    targetsCallback(prevAz,prevEl,prevRoll);
    getCurrPosition();

}

void PositionControl::setControlMethod(uint8_t cm)
{
    this->controlMethod = cm;
    //Serial.print("Control Method set to "); Serial.println(controlMethod);
}

uint8_t PositionControl::servoDirection(int targ, int prev, int servo)
{
    
    if (servo==AZIMUTH_SERVO)
    {
        int absDiff = abs(targ-prev); // determine the absolute value difference between target and previous to determine the shortest route
        //Serial.print("\n\nabsDiff: "); Serial.print(absDiff);  Serial.println("\n\n");  
        if (targ<prev)
        {
            if (absDiff<=180)
            {
                return 1;
            }
            else if (absDiff>180)
            {
                return 0;
            }
        }
        else if (targ>prev)
        {
            if (absDiff<=180)
            {
                return 0;
            }
            else if (absDiff>180)
            {
                return 1;
            }
        }
        else
        {
            return 2;
        }
    }
    else if (servo==ELEVATION_SERVO)
    {
        if (targ<prev)
        {
            return 0;
        }
        else if (targ>prev)
        {
            return 1;
        }
        else
        {
            return 2;
        }

    }
    else if (servo==ROLL_MOTOR)
    {
        if (targ<prev)
        {
            return 1;
        }
        else if (targ>prev)
        {
            return 0;
        }
        else
        {
            return 2;
        }
    }
    
    
    
    // // if (!prev)
    // // {
    // //     prev=currAz;
    // // }
    // if (targ>prev)
    // {
        
    //     if (el)
    //     {
    //         return 1; // COUNTER_CLOCKWISE
    //     }
    //     else
    //     {
    //         return 0; // CLOCKWISE
    //     }
    //  }
    // else if (targ<prev)
    // {  
        
    //     if (el)
    //     {
    //         return 0; // CLOCKWISE
    //     }
    //     else
    //     {
    //         return 1; // COUNTER_CLOCKWISE
    //     }
    // }
    // else
    // {
    //     return 2; // FULL_STOP
    // }
}

uint8_t PositionControl::motorDirection(uint16_t targ, uint16_t prev)
{

    if (prev>0)
    {
        return 0; // CLOCKWISE
    }
    else if (prev<0)
    {
        return 1; // COUNTER_CLOCKWISE
    }
    else
    {
        return 2; // FULL_STOP
    }

}

void PositionControl::setCalibrationCallback(std::function<void()> cb)
{
    this->calibrationCallback=cb;
}

void PositionControl::setTargetsCallback(std::function<void(int az, int el, int roll)> cb)

{
    this->targetsCallback=cb;
}

void PositionControl::setPositionCallback(std::function<void(int az, int el, int roll)> cb)

{
    this->positionCallback=cb;
}

bool PositionControl::getCalibrationStatus()
{
    uint8_t system, gyro, accel, mag;
    system = gyro = accel = mag = 0;
    bno.getCalibration(&system, &gyro, &accel, &mag);
    
    systemCalibrationScore = system;

    return systemCalibrated;
    
}


template <typename T>
void PositionControl::putEEPROM(int adr, T val)
{
    
    Serial.print("Saving values to EEPROM at address "); Serial.println(adr);
    EEPROM.put(adr, val);

}

template <typename T>
adafruit_bno055_offsets_t PositionControl::getEEPROM(int adr, T tp)
{
    
    adafruit_bno055_offsets_t ofs = EEPROM.get(adr,tp);

    Serial.print("Accelerometer: ");
    Serial.print(ofs.accel_offset_x); Serial.print(" ");
    Serial.print(ofs.accel_offset_y); Serial.print(" ");
    Serial.print(ofs.accel_offset_z); Serial.print(" ");

    Serial.print("\nGyro: ");
    Serial.print(ofs.gyro_offset_x); Serial.print(" ");
    Serial.print(ofs.gyro_offset_y); Serial.print(" ");
    Serial.print(ofs.gyro_offset_z); Serial.print(" ");

    Serial.print("\nMag: ");
    Serial.print(ofs.mag_offset_x); Serial.print(" ");
    Serial.print(ofs.mag_offset_y); Serial.print(" ");
    Serial.print(ofs.mag_offset_z); Serial.print(" ");

    Serial.print("\nAccel Radius: ");
    Serial.print(ofs.accel_radius);

    Serial.print("\nMag Radius: ");
    Serial.println(ofs.mag_radius);

    return ofs;

}

adafruit_bno055_offsets_t PositionControl::getSensorOffsets(adafruit_bno055_offsets_t &calibData)
{

    Serial.print("Accelerometer: ");
    Serial.print(calibData.accel_offset_x); Serial.print(" ");
    Serial.print(calibData.accel_offset_y); Serial.print(" ");
    Serial.print(calibData.accel_offset_z); Serial.print(" ");

    Serial.print("\nGyro: ");
    Serial.print(calibData.gyro_offset_x); Serial.print(" ");
    Serial.print(calibData.gyro_offset_y); Serial.print(" ");
    Serial.print(calibData.gyro_offset_z); Serial.print(" ");

    Serial.print("\nMag: ");
    Serial.print(calibData.mag_offset_x); Serial.print(" ");
    Serial.print(calibData.mag_offset_y); Serial.print(" ");
    Serial.print(calibData.mag_offset_z); Serial.print(" ");

    Serial.print("\nAccel Radius: ");
    Serial.print(calibData.accel_radius);

    Serial.print("\nMag Radius: ");
    Serial.println(calibData.mag_radius);

    return calibData;


}

void PositionControl::setSensorOffsets(adafruit_bno055_offsets_t &calibData)
{
    bno.setSensorOffsets(calibData);
    Serial.println("Calibration offsets saved (until power down) to BNO");
}


void PositionControl::loop() {
    azTimer.loop();
    elTimer.loop();
    rollTimer.loop();
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


