#include "MotorControl.h"
// #include "WS_Client.h"

MotorControl::MotorControl()
{
    std::cout << "MotorControl init\n";
}

// MotorControl::MotorControl(WS_Client * wsClient)
// {
//     //std::cout << "MotorControl init";
//     this->wscMc = wsClient;
   
// }

void MotorControl::initMotors()
{
    
    //Serial.println("MotorControl::initMotors");
    
    AFMS.begin();
    myMotor->setSpeed(190);
    myMotor->run(RELEASE);
   
    pwm.begin();
    //pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
    delay(50);

    //wscMc->sendTextToServer("Motor Init");
}

void MotorControl::moveMotor(int pos)
{
    Serial.print("MotorControl::moveMotor - pwm: "); Serial.println(pos);
    double lowRange = 0;
    int highRange = 20;
    int MIN_PULSE_WIDTH = 212;
    int MAX_PULSE_WIDTH = 416;
    float FREQUENCY = 50;

    double pulse_wide, pulse_width;

    pulse_wide = map(pos, lowRange, highRange, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
    pulse_width = int(float(pulse_wide) / 1000000 * FREQUENCY * 4096);

    pwm.setPWMFreq(FREQUENCY);

    Serial.print("pulse_wide: "); Serial.println(pulse_wide);
    pwm.setPWM(0, 0, pulse_wide);
    //pwm.setPWM(0, 0, pos);
    
}




