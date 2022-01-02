#include "MotorControl.h"
// #include "WS_Client.h"


MotorControl::MotorControl()
{
    std::cout << "MotorControl init\n";
}

void MotorControl::begin()
{
    
    AFMS.begin();
    myMotor->setSpeed(255);
    myMotor->run(RELEASE);
   
    pwm.begin();
    pwm.setPWMFreq(PWM_FREQUENCY);  // Analog servos run at ~50 Hz updates
    moveServo(0,0,FULL_STOP);
    moveServo(1,0,FULL_STOP);
    //delay(50);
}

// void MotorControl::moveServo(int spd)
// {
//     Serial.print("MotorControl::moveMotor - pwm: "); Serial.println(spd);
//     uint16_t pulse_wide;
//     pulse_wide = map(spd, SERVO_LOW_RANGE, SERVO_HIGH_RANGE, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
    
//     //Serial.print("pulse_wide: "); Serial.println(pulse_wide);
//     pwm.setPWM(0, 0, pulse_wide);
// }

/*!
 *  @brief  Moves one of the two servo motors - Azimuth or Elevation
 *  @param  svo Servo number to control
 *  @param  spd Speed of the move 0 (full off) - 9 (full on)
 *  @param  dir CLOCKWISE, COUNTER_CLOCKWISE, FULL_STOP
 */
void MotorControl::moveServo(int svo, int spd, int dir)
{
   // std::cout << "Incoming speed: " << spd << "\n";
    int pw;
    switch (dir) {
        case CLOCKWISE:
        //spd=(SERVO_HIGH_RANGE/2)-spd;
        pw = calcPWM((SERVO_HIGH_RANGE/2)-spd);
        break;
       
        case COUNTER_CLOCKWISE:
        pw = calcPWM(spd+=(SERVO_HIGH_RANGE/2));
        break;

        case FULL_STOP:
        //pw = calcPWM(SERVO_HIGH_RANGE/2);
        pw = MAX_PULSE_WIDTH-(MIN_PULSE_WIDTH*2);
        break;
    }
    //Serial.print("MotorControl::moveMotor: speed: "); Serial.print(spd);
    //std::cout << "\t\t\tInverted Speed: " << spd << "  |  Direction: " << dir << "\n";
    //Serial.print("pulse_wide: "); Serial.println(pw);
    pwm.setPWM(svo, 0, pw);
}

int MotorControl::calcPWM(int spd)
{
    double pulse_wide = map(spd, SERVO_LOW_RANGE, SERVO_HIGH_RANGE, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
    return pulse_wide;
}

void MotorControl::moveDCMotor(int dir)
{
    switch (dir)
    {
        case CLOCKWISE:
        myMotor->run(FORWARD);
        break;

        case COUNTER_CLOCKWISE:
        myMotor->run(BACKWARD);
        break;

        case FULL_STOP:
        myMotor->run(RELEASE);
        break;
    }
}

