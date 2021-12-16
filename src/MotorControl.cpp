#include "MotorControl.h"
// #include "WS_Client.h"


MotorControl::MotorControl()
{
    std::cout << "MotorControl init\n";
}

void MotorControl::begin()
{
    
    AFMS.begin();
    myMotor->setSpeed(190);
    myMotor->run(RELEASE);
   
    pwm.begin();
    pwm.setPWMFreq(PWM_FREQUENCY);  // Analog servos run at ~50 Hz updates
    //delay(50);


}


void MotorControl::moveServo(int spd)
{
    Serial.print("MotorControl::moveMotor - pwm: "); Serial.println(spd);
    uint16_t pulse_wide;
    pulse_wide = map(spd, SERVO_LOW_RANGE, SERVO_HIGH_RANGE, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
    
    //Serial.print("pulse_wide: "); Serial.println(pulse_wide);
    pwm.setPWM(0, 0, pulse_wide);
}

/*!
 *  @brief  Moves one of the two servo motors - Azimuth or Elevation
 *  @param  svo Servo number to control
 *  @param  spd Speed of the move 0 (full off) - 9 (full on)
 *  @param  dir CLOCKWISE, COUNTER_CLOCKWISE, FULL_STOP
 */
void MotorControl::moveServo(int svo, int spd, int dir)
{
    std::cout << "Incoming speed: " << spd << "\n";
    //spd += 7;
    switch (dir) {
        case CLOCKWISE:
        spd=9-spd;
        break;
       
        case COUNTER_CLOCKWISE:
        spd+=10;
        break;

        case FULL_STOP:
        spd=9;
        break;
    }

    //Serial.print("MotorControl::moveMotor: speed: "); Serial.print(spd);
    std::cout << "Inverted Speed: " << spd << "  |  Direction: " << dir << "\n";
    uint16_t pulse_wide = map(spd, SERVO_LOW_RANGE, SERVO_HIGH_RANGE, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
    
    Serial.print("pulse_wide: "); Serial.println(pulse_wide);
    pwm.setPWM(svo, 0, pulse_wide);
}

void MotorControl::moveDCMotor(int pos)
{

}

