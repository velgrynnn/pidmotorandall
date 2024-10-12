#include "Motor.CON.h"
#include "Arduino.h"

MotorDagoz :: MotorDagoz (int directionCW, int directionCCW, int pwmpin) :  _directionCW(directionCW), _directionCCW(directionCCW), _pwmpin(pwmpin)
{

}

void MotorDagoz::setpwm(float inputpwm )
{
    if ( inputpwm >= 0 )
    {
        digitalWrite(_directionCW,1);
        digitalWrite(_directionCCW,0);
        analogWrite(_pwmpin, inputpwm);
    }
    else
    {
        digitalWrite(_directionCCW,0);
        digitalWrite(_directionCW,1);
        analogWrite(_pwmpin,-1*inputpwm);
    }   
}
