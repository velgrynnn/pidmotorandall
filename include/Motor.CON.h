#ifndef MOTOR.CON_H
#define MOTOR.CON_H

class MotorDagoz
{
public:
    MotorDagoz(int directionCW, int directionCCW, int pwmpin);

    void setpwm(float inputpwm);

private:
    int _pwmpin;
    
    int _directionCW;
    int _directionCCW;

};
#endif