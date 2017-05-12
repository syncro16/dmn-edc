// Generic PID control

#ifndef PID_H
#define PID_H

// #include "Arduino.h"

class PID {
private:
    int *Kp; // p factor
    int *Ki; // i factor
    int *Kd; // d factor
    int error,errorOld;
    float p,i,d,integral,derivate;
    int *minOutput; // Saturate output
    int *maxOutput; // Saturate output
    int *speed;
    int setPoint;
    int targetSetPoint;
    int *input;
    int *output;
    int *bias;

public:

public:
    PID(int *p,int *i,int *d,int *minOutput,int *maxOutput,int *speed,int *input,int *output);    
    PID(int *p,int *i,int *d,int *minOutput,int *maxOutput,int *speed,int *bias,int *input,int *output);
    void setPosition(int s);
    void calculate();
};

#endif

