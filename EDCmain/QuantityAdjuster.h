// VP37 QuantityAdjuster "servo" control.

#include "Core.h"

class QuantityAdjuster {
private:
    static const char HIT_TRIGGER = 1;
    
    unsigned char pinIn;
    unsigned char pinOut;
    unsigned int smoothOutput;
    unsigned long ticks;
    unsigned char runningStage;
    float oldOutput;
    float error;
    float integral;
    float errorOld;
    float derivate;
    volatile char statusBits;
    volatile int targetSetPoint; 
    volatile int setPoint;
    unsigned int speed;


public:
    float Kp,p;
    float Ki,i;
    float Kd,d;

    int currentDutyCycle;
    int currentActuatorPosition;
    /*
    volatile unsigned char dumpBits;
    volatile int dumpData[7][DUMP_SIZE]; // setPoint, Pos, P, I, D, Bits, RES
    volatile unsigned char dumpPos;  
    */
    volatile char lastError;
    volatile char previousError;
    volatile int accuracy;
    volatile int setPointMax;
    volatile int setPointMin;
public:
    void (*callback)();
    QuantityAdjuster();
    void update();
    void initialize();
    void setPosition(int val);
    void triggerHit();
};

extern Core core;

