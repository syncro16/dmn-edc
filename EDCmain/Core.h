#ifndef CORE_H
#define CORE_H

#include "defines.h"
#include "Arduino.h"

// Purpose of Core class is to provide one place where introduce new variables, their limits and binds to actual sensor values
// So, all variables (See Core.cpp ::constructor) are automatically configurable via configuration interface

#define VALUE_INT 0
#define VALUE_HEX 1
#define VALUE_CELSIUS 2 //(0 = -35C, 255 = 220C)
#define VALUE_FIXED_POINT_2 3
#define VALUE_PERCENTAGE 4
#define VALUE_DEGREE 4
#define VALUE_KPA 5
#define VALUE_VOLTAGE 6
#define VALUE_BOOLEAN 7
#define VALUE_INJECTION_TIMING 8
#define VALUE_BATTERY_VOLTAGE 9
#define VALUE_MS 10
#define VALUE_PWM8 11

#define CONFIGURATION_FILE_4BYTE_ID "DC56" 
#define CONFIGURATION_EEPROM_OFFSET (0)

#define NODE_PROPERTY_HIDDEN 0
#define NODE_PROPERTY_EDITABLE 1
#define NODE_PROPERTY_LOCKED 2


struct nodeStruct {
    unsigned int fileId;
    volatile int value;
    int min;
    int max;
    int step;
    unsigned char rawValueKey;
    unsigned char actualValueKey;
    char properties;
    char type;
} __attribute__ ((packed));

class Core {
public:
    // configuration file id

    // bindings for input values (value index in EDCmain configuration array)
    static const unsigned char valueNone = 0;
    static const unsigned char valueTPSRaw = 1;
    static const unsigned char valueTPSActual = 2;
    static const unsigned char valueTempEngine = 3;
    static const unsigned char valueTempFuel = 4;
    static const unsigned char valueTempAir = 5;
    static const unsigned char valueBoostPressure = 6;
    static const unsigned char valueEngineRPM = 7;
    static const unsigned char valueEngineTimingTarget = 8;
    static const unsigned char valueEngineTimingActual = 9;
    static const unsigned char valueQAfeedbackActual = 10;  
    static const unsigned char valueQAfeedbackRaw = 11;  
    static const unsigned char valueQAfeedbackSetpoint = 12;  
    static const unsigned char valueQAPWMActual = 13;    
    static const unsigned char valueMAPRaw = 14;
    static const unsigned char valueMAPActual = 15;    
    static const unsigned char valueEngineRPMDurationBetweenTeeths = 16;
    static const unsigned char valueEngineTimingMeasuredDuration = 17;
    static const unsigned char valueEngineRPMJitter = 18;
    static const unsigned char valueQAPIDPparam = 19;
    static const unsigned char valueQAPIDIparam = 20;
    static const unsigned char valueQAPIDDparam = 21;
    static const unsigned char valueBatteryVoltage = 22;
    static const unsigned char valueQAJitter = 23;
    static const unsigned char valueRunMode = 24;
    static const unsigned char valueEngineRPMFiltered = 25;
    static const unsigned char valueEngineRPMMin = 26;
    static const unsigned char valueEngineRPMMax = 27;
    static const unsigned char valueInjectionThresholdVoltage = 28;
    static const unsigned char valueOutputGlow = 29;
    static const unsigned char valueOutputFan = 30;
    static const unsigned char valueEngineTimingDutyCycle = 31;    
    static const unsigned char valueEngineTimingDiff = 32;
    static const unsigned char valueN75DutyCycle = 33;
    static const unsigned char valueOutputTestMode = 34;  
    static const unsigned char valueFuelAmount = 35;      
    static const unsigned char valueFuelAmount8bit = 36;
    static const unsigned char valueRPM8bit = 37;
    static const unsigned char valueFuelBaseAmount = 38;
    static const unsigned char valueFuelEnrichmentAmount = 39;
    static const unsigned char valueFuelLimitAmount = 40;
    static const unsigned char valueTempEngineInput = 41;
    static const unsigned char valueTempFuelInput = 42;
    static const unsigned char valueTempAirInput = 43;  
    static const unsigned char valueFuelTrim = 44; 
    static const unsigned char valueIdlePIDCorrection = 45; 
    static const unsigned char valueBoostPIDCorrection = 46;    
    static const unsigned char valueTimingPIDAmount = 47;
    static const unsigned char valueBoostTarget = 48; 
    static const unsigned char valueBoostValveDutyCycle = 49;   
    static const unsigned char valueCurrentTeeth = 50; 
    static const unsigned char valueBoostActuatorClipReason = 51;   
    static const unsigned char valueSRAPosition = 52;   
    static const unsigned char valueSRATarget = 53;   
    static const unsigned char valueQADebug = 54;   
    static const unsigned char valueActuatorSetPoint = 55;
    static const unsigned char valueActuatorActualPosition = 56;    
    static const unsigned char valueEngineRPMRaw = 57;    
    static const unsigned char VALUE_MAX = 57;

    // Storage for sensors values and such
    volatile int controls[Core::VALUE_MAX+1];    
    
    // nodes for configurable items (itemNode)
    static const unsigned char nodeSoftwareVersion = 0;  
    static const unsigned char nodeEngineRPM = 1;
    static const unsigned char nodeEngineTiming = 2;
    static const unsigned char nodeTempEngine = 3;
    static const unsigned char nodeTempFuel = 4;
    static const unsigned char nodeTempAir = 5;   
    static const unsigned char nodePressure = 6;   
    static const unsigned char nodeHeartBeat = 7; 
    static const unsigned char nodeInjectionThresholdVoltage = 8;
    static const unsigned char nodeBatteryVoltage = 9;
    static const unsigned char nodeRunMode = 10;

    static const unsigned char nodeFuelTrim = 11;
    static const unsigned char nodeFuelMapSmoothness = 12;
    static const unsigned char nodeInitialInjectionQuantity = 13;
    static const unsigned char nodeQASync = 14;
    static const unsigned char nodeFree4 = 15;
    static const unsigned char nodeFree5 = 16;
 
    // 7 .. 16 is reserved values to be observed on main screen
    
    static const unsigned char nodeTPSMin = 17;
    static const unsigned char nodeTPSMax = 18;

    static const unsigned char nodeTPSSafetyBits = 19;
    static const unsigned char nodeMAPMin = 20;
    static const unsigned char nodeMAPMax = 21;
    static const unsigned char nodeMAPkPa = 22;
    static const unsigned char nodeControlMapScaleRPM = 23;
    static const unsigned char nodeRPMDSP = 24;
    static const unsigned char nodeProbeSignalOutput = 25;
    static const unsigned char nodeFreqConvRatio = 26;
    static const unsigned char nodeFuelCutAtStall = 27;
    static const unsigned char nodeTimingMethod = 28;
    static const unsigned char nodeQAInjectorBalance = 29;
    static const unsigned char nodeIdleAdjusting = 30;
    static const unsigned char nodeIdleSpeedTarget = 31;
    static const unsigned char nodeIdleKp = 32;
    static const unsigned char nodeIdleKi = 33;
    static const unsigned char nodeIdleKd = 34;
    static const unsigned char nodeIdlePIDSpeed = 35;  
    static const unsigned char nodeIdlePIDBias = 36;    

    static const unsigned char nodeIdleMaxFuel = 37;
    static const unsigned char nodeIdleMinFuel = 38;
    
    // QA Control & timing control
    static const unsigned char nodeQAFeedbackMin = 39;
    static const unsigned char nodeQAFeedbackMax = 40;    
    static const unsigned char nodeQAReferenceEnabled = 41;
    static const unsigned char nodeQAMinPWM = 42;
    static const unsigned char nodeQAMaxPWM = 43;
    static const unsigned char nodeQASetPoint= 44;        
    static const unsigned char nodeGenericDebugValue = 45;
    static const unsigned char nodeQADebugJitter = 46;
    static const unsigned char nodeQAPIDKp = 47;  
    static const unsigned char nodeQAPIDKi = 48;       
    static const unsigned char nodeQAPIDKd = 49;
    static const unsigned char nodeQAPIDSpeed = 50;
    static const unsigned char nodeQAPIDBias = 51;
    static const unsigned char nodeBoostAdjusting = 52;
    static const unsigned char nodeBoostSpeed = 53;
    static const unsigned char nodeBoostKp = 54;
    static const unsigned char nodeBoostKi = 55;
    static const unsigned char nodeBoostKd = 56;    
    static const unsigned char nodeBoostBias = 57;       
    static const unsigned char nodeBoostPIDRange = 58; 
    static const unsigned char nodeBoostActualPressure = 59;       
    static const unsigned char nodeBoostTargetPressure = 60;        
    static const unsigned char nodeTimingKp = 61;
    static const unsigned char nodeTimingKi = 62;     
    static const unsigned char nodeActuatorPos = 63;    
    static const unsigned char nodeActuatorMinPos = 64;
    static const unsigned char nodeActuatorMaxPos = 65;
    static const unsigned char nodeActuatorP = 66;
    static const unsigned char nodeActuatorI = 67;   
    static const unsigned char nodeActuatorPWM = 68;   
    static const unsigned char nodeActuatorMaxPWM = 69;    
    static const unsigned char nodeActuatorHysteresis = 70;   
    static const unsigned char nodeActuatorDirection = 71;

    static const unsigned char nodeActuatorSteppingDelay = 72; 

    static const unsigned char NODE_MAX = 73; 
    
    static const unsigned char LIST_RESET = -1;
    // Storage for configuragble items
    nodeStruct node[NODE_MAX+1];
    
    // Maps
    static const unsigned char mapIdxFuelMap = 0;
    static const unsigned char mapIdxBoostMap = 1;
    static const unsigned char mapIdxIdleMap = 2;
    static const unsigned char mapIdxIdlePidP = 3;
    static const unsigned char mapIdxOpenLoopAdvanceMap = 4;
    static const unsigned char mapIdxClosedLoopAdvanceMap = 5;
    static const unsigned char mapIdxTurboControlMap = 6;    
    static const unsigned char mapIdxTurboTargetPressureMap = 7;   
    static const unsigned char mapIdxGlowPeriodMap = 8;
    static const unsigned char mapIdxFuelTrimFuelTemp = 9;
    static const unsigned char mapIdxFuelTrimAirTemp = 10;   

    static const unsigned char mapIdxEngineTempSensorMap = 11;    
    static const unsigned char mapIdxFuelTempSensorMap = 12;    
    static const unsigned char mapIdxAirTempSensorMap = 13;   
    static const unsigned char mapIdxEcuTempSensorMap = 14;    
    static const unsigned char mapIdxActuatorTension = 15;

    
    // unsigned char basicFuelMap[13+6*6];
    // unsigned char boostMap[13+8*8];
    unsigned char *maps[32];
    const char *mapNames[32];
    unsigned char numberOfMaps;
    
private:
    unsigned int currentNode;
    
    
public:    
    Core();
    void save();
    bool load();
    void save_old();
    bool load_old();

    void setCurrentNode(int start = LIST_RESET);
    nodeStruct* getNextNode();
    
    int seekNextNode();
    nodeStruct* getNodeData();
    void incValue();
    void decValue();
    void setValue(int value);
};

extern Core core;

#endif
