#include "Arduino.h"
#include "Core.h"
#include "Utils.h"
#include "DTC.h"
#include "ConfEditor.h"
//#include "SimpleRotatingActuator.h"


const char nodeDescription[][55] PROGMEM = {
	"DMN-EDC Software Version", // 0
	"Engine RPM", // 1
	"Injection Advance", // 2
	"Engine temperature",  // 3
	"Fuel temperature", // 4
	"Air temperature", // 5
	"Boost pressure", // 6
	"Injection trigger threshold voltage", // 7
	"Battery Voltage", // 8
	"Running State", // 9
	"Rpm deviation cyl#1", // 10
	"Rpm deviation cyl#2", // 11
	"Rpm deviation cyl#3", // 12
	"Rpm deviation cyl#4", // 13
	"Rpm deviation cyl#5", // 14
	"Rpm deviation cyl#6", // 15
	"", // 16 
	"TPS: signal min limit", // 17
	"TPS: signal max limit", // 18
	"TPS: safety bits (0=off, 1=idleSw/WotSW)", // 19
	"MAP: signal min limit (at 100kPa)", // 20
	"MAP: signal max limit", // 21
	"MAP: sensor max pressure", // 22
	"RPM: engine max RPM (control-map scale)", // 23
	"RPM: DSP (0=direct/1=avg/2=2xteeth)", // 24
	"Probe: signal out (0=RPM/1=injSig/2=freqConv)",  // 25
	"Probe: freq conv ratio (1.00=100)", // 26
	"Injection: fuel cut when engine is stopped", // 27
	"Injection: advance (0=off/1=open/2=closed)",  // 28
	"Injection: automatic balance", // 29
	"Idle: adjusting enabled", // 30
	"Idle: speed target", // 31
	"Idle: PID Control Kp", // 32
	"Idle: PID Control Ki", // 33
	"Idle: PID Control Kd", // 34
	"Idle: PID Control Speed", // 35
	"Idle: PID Control Bias", // 36
	"Idle: PID Control Max Fuel", // 37
	"Idle: PID Control Min Fuel", // 38
	"QA Feedback: signal min limit", // 39
	"QA Feedback: signal max limit", // 40
	"QA Feedback: HDK reference enabled", // 41
	"QA servo: min PWM cycle (x/1024)", // 42
	"QA servo: max PWM cycle (x/1024)",// 43
	"QA servo: setPoint", // 44
	"Generic variable for debug", // 45
	"QA Debug: setPoint diff / FB Jitter", // 46
	"QA PID Control: Kp factor", // 47
	"QA PID Control: Ki factor", // 48
	"QA PID Control: Kd factor", // 49
	"QA PID Control: Speed", // 50
	"QA PID Control: Bias", // 51
	"Boost Control (0=off/1=open/2=closed/3/4)", // 52
	"Boost Control: PID Speed", // 53
	"Boost Control: PID Kp factor", // 54
	"Boost Control: PID Ki factor", // 55
	"Boost Control: PID Kd factor", // 55
	"Boost Control: PID Control Bias", // 57
	"Boost Control: PID Range", // 58
	"Boost Control: Current Pressure", // 59	
	"Boost Control: Target Pressure", // 60
	"Advance Control: PID Kp factor", // 61
	"Advance Control: PID Ki factor", // 62
 	"Actuator: Target position", // 63
	"Actuator: low position stop", // 64
 	"Actuator: high position stop",  // 65 
 	"Actuator: P-value (responsiveness, speed)", // 66
 	"Actuator: I-value (stability)", // 67
	"Actuator: calculated current PWM", // 68
	"Actuator: Max PWM", // 69
	"Actuator: hysteresis", // 70
	"Actuator: motor direction", // 71
 	"Actuator: stepping delay (0=continuous loop)", // 72
	"Temp Sensor, engine: value",
	"Temp Sensor, engine: B-coefficient",
	"Temp Sensor, engine: nominal resistance",
	"Temp Sensor, engine: nominal temperature",
	"Temp Sensor, intake: value",
	"Temp Sensor, intake: B-coefficient",
	"Temp Sensor, intake: nominal resistance",
	"Temp Sensor, intake: nominal temperature",
	"Temp Sensor, fuel: value",
	"Temp Sensor, fuel: B-coefficient",
	"Temp Sensor, fuel: nominal resistance",
	"Temp Sensor, fuel: nominal temperature",	
	"Fuelmap smoothness", // 85
	"Initial Injection Quantity", // 86
	"Maximal Injection Quantity", // 87	
	"Fan#1 Switch on temperature" // 88
	""
};


Core core;

Core::Core() {
	// Initialize core parameters
	// (file_id in EEPROM, initial_value, min, max, increment_step, bindedRawValue, bindedActualValue,isLocked?,description of this value (max 45 chars!))

	node[nodeSoftwareVersion] = (nodeStruct) {0x0000,VERSION_NUMBER,0x0103,9999,1,valueEngineRPMMin,valueEngineRPMMax,NODE_PROPERTY_LOCKED,VALUE_INT};  
	node[nodeEngineRPM] =       (nodeStruct) {0x1001,0,0,0,1,valueEngineRPMFiltered,valueEngineRPMJitter, NODE_PROPERTY_LOCKED,VALUE_INT};
	node[nodeEngineTiming] =    (nodeStruct) {0x1002,0,0,0,1,valueEngineTimingActual,valueEngineRPMErrors, NODE_PROPERTY_LOCKED,VALUE_INJECTION_TIMING};  
	node[nodeTempEngine] =      (nodeStruct) {0x10e3,47,255,0,1,valueTempEngine,valueTempEngineRaw, NODE_PROPERTY_LOCKED,VALUE_CELSIUS};  
	node[nodeTempFuel] =        (nodeStruct) {0x10e4,47,0,255,1,valueTempFuel,valueTempFuelRaw, NODE_PROPERTY_LOCKED,VALUE_CELSIUS};  
	node[nodeTempIntake] =         (nodeStruct) {0x10e5,47, 0,255,1,valueTempIntake,valueTempIntakeRaw, NODE_PROPERTY_LOCKED,VALUE_CELSIUS};  
	node[nodePressure] =        (nodeStruct) {0x1006,0,0,0,1,valueBoostPressure,valueNone, NODE_PROPERTY_LOCKED,VALUE_KPA};  
	node[nodeInjectionThresholdVoltage] = 
	                            (nodeStruct) {0x1007,0,0,1,1,valueInjectionThresholdVoltage,valueNone, NODE_PROPERTY_LOCKED,VALUE_VOLTAGE};     
	node[nodeBatteryVoltage] =  (nodeStruct) {0x1008,0,0,1,1,valueBatteryVoltage,valueNone, NODE_PROPERTY_LOCKED,VALUE_BATTERY_VOLTAGE};     
	node[nodeRunMode] =         (nodeStruct) {0x1009,0,0,1,1,valueRunMode,valueNone, NODE_PROPERTY_LOCKED,VALUE_INT};     
	node[nodeRpmDeviation1] =   (nodeStruct) {0x10FA,0,0,1,1,valueNone,valueRpmDeviation1, NODE_PROPERTY_LOCKED,VALUE_INT};
	node[nodeRpmDeviation2] =   (nodeStruct) {0x10FB,0,0,1,1,valueNone,valueRpmDeviation2, NODE_PROPERTY_LOCKED,VALUE_INT};
	node[nodeRpmDeviation3] =   (nodeStruct) {0x10FC,0,0,1,1,valueNone,valueRpmDeviation3, NODE_PROPERTY_LOCKED,VALUE_INT};
	node[nodeRpmDeviation4] =   (nodeStruct) {0x10FD,0,0,1,1,valueNone,valueRpmDeviation4, NODE_PROPERTY_LOCKED,VALUE_INT};
	node[nodeRpmDeviation5] =   (nodeStruct) {0x10FE,0,0,1,1,valueNone,valueRpmDeviation5, NODE_PROPERTY_LOCKED,VALUE_INT};
	node[nodeRpmDeviation6] =   (nodeStruct) {0x10FF,0,0,1,1,valueNone,valueRpmDeviation6, NODE_PROPERTY_LOCKED,VALUE_INT};

	node[nodeTPSMin] = 	        (nodeStruct) {0x1010,201,25,1000,1,valueTPSRaw,valueTPSActual,NODE_PROPERTY_EDITABLE,VALUE_VOLTAGE};  
	node[nodeTPSMax] =          (nodeStruct) {0x1011,885,25,1000,1,valueTPSRaw,valueTPSActual,NODE_PROPERTY_EDITABLE,VALUE_VOLTAGE};
	node[nodeTPSSafetyBits] =   (nodeStruct) {0x1012,0,0,255,1,valueNone,valueNone,NODE_PROPERTY_EDITABLE,VALUE_INT};  
	node[nodeFuelCutAtStall] =  (nodeStruct) {0x1013,1,0,1,1,valueNone,valueNone,NODE_PROPERTY_EDITABLE,VALUE_BOOLEAN};  
	node[nodeQAInjectorBalance] = 
	                            (nodeStruct) {0x1015,0,0,1,1,valueNone,valueNone,NODE_PROPERTY_EDITABLE,VALUE_BOOLEAN};  
	node[nodeTimingMethod] =    (nodeStruct) {0x1014,0,0,2,1,valueNone,valueTimingPIDAmount,NODE_PROPERTY_EDITABLE,VALUE_INT};  
	                            
	node[nodeProbeSignalOutput] = 
	                            (nodeStruct) {0x1016,0,0,10,1,valueNone,valueNone,NODE_PROPERTY_EDITABLE,VALUE_INT};  
	node[nodeFreqConvRatio] =   (nodeStruct) {0x1017,100,1,1000,1,valueNone,valueNone,NODE_PROPERTY_EDITABLE,VALUE_INT};  

	node[nodeIdleAdjusting] =   (nodeStruct) {0x1018,0,0,1,1,valueNone,valueIdlePIDCorrection,NODE_PROPERTY_EDITABLE,VALUE_BOOLEAN};  
	node[nodeIdleSpeedTarget] = (nodeStruct) {0x1019,830,350,1600,1,valueEngineRPMFiltered,valueNone,NODE_PROPERTY_EDITABLE,VALUE_INT};  
	node[nodeIdleKp] =          (nodeStruct) {0x101A,2,0,300,1,valueNone,valueNone,NODE_PROPERTY_EDITABLE,VALUE_INT};  
	node[nodeIdleKi] =          (nodeStruct) {0x101B,1,0,300,1,valueNone,valueNone,NODE_PROPERTY_EDITABLE,VALUE_INT};  
	node[nodeIdleKd] =          (nodeStruct) {0x1032,1,0,300,1,valueNone,valueNone,NODE_PROPERTY_EDITABLE,VALUE_INT};  
	node[nodeIdlePIDSpeed] =    (nodeStruct) {0x1035,15,1,140,1,valueNone,valueNone,NODE_PROPERTY_EDITABLE,VALUE_INT};  	
	node[nodeIdlePIDBias] =     (nodeStruct) {0x1036,33,1,200,1,valueNone,valueNone,NODE_PROPERTY_EDITABLE,VALUE_INT};  
	node[nodeIdleMaxFuel] =     (nodeStruct) {0x1033,40,0,1024,5,valueNone,valueNone,NODE_PROPERTY_EDITABLE,VALUE_INT};  
	node[nodeIdleMinFuel] =     (nodeStruct) {0x1034,0,-40,1024,5,valueNone,valueNone,NODE_PROPERTY_EDITABLE,VALUE_INT};  

	node[nodeRPMDSP] =          (nodeStruct) {0x101C,0,0,3,1,valueNone,valueNone,NODE_PROPERTY_EDITABLE,VALUE_INT};  

	node[nodeQAFeedbackMin] =   (nodeStruct) {0x101D,205,1,1022,1,valueQAfeedbackActual,valueQAfeedbackRaw,NODE_PROPERTY_EDITABLE,VALUE_VOLTAGE};
	node[nodeQAFeedbackMax] =   (nodeStruct) {0x101E,881,1,1023,1,valueQAfeedbackActual,valueQAfeedbackRaw,NODE_PROPERTY_EDITABLE,VALUE_VOLTAGE};
	node[nodeQAReferenceEnabled] = 
								(nodeStruct) {0x101F,0,0,1,1,valueNone,valueNone,NODE_PROPERTY_EDITABLE,VALUE_BOOLEAN};  
	node[nodeQASetPoint] =      (nodeStruct) {0x1020,0,0,1023,1,valueNone,valueQAfeedbackSetpoint,NODE_PROPERTY_LOCKED,VALUE_INT};  
	node[nodeQAMinPWM] =        (nodeStruct) {0x1021,100,1,700,1,valueNone,valueQAPWMActual,NODE_PROPERTY_EDITABLE,VALUE_INT};      
	node[nodeQAMaxPWM] =        (nodeStruct) {0x1022,650,1,800,1,valueNone,valueQAPWMActual,NODE_PROPERTY_EDITABLE,VALUE_INT};     
	node[nodeMAPMin] =          (nodeStruct) {0x1023,135,0,1000,5,valueMAPRaw,valueMAPActual,NODE_PROPERTY_EDITABLE,VALUE_VOLTAGE};  
	node[nodeMAPMax] =          (nodeStruct) {0x1024,935,0,1000,5,valueMAPRaw,valueMAPActual,NODE_PROPERTY_EDITABLE,VALUE_VOLTAGE};
	node[nodeMAPkPa] =          (nodeStruct) {0x1025,300,100,1000,5,valueNone,valueNone,NODE_PROPERTY_EDITABLE,VALUE_INT};  
	node[nodeControlMapScaleRPM] = 
	                            (nodeStruct) {0x1026,5000,100,10000,100,valueNone,valueNone,NODE_PROPERTY_EDITABLE,VALUE_INT};  
	node[nodeGenericDebugValue] = 
	                            (nodeStruct) {0x1027,0,0,1023,16,valueNone,valueNone,NODE_PROPERTY_EDITABLE,VALUE_KPA};  
	node[nodeQADebugJitter] =   (nodeStruct) {0x1028,0,0,1024,1,valueNone,valueQAJitter,NODE_PROPERTY_LOCKED,VALUE_INT};  
	node[nodeQAPIDKp] =         (nodeStruct) {0x1029,54/*80*/,0,1000,1,valueNone,valueQAPIDPparam,NODE_PROPERTY_EDITABLE,VALUE_INT};  
	node[nodeQAPIDKi] =         (nodeStruct) {0x102A,10/*3*/,0,1000,1,valueNone,valueQAPIDIparam,NODE_PROPERTY_EDITABLE,VALUE_INT};  
	node[nodeQAPIDKd] =         (nodeStruct) {0x102B,0,0,1000,1,valueNone,valueQAPIDDparam,NODE_PROPERTY_EDITABLE,VALUE_INT};  
	node[nodeQAPIDSpeed] =      (nodeStruct) {0x102C,25,1,128,1,valueNone,valueNone,NODE_PROPERTY_EDITABLE,VALUE_INT};     
	node[nodeQAPIDBias] =       (nodeStruct) {0x102D,55,1,200,5,valueNone,valueNone,NODE_PROPERTY_EDITABLE,VALUE_INT};      

	node[nodeBoostAdjusting] =  (nodeStruct) {0x102E,0,0,4,1,valueBoostValveDutyCycle,valueBoostPIDCorrection,NODE_PROPERTY_EDITABLE,VALUE_INT};      
	node[nodeBoostSpeed] =      (nodeStruct) {0x102F,5,1,200,10,valueNone,valueNone,NODE_PROPERTY_EDITABLE,VALUE_INT};      
	node[nodeBoostKp] =         (nodeStruct) {0x1030,13,1,2000,1,valueNone,valueNone,NODE_PROPERTY_EDITABLE,VALUE_INT};      
	node[nodeBoostKi] =         (nodeStruct) {0x1031,6,1,2000,1,valueNone,valueNone,NODE_PROPERTY_EDITABLE,VALUE_INT};      
	node[nodeBoostKd] =         (nodeStruct) {0x1032,0,0,2000,1,valueNone,valueNone,NODE_PROPERTY_EDITABLE,VALUE_INT};  
	node[nodeBoostBias] =    	(nodeStruct) {0x1038,100,1,200,1,valueNone,valueNone,NODE_PROPERTY_EDITABLE,VALUE_INT};  
	node[nodeBoostPIDRange] =   (nodeStruct) {0x1039,20,1,200,1,valueNone,valueBoostPIDCorrection,NODE_PROPERTY_EDITABLE,VALUE_INT};  
	node[nodeBoostTargetPressure] =   (nodeStruct) {0x0,0,1,200,1,valueBoostTarget,valueNone,NODE_PROPERTY_LOCKED,VALUE_KPA};  
	node[nodeBoostActualPressure] =   (nodeStruct) {0x0,0,1,200,1,valueBoostPressure,valueNone,NODE_PROPERTY_LOCKED,VALUE_KPA};  

	node[nodeTimingKp] =         (nodeStruct) {0x103A,10,1,200,1,valueNone,valueNone,NODE_PROPERTY_EDITABLE,VALUE_INT};      
	node[nodeTimingKi] =         (nodeStruct) {0x103B,2,1,200,1,valueNone,valueNone,NODE_PROPERTY_EDITABLE,VALUE_INT};   

	node[nodeActuatorPos] = (nodeStruct) {0x5000,800,80,920,5,valueNone,valueActuatorActualPosition,NODE_PROPERTY_EDITABLE,VALUE_INT};  	
	node[nodeActuatorMinPos] = (nodeStruct) {0x5001,200,80,920,5,valueNone,valueActuatorSetPoint,NODE_PROPERTY_EDITABLE,VALUE_INT};  
	node[nodeActuatorMaxPos] = (nodeStruct) {0x5002,800,80,920,5,valueNone,valueActuatorSetPoint,NODE_PROPERTY_EDITABLE,VALUE_INT};  
	node[nodeActuatorMaxPWM] = (nodeStruct) {0x5003,200,5,255,5,valueNone,valueNone,NODE_PROPERTY_EDITABLE,VALUE_PWM8};  

	node[nodeActuatorP] = (nodeStruct) {0x5004,135,0x0,4000,5,valueNone,valueNone,NODE_PROPERTY_EDITABLE,VALUE_INT};  
	node[nodeActuatorI] = (nodeStruct) {0x5005,20,0x0,4000,5,valueNone,valueNone,NODE_PROPERTY_EDITABLE,VALUE_INT};  
	node[nodeActuatorPWM] = (nodeStruct) {0x5006,0,0,0,1,valueNone,valueNone,NODE_PROPERTY_LOCKED,VALUE_PWM8};  
	node[nodeActuatorHysteresis] = (nodeStruct) {0x5007,11,0,100,1,valueNone,valueNone,NODE_PROPERTY_EDITABLE,VALUE_INT};  
	node[nodeActuatorDirection] = (nodeStruct) {0x5008,0,0,200,1,valueNone,valueActuatorSetPoint,NODE_PROPERTY_LOCKED,VALUE_INT};  
	node[nodeActuatorSteppingDelay] = (nodeStruct) {0x5009,0,0,200,1,valueNone,valueNone,NODE_PROPERTY_EDITABLE,VALUE_MS}; 

	node[nodeEngineTemp] =      (nodeStruct) {0x6000,47,255,0,1,valueTempEngine,valueNone, NODE_PROPERTY_LOCKED,VALUE_CELSIUS}; 
	node[nodeEngineTempSensorBcoefficient] = (nodeStruct) {0x6001,3520,100,20000,10,valueNone,valueTempEngineRaw,NODE_PROPERTY_EDITABLE,VALUE_INT};  	
	node[nodeEngineTempSensorNResistance] = (nodeStruct) {0x6002,450,100,20000,10,valueNone,valueNone,NODE_PROPERTY_EDITABLE,VALUE_INT};  	
	node[nodeEngineTempSensorNTemp] = (nodeStruct) {0x6003,70,0,255,1,valueNone,valueNone,NODE_PROPERTY_EDITABLE,VALUE_INT};  	

	node[nodeIntakeTemp] =      (nodeStruct) {0x6004,47,255,0,1,valueTempIntake,valueNone, NODE_PROPERTY_LOCKED,VALUE_CELSIUS}; 
	node[nodeIntakeTempSensorBcoefficient] = (nodeStruct) {0x6005,3800,100,20000,10,valueNone,valueTempIntakeRaw,NODE_PROPERTY_EDITABLE,VALUE_INT};  	
	node[nodeIntakeTempSensorNResistance] = (nodeStruct) {0x6006,450,100,20000,10,valueNone,valueNone,NODE_PROPERTY_EDITABLE,VALUE_INT};  	
	node[nodeIntakeTempSensorNTemp] = (nodeStruct) {0x6007,70,0,255,1,valueNone,valueNone,NODE_PROPERTY_EDITABLE,VALUE_INT};  	

	node[nodeFuelTemp] =      (nodeStruct) {0x6008,47,255,0,1,valueTempFuel,valueNone, NODE_PROPERTY_LOCKED,VALUE_CELSIUS}; 
	node[nodeFuelTempSensorBcoefficient] = (nodeStruct) {0x6009,3450,100,20000,10,valueNone,valueTempFuelRaw,NODE_PROPERTY_EDITABLE,VALUE_INT};  	
	node[nodeFuelTempSensorNResistance] = (nodeStruct) {0x600a,4000,100,20000,10,valueNone,valueNone,NODE_PROPERTY_EDITABLE,VALUE_INT};  	
	node[nodeFuelTempSensorNTemp] = (nodeStruct) {0x600b,1,0,255,1,valueNone,valueNone,NODE_PROPERTY_EDITABLE,VALUE_INT};  	

	node[nodeFuelMapSmoothness] =  (nodeStruct) {0x1100,0,0,100,5,valueNone,valueNone, NODE_PROPERTY_EDITABLE,VALUE_INT};     
	node[nodeInitialInjectionQuantity] =   (nodeStruct) {0x1101,120,0,1000,5,valueNone,valueNone, NODE_PROPERTY_EDITABLE,VALUE_INT};     
	node[nodeMaximalInjectionQuantity] =   (nodeStruct) {0x1103,1020,0,1023,5,valueNone,valueNone, NODE_PROPERTY_EDITABLE,VALUE_INT};     
	node[nodeFan1SwitchOnTemp] =  (nodeStruct) {0x1102,132,0,255,1,valueNone,valueFan1State, NODE_PROPERTY_EDITABLE,VALUE_CELSIUS};     

	currentNode = LIST_RESET;

	// (file_id in EEPROM, initial_value, min, max, increment_step, bindedRawValue, bindedActualValue,isLocked?,description of this value (max 45 chars!))

	/* MAP header <map-id>, 0xf0, .... 
	Map id is used for saving/loading maps from eeprom, use unique id
	*/ 
   
	/* 0xf? Fuel injections maps */
	/* basic injection amount */

	static unsigned char fuelMap[] = {
	  0xF0,0xF0,'M','2','D',
	  0x8,0x6,MAP_AXIS_RPM,MAP_AXIS_TPS,MAP_AXIS_INJECTED_FUEL,
 		100,	70,     35,     0,      0,      0,      0,      0,
		100,    100,    78,    37,     0,      0,      0,      0,
	    100,    100,    80,    75,    37,     10,      0,      0,
	    100,    100,    100,    100,    60,    77,     10,     0,
        100,    100,    100,    100,    100,    100,    77,     0,
        100,    100,    100,    100,    100,    100,    100,    0,      
		0,0,0,0,0,1,1                // lastX,lastY,lastRet,lastRet 10bit (2 bytes),idxX,idxY
	 };

	/* additive (injection) fuel map for boost */

	static unsigned char boostMap[] = {
	  0xF1,0xF0,'M','2','D',
	  0x8,0x6,MAP_AXIS_RPM,MAP_AXIS_KPA,MAP_AXIS_INJECTED_FUEL,
	  0,0,0,0,     0,0,0,0,
	  31,31,31,31, 31,31,31,0,
	  53,47,65,65, 75,75,75,0,
	  30,30,78,78, 85,85,85,0,	  
	  0,0,0,0, 0,0,0,0,
	  0,0,0,0, 0,0,0,0,
		0,0,0,0,0,1,1                // lastX,lastY,lastRet,lastRet 10bit (2 bytes),idxX,idxY
	};

	/* Cold Start & idle fuel map */
	static unsigned char idleMap[] = {
		0xF2,0xF0,'M','2','D',
		0x8,0x4,MAP_AXIS_IDLERPM,MAP_AXIS_CELSIUS,MAP_AXIS_INJECTED_FUEL,
		255,255,255,114, 80,60,60,50,
		190,130,90,80, 70,60,50,40,		
		190,120,90,80, 70,60,50,40,
		190,110,90,80, 70,60,50,40,
		0,0,0,0,0,1,1                // lastX,lastY,lastRet,lastRet 10bit (2 bytes),idxX,idxY
	};

	static unsigned char coastMap[] = {
		0xF5,0xF0,'M','1','D',
		0x8,0x1,MAP_AXIS_RPM,MAP_AXIS_NONE,MAP_AXIS_INJECTED_FUEL,
		100,100,50,10, 0,0,0,0,
		0,0,0,0,0,1,1                // lastX,lastY,lastRet,lastRet 10bit (2 bytes),idxX,idxY
	};

	static unsigned char fuelTrimFuelTemp[] = {
		0xF3,0xF0,'M','1','D',
		0x8,0x1,MAP_AXIS_CELSIUS,MAP_AXIS_NONE,MAP_AXIS_FUEL_TRIM_AMOUNT,
		128,128,128,128,128,128,128,128,
		0,0,0,0,0,1,1                // lastX,lastY,lastRet,lastRet 10bit (2 bytes),idxX,idxY
	};
	
	static unsigned char fuelTrimAirTemp[] = {
		0xF4,0xF0,'M','1','D',
		0x8,0x1,MAP_AXIS_CELSIUS,MAP_AXIS_NONE,MAP_AXIS_FUEL_TRIM_AMOUNT,
		128,128,128,128,128,128,128,128,
		0,0,0,0,0,1,1                // lastX,lastY,lastRet,lastRet 10bit (2 bytes),idxX,idxY
	};

	/* 0xe? engine timing */
	/*  static mode */

	static unsigned char openLoopAdvanceMap[] = {
		0xe0,0xF0,'M','2','D',
		0x6,0x6,MAP_AXIS_RPM,MAP_AXIS_INJECTED_FUEL,MAP_AXIS_DUTY_CYCLE,
		255,255,255, 255,255,255, 
		255,255,255, 255,255,210, 
		255,255,255, 255,180,180, 
		255,255,255,    190,140,0, 
		255,255,255,    100,80,0, 
		100,100,100,    50,50,0, 
		0,0,0,0,0,1,1                // lastX,lastY,lastRet,lastRet 10bit (2 bytes),idxX,idxY
	};

	/* dynamic mode */

	static unsigned char closedLoopAdvanceMap[] = {
		0xe1,0xF0,'M','2','D',
		0x6,0x6,MAP_AXIS_RPM,MAP_AXIS_INJECTED_FUEL,MAP_AXIS_INJECTION_TIMING,
		0,0,0, 0,0,0, 
		0,0,0, 0,0,0, 
		0,0,0, 0,0,0,  

		0,0,0, 0,0,0, 
		0,0,0, 0,0,0, 
		0,0,0, 0,0,0,  
		0,0,0,0,0,1,1                // lastX,lastY,lastRet,lastRet 10bit (2 bytes),idxX,idxY
	};


	/*  0xd? - Turbo wastegate / VNT control maps */

	/* basic duty cycle vs rpm*injection amount */

	static unsigned char turboControlMap[] = {
		0xd0,0xF0,'M','2','D',
		0x6,0x6,MAP_AXIS_RPM,MAP_AXIS_INJECTED_FUEL,MAP_AXIS_DUTY_CYCLE,
		201,227,210,201,171,158,
		201,227,208,192,169,158,	
		201,182,198,180,162,158,
		201,201,195,166,153,140,
		201,182,118,151,151,28,
		201,182,156,103,59,28,		
		0,0,0,0,0,1,1                // lastX,lastY,lastRet,lastRet 10bit (2 bytes),idxX,idxY
	};

	/* target pressure map */
	static unsigned char turboTargetPressureMap[] = {
		0xd1,0xF0,'M','2','D',
		0x6,0x4,MAP_AXIS_RPM,MAP_AXIS_TPS,MAP_AXIS_KPA,
		0,0,0,0,0,0,
		0,0,0,10,20,100,	
		0,0,10,30,100,128,
		100,100,100,100,100,100,
		0,0,0,0,0,1,1                // lastX,lastY,lastRet,lastRet 10bit (2 bytes),idxX,idxY
	};

	/* actuator opening/operating curve */
	static unsigned char actuatorTension[] = {
		0xd2,0xF0,'M','1','D',
		0x8,0x1,MAP_AXIS_RAW,MAP_AXIS_NONE,MAP_AXIS_RAW,
		255,215,192,143,90,55,23,0,
		0,0,0,0,0,1,1                // lastX,lastY,lastRet,lastRet 10bit (2 bytes),idxX,idxY
	};			

	/* 0xc? - temperature related maps */

	static unsigned char glowPeriodMap[] = {
		0xc4,0xF0,'M','1','D',
		0x8,0x1,MAP_AXIS_CELSIUS,MAP_AXIS_NONE,MAP_AXIS_HALF_SECONDS,
		40,22,8,6,3,0,0,0,
		0,0,0,0,0,1,1                // lastX,lastY,lastRet,lastRet 10bit (2 bytes),idxX,idxY
	};

	static unsigned char tempSenderMap[] = {
		0xc5,0xF0,'M','1','D',
		0x8,0x1,MAP_AXIS_CELSIUS,MAP_AXIS_NONE,MAP_AXIS_RAW,
		0,0,18,66,130,170,255,255,
		0,0,0,0,0,1,1                // lastX,lastY,lastRet,lastRet 10bit (2 bytes),idxX,idxY
	};
	

	/* 0xb? - temperature sensor calibration maps */

	/* 0x7? Generic control maps */

	static unsigned char idlePidP[] = {
		0x70,0xF0,'M','1','D',
		10,0x1,MAP_AXIS_IDLERPM,MAP_AXIS_NONE,MAP_AXIS_RAW,
		12,12,10,6,4, 2,2,2,5,10,
		0,0,0,0,0,1,1                // lastX,lastY,lastRet,lastRet 10bit (2 bytes),idxX,idxY
	};		

	mapNames[Core::mapIdxFuelMap] = "Basic Injection Map";
	mapNames[Core::mapIdxBoostMap] = "Additive Injection Map (Boost)";
	mapNames[Core::mapIdxIdleMap] = "Injection quantity when starting / idling";
	mapNames[Core::mapIdxOpenLoopAdvanceMap] = "Open Loop advance";
	mapNames[Core::mapIdxClosedLoopAdvanceMap] = "Closed Loop advance";
	mapNames[Core::mapIdxTurboControlMap] = "Turbo actuator; duty cycle base map";
	mapNames[Core::mapIdxTurboTargetPressureMap] = "Turbo actuator; target pressure";
	mapNames[Core::mapIdxGlowPeriodMap] = "Glow period (half seconds)";
	mapNames[Core::mapIdxtempSenderMap] = "Temp Sender emu. output";


	mapNames[Core::mapIdxFuelTrimFuelTemp] = "Fuel Trim amount vs. Fuel Temp.";
	mapNames[Core::mapIdxFuelTrimAirTemp] = "Fuel Trim amount vs. Air Temp.";
	mapNames[Core::mapIdxActuatorTension] = "Turbo Actuator Operating Curve";
	mapNames[Core::mapIdxIdlePidP] = "Idle PID P-parameter during idle";
	mapNames[Core::mapIdxCoastingFuelLimit] = "Fuel limit during coasting";


	maps[Core::mapIdxFuelMap] = (unsigned char*)&fuelMap;
	maps[Core::mapIdxIdleMap] = (unsigned char*)&idleMap;
	maps[Core::mapIdxBoostMap] = (unsigned char*)&boostMap;
	maps[Core::mapIdxOpenLoopAdvanceMap] = (unsigned char*)&openLoopAdvanceMap;
	maps[Core::mapIdxClosedLoopAdvanceMap] = (unsigned char*)&closedLoopAdvanceMap;
	maps[Core::mapIdxTurboControlMap] = (unsigned char*)&turboControlMap;
	maps[Core::mapIdxTurboTargetPressureMap] = (unsigned char*)&turboTargetPressureMap;
	maps[Core::mapIdxGlowPeriodMap] = (unsigned char*)&glowPeriodMap;
	maps[Core::mapIdxtempSenderMap] = (unsigned char*)&tempSenderMap;


//	maps[Core::mapIdxFuelTrimMap] = (unsigned char*)&fuelTrimMap;
	maps[Core::mapIdxFuelTrimFuelTemp] = (unsigned char*)&fuelTrimFuelTemp;
	maps[Core::mapIdxFuelTrimAirTemp] = (unsigned char*)&fuelTrimAirTemp;
	maps[Core::mapIdxActuatorTension] = (unsigned char*)&actuatorTension;
	maps[Core::mapIdxIdlePidP] = (unsigned char*)&idlePidP;
	maps[Core::mapIdxCoastingFuelLimit] = (unsigned char*)&coastMap;

	numberOfMaps=14;
}

/*
New EEPROM Structure 

0000 ROOT ID 4B
0004 file id 2B  // chunk 1
0006 Size 2B
0008 Data (length=size)
000X file id     // chunk n
000X+2 Size
000X+4 Data (length=size)
...
0??? file id=FFFF  // EOF

3584-4096 -> Reserved for DTC


item_id = 0x10xx control values
item_id = 0xf0xx control maps
*/


void Core::save() {
	//SimpleRotatingActuator::disable();

	int ofs = CONFIGURATION_EEPROM_OFFSET;    
	int size;
	// write header
	EEPROMwriteData(ofs,(char*)CONFIGURATION_FILE_4BYTE_ID,4);
	ofs += 4;
	// Write nodes 
	unsigned char idx;
	for (idx = 0;idx<NODE_MAX;idx++) {
		EEPROMwriteData(ofs,(char*)&node[idx].fileId,2); // File id 
		ofs += 2;				
		size = 2;
		EEPROMwriteData(ofs,(char*)&size,2);					// Size
		ofs += 2;				
		EEPROMwriteData(ofs,(char*)&node[idx].value,2);	// Data
		ofs += 2;		
	}
	// Write maps
	for (idx = 0;idx<numberOfMaps;idx++) {
		char x = core.maps[idx][5];
		char y = core.maps[idx][6];
		
		size = x*y+15;

		EEPROMwriteData(ofs,(char*)core.maps[idx],2); // File id 
		ofs += 2;				
		EEPROMwriteData(ofs,(char*)&size,2);				// Size
		ofs += 2;				
		EEPROMwriteData(ofs,(char*)core.maps[idx],15+x*y);    // Data
		ofs += size;

	}
	// FILE ID for EOF
	size = 0xFFFF;
	EEPROMwriteData(ofs,(char*)&size,2);

	//SimpleRotatingActuator::enable();	
}


bool Core::load() {
	unsigned char c1=0,c2=0;
	int ofs = CONFIGURATION_EEPROM_OFFSET;  
	unsigned int fileId;
	int size;  
	int value;
	char buf[7];
	EEPROMreadData(ofs,buf,4);
	buf[4] = 0;
	ofs += 4;

	if (strcmp(buf,CONFIGURATION_FILE_4BYTE_ID) == 0) {
		do {
			EEPROMreadData(ofs,(char*)&fileId,2);
			ofs += 2;
			EEPROMreadData(ofs,(char*)&size,2);
			ofs += 2;

			switch ((fileId & 0xFF00)) {
				case 0x1100:
				case 0x1000:
					if (size == 2) {
						// Core item, length should be 2

						for (unsigned char idx = 0;idx<NODE_MAX;idx++) {
							if (node[idx].fileId == fileId) {
								EEPROMreadData(ofs,(char*)&value,size);
								c1++;
								node[idx].value = value;
							}
						}
					} else {
						dtc.setError(DTC_CONFIGURATION_MISMATCH);
					}

					break;
				case 0xF000:
					// Map item, check matching length/header before loading 
					// EEPROMreadData(ofs,(char*)&buf,7);

					for (unsigned char idx = 0;idx<numberOfMaps;idx++) {
						unsigned int mapId;
						mapId = (int)core.maps[idx][1]*256+core.maps[idx][0];

						if (mapId == fileId) {
							if (size == (core.maps[idx][5]*core.maps[idx][6]+15)) {
								c2++;
								EEPROMreadData(ofs,(char*)core.maps[idx],size);
							} else {
								dtc.setError(DTC_CONFIGURATION_MISMATCH);
							}
						}
					}
					break;
			}

			ofs += size;
		} while (fileId != 0xFFFF);
		Serial.print(c1);
		Serial.print(" nodes, ");			
		Serial.print(c2);
		Serial.print(" maps..");		

		return true;
	} else {
		// file id not detected, do not load
		dtc.setError(DTC_CONFIGURATION_ERROR);
	}
	return false;
}

void Core::save_old() {
	// TODO new format
	// <id><len><data><cksum>
	// <id><len><data><cksum> ...
	// 
	// conf saved as node<->value pairs
	// tables "as it is" (check allocated size in preinitalized header vs. stored size)
	
	int ofs = CONFIGURATION_EEPROM_OFFSET;    
	// write header
	EEPROMwriteData(ofs,(char*)CONFIGURATION_FILE_4BYTE_ID,4);
	ofs += 4;
	// write size
	int size = sizeof(node);
	EEPROMwriteData(ofs,(char*)&size,2);
	ofs += 2;
	// write contents - core nodes
	EEPROMwriteData(ofs,(char*)&node,size);
	ofs += size;  

	// write map data
	for (unsigned char i=0;i<numberOfMaps;i++) {
		char x = core.maps[i][3];
		char y = core.maps[i][4];
		EEPROMwriteData(ofs,(char*)core.maps[i],13+x*y);
		ofs += 13+x*y;
	}
	// write contents - map data -- todo better structure!!
	//EEPROMwriteData(ofs,(char*)&basicFuelMap,sizeof(basicFuelMap));
	
	/* ****
	EEPROMwriteData(ofs,(char*)core.maps[0],13+8*6);
	ofs += 13+8*6;
	
	EEPROMwriteData(ofs,(char*)core.maps[1],13+8*4);
	ofs += 13+8*4;

	**** */

	//EEPROMwriteData(ofs,(char*)core.maps[2],13+5*1);
	//ofs += 13+5*1;
	
	confeditor.setSystemStatusMessage("Saved");
}

void Core::setCurrentNode(int start) {
	if (start<=NODE_MAX) { 
		currentNode = start;
	} else {
		currentNode = LIST_RESET;
	}
}

int Core::seekNextNode() {
	if (currentNode+1<=NODE_MAX) { 
		currentNode++;
	} else {
		currentNode = LIST_RESET;
	}
	return currentNode;
}

nodeStruct* Core::getNextNode() {
	seekNextNode();
	return getNodeData();
}

nodeStruct* Core::getNodeData() {
	if (currentNode == LIST_RESET)
		return NULL;
	return &node[currentNode];
}

void Core::incValue() {
	setValue(node[currentNode].value+node[currentNode].step);
}

void Core::decValue() {
	setValue(node[currentNode].value-node[currentNode].step);
}

void Core::setValue(int value) {
	if (node[currentNode].properties != NODE_PROPERTY_LOCKED) {
		if (value < node[currentNode].min) 
			value = node[currentNode].min;
		if (value > node[currentNode].max) 
			value = node[currentNode].max;
	
		node[currentNode].value = value;
	}
}
