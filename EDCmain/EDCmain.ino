/*
2012-2017 Juho Pesonen, syncro16@outlook.com (dmn@qla.fi) - Public Domain - Use as you wish.

Program layout:

+ EDCmain (sets interrupt services and handles user interface communication in main loop)
  - refreshQuantityAdjuster is called periodically, handles control of the quantity adjuster
  - refreshFastSensors is called periodically, sets fuel amount based on current engine parameters
  - refreshSlowSensors is called periodically, reads TPS, MAP and other sensor values. Also set updates output ports.
  - rpmTrigger is called when rpm interrupt is high (actual calculation is based on timer1 counter value )
  - needleTrigger is called when injection needle interrupt is high (injection offset is timer 1 counter value after rpm trigger)

  + QuantityAdjuster (PID control of QA)
  + Core (contains configurable items, maps and also storage for sensor & other processing data)
  + ConfEditor (logic for text mode configuration)
  + DTC (non-volatile storage of diagnostic errors)
  + PID (general purpose PID routine)
  + Utils (map lookup & interpolation routines, some ui routines as well)
  + defines.h PIN mapping and other low level data


TODO:
- timer2: based freq converter (pwm or one shot) for tacho etc..
- overboost DTC + safe mode
- Boost map scaler -> separate map -> better drivability
- Overrun -> set Idle PID control to max to allow smooth curve for engine rpm decrease (or 1D(time) map ->  fuel addition)
- PID Control routine: check that output saturation values doesnt let the original control values to run away (during deceleration etc)

idle PID 33/2/1   (2-stable when engine load differs, 33 for rpm hunting to settle)
qa   PID 54/7/1


qa debugVal 
	0 - as it is
	1 - median filtering
	2 - read pos when trigger happens


2015-05:

!!!! IDLE RPM menee ympäri kierroksilla ja PID laskenta koko ajan !!!

TODO: Anti shudder control:

Make injFuel "smoothing"
if injFuel > sweetSpot then injFuel = injFuel * map[injFuel];

or 

maxRpm-minRpm = shudderAmount -> injFuel = injFuel * map[shudderAmount]

Pump head wires
1-pot signal
2-pot +
3-pot -


5-ftemp -
6-ftemp + (2300ohm 23C)


*/
#include "defines.h"
#include "ConfEditor.h"
#include "Core.h"
#include "QuantityAdjuster.h"
#include "TimerThree.h"
#include "utils.h"
#include "DTC.h"
#include "MemoryFree.h"
#include "PID.h"


#ifdef RPM_SENSOR_TYPE_DEFAULT
#include "RPMDefaultCPS.h"
RPMDefaultCPS rpm;
#endif

#ifdef RPM_SENSOR_TYPE_CUSTOM
#include "RPMCustomCPS.h"
RPMCustomCPS rpm;
#endif



// VP37 Quantity adjuster module
static QuantityAdjuster adjuster;

// Periodic routines called by 2500Hz (or less, set divider to 2 or greater)
#define interruptHandlerMax 8

volatile struct interruptHandlerStruct { 
	void (*handler)();
	unsigned char divider;
} interruptHandlerArray[interruptHandlerMax];

volatile unsigned char shortTicks=0;

// Called when Timer3 overflow occurs. Then calls handler routines according to their divider value
void mainInterruptHandler() {
	// Enable nested interrupts to not miss (or wrongly) calculate RPM signal 
	sei();    
	for (unsigned char i=0;i<interruptHandlerMax;i++) {
		if (interruptHandlerArray[i].handler != NULL 
			&& (interruptHandlerArray[i].divider == 0
				|| (shortTicks % interruptHandlerArray[i].divider == 0))) {
			interruptHandlerArray[i].handler();
		}
	}
	shortTicks++;
	//cli(); // brainfart
} 

void refreshQuantityAdjuster() {
	adjuster.update();
}


static int rpmIdle;
volatile static char calls=0;

// Last idle calculated loop fuel amount. This decreases slowly on non-idle run mode, providing smooth transition between maps
// baseFuelAmount is set to idleLastCalculatedFuelAmount if it is greater than value read from control lookup table (map)

static int idleLastCalculatedFuelAmount; 
void doIdlePidControl() {
	/* PID */ 
	if (idleLastCalculatedFuelAmount>0)
		idleLastCalculatedFuelAmount--;

	static int idleMinFuel,idleMaxFuel,pidP;

	/* Use Idle PID p-parameter lookup table if static value of P is below 2*/
	if (core.node[Core::nodeIdleKp].value<2) {
		pidP = core.node[Core::nodeIdleKp].value;
	} else {
		pidP = mapLookUp(core.maps[Core::mapIdxIdlePidP],rpmIdle,0);	
	}

	static PID idlePidControl(
		(int*)&pidP,
//		(int*)&core.node[Core::nodeIdleKp].value,
		(int*)&core.node[Core::nodeIdleKi].value,
		(int*)&core.node[Core::nodeIdleKd].value,
			(int*)&idleMinFuel, //(int*)&core.node[Core::nodeIdleMinFuel].value,
			(int*)&idleMaxFuel, // (int*)&core.node[Core::nodeIdleMaxFuel].value,
			(int*)&core.node[Core::nodeIdlePIDSpeed].value,
			(int*)&core.node[Core::nodeIdlePIDBias].value,			
			(int*)&core.controls[Core::valueEngineRPMFiltered],
			(int*)&core.controls[Core::valueIdlePIDCorrection]
			);


	idleMinFuel = core.node[Core::nodeIdleMinFuel].value; // JOOSE
	idleMaxFuel = core.node[Core::nodeIdleMaxFuel].value;

	idlePidControl.setPosition(core.node[Core::nodeIdleSpeedTarget].value);
	idlePidControl.calculate(); // interrupt safe!
}

/* Read sensor values here which are not tied into interrupt handlers (see refreshFastSensors) 
   If changing over 8bit values which are used by interrupt controller (refreshFastSensor), remember to disable intterrupts while assigning parameter
*/
void refreshSlowSensors() {
	// read slow changing sensors
	int value;
	int scaledValue;

 	// Engine TEMP
	value = analogRead(PIN_ANALOG_TEMP_COOLANT);
	scaledValue = mapLookUp(core.maps[Core::mapIdxEngineTempSensorMap], value / 4, 0);		
	if (value > ANALOG_INPUT_HIGH_STATE_LIMIT) {
		// generate error only if map is configured and sensor reading is not present
		if (scaledValue > 0)
		dtc.setError(DTC_ENGINE_TEMP_UNCONNECTED);
		scaledValue = core.node[Core::nodeTempEngine].value; // use configuration setpoint value sensor's failback substitute value
	} 
	core.controls[Core::valueTempEngine]=scaledValue;
	
	// Fuel TEMP
	value = analogRead(PIN_ANALOG_TEMP_FUEL);
	scaledValue = mapLookUp(core.maps[Core::mapIdxFuelTempSensorMap], value / 4, 0);		
	if (value > ANALOG_INPUT_HIGH_STATE_LIMIT) {
		// generate error only if map is configured and sensor reading is not present
		if (scaledValue > 0)
		dtc.setError(DTC_FUEL_TEMP_UNCONNECTED);
		scaledValue = core.node[Core::nodeTempFuel].value; // use configuration setpoint value sensor's failback substitute value
	}
	core.controls[Core::valueTempFuel]=scaledValue;

	// Air TEMP
	value = analogRead(PIN_ANALOG_TEMP_INTAKE);
	scaledValue = mapLookUp(core.maps[Core::mapIdxAirTempSensorMap], value / 4, 0);		
	if (value > ANALOG_INPUT_HIGH_STATE_LIMIT) {
		// generate error only if map is configured and sensor reading is not present
		if (scaledValue > 0)
		dtc.setError(DTC_AIR_TEMP_UNCONNECTED);
		scaledValue = core.node[Core::nodeTempAir].value; // use configuration setpoint value sensor's failback substitute value
	}
	core.controls[Core::valueTempAir]=scaledValue;

	// Gearbox TEMP
	value = analogRead(PIN_ANALOG_TEMP_GEARBOX);
/*	scaledValue = mapLookUp(core.maps[Core::mapIdxAirTempSensorMap], value / 4, 0);		
	if (value > ANALOG_INPUT_HIGH_STATE_LIMIT) {
		// generate error only if map is configured and sensor reading is not present
		if (scaledValue > 0)
		dtc.setError(DTC_AIR_TEMP_UNCONNECTED);
		scaledValue = core.node[Core::nodeTempAir].value; // use configuration setpoint value sensor's failback substitute value
	}
	core.controls[Core::valueTempAir]=scaledValue;*/

	// Throttle position sensor(s) - Common VW sensor type
	// swiches for idle and full gas (later one not used)
	value = analogRead(PIN_ANALOG_TPS_POS);
	core.controls[Core::valueTPSRaw]=value;

	// Check TPS it is connected, otherwise apply limp mode amount (about 15%) 
	if (value > ANALOG_INPUT_HIGH_STATE_LIMIT) {
		dtc.setError(DTC_TPS_UNCONNECTED);
		core.controls[Core::valueTPSActual] = TPS_LIMP_MODE_AMOUNT; 
	} else {
		scaledValue = mapValues(core.controls[Core::valueTPSRaw],
				core.node[Core::nodeTPSMin].value,
				core.node[Core::nodeTPSMax].value);
		core.controls[Core::valueTPSActual] = scaledValue; 

	}

	/* override for gas pedal idle position TODO!*/
//	if ((core.node[Core::nodeTPSSafetyBits].value & TPS_SAFETY_BITS_IDLESW) == TPS_SAFETY_BITS_IDLESW) {
//		value = digitalRead(PIN_TPS_IDLE);
//		if (!value) {
//			core.controls[Core::valueTPSActual] = 0;
////		}
//	}

	value = analogRead(PIN_ANALOG_MAP);
	core.controls[Core::valueMAPRaw] = value; 
	static int mapFailCount = 0;
	if (value > ANALOG_INPUT_HIGH_STATE_LIMIT) {
		if (mapFailCount > SENSOR_FAIL_COUNT) {
			dtc.setError(DTC_MAP_UNCONNECTED);
		 	// Map failback is zero kPa
		 	core.controls[Core::valueBoostPressure] = 0; 
	 	} else {
	 		mapFailCount++;	
	 	}
	} else {
	 	core.controls[Core::valueBoostPressure] = mapValues(core.controls[Core::valueMAPRaw],
	 		core.node[Core::nodeMAPMin].value,
	 		core.node[Core::nodeMAPMax].value);
	 	mapFailCount = 0;
	}

 	core.controls[Core::valueBatteryVoltage] = analogRead(PIN_ANALOG_BATTERY_VOLTAGE);  

	// Log adjuster accuracy for debugging
	core.controls[Core::valueQAJitter] = adjuster.accuracy; 
	core.node[Core::nodeQADebugJitter].value = adjuster.setPointMax - adjuster.setPointMin;
	adjuster.setPointMax=0;
	adjuster.setPointMin=2000;
	
	// Register current injection timing (*10 fixed point)
	core.controls[Core::valueEngineTimingActual] = rpm.getInjectionTiming();
}

/*
	Calculates injection pump advance, controls N108 duty cycle which affects pump internal pressure and thus advance piston position
	- Operating modes 
		0 = off
		1 = open loop control, duty cycle is read from table (injFuel x RPM)
		2 = closed loop control, PID control from target table

*/
void doTimingControl() {
	if (core.controls[Core::valueOutputTestMode] != 0 || core.node[Core::nodeTimingMethod].value == 0) {	
		analogWrite(PIN_PWM_TIMING_SOLENOID,core.controls[Core::valueEngineTimingDutyCycle]);	

		return;
	}

	if (core.node[Core::nodeTimingMethod].value == 2) {
		/* closed loop mode */


		core.controls[Core::valueEngineTimingTarget] = mapLookUp(core.maps[Core::mapIdxClosedLoopAdvanceMap],
			core.controls[Core::valueRPM8bit],
			core.controls[Core::valueFuelAmount8bit]);	

		static int timingKd = 0, timingSpeed=16, timingMin=N108_MIN_DUTY_CYCLE, timingMax=N108_MAX_DUTY_CYCLE;
		static PID timingPidControl(
			(int*)&core.node[Core::nodeTimingKp].value,
			(int*)&core.node[Core::nodeTimingKi].value,
			&timingKd,
			&timingMin,
			&timingMax,
			&timingSpeed,
			NULL,
			(int*)&core.controls[Core::valueEngineTimingActual], // Current timing
			(int*)&core.controls[Core::valueTimingPIDAmount] // output DC
			);

		timingPidControl.setPosition(core.controls[Core::valueEngineTimingTarget]);
		timingPidControl.calculate();

		core.controls[Core::valueEngineTimingDutyCycle] = core.controls[Core::valueTimingPIDAmount];

		// for ui
		core.node[Core::nodeEngineTiming].value = core.controls[Core::valueEngineTimingTarget];
	} else {
		/* open loop mode */
		core.controls[Core::valueEngineTimingDutyCycle] = mapLookUp(core.maps[Core::mapIdxOpenLoopAdvanceMap],
				core.controls[Core::valueRPM8bit],
				core.controls[Core::valueFuelAmount8bit]);			
	}

	analogWrite(PIN_PWM_TIMING_SOLENOID,core.controls[Core::valueEngineTimingDutyCycle]);	
}

/*
	Calculates boost control parameters, controls N75 duty cycle which actuates Turbo VNT vanes
	- Operating modes 
		0 = off
		1 = open loop control, N75 duty cycle is read from table (injFuel x RPM)
		2 = closed loop control, N75 duty cycle is read from table + small amount of PID is applied
		*/

void doBoostControl() {
/*	static SimpleRotatingActuator SRA((int*)&core.node[Core::nodeSRAMinPos].value,
		(int*)&core.node[Core::nodeSRAMaxPos].value,
		(int*)&core.node[Core::nodeSRAInverseOperation].value);
*/
	if (core.controls[Core::valueOutputTestMode] != 0 || core.node[Core::nodeBoostAdjusting].value == 0) {	
	//		SRA.setPosition(core.controls[Core::valueN75DutyCycle]);
	//		SRA.calculate();
		analogWrite(PIN_PWM_BOOST_SOLENOID,core.controls[Core::valueN75DutyCycle]);
		return;
	}
	static int boostMin=0, boostMax=255;
	static PID boostPidControl(
		(int*)&core.node[Core::nodeBoostKp].value,
		(int*)&core.node[Core::nodeBoostKi].value,
		(int*)&core.node[Core::nodeBoostKd].value,
		&boostMin,
		&boostMax,
		(int*)&core.node[Core::nodeBoostSpeed].value,
		(int*)&core.node[Core::nodeBoostBias].value,						
		(int*)&core.controls[Core::valueBoostPressure], // input
		(int*)&core.controls[Core::valueBoostPIDCorrection] // output
		);


	/* Open loop control */


	/* Look up for requested boost level (RPM x TPS) */
	core.controls[Core::valueBoostTarget] = mapLookUp(core.maps[Core::mapIdxTurboTargetPressureMap],
		core.controls[Core::valueRPM8bit],
		core.controls[Core::valueTPSActual]);	

	/* TODO: add check for overboost situations and set up DTC and safe mode */

	core.controls[Core::valueN75DutyCycleBaseForPid] =  mapLookUp(core.maps[Core::mapIdxTurboControlMap],
		core.controls[Core::valueRPM8bit],
		core.controls[Core::valueFuelAmount8bit]);	


	// for ui
	core.node[Core::nodePressure].value = core.controls[Core::valueBoostTarget];

	long int idx;
	unsigned char out;
	core.controls[Core::valueBoostActuatorClipReason] = BOOST_OK;

	static char boostRunCount;
	int joo;
	
	boostRunCount++;

	switch (core.node[Core::nodeBoostAdjusting].value) {
		case 2:
			/* Closed loop control 
			- Duty cycle table predefines max opening of vanes
			- Small amount of pid correction is subtracted of max opening position
			*/

			// Saturate PID control min/max according to fixed duty cycle table	
			// boostMin = 0;//-core.node[Core::nodeBoostPIDRange].value;
			// Fixed DC is the maximum opening
			// boostMax = core.node[Core::nodeBoostPIDRange].value;

			if (boostRunCount % 4 == 0) {
				boostMin = -core.node[Core::nodeBoostPIDRange].value; 
				boostMax = core.node[Core::nodeBoostPIDRange].value; 

				boostPidControl.setPosition(core.controls[Core::valueBoostTarget]);
				boostPidControl.calculate();
				core.controls[Core::valueBoostPIDCorrection] = -core.controls[Core::valueBoostPIDCorrection];
			}

			idx = core.controls[Core::valueN75DutyCycleBaseForPid] + core.controls[Core::valueBoostPIDCorrection];
			if (idx<0) {
				idx = 0;
			}
			if (idx>255) {
				idx = 255;
			}
			out = mapLookUp(core.maps[Core::mapIdxActuatorTension],idx,0); 

			core.controls[Core::valueN75DutyCycle] = out;
			break;
		case 3:			
			// Fixed openin based on control table
			idx = (((long int)core.controls[Core::valueBoostPressure]*100/(long int)core.controls[Core::valueBoostTarget])*256)/100;

			if (idx<0) {
				idx = 0;
			}
			if (idx>255) {
				idx = 255;
			}
			idx = idx;
	   	
		   	core.controls[Core::valueN75DutyCycle] =mapLookUp(core.maps[Core::mapIdxActuatorTension],idx,0); 
		   	break;
		  	case 4:
			// Classic vnt lda style + small PID correction
			idx = idx;

		    idx = (((long int)core.controls[Core::valueBoostPressure]*100/(long int)core.controls[Core::valueBoostTarget])*256)/100;

		    if (boostRunCount % 4 == 0) {
		    	boostMin = -core.node[Core::nodeBoostPIDRange].value; 
		    	boostMax = core.node[Core::nodeBoostPIDRange].value; 

		    	boostPidControl.setPosition(core.controls[Core::valueBoostTarget]);
		    	boostPidControl.calculate();
		    	core.controls[Core::valueBoostPIDCorrection] = -core.controls[Core::valueBoostPIDCorrection];
		    }

		    if (idx<0) {
		    	idx = 0;
		    }
		    if (idx>255) {
		    	idx = 255;
		    }

		    out = 255-mapLookUp(core.maps[Core::mapIdxActuatorTension],idx,0); 

/*
			out = 255-mapLookUp(core.maps[Core::mapIdxActuatorTension],idx,0); 

		   	// scale value
		   	joo = map(out,0,255,0,core.controls[Core::valueN75DutyCycleBaseForPid]);   	
		   	*/
		   	// scale value
		   	joo = map(out,0,255,0,core.controls[Core::valueN75DutyCycleBaseForPid]+core.controls[Core::valueBoostPIDCorrection]);   		   	
		   	core.controls[Core::valueN75DutyCycle] = joo;
		   	break;
		   }

		static unsigned int idleRunCycleCount = 0;


		/* Open vanes when idling */
		if (core.controls[Core::valueTPSActual] == 0 && core.controls[Core::valueEngineRPMFiltered]<1200) {
		   	idleRunCycleCount++;
		   	/* quick test: VNT autoclean */ 
		   	if (VNT_OPEN_DELAY_ON_IDLE != 0 && idleRunCycleCount > VNT_OPEN_DELAY_ON_IDLE ) {
		   		core.controls[Core::valueN75DutyCycle] = 255;
		   	}
		   	if (VNT_OPEN_DELAY_ON_IDLE != 0 && idleRunCycleCount > VNT_OPEN_DELAY_ON_IDLE*10 ) {
		   		core.controls[Core::valueN75DutyCycle] = 0;
		   	}
		   	if (VNT_OPEN_DELAY_ON_IDLE != 0 && idleRunCycleCount > VNT_OPEN_DELAY_ON_IDLE*11 ) {
		   		idleRunCycleCount = 0;
		   	}

		   } 
		   if (core.controls[Core::valueEngineRPMFiltered]>1200 || core.controls[Core::valueTPSActual] > 0) {
		   	idleRunCycleCount = 0;
		   }

/*
	SRA.calculate();	
	SRA.setPosition(core.controls[Core::valueN75DutyCycle]);
	*/

	analogWrite(PIN_PWM_BOOST_SOLENOID,core.controls[Core::valueN75DutyCycle]);

}

void doUserStuff() {
	/* Add your code to here */
}

unsigned char safetyStop = 0;

/* Fast Sensor control. Use only for sensors that need over 60Hz poll rate and/or are related to interrupt services, 
especially if over 8bit variables. 
- QA Control

*/

void refreshFastSensors() {
	int fuelAmount;
	static unsigned int runCount;

	/* call rpm measurement function which converts internal timer state to rotation per minute (and stores it to control structure) */
	rpm.measure();
	
	if (core.controls[Core::valueEngineRPMFiltered] == 0 && core.node[Core::nodeFuelCutAtStall].value == 1) {
		if (runCount > FAST_START_DELAY)
		safetyStop = true;
		if (runCount<255)
		runCount++;
		core.controls[Core::valueRunMode]=0; // Stopped  		
	}

	if (safetyStop) {
		// When engine is not running, do not inject anything to prevent runaway situation when RPM signal is missing and engine is running
		fuelAmount = 0;
		core.controls[Core::valueRunMode]=0; // Stopped  
	} else { 
		safetyStop = false;
		unsigned int rpm;

		cli();
		rpm = core.controls[Core::valueEngineRPMFiltered];
		sei();
		int rpmCorrected = mapValues(rpm,0,core.node[Core::nodeControlMapScaleRPM].value);   

		core.controls[Core::valueRPM8bit] = rpmCorrected;
/*
		if (rpm<350) {
			core.controls[Core::valueRunMode] = 1; // starting 
		} else {
			if (core.controls[Core::valueRunMode]<98)
				core.controls[Core::valueRunMode]++;
				}*/
/*
				if (core.controls[Core::valueRunMode] == 0) {
			// Saturate PID control before start
			// core.controls[Core::valueIdlePIDCorrection] = core.node[Core::nodeIdleMinFuel].value*2;
			}
			*/

	// Use or switch to PID idle control if gas is not pressed and RPM below threshold. When PID is activated, it is switched of only when gas is pressed (no RPM threshold check)
	if (/*(core.controls[Core::valueRunMode] == 2 ||*/  
			/* rpm<(core.node[Core::nodeIdleSpeedTarget].value+400)) && */
		core.controls[Core::valueTPSActual] == 0) {

		rpmIdle = mapValues(rpm,0,1024);

		if (core.node[Core::nodeIdleAdjusting].value && rpm>270) {
			// TODO fix valueRunMode!!
			fuelAmount = mapLookUp10bit(core.maps[Core::mapIdxIdleMap],
				rpmIdle,
				core.controls[Core::valueTPSActual]);	
//			idleMinFuel = core.node[Core::nodeIdleMinFuel].value; // JOOSE
//			idleMaxFuel = core.node[Core::nodeIdleMaxFuel].value;
			core.controls[Core::valueRunMode] = 2; // pid idle 
//			idlePidControl.setPosition(core.node[Core::nodeIdleSpeedTarget].value);
//			idlePidControl.calculate();
			fuelAmount += core.controls[Core::valueIdlePIDCorrection];
			idleLastCalculatedFuelAmount = fuelAmount;
		} else {
			// Use idle & cold start map.
			fuelAmount = mapLookUp10bit(core.maps[Core::mapIdxIdleMap],
						rpmIdle,
						core.controls[Core::valueTPSActual]);
			
			idleLastCalculatedFuelAmount = fuelAmount;
			core.controls[Core::valueRunMode] = 1; // starting 

			// Sets up the integral part of PID calculation
			// idlePidControl.setPositionHint(fuelAmount);
		}					 

	} else {
		// Return upscaled interpolated value of range 0..1023 according to fuel map data (0..255)
		core.controls[Core::valueRunMode]=100; // Engine running 

		// Base injection quantity 
		// TODO::: generate fuelMapLookup which uses upscaled fuel trim values before final interpolation

		core.controls[Core::valueFuelBaseAmount] = mapLookUp10bit(core.maps[Core::mapIdxFuelMap],
			rpmCorrected,
			core.controls[Core::valueTPSActual]);

		/* Limit enrichment amount to by REQUESTED boost level (turbo control table), not the actual */
		unsigned char boostRequlated = core.controls[Core::valueBoostPressure];

		if (boostRequlated > core.controls[Core::valueBoostTarget])
			boostRequlated = core.controls[Core::valueBoostTarget];

		// Enrichment based on boost, amount is TPS% * fuel enrichment map value to smooth apply of enrichment
		core.controls[Core::valueFuelEnrichmentAmount] = 
		((unsigned long)(mapLookUp10bit(core.maps[Core::mapIdxBoostMap],
				rpmCorrected,
				boostRequlated)) 
			*(unsigned  long)(core.controls[Core::valueTPSActual])
			/ (unsigned long)256);

		// Smoke limit map (TODO: add wide band lambda support and auto-map feature)
		//core.controls[Core::valueFuelLimitAmount] = mapLookUp10bit(core.maps[Core::mapIdxMaxFuelMap],
		//		rpmCorrected,
		//		core.controls[Core::valueBoostPressure]);	

		fuelAmount = core.controls[Core::valueFuelBaseAmount];
	
		// Smooth transtion from idle
		if (idleLastCalculatedFuelAmount>fuelAmount) 
			fuelAmount = idleLastCalculatedFuelAmount;

		if (fuelAmount) {
			// Enrichment amount is TPS% * fuel enrichment map value to smooth apply of enrichment
			fuelAmount += core.controls[Core::valueFuelEnrichmentAmount];
		}

		if (fuelAmount>core.controls[Core::valueFuelLimitAmount])	
		fuelAmount = core.controls[Core::valueFuelLimitAmount];		

		}
	}
	core.controls[Core::valueFuelAmount] = fuelAmount;	
	core.controls[Core::valueFuelAmount8bit] = fuelAmount/4;
	adjuster.setPosition(fuelAmount);
}

void doRelayControl() {

}


const char main_pressAKeyString[] PROGMEM = " bytes free.\r\n\r\nPress a key for configuration interface ...";

void setup() {
	//pinMode(PIN_INPUT_RPM,INPUT_PULLUP);
	pinMode(PIN_INPUT_NEEDLELIFTSENSOR,INPUT_PULLUP);

	pinMode(PIN_ANALOG_QA_POS,INPUT);
	pinMode(PIN_ANALOG_TEMP_FUEL,INPUT);
	pinMode(PIN_ANALOG_BATTERY_VOLTAGE,INPUT);
	//pinMode(PIN_ANALOG_UNDEF,INPUT);
	//pinMode(PIN_ANALOG_UNDEF,INPUT);
	pinMode(PIN_ANALOG_SERVO_POS,INPUT);
	pinMode(PIN_ANALOG_LAMBDA,INPUT);
	pinMode(PIN_ANALOG_EGT,INPUT);
	pinMode(PIN_ANALOG_MAP,INPUT);
	pinMode(PIN_ANALOG_TEMP_INTAKE,INPUT);
	pinMode(PIN_ANALOG_TEMP_GEARBOX,INPUT);
	pinMode(PIN_ANALOG_TEMP_COOLANT,INPUT);

	/* TPS Sensor */
	pinMode(PIN_ANALOG_TPS_POS,INPUT);
	pinMode(PIN_INPUT_TPS_WOT_SW,INPUT);  // TODO: swap this with SET_SW (and use pullup ~2.5v (GND=decrease speed, +12v=set/Accel))
	pinMode(PIN_INPUT_TPS_IDLE_SW,INPUT);
	pinMode(PIN_INPUT_BRAKE_SW,INPUT);
	pinMode(PIN_INPUT_CLUTCH_SW,INPUT);
	pinMode(PIN_INPUT_CRUISE_SET_SW,INPUT);

	/* FETs and/or PWM */
	pinMode(PIN_PWM_QA,OUTPUT);
	pinMode(PIN_PWM_TIMING_SOLENOID,OUTPUT);
	pinMode(PIN_PWM_AUX,OUTPUT);
	pinMode(PIN_PWM_BOOST_SOLENOID,OUTPUT);
	pinMode(PIN_PWM_SERVO,OUTPUT);
	pinMode(PIN_PWM_NLS_REF_VOLTAGE,OUTPUT);

	/* servo control */
	pinMode(PIN_SERVO_DIR,OUTPUT);
	pinMode(PIN_INPUT_SERVO_FAULT,INPUT_PULLUP);

	/* relays */
	pinMode(PIN_RELAY_POWER ,OUTPUT);
	pinMode(PIN_RELAY_FUEL_SOLENOID,OUTPUT);
	pinMode(PIN_RELAY_FAN1,OUTPUT);
	pinMode(PIN_RELAY_FAN2,OUTPUT);
	pinMode(PIN_RELAY_ENGINE_GLOW,OUTPUT);
	pinMode(PIN_RELAY_COOLANT_RELAY1,OUTPUT);
	pinMode(PIN_RELAY_COOLANT_RELAY2,OUTPUT);
	pinMode(PIN_RELAY_AUX,OUTPUT);

	/* general outputs */
	pinMode(PIN_TACHO_OUT,OUTPUT);
	pinMode(PIN_GLOW_LIGHT,OUTPUT);

	/* general inputs */
	pinMode(PIN_INPUT_POWER_ON,INPUT);
	pinMode(PIN_INPUT_VSS,INPUT_PULLUP);
	//	analogReference(EXTERNAL); // connect +5 power supply to aref



	//tacho.init();
	/*
	// Setting ADC prescaler to /16 (http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1208715493/11 )
	sbi(ADCSRA,ADPS2) ;
	cbi(ADCSRA,ADPS1) ;
	cbi(ADCSRA,ADPS0) ;
	*/
	Serial.begin(115200);
	ansiClearScreen();
	
	interruptHandlerArray[2].handler=refreshFastSensors; 
	interruptHandlerArray[2].divider=4;
	//interruptHandlerArray[3].handler=refreshSlowSensors; 
	//interruptHandlerArray[3].divider=16;
	
//RPM	attachInterrupt(0, rpmTrigger, RISING);  // Interrupt 0 -- PIN2 -- LM1815 gated output 
//RPM	attachInterrupt(1, needleTrigger,FALLING );  // From voltage comparator, default +5v
	
	// Timer info - see: http://sobisource.com/?p=195

	// Timer 4 base freq -- timing solenoid 0x03 - 500Hz, 0x04 - 125Hz
	TCCR4B = (TCCR4B & 0xF8) | 0x05; // 490Hz was 0x04
	
	// Reference pulse for testing RPM signalling
	//TCCR4B = (TCCR4B & 0xF8) | 0x05;
	//analogWrite(PIN_REFPULSE,128); // TCCR4B - timer 4
	
	Serial.print("BOOT: ");
	if (core.load()) {
		const char *msg="OK";
		confeditor.setSystemStatusMessage(msg);
	} else {
		const char *msg="Conf. error";
		Serial.print(msg);
		confeditor.setSystemStatusMessage(msg);
	}

	setupQATimers();
	rpm.init();

	Serial.print("... ");	
	Serial.print(freeMemory());
	Serial.print(fetchFromFlash(main_pressAKeyString));

	TCCR2B = TCCR2B & (B11111000 | B00000010);    // set timer 2 divisor to     8 for PWM frequency of  3921.16 Hz
	//TCCR2B = TCCR2B & (B11111000 | B00000001);    // set timer 2 divisor to     1 for PWM frequency of 31372.55 Hz

	analogWrite(PIN_PWM_NLS_REF_VOLTAGE,73);

}

void setupQATimers() {
	// 2000 = 500ticks -// was 500
	Timer3.initialize(4000); // in microseconds, also sets PWM base frequency for "mega" pins 5,2,3 // 2000 = old default
	adjuster.initialize();

	Timer3.attachInterrupt(mainInterruptHandler,0);
	interruptHandlerArray[0].handler=refreshQuantityAdjuster; 
	interruptHandlerArray[0].divider=2; 
}

int i;
#define BUFFER_SIZE 64
char buffer[BUFFER_SIZE];
char lastKey;
long time;

boolean confChanged = 0;
char loopCount = 0;


void loop() {


	//boolean printValues = 0;
	boolean ignoreSleep = false;
//	static unsigned char testMapX = 0;
//	static unsigned char testMapY = 0;

	refreshSlowSensors();
	doBoostControl();
	doTimingControl();
	doRelayControl();
	doIdlePidControl();


	if (loopCount == 60) {
		// Log some "short term" differencies
//RPM		core.controls[Core::valueEngineRPMJitter]=rpmMax-rpmMin;
//RPM		rpmMin = core.controls[Core::valueEngineRPM];
//RPM		rpmMax = core.controls[Core::valueEngineRPM];
		loopCount=0;    
	}   
	loopCount++;
	
	lastKey = 0;
	// Read incoming command from serial interface (USB)
	if (Serial.available()>0) {
		char c = Serial.read();
		if (c == 27) {
			c = Serial.read();
			if (c == '[') {
				c = Serial.read();
				// Cursor to "vi" nodes
				switch (c) { 
					case 'A':
					lastKey = KEY_UP;
					break;
					case 'B':
					lastKey = KEY_DOWN;
					break;
					case 'C':
					lastKey = KEY_RIGHT;
					break;
					case 'D':
					lastKey = KEY_LEFT;
					break;
				}
			}
			confeditor.handleInput(lastKey);
			} else {
			// Special commands
			switch (c) {
/*                case 2: // STX
					ignoreSleep = 1;
					i=0;
					int node;
					int val;
					node=-1;
					val=-1;
					while (1) {
						if (Serial.available()) {
							c = Serial.read();
							if (c==3) // ETX
								break;
							if (c==':') {
								buffer[i]=0;           
								node = atoi(buffer);
								i=0;
							} else if (c==';') {
								buffer[i]=0;           
								val = atoi(buffer);
								i=0;
							} else if (i<BUFFER_SIZE-1) {
								buffer[i]=c;
								i++;
							}
						}
					}
					buffer[i]=0;
					int cksum;
					cksum=atoi(buffer);    
									  
					if (cksum == 66 && node == 255) {
						core.save();
					}       
					if (cksum == 66 && node > 0 && node <= Core::KEY_MAX) {
						core.setCurrentNode(node);
						core.setValue(val);
						confChanged=1;
						//if (node == Core::nodeQAPWMBase || node == Core::nodeQATimerInterval)
						  //§  setupQATimers();
					}
					break;
					*/
					default:
					confeditor.handleInput(c);
				}
			}
		}

	/*
	if (printValues) {
		// print one packet of internal status (RPM,TPS, QA SetPoint, QA Position)
		Serial.write(2); // stx
		Serial.print("0,"); // Type=0 packet
		Serial.print(core.controls[Core::valueEngineRPM],DEC); // RPM
		Serial.write(',');
		Serial.print(core.controls[Core::valueTPSActual],DEC);
		Serial.write(',');
		Serial.print(core.controls[Core::valueQAfeedbackSetpoint],DEC);
		Serial.write(',');
		Serial.print(core.controls[Core::valueQAfeedbackRaw],DEC);
		Serial.write(',');
		Serial.print(core.controls[Core::valueQAPWMActual],DEC);
		Serial.write(',');
		Serial.print((int)adjuster.p,DEC); 
		Serial.write(',');
		Serial.print((int)adjuster.i,DEC); 
		Serial.write(',');
		Serial.print((int)adjuster.d,DEC);        
		Serial.write(',');
		Serial.print(core.controls[Core::valueEngineTimingActual],DEC);    
		Serial.write(3); // etx;
		printValues = false;
		}*/

	confeditor.refresh();

	if (!ignoreSleep) {
		static int debug;
		debug = !debug;
		digitalWrite(13,debug);
		delay(1000/60);
	} 

	// Handle errors (generated by interrupt service)
//RPM
/*	
	if (unplausibleRpm) {
		dtc.setError(DTC_RPM_UNPLAUSIBLE_SIGNAL);
		unplausibleRpm = 0;
		// TODO: switch over backup signal (needlelift) after nnn failing signals  
	}
	if (unplausibleNeedleLiftSensor) {
		//dtc.setError(DTC_NEEDLESENSOR_UNPLAUSIBLE_SIGNAL);
		unplausibleNeedleLiftSensor = 0;
	}
	*/

	// Saves any DTC codes generated .. 
	dtc.save();

}


/*
 Useful data :-)
 
 http://arduino.cc/en/Hacking/PinMapping2560
 
 Phase correct mode?
 
 Most Arduino boards have two external interrupts: numbers 0 (on digital pin 2) and 1 (on digital pin 3). 
 The Arduino Mega has an additional four: numbers 2 (pin 21), 3 (pin 20), 4 (pin 19), and 5 (pin 18).
 
 Mega timers: http://sobisource.com/?p=195
 
 */

