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

Pump head wires
1-pot signal
2-pot +
3-pot -


5-ftemp -
6-ftemp + (2300ohm 23C)

Todo:
- sync qa pos reading with rpm sensor on runmode 100 ?? -- problems with analogRead on -> todo
- or smooth qa signal with RC circuit? -- maybe better
- serial line errors?
- log injection differences on rpm

- cap mod rpm + stronger pullup
- qa feedback unstable on middle range ? worn out? try faster pwm?
- idle pid to be negative too

*/
#include "RPMDefaultCPS.h"
//#include "RPMCustomCps.h"
#include "ConfEditor.h"
#include "Core.h"
#include "QuantityAdjuster.h"
#include "TimerThree.h"
#include "utils.h"
#include "DTC.h"
#include "defines.h"
#include "MemoryFree.h"
#include "PID.h"
#include "TachoOut.h"
#include "BackgroundADC.h"

extern volatile long rpmMax;
extern volatile long rpmMin;

// Crankshaft position sensor decoding, use RPMDefaultCps or RPMHiDensityCps 
static RPMDefaultCps rpm;

// VP37 Quantity adjuster module
static QuantityAdjuster adjuster;

//TachoOut tacho;

// Periodic routines called by 2500Hz (or less, set divider to 2 or greater)
#define interruptHandlerMax 8

volatile struct interruptHandlerStruct { 
	void (*handler)();
	unsigned char divider;
} interruptHandlerArray[interruptHandlerMax];

volatile unsigned char shortTicks=0;
volatile unsigned int intHandlerCalls;

// Called when Timer3 overflow occurs. Then calls handler routines according to their divider value
void mainInterruptHandler() {
	intHandlerCalls++;
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
	intHandlerCalls--;
} 

void refreshQuantityAdjuster() {
	adjuster.update();
}


static int rpmIdle;
volatile static char calls=0;

void doIdlePidControl() {
	if (core.node[Core::nodeIdleAdjusting].value == 0) {
		core.controls[Core::valueIdlePIDCorrection] = 0;
		return;
	}
	
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

	if (core.controls[Core::valueRunMode] < ENGINE_STATE_PID_IDLE) {
		// do not process pid if not yet running (overshoot on initial injection quantity)
		idlePidControl.reset();	
		core.controls[Core::valueIdlePIDCorrection] = 0;
		return;
	}

	if (core.controls[Core::valueEngineRPMFiltered] == 0)
		idlePidControl.reset();		


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
	value = adc.readValueAvarage(PIN_ANALOG_TEMP_COOLANT);
	if (value > ANALOG_INPUT_HIGH_STATE_LIMIT) {
		// generate error only if map is configured and sensor reading is not present
		dtc.setError(DTC_ENGINE_TEMP_UNCONNECTED);
		core.controls[Core::valueTempEngineRaw] = 512; // use configuration setpoint value sensor's failback substitute value
	} else {
		cli();
		core.controls[Core::valueTempEngineRaw] = value;
		sei();
	}
	
	// Fuel TEMP
	value = adc.readValueAvarage(PIN_ANALOG_TEMP_FUEL);
	if (value > ANALOG_INPUT_HIGH_STATE_LIMIT) {
		// generate error only if map is configured and sensor reading is not present
		dtc.setError(DTC_FUEL_TEMP_UNCONNECTED);
		core.controls[Core::valueTempFuelRaw] = 512; // use configuration setpoint value sensor's failback substitute value
	} else {
		cli();
		core.controls[Core::valueTempFuelRaw] = value;
		sei();
	}

	// Air TEMP
	value = adc.readValueAvarage(PIN_ANALOG_TEMP_INTAKE);
	if (value > ANALOG_INPUT_HIGH_STATE_LIMIT) {
		// generate error only if map is configured and sensor reading is not present
		dtc.setError(DTC_AIR_TEMP_UNCONNECTED);
		core.controls[Core::valueTempIntakeRaw] = 512; // use configuration setpoint value sensor's failback substitute value
	} else {
		cli();
		core.controls[Core::valueTempIntakeRaw] = value;
		sei();
	}

	// Gearbox TEMP
//	value = adc.readValueAvarage(PIN_ANALOG_TEMP_GEARBOX);
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
	value = adc.readValueAvarage(PIN_ANALOG_TPS_POS);
	core.controls[Core::valueTPSRaw]=value;

	// Check TPS it is connected, otherwise apply limp mode amount (about 15%) 
	if (value > ANALOG_INPUT_HIGH_STATE_LIMIT) {
		dtc.setError(DTC_TPS_UNCONNECTED);
		core.controls[Core::valueTPSActual] = TPS_LIMP_MODE_AMOUNT; 
	} else {
		scaledValue = mapValues(core.controls[Core::valueTPSRaw],
				core.node[Core::nodeTPSMin].value,
				core.node[Core::nodeTPSMax].value);
		cli();
		core.controls[Core::valueTPSActual] = scaledValue; 
		sei();

	}

	/* override for gas pedal idle position TODO!*/
//	if ((core.node[Core::nodeTPSSafetyBits].value & TPS_SAFETY_BITS_IDLESW) == TPS_SAFETY_BITS_IDLESW) {
//		value = digitalRead(PIN_TPS_IDLE);
//		if (!value) {
//			core.controls[Core::valueTPSActual] = 0;
////		}
//	}

	value = adc.readValueAvarage(PIN_ANALOG_MAP);
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
		unsigned int res = mapValues(core.controls[Core::valueMAPRaw],
	 		core.node[Core::nodeMAPMin].value,
	 		core.node[Core::nodeMAPMax].value);
		cli();
		core.controls[Core::valueBoostPressure] = res;
		sei();
	 	mapFailCount = 0;
	}

 	core.controls[Core::valueBatteryVoltage] = adc.readValueAvarage(PIN_ANALOG_BATTERY_VOLTAGE);

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
			analogWrite(PIN_PWM_TIMING_SOLENOID,0);					
		return;
	}
	if (core.node[Core::nodeTimingMethod].value == 1) {
		if (core.controls[Core::valueRunMode]>=ENGINE_STATE_IDLE) {
			core.controls[Core::valueEngineTimingDutyCycle] = mapLookUp(core.maps[Core::mapIdxOpenLoopAdvanceMap],
					core.controls[Core::valueRPM8bit],
					core.controls[Core::valueFuelAmount8bit]);				
			analogWrite(PIN_PWM_TIMING_SOLENOID,core.controls[Core::valueEngineTimingDutyCycle]);	
		} else {
			analogWrite(PIN_PWM_TIMING_SOLENOID,0);				
			core.controls[Core::valueEngineTimingDutyCycle]=0;
		}	
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
		analogWrite(PIN_PWM_TIMING_SOLENOID,core.controls[Core::valueEngineTimingDutyCycle]);	

		return;
	} 

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
		// Usually n75 is responsive only over certain PWM range, so remap the 0..255 value 

		analogWrite(PIN_PWM_BOOST_SOLENOID,core.controls[Core::valueN75DutyCycle]);
		return;
	}
	static int boostMin=0, boostMax=128;
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
	unsigned char res = mapLookUp(core.maps[Core::mapIdxTurboTargetPressureMap],
		core.controls[Core::valueRPM8bit],
		core.controls[Core::valueTPSActual]);	
	cli();
	core.controls[Core::valueBoostTarget] = res;
	sei();

	/* TODO: add check for overboost situations and set up DTC and safe mode */

	core.controls[Core::valueBoostValveDutyCycle] =  mapLookUp(core.maps[Core::mapIdxTurboControlMap],
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

			boostMin = -core.node[Core::nodeBoostPIDRange].value; 
			boostMax = core.node[Core::nodeBoostPIDRange].value; 

			boostPidControl.setPosition(core.controls[Core::valueBoostTarget]);
			boostPidControl.calculate();
//				core.controls[Core::valueBoostPIDCorrection] = -core.controls[Core::valueBoostPIDCorrection];
			core.controls[Core::valueBoostPIDComponentP] = boostPidControl.lastP;
			core.controls[Core::valueBoostPIDComponentI] = boostPidControl.lastI;
			core.controls[Core::valueBoostPIDComponentD] = boostPidControl.lastD;			

			idx = core.controls[Core::valueBoostValveDutyCycle] + core.controls[Core::valueBoostPIDCorrection];
			if (idx<0) {
				idx = 0;
			}
			if (idx>255) {
				idx = 255;
			}
			core.controls[Core::valueBoostCalculatedAmount] = idx;
			
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
	   	
		   	core.controls[Core::valueN75DutyCycle] = mapLookUp(core.maps[Core::mapIdxActuatorTension],idx,0); 
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
//		    	core.controls[Core::valueBoostPIDCorrection] = -core.controls[Core::valueBoostPIDCorrection];

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
		   	joo = map(out,0,255,0,core.controls[Core::valueBoostValveDutyCycle]);   	
		   	*/
		   	// scale value
		   	joo = map(out,0,255,0,core.controls[Core::valueBoostValveDutyCycle]+core.controls[Core::valueBoostPIDCorrection]);   		   	
		   	core.controls[Core::valueN75DutyCycle] = joo;
		   	break;
		   }

//		static unsigned int idleRunCycleCount = 0;


		/* Open vanes when idling */
/*		if (core.controls[Core::valueTPSActual] == 0 && core.controls[Core::valueEngineRPMFiltered]<1200) {
		   	idleRunCycleCount++;
		   	// quick test: VNT autoclean 
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
*/
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
	int fuelAmount=0;
	// --------------------------------------------------------------
	// All 16bit variables should be encapsulated with cli()/sei() if
	// updated outside of interrupt handler
	// --------------------------------------------------------------

	/* call rpm measurement function which converts internal timer state to rotation per minute (and stores it to control structure) */
	rpm.measure();	

	if (core.controls[Core::valueEngineRPMFiltered] == 0) {
		core.controls[Core::valueRunMode]=ENGINE_STATE_STOPPED;
	}
	if (core.controls[Core::valueRunMode] == ENGINE_STATE_STOPPED && 
		core.controls[Core::valueBatteryVoltage] == 0) {
		// reset PID control when power is off/
		adjuster.reset();
	}

	if (core.controls[Core::valueEngineRPMFiltered] == 0 && core.node[Core::nodeFuelCutAtStall].value == 1) {
		core.controls[Core::valueRunMode]=ENGINE_STATE_STOPPED; // Stopped  		
		// fuelAmount = 0;

		fuelAmount = core.node[Core::nodeInitialInjectionQuantity].value;
	} else {
		int rpmCorrected = mapValues(core.controls[Core::valueEngineRPMFiltered],0,core.node[Core::nodeControlMapScaleRPM].value);   
		core.controls[Core::valueRPM8bit] = rpmCorrected;

		// TODO Use idle pid control map or basic map, depending which one is larger to provide smooth transition 
		// TODO May cause bad engine braking, check it out
		rpmIdle = mapValues(core.controls[Core::valueEngineRPMFiltered],0,1024);
		int amountPIDIdle = mapLookUp10bit(core.maps[Core::mapIdxIdleMap],
			rpmIdle,
			core.controls[Core::valueTempEngine]);

		if (core.controls[Core::valueEngineRPMFiltered]<=ENGINE_RPM_CRANKING_LIMIT) {
			core.controls[Core::valueRunMode]=ENGINE_STATE_CRANKING;			
		} 

		rpmCorrected = mapValues(core.controls[Core::valueEngineRPMFiltered],0,core.node[Core::nodeControlMapScaleRPM].value);   
		core.controls[Core::valueRPM8bit] = rpmCorrected;

		int amountBase = mapLookUp10bit(core.maps[Core::mapIdxFuelMap],
			rpmCorrected,
			core.controls[Core::valueTPSActual]);

		if (amountBase>amountPIDIdle) {
			core.controls[Core::valueFuelBaseAmount] = amountBase;
			core.controls[Core::valueRunMode] = ENGINE_STATE_LOW_RPM_RANGE;
		} else {
			core.controls[Core::valueFuelBaseAmount] = amountPIDIdle;
			core.controls[Core::valueRunMode] = ENGINE_STATE_PID_IDLE;
		}

	//	if (core.controls[Core::valueEngineRPMFiltered]<=ENGINE_RPM_CRANKING_LIMIT) {
	//		core.controls[Core::valueRunMode]=ENGINE_STATE_CRANKING;			
	//	} 


		// Smooth transition to PID Idle to engine coast
		if (core.controls[Core::valueTPSActual] == 0) {
			unsigned int coastAmount = mapLookUp10bit(core.maps[Core::mapIdxCoastingFuelLimit],rpmCorrected,0);
			if (core.controls[Core::valueFuelBaseAmount]>coastAmount)
				core.controls[Core::valueFuelBaseAmount] = coastAmount;
		}

		// Limit enrichment amount to by REQUESTED boost level (turbo control table), not the actual 
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

		fuelAmount = core.controls[Core::valueFuelBaseAmount];

		if (fuelAmount) {
			// Enrichment amount is TPS% * fuel enrichment map value to smooth apply of enrichment
			fuelAmount += core.controls[Core::valueFuelEnrichmentAmount];
		}
	
	}

	if (core.node[Core::nodeFuelMapSmoothness].value>0) {
		float input = fuelAmount;
		float output = core.controls[Core::valueFuelAmount];
	    output += (input-output) * ((float)(100-core.node[Core::nodeFuelMapSmoothness].value)/100.0);
		core.controls[Core::valueFuelAmount] = output;	
	} else {		
		core.controls[Core::valueFuelAmount] = fuelAmount;	
	}

	if (core.controls[Core::valueRunMode] != ENGINE_STATE_STOPPED)
		core.controls[Core::valueFuelAmount] += core.controls[Core::valueIdlePIDCorrection];
	core.controls[Core::valueFuelAmount8bit] = core.controls[Core::valueFuelAmount]/4;

//	if (fuelAmount>core.node[Core::nodeMaximalInjectionQuantity].value)	
//		fuelAmount = core.node[Core::nodeMaximalInjectionQuantity].value;		


	adjuster.setPosition(core.controls[Core::valueFuelAmount]);

}

static unsigned char halfSeconds = 0;

void doRelayControl() {


	if (core.controls[Core::valueOutputTestMode] == 0) {
		// if not in output test mode, performn normal operation
		if (core.controls[Core::valueEngineRPM] == 0) {
			unsigned char len = mapLookUp(core.maps[Core::mapIdxGlowPeriodMap],core.controls[Core::valueTempEngine],0);
			if (halfSeconds<=len) {
				int step = 500+((int)len-(int)halfSeconds)*500;
				tacho.setRpm(step);		
				core.controls[Core::valueOutputGlow] = true;							
			} else {
				core.controls[Core::valueOutputGlow] = false;		
				tacho.setRpm(0);			
			}
		} else {
			tacho.setRpm(core.controls[Core::valueEngineRPM]);
	//		digitalWrite(PIN_RELAY_ENGINE_GLOW,LOW);		
		}
		
		if (core.controls[Core::valueFan1State]) {
			if (core.controls[Core::valueTempIntake] <= core.node[Core::nodeFan1SwitchOnTemp].value-10) {
				core.controls[Core::valueFan1State] = 0;
			} else {
				core.controls[Core::valueFan1State] = 1;
			}
		} else {
			if (core.controls[Core::valueTempIntake] >= core.node[Core::nodeFan1SwitchOnTemp].value) {
				core.controls[Core::valueFan1State] = 1;
			} else {
				core.controls[Core::valueFan1State] = 0;
			}
		}
	} else {
		tacho.setRpm(2500);
	}
	digitalWrite(PIN_RELAY_ENGINE_GLOW,core.controls[Core::valueOutputGlow]);
	digitalWrite(PIN_RELAY_FAN1,core.controls[Core::valueFan1State]);	

	unsigned char emulatedOutput = mapLookUp(core.maps[Core::mapIdxtempSenderMap],core.controls[Core::valueTempEngine],0);	
	analogWrite(PIN_PWM_TEMP_SENDER,emulatedOutput);
}


const char main_pressAKeyString[] PROGMEM = " bytes free.\r\n\r\nPress a key for configuration interface ...";

void setup() {
	pinMode(PIN_INPUT_RPM,INPUT);
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
	pinMode(PIN_PWM_TEMP_SENDER,OUTPUT);

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
	pinMode(PIN_RPM_PROBE,OUTPUT);

	/* general inputs */
	pinMode(PIN_INPUT_POWER_ON,INPUT);
	pinMode(PIN_INPUT_VSS,INPUT_PULLUP);
	//	analogReference(EXTERNAL); // connect +5 power supply to aref




	tacho.init();

	/*
	// Setting ADC prescaler to /16 (http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1208715493/11 )
	sbi(ADCSRA,ADPS2) ;
	cbi(ADCSRA,ADPS1) ;
	cbi(ADCSRA,ADPS0) ;
	*/
	Serial.begin(115200);
	ansiClearScreen();

	interruptHandlerArray[1].handler=refreshFastSensors; 
	interruptHandlerArray[1].divider=4;



//	interruptHandlerArray[3].handler=refreshSlowSensors; 
//	interruptHandlerArray[3].divider=32;
	
//	attachInterrupt(0, rpmTrigger, RISING);  // Interrupt 0 -- PIN2 -- LM1815 gated output 
//	attachInterrupt(1, needleTrigger,FALLING );  // From voltage comparator, default +5v
	
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
	adc.init();

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

void edcConfSendMessage(char *key, char *value) {
	unsigned char checksum = 0;
	Serial.write(0x02);
	for (unsigned char i=0;i<strlen(key);i++) {
		checksum = (checksum<<1) ^ key[i];
		Serial.write(key[i]);
	}
	Serial.write(':');
	for (unsigned char i=0;i<strlen(value);i++) {
		checksum = (checksum<<1) ^ value[i];
		Serial.write(value[i]);
	}
	Serial.write(0x1f);
	Serial.write(checksum);
	Serial.write(0x03);
}

void edcConfSendStatus(unsigned char id,int val) {
	char buf1[4];
	char buf2[7];
    itoa(id,buf1,10);
    itoa(val,buf2,10);
	edcConfSendMessage(buf1,buf2);
}

int i;
#define BUFFER_SIZE 64
char buffer[BUFFER_SIZE];
char lastKey;
long time;

boolean confChanged = 0;
char loopCount = 0;
boolean ecdConfEnabled = 0;

void setup_old2() {
	Serial.begin(115200);
	Serial.write(0x02);
	Serial.print("_RDY:dmn-edc 1.0");
	Serial.write(0x03);
	Serial.print("edcConf enabled");						
	Serial.flush();
}
void loop_old2() {
	edcConfSendStatus(42,6666);
}
void loop() {
	boolean ignoreSleep = false;

	// test only
	// refreshFastSensors();
	
	refreshSlowSensors();
	doBoostControl();
	doTimingControl();

//	static int rpmMin,rpmMax;
	if (loopCount % 30 == 0) {	
		if (halfSeconds<255) 
			halfSeconds++;
	}
	if (loopCount % 2 == 0) {
		doIdlePidControl();
	}
	
	if (loopCount % 2 == 0) {
		doRelayControl();
	}
	if (loopCount == 60) {
		// Log some "short term" differencies
		unsigned int a=rpmMax;
		unsigned int b=rpmMin;
		core.controls[Core::valueEngineRPMMin]=rpmMin;
		core.controls[Core::valueEngineRPMMax]=rpmMax;

		core.controls[Core::valueEngineRPMJitter]=a-b;
		core.controls[Core::valueEngineRPMErrors]=rpm.errorCount;		
		cli();
		rpmMin = 0xffff;
		rpmMax = 0;
		sei();
		loopCount=0;    

		int ret = tempSensorBcoefficientCalc(
			core.controls[Core::valueTempEngineRaw],
			core.node[Core::nodeEngineTempSensorBcoefficient].value,
			core.node[Core::nodeEngineTempSensorNResistance].value,
			core.node[Core::nodeEngineTempSensorNTemp].value
			);
		core.controls[Core::valueTempEngine] = ret;

		ret = tempSensorBcoefficientCalc(
			core.controls[Core::valueTempFuelRaw],
			core.node[Core::nodeFuelTempSensorBcoefficient].value,
			core.node[Core::nodeFuelTempSensorNResistance].value,
			core.node[Core::nodeFuelTempSensorNTemp].value
			);
		core.controls[Core::valueTempFuel] = ret;

		ret = tempSensorBcoefficientCalc(
			core.controls[Core::valueTempIntakeRaw],
			core.node[Core::nodeIntakeTempSensorBcoefficient].value,
			core.node[Core::nodeIntakeTempSensorNResistance].value,
			core.node[Core::nodeIntakeTempSensorNTemp].value
			);
		core.controls[Core::valueTempIntake] = ret;				
	}  
/*	if (core.controls[Core::valueEngineRPM]<rpmMin) 
		rpmMin = core.controls[Core::valueEngineRPM];
	if (core.controls[Core::valueEngineRPM]>rpmMax) 
		rpmMax = core.controls[Core::valueEngineRPM];
*/
	
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
                case 2: // STX
					while (1) {
						
						if (Serial.available()) {
							c = Serial.read();
							if (c==3) // ETX
								break;
							if (i<BUFFER_SIZE-1) {
								buffer[i]=c;
								i++;
							}
						}
					}
					buffer[i]=0;
					buffer[4]=0;					
					if (strcmp(buffer,"_INI") == 0) {
						// EDC Configurator enabled
						ecdConfEnabled = true;
						edcConfSendMessage("_RDY","dmn-edc 1.0");
						Serial.print("edcConf enabled");						
					}

				break;
				default:
					confeditor.handleInput(c);
			}
		}
	}

	if (ecdConfEnabled) {
		unsigned long int l = millis();
		l += 16;
		do { 
			
			edcConfSendStatus(EDCCONF_RPM,core.controls[Core::valueEngineRPM]);
			edcConfSendStatus(EDCCONF_TPS,core.controls[Core::valueTPSActual]);
			edcConfSendStatus(EDCCONF_QA_SETPOINT,core.controls[Core::valueQAfeedbackSetpoint]);
			edcConfSendStatus(EDCCONF_QA_ACTUAL,core.controls[Core::valueQAfeedbackActual]);
			edcConfSendStatus(EDCCONF_MAP_SETPOINT,core.controls[Core::valueBoostPressure]);
			edcConfSendStatus(EDCCONF_MAP_ACTUAL,core.controls[Core::valueBoostTarget]);		
			edcConfSendStatus(EDCCONF_QA_PID_P,adjuster.p);
			edcConfSendStatus(EDCCONF_QA_PID_P,adjuster.i);
			edcConfSendStatus(EDCCONF_QA_PID_P,adjuster.d);

			edcConfSendStatus(EDCCONF_TEMP_COOLANT,core.controls[Core::valueTempEngine]);
			edcConfSendStatus(EDCCONF_TEMP_INTAKE,core.controls[Core::valueTempIntake]);
			edcConfSendStatus(EDCCONF_TEMP_FUEL,core.controls[Core::valueTempFuel]);
			
			//edcConfSendStatus(42,6666);
			
		} while (millis() <= l);
		
	} else {
		delay(1000/60);
	}
	static int debug;
	debug = !debug;
	digitalWrite(13,debug);
	confeditor.refresh();


	// Handle errors (generated by interrupt service)
//	if (rpm.getError()) {
//		dtc.setError(DTC_RPM_UNPLAUSIBLE_SIGNAL);
		// TODO: switch over backup signal (needlelift) after nnn failing signals  
//	}
	/*
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

