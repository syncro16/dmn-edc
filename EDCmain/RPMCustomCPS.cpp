#include "defines.h"
#include "RPMCustomCPS.h"

#ifdef RPM_SENSOR_TYPE_CUSTOM

/* 
 *  Some low level functions for RPM counting 
 *  
 *  This counter is optimized for 144 teeth / rotation
 *  
 *  RPM value is calculated from 144 / NO_OF_CYLINDER samples for stable output.
 *  
 *  TODO
 *  
*/

volatile unsigned int rpmDuration;

static inline void rpmTimerSetup() __attribute__((always_inline));
static inline void rpmTimerEnable() __attribute__((always_inline));
static inline void rpmTimerDisable() __attribute__((always_inline));
void rpmTrigger();

static inline void rpmTimerSetup()  {
	cli();
	TCCR1A = 0; 
	TCCR1B = 0; 
	TCNT1 = 0;
	OCR1A = 0xffff;

 	TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt
 	sei();
	attachInterrupt(0, rpmTrigger, RISING);  // Interrupt 0 -- PIN2 -- LM1815 gated output 

 }

 static inline void rpmTimerEnable() {
 	cli();
 	TCNT1 = 0;
 	TCCR1B = 9; // WGM12 = 8 + CS10 = 1  - 16MHz clock

 	sei();
 }

 static void rpmTimerDisable() {
 	cli();
 	TCCR1A = 0;
 	TCCR1B = 0;
 	sei();
 	rpmDuration = 0;
 }
 
ISR(TIMER1_COMPA_vect) 
{	
	// Timer has waited rotation for too long, declare engine as stopped
	rpmTimerDisable();
	core.controls[Core::valueEngineRPM] = 0;
	core.node[Core::nodeEngineRPM].value = 0;
	core.controls[Core::valueEngineRPMFiltered] = 0;   
}


unsigned char volatile skipInjectionTrigger;


void rpmTrigger() { 
	static unsigned char idx=0;;
	cli();
	rpmDuration = TCNT1;
	rpmTimerEnable();	
	rawValues[idx]=rpmDuration;
	sei();
	idx++;
	if (idx>RPM_TEETH_PER_CYL)
		idx=0;

}

// Class methods

void RPMCustomCPS::init() {
 	rpmDuration = 0;
	rpmTimerSetup(); 
	rpmTimerEnable();		
}

unsigned int RPMCustomCPS::getLatestMeasure() {
	if (rpmDuration==0)
		return 0;
	/* do not process values on rawValues directly because they may get changed (interrupt routine)*/
	cli();
	memcpy(storedValues,rawValues,RPM_TEETH_PER_CYL*2);
	sei();

	/* Return an avarage value of one cylinder cycle  */
	unsigned long duration=0;
	for (unsigned char i=0;i<RPM_TEETH_PER_CYL;i++) {
		duration += (unsigned long)storedValues[i];	
	}
	duration = duration/RPM_TEETH_PER_CYL; 
	// 64 = timer divider
	return ((unsigned long)(60*F_CPU/RPM_TEETH_COUNT)/((unsigned long)duration)); 

}
unsigned int RPMCustomCPS::getLatestRawValue() {
	return rpmDuration;
}




/*




//((16000000/64)/65536)*60

#define RPMTIMER_COUNTER TCNT1

// TODO Set scaler to /16 when running and to /64 when starting !! (http://www.engblaze.com/microcontroller-tutorial-avr-and-arduino-timer-interrupts/)

volatile int rpmStore[NUMBER_OF_CYLINDERS*2]; 

 ISR(TIMER1_COMPA_vect) 
 {	
	// Timer has waited rotation for too long, declare engine as stopped
	rpmTimerDisable();
	core.controls[Core::valueEngineRPM] = 0;
	core.node[Core::nodeEngineRPM].value = 0;
	core.controls[Core::valueEngineRPMFiltered] = 0;   
}

*/




/*
volatile unsigned long injectionBegin;
volatile char unplausibleRpm = 0;
volatile char unplausibleNeedleLiftSensor = 0;
volatile unsigned long rpmMin;
volatile unsigned long rpmMax;
volatile int skipNextTeeth = 4;
volatile unsigned int rpmDuration;
volatile unsigned char skipInjectionTrigger;

void rpmTrigger() { 
	static unsigned long int previousTeethDuration = 0;  
	unsigned int rpmCalculated;     
	cli();
	if (skipNextTeeth > 0) {
		skipNextTeeth--;
		sei();
		return;
	}     
	rpmDuration = RPMTIMER_COUNTER;
	rpmTimerEnable();

	if (rpmDuration == 0xffff)
	rpmCalculated=0;
  	//sei();

  	// combine duration from last two teeth 
  	unsigned long int t=(rpmDuration+previousTeethDuration)/2;
  	rpmCalculated = RPMTIMER_DURATION_TO_RPM(t);

	// Log unplausible signals. 
	if (rpmDuration<RPMTIMER_MIN_DURATON) {
		skipNextTeeth=1; // do not trust next teeth, in case of error
		unplausibleRpm = 1;
		sei();
		return;
	} 
	previosTeethDuration = rpmDuration;	
	core.node[Core::nodeEngineRPM].value = rpmCalculated;
	
//  	cli();
	core.controls[Core::valueEngineRPM] = rpmCalculated;
 //	sei();

  core.controls[Core::valueEngineRPMDurationBetweenTeeths] = rpmDuration;


	// TODO, combine two teeths to one value
	core.controls[Core::valueCurrentTeeth]++;
	if (core.controls[Core::valueCurrentTeeth]>=NUMBER_OF_CYLINDERS)
	core.controls[Core::valueCurrentTeeth] = 0;
	rpmStore[core.controls[Core::valueCurrentTeeth]] = rpmCalculated;
	
	unsigned long int avg = rpmCalculated;

	// Always force per teeth calculation when starting to maintain better engine control, when critical point (700rpm) is reached
	// RPM can be avaraged

	if (avg>600) {
		unsigned char teeth=rpmStore[core.controls[Core::valueCurrentTeeth]];

		switch (core.node[Core::nodeRPMDSP].value) {
			case 2: // rpm based on last two teeths
			avg = rpmStore[teeth];
			if (teeth == 0) {
				teeth = (NUMBER_OF_CYLINDERS*2)-1;
				} else {
					teeth--;
				}
				avg += rpmStore[teeth];
				avg = avg / 2;
				break;
			case 1: // return avarage rpm per last full turn
			avg = 0;
			for (unsigned char i=0;i<NUMBER_OF_CYLINDERS;i++) {
				avg += rpmStore[i];
			}

			avg = avg/NUMBER_OF_CYLINDERS;
			break;
		}
	}
	//cli();
	core.controls[Core::valueEngineRPMFiltered] = avg;
//	sei();
	if (avg>rpmMax) {
		rpmMax = avg;
		core.controls[Core::valueEngineRPMMax] = avg;
	} else if (avg<rpmMin) {
		rpmMin = avg;
		core.controls[Core::valueEngineRPMMin] = avg;
	}
	
	attachInterrupt(1, needleTrigger, FALLING);
	// Skip first interrupt (interrupt pending flag is not cleared by attachIntterrupt?? )
	skipInjectionTrigger = 2; 
	injectionBegin = 0;

	sei();
	safetyStop=false;
}


///Stores timer values when injection begins;

void needleTrigger() { 
	cli();
	skipInjectionTrigger--;

	if (skipInjectionTrigger) {
		sei();
		return;
	}
	injectionBegin = RPMTIMER_COUNTER; 
	detachInterrupt(1); // disable interrupt handler for a while

	if ((unsigned int)core.controls[Core::valueEngineTimingDiff]<32768) {
		core.controls[Core::valueEngineTimingDiff] = (core.controls[Core::valueEngineTimingDiff]+injectionBegin)/2; 
		} else {
			core.controls[Core::valueEngineTimingDiff] = injectionBegin;
		}
		core.node[Core::nodeHeartBeat].value++;

	sei();
}

*/
#endif
