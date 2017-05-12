#include "RPMDefaultCPS.h"


/* 
 *  Some low level functions for RPM counting (and also for injection timing measurement) 
 *  
 *  Timer 1 is set to 0 when RPM interrupt happens and old Timer 1 value is stored. 
 *  This value can be directly converted for revolutions per minute. 
 *  If timer reaches 65535, overflow interrupt handler is activated and it engine is 
 *  declared as stopped.
 *  
 *  Injection timing is also recorded. Because the mark on the flywheel is on a known position, the
 *  counter value can be converted to actual degree of advance/retard
 *  
 *  Timer 1 frequency (250KHz) is optimized for 4 ... 6 marks on flywheel (same as number of cylinders, usually) 
 *  Minimum rotating speed detected is about 57RPM (4cyl engine)
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
 	TCCR1B = 11; // WGM12 = 8 +CS11 = 2+CS10 = 1
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


void rpmTrigger() { 
	cli();
	rpmDuration = TCNT1;
	sei();
	rpmTimerEnable();


	//attachInterrupt(1, needleTrigger, FALLING);
	// Skip first interrupt (interrupt pending flag is not cleared by attachIntterrupt?? )
	//skipInjectionTrigger = 2; 
	//injectionBegin = 0;

}

/*
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

// Class methods

void RPMDefaultCps::init() {
 	rpmDuration = 0;
	rpmTimerSetup(); 
	rpmTimerEnable();		
}

unsigned int RPMDefaultCps::getLatestMeasure() {
	if (rpmDuration==0)
		return 0;
	// 64 = timer divider
	return ((unsigned long)(60*F_CPU/64/NUMBER_OF_CYLINDERS)/((unsigned long)rpmDuration)); 

}
unsigned int RPMDefaultCps::getLatestRawValue() {
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
