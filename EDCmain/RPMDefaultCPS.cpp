#include "defines.h"
#include "RPMDefaultCPS.h"

#ifdef RPM_SENSOR_TYPE_DEFAULT

/* sd
 *  Some low level functions for RPM counting (and also for injection timing measurement) 
 *  
 *  Timer 1 frequency (250KHz) is optimized for 4 ... 6 marks on flywheel (same as number of cylinders, usually) 
 *  Minimum rotating speed detected is about 57RPM (4cyl engine)
 *  
 *  Timer 1 is set to 0 when RPM interrupt happens and old Timer 1 value is stored. 
 *  This value can be directly converted for revolutions per minute. 
 *  If timer reaches 65535, overflow interrupt handler is activated and it engine is 
 *  declared as stopped.
 *  
 *  Injection timing is also recorded. Because the mark on the flywheel is on a known position, the
 *  counter value can be converted to actual degree of advance/retard
 *  
 *  
*/

volatile unsigned int rpmDuration;


static inline void rpmTimerSetup() __attribute__((always_inline));
static inline void rpmTimerEnable() __attribute__((always_inline));
static inline void rpmTimerDisable() __attribute__((always_inline));
static inline void needleTrigger()  __attribute__((always_inline));
void rpmTrigger();

static inline void rpmTimerSetup()  {
	// Arduino defaults 490Hz base freq which is fine to us
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
 	TCCR1B = 11; // WGM12 = 8 +CS11 = 2+CS10 = 1  // Clock select 490Hz
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
	static unsigned char cylinder=0;;
	cli();
	rpmDuration = TCNT1;
	rawValues[cylinder]=rpmDuration;
	injectionBegin = 0;
	sei();
	cylinder++;
	if (cylinder>NUMBER_OF_CYLINDERS)
		cylinder=0;
	rpmTimerEnable();


	attachInterrupt(1, needleTrigger, FALLING);
	// Skip first interrupt (interrupt pending flag is not cleared by attachIntterrupt?? )
	skipInjectionTrigger = 2; 
}

void needleTrigger() { 
	cli();
	skipInjectionTrigger--;

	if (skipInjectionTrigger) {
		sei();
		return;
	}
	injectionBegin = TCNT1; 
	detachInterrupt(1); // disable interrupt handler for a while

	sei();
}

// Class methods

void RPMDefaultCPS::init() {
 	rpmDuration = 0;
	rpmTimerSetup(); 
	rpmTimerEnable();		
}

unsigned int RPMDefaultCPS::getLatestMeasure() {
	if (rpmDuration==0)
		return 0;
	/* do not process values on rawValues directly because they may get changed (interrupt routine)*/
	cli();
	memcpy(storedValues,rawValues,NUMBER_OF_CYLINDERS*2);
	sei();

	/* Return an avarage value of full last rotation. It may be unnecessary, but my engine has a little unevenly spaced markers on the flywheel */
	unsigned long duration=0;
	for (unsigned char i=0;i<NUMBER_OF_CYLINDERS;i++) {
		duration += (unsigned long)storedValues[i];	
	}
	duration = duration/5; 
	// 64 = timer divider
	return ((unsigned long)(60*F_CPU/64/NUMBER_OF_CYLINDERS)/((unsigned long)duration)); 

}
unsigned int RPMDefaultCPS::getLatestRawValue() {
	return rpmDuration;
}

int RPMDefaultCPS::getInjectionTiming() {
	if (injectionBegin) {
			unsigned int timerTicksPerDegree = rpmDuration / (360/NUMBER_OF_CYLINDERS);
			// Calculate relative position, using *10 fixed point for internal presentation 
			unsigned int advanceRelative = ((unsigned long)core.controls[Core::valueEngineTimingDiff]*(unsigned long)10)/(unsigned long)timerTicksPerDegree;
			// Apply BTDC Mark correction
			return BTDC_MARK-advanceRelative;		
	} else {
		return 0;
	}
}

#endif
