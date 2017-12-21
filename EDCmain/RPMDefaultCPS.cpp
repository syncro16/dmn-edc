#include "RPMDefaultCPS.h"

/* 
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
void rpmTrigger();

static inline void rpmTimerSetup()  {
	cli();
	TCCR1A = 0; 
	TCCR1B = 0; 
	TCNT1 = 0;
	OCR1A = 0xffff;

 	TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt
 	sei();
	//attachInterrupt(0, rpmTrigger, RISING);  // Interrupt 0 -- PIN2 -- LM1815 gated output 
	attachInterrupt(0, rpmTrigger, FALLING);  // Interrupt 0 -- PIN2 -- Cherry GS sensor 

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



volatile unsigned char *errCnt;
volatile long rpmAvg;

void rpmTrigger() { 
	cli();
	unsigned int dur = TCNT1;
	/*if (core.controls[Core::valueRunMode] >= ENGINE_STATE_IDLE ) {
		// allow 10% error, otherwise 
		long maxErr = dur/10;
		if (abs((long)rpmAvg-dur)>maxErr) {
			errCnt++;
			sei();
			rpmTimerEnable();
			return;	
		} else {
			rpmAvg += (rpmAvg*3+dur)/4;
		}
	} else {
		rpmAvg = dur;
	}
	*/
	rpmDuration = dur; // store duration to be calculated as RPM later
	injectionBegin = 0;
	sei();
	rpmTimerEnable();

	/* This reads (and smoothes) Quantity Adjuster position syncronized with RPM, 
	this minimizes errors during reading the position (there is mechanical jitter on qa position) */
	if (core.node[Core::nodeQASync].value 
		&& (core.controls[Core::valueRunMode] >= ENGINE_STATE_IDLE 
			&& core.controls[Core::valueRunMode] <= ENGINE_STATE_LOW_RPM_RANGE )) {
		cli();
		core.controls[Core::valueQAfeedbackActual] = (core.controls[Core::valueQAfeedbackActual]+analogRead(PIN_ANALOG_QA_POS))/2;     
		sei();
	}

}

// Class methods
void RPMDefaultCps::init() {
	errorCount = 0;
	errCnt = &errorCount;
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

int RPMDefaultCps::getInjectionTiming() {
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

