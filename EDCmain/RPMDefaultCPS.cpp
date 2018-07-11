#include "RPMDefaultCPS.h"
#include "BackgroundADC.h"

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


static volatile unsigned int injectionBegin;
// record rotation speeds for a full engine cycle (720°)
volatile unsigned int measurements[NUMBER_OF_CYLINDERS*2]; 
unsigned char currentTick;

volatile unsigned int rpmDuration;

extern volatile unsigned int intHandlerCalls;

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
 	TCNT1 = 0;
 	TCCR1B = 11; // WGM12 = 8 +CS11 = 2+CS10 = 1
 }

 static void rpmTimerDisable() {
 	TCCR1A = 0;
 	TCCR1B = 0;
 	rpmDuration = 0;
 }
 

ISR(TIMER1_COMPA_vect) 
{	
	// Timer has waited rotation for too long, declare engine as stopped
	rpmTimerDisable();
	core.controls[Core::valueEngineRPM] = 0;
	core.node[Core::nodeEngineRPM].value = 0;
	core.controls[Core::valueEngineRPMFiltered] = 0;   
	rpmDuration = 0;
	memset(measurements,0,sizeof(measurements));
}



volatile unsigned char *errCnt;
volatile long rpmMax;
volatile long rpmMin;

void rpmTrigger() { 
	unsigned int dur = TCNT1;
	unsigned int on=0,off=0;
	if (intHandlerCalls)
		*errCnt = intHandlerCalls;

	TCNT1 = 0;
	digitalWrite(PIN_RPM_PROBE,HIGH);

	for (unsigned int i=0;i<10;i++) {
		if (digitalRead(PIN_INPUT_RPM)) {
			on++;
		} else {
			off++;
		}
	}
	digitalWrite(PIN_RPM_PROBE,LOW);	
	if (on) {
		TCNT1 += dur; // restore
		//*errCnt++;
		return;
	}


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
	if (dur<rpmMin)
		rpmMin = dur;
	if (dur>rpmMax)
		rpmMax = dur;
		
	injectionBegin = 0;
	measurements[currentTick] = dur;
	currentTick++;

//	0 1 2 3 4 5 6 7 8 9
	if (currentTick>(NUMBER_OF_CYLINDERS*2-1))
		currentTick = 0;

	if (core.controls[Core::valueRunMode] >= ENGINE_STATE_PID_IDLE)
		core.controls[Core::valueQAfeedbackActual] = adc.readValue_interrupt_safe(PIN_ANALOG_QA_POS);   

//	rpmTimerEnable();
 	TCCR1B = 11; // WGM12 = 8 +CS11 = 2+CS10 = 1

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
	if (rpmDuration<10)
		return 0;
	// 64 = timer divider
	// cli sei is to moved parent class
//	cli();
	unsigned int ret=((unsigned long)(60*F_CPU/64/NUMBER_OF_CYLINDERS)/((unsigned long)rpmDuration)); 
//	sei();
	return ret;
}

/*
	todo: add teeth mapping function for correct cylinder phasing
*/
unsigned int RPMDefaultCps::getLatestMeasureFiltered() {
	// 64 = timer divider
	unsigned long total = 0;
	char t = currentTick;
	
	for (unsigned char idx=0;idx<NUMBER_OF_CYLINDERS;idx++) {
		t--;
		if (t<0) t=NUMBER_OF_CYLINDERS*2-1;
		total += measurements[(unsigned)t];
	}
	total = total/(NUMBER_OF_CYLINDERS);
	//TODO 
	if (total==0)
		return 0;	
	cli();
	unsigned int ret = ((unsigned long)(60*F_CPU/64/NUMBER_OF_CYLINDERS)/((unsigned long)total)); 
	sei();
	return ret;
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

unsigned int RPMDefaultCps::getDeviationForCylinder(unsigned char cyl) {
	cli();
	unsigned int v1=measurements[cyl*2];
	unsigned int v2=measurements[cyl*2+1];
	sei();
	static long cyl0Timing;
	
	if (cyl == 0) {
		// cache some things to speed up things		
		cyl0Timing = (v1 + v2)/2;
		return cyl0Timing;
	}
	if (cyl >= NUMBER_OF_CYLINDERS)
		return 0;
	long deviation = cyl0Timing-(v1+v2)/2;
	if (deviation<-32000)
		return -32000;
	if (deviation>32000)
		return 32000;
		
	return deviation;
}
