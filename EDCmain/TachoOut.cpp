#include "TachoOut.h"
#include <avr/io.h>
#include <avr/interrupt.h>

TachoOut::TachoOut() {


}

void TachoOut::init() {
	/*
	TCCR5A = 0; 
 	TCCR5B = 0; 
 	TCNT5 = 0;
 	OCR5A = 0x4ff;
  
 	TIMSK5 |= (1 << OCIE5A); // enable timer compare interrupt
 
 	TCNT5 = 0;
 	TCCR5B = 11; // WGM12 = 8 +CS11 = 2+CS10 = 1
 	*/
}

void TachoOut::updateAutomatically() {


}

void TachoOut::setRpm(int rpm) {
	/*
	float scale = 2.35;	
	unsigned int ticksPerSeconds = ((float)rpm*scale)/60;
	unsigned int ticksCorrected = (float)ticksPerSeconds/(1000.0/(float)TACHO_TIMER_PIN_HZ);
	//OCR5A = ticksCorrected;

	float dur = (1.0/((float)rpm))*450000.0;
    OCR5A = dur;
    */
}

static bool tachoOutputState;

/*
//ISR(TIMER5_OVF_vect)  
ISR(TIMER5_COMPA_vect) 
{	
	TCNT5 = 0;
	tachoOutputState = !tachoOutputState;
	digitalWrite(PIN_RPM_OUT,tachoOutputState);
}
*/