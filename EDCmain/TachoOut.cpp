#include "TachoOut.h"
#include <avr/io.h>
#include <avr/interrupt.h>

TachoOut::TachoOut() {


}

static unsigned int period=4800;
//static unsigned int period=65535;
// 10000 = 3000rpm
// 4800 == 6000rpm
// 4500 = 6500rpm


void TachoOut::init() {
	TCCR5A = 0;                
  	TCCR5B = _BV(WGM13);     
  	//char clockSelectBits = _BV(CS51); //8
	char clockSelectBits = _BV(CS50);   	
 	ICR5 = period;                                                   // ICR1 is TOP in p & f correct pwm mode
 	OCR5A = period/2;
  	TCCR5B &= ~(_BV(CS50) | _BV(CS51) | _BV(CS52));
  	TCCR5B |= clockSelectBits;  
  	// DDRL |= _BV(PORTL3); TCCR5A |= _BV(COM5A1);   
  	DDRL |= _BV(PORTL4); TCCR5A |= _BV(COM5B1);  	// pin45
}

void TachoOut::setRpm (unsigned int rpm) {
	// 65535*(450/1000)
	if (rpm>6000) rpm=6000;
	long int base=(65535*445)/rpm;
	if (base>0xffff)
		base=0xffff;
	ICR5 = base;                                                   // ICR1 is TOP in p & f correct pwm mode
 	//OCR5A = base/2;
 	OCR5B = base/2;
}
