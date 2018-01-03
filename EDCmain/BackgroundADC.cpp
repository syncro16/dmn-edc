#include "BackgroundADC.h"

/*
	Reads ADC value periodically in the background. Set ADC clock to slowest value (interrupt rate 9000hz),
	adc buffer is filled one channel at a time, when interrupt hits

	Return values can be raw last sample or avaraged value.
*/

BackgroundADC adc;

ISR(ADC_vect) {
	static unsigned char adcPin=0;

	unsigned char h,l;
	l = ADCL; // must read first
	h = ADCH;
	adcBuffer[adcPin] = h << 8 | l;
	// set up for the next pin
	adcPin++;
	adcPin &= 0x0f;

	// the MUX5 bit of ADCSRB selects whether we're reading from channels
	// 0 to 7 (MUX5 low) or 8 to 15 (MUX5 high).
	ADCSRB = (ADCSRB & ~(1 << MUX5)) | (((adcPin >> 3) & 0x01) << MUX5);
  
	// set the analog reference (high two bits of ADMUX) and select the
	// channel (low 4 bits).  this also sets ADLAR (left-adjust result)
	// to 0 (the default).
	
	// analog reference (default)
	ADMUX = (1 << REFS0) | (adcPin & 0x07);

	// start the conversion
	sbi(ADCSRA, ADSC);
}

void BackgroundADC::init() {
//	for (unsigned char i=A0;i<=A15;i++)
//		pinMode(i,INPUT_PULLUP);
	
	cli();//disable interrupts
	ADCSRA = 0;
	ADCSRB = 0;

	ADMUX |= (1 << REFS0); //set reference voltage
	//	ADCSRA |= (1 << ADPS2) | (1 << ADPS0); //set ADC clock with 32 prescaler- 16mHz/32=500kHz
	ADCSRA |= (1 << ADPS2) | (1 << ADPS0) | (1 << ADPS1); //set ADC clock with 32 prescaler- 16mHz/32=500kHz
	// ADCSRA |= (1 << ADATE); //enabble auto trigger
	ADCSRA |= (1 << ADIE); //enable interrupts when measurement complete
	ADCSRA |= (1 << ADEN); //enable ADC
	ADCSRA |= (1 << ADSC); //start ADC measurements

	// start the conversion
	sbi(ADCSRA, ADSC);

	sei();//enable interrupt
	delay(10); // allow some interrupts happen to fill buffer to make readAvarageValue work perfectly from start
}

unsigned int BackgroundADC::readValue(unsigned char pin) {
	if (pin >= PIN_A0)
		pin = pin-PIN_A0;
	unsigned int ret;
	cli();
	ret = adcBuffer[pin & 0xf];
	sei();
	return ret;
}

unsigned int BackgroundADC::readValueAvarage(unsigned char pin) {
	if (pin >= PIN_A0)
		pin = pin-PIN_A0;
	unsigned int input = readValue(pin);
    if (aOutput[pin] == -1)
        aOutput[pin] = input;
    aOutput[pin] += (input-aOutput[pin]) * aOversamplingFactor[pin];
    return round(aOutput[pin]);
}

