#ifndef BACKGROUNDADC_H
#define BACKGROUNDADC_H

#include "defines.h"
#include "Arduino.h"

static volatile unsigned int adcBuffer[PIN_A15-PIN_A0+1];

class BackgroundADC {
public:
	
private:
	float aOutput[PIN_A15-PIN_A0+1]={-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1};
	float const aOversamplingFactor[PIN_A15-PIN_A0+1] = {
			PIN_ANALOG_SMOOTHING_A0,
			PIN_ANALOG_SMOOTHING_A1,
			PIN_ANALOG_SMOOTHING_A2,
			PIN_ANALOG_SMOOTHING_A3,
			PIN_ANALOG_SMOOTHING_A4,
			PIN_ANALOG_SMOOTHING_A5,
			PIN_ANALOG_SMOOTHING_A6,
			PIN_ANALOG_SMOOTHING_A7,
			PIN_ANALOG_SMOOTHING_A8,
			PIN_ANALOG_SMOOTHING_A9,
			PIN_ANALOG_SMOOTHING_A10,
			PIN_ANALOG_SMOOTHING_A11,
			PIN_ANALOG_SMOOTHING_A12,
			PIN_ANALOG_SMOOTHING_A13,
			PIN_ANALOG_SMOOTHING_A14,
			PIN_ANALOG_SMOOTHING_A15,
		};
public:
	void init();
	unsigned int readValue(unsigned char pin);
	unsigned int readValueAvarage(unsigned char pin);
	unsigned int readValue_interrupt_safe(unsigned char pin);
	unsigned int readValueAvarage_interrupt_safe(unsigned char pin);

	
};

extern BackgroundADC adc;

#endif