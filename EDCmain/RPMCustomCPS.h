#ifdef RPM_SENSOR_TYPE_CUSTOM

#ifndef RPMCUSTOMCPS_H
#define RPMCUSTOMCPS_H

#include "RPMBase.h"
#include "Arduino.h"
#include "Core.h"

/* 
 * RPM calculation - standard type (4/5/6 marks  per rotation)
 */


// ((((16000000/64)/65535)*60/5                         16000000L             4000 ~ 780rpm
#define RPMTIMER_DURATION_TO_RPM(x) ((unsigned long)(60*F_CPU/64/NUMBER_OF_CYLINDERS)/((unsigned long)x))  // 250 000hz frequency (max. 65535 ticks per teeth ~45rpm)
#define RPMTIMER_MIN_DURATON 400 // 7500rpm

#define RPM_TEETH_COUNT (144)
#define RPM_TEETH_PER_CYL (144/NUMBER_OF_CYLINDERS)

static volatile unsigned int rawValues[RPM_TEETH_PER_CYL];


class RPMCustomCPS : public RPMBase {
	private:
	unsigned int storedValues[RPM_TEETH_PER_CYL];
	public:
	void init();
	unsigned int getLatestMeasure();
	unsigned int getLatestRawValue();
	
	private:
	void setupTimers();

};


#endif
#endif
