#include "RPMBase.h"
#include "core.h"

RPMBase::RPMBase()  {
	
}

void RPMBase::init() {

}
unsigned int RPMBase::getLatestMeasure() {
	return 0;
}
unsigned int RPMBase::getLatestRawValue() {
	return 0;
}
int RPMBase::getInjectionTiming() {
	return 0;
}

void RPMBase::measure() {
	core.controls[Core::valueEngineRPMFiltered] = getLatestMeasureFiltered();
	core.controls[Core::valueEngineRPM] = getLatestMeasure();
	core.controls[Core::valueEngineRPMRaw] = getLatestRawValue();

	core.controls[Core::valueRpmDeviation1] = getDeviationForCylinder(0);
	core.controls[Core::valueRpmDeviation2] = getDeviationForCylinder(1);
	core.controls[Core::valueRpmDeviation3] = getDeviationForCylinder(2);
	core.controls[Core::valueRpmDeviation4] = getDeviationForCylinder(3);
	core.controls[Core::valueRpmDeviation5] = getDeviationForCylinder(4);
	core.controls[Core::valueRpmDeviation6] = getDeviationForCylinder(5);

}

unsigned char RPMBase::getError() {
	unsigned char ret = errorCount;
	errorCount=0;
	return ret;
}




