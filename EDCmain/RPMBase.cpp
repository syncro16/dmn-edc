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
	core.controls[Core::valueEngineRPMFiltered] = getLatestMeasure();
	core.controls[Core::valueEngineRPM] = getLatestMeasure();
	core.controls[Core::valueEngineRPMRaw] = getLatestRawValue();

}



