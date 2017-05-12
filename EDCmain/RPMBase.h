#ifndef RPMBASE_H
#define RPMBASE_H
/*
 * Implement engine type RPM counter (and injection timing counter) using this base class
 */
class RPMBase {
	public:
	RPMBase();
	virtual void init();
	virtual unsigned int getLatestMeasure();
	virtual unsigned int getLatestRawValue();
	virtual int getInjectionTiming();
	void measure();
	//void fullRotationTrigger();
};
#endif
