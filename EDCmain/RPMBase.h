#ifndef RPMBASE_H
#define RPMBASE_H
/*
 * Implement engine type RPM counter (and injection timing counter) using this base class
 */
class RPMBase {
	public:
	volatile unsigned char errorCount;

	public:
	RPMBase();
	virtual void init();
	virtual unsigned int getLatestMeasure();
	virtual unsigned int getLatestMeasureFiltered();	
	virtual unsigned int getLatestRawValue();
	virtual int getInjectionTiming();
	void measure();
	unsigned char getError();
	//void fullRotationTrigger();
};
#endif
