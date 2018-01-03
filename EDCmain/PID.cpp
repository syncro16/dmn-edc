#include "PID.h"
#include <avr/io.h>
#include <avr/interrupt.h>

PID::PID(int *p,int *i,int *d,int *minOutput,int *maxOutput,int *speed,int *bias,int *input,int *output)
{
	Kp=p;
 	Ki=i;
 	Kd=d;
	this->minOutput = minOutput;
	this->maxOutput = maxOutput;
	this->speed = speed;
	this->input = input;
	this->output = output;
	this->bias = bias;	
	setPoint=0;
	targetSetPoint=0;
}

void PID::setPosition(int s) {
	this->targetSetPoint = s;
}

void PID::reset() {
	integral = 0;
}

void PID::calculate() {
	if (setPoint < targetSetPoint) {
		setPoint = setPoint + *speed;
		if (setPoint > targetSetPoint)
			setPoint = targetSetPoint;
	} else if (setPoint > targetSetPoint) {
		setPoint -= *speed;
		if (setPoint < targetSetPoint)
			setPoint = targetSetPoint;
	}
	setPoint = targetSetPoint;

	error = setPoint - *input;


	if (bias && error<0)
		error = (error * (*bias) / 100.0); 


	integral = integral + (error * ((float)(*Ki)/100.0));

	if (integral>*maxOutput)
		integral = *maxOutput; 
	
	if (integral<*minOutput)
		integral = *minOutput;

	derivate = (error - errorOld);
	errorOld = error;

	p = (*Kp/100.0) * (float)error;
	i = integral;
	d = ((float)(*Kd)/100.0) * (float)derivate;
	int o = (float)(p+i+d);

	lastP = p;
	lastI = i;
	lastD = d;	

	//cli(); // disable interrupts during set 
	if (o>*maxOutput) {
		*output = *maxOutput;
	} else if (o<*minOutput) {
		*output = *minOutput;
	} else {
		*output = o;
	}
	//sei();

	//printf("error=%d p=%d (%f) i=%d (%f) d=%d (%f), sp=%d, tsp=%d, in=%d, out=%d\n", error,*Kp,p, *Ki,i ,*Kd,d, setPoint,targetSetPoint,*input,*output);
}
