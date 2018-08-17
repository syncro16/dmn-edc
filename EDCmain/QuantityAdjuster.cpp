#include "QuantityAdjuster.h"
#include "Arduino.h"
#include "defines.h"
#include "TimerThree.h"
#include "utils.h"
#include "DTC.h"
#include "BackgroundADC.h"

QuantityAdjuster::QuantityAdjuster() {
}

void QuantityAdjuster::initialize() {
	Timer3.pwm(PIN_PWM_QA,0); // 100
	Timer3.start();
	errorOld=0;
	integral=0;
	setPoint = 42;
	speed=1;
}

static char qaCalls=0;

void QuantityAdjuster::reset() {
	integral=0;
	error=0;
}
void QuantityAdjuster::update(char skip) {
	if (qaCalls)
		dtc.setError(DTC_TRAP_1);

	qaCalls++;

	float output;

	Kp = (float)(core.node[Core::nodeQAPIDKp].value)/256;
	Ki = (float)(core.node[Core::nodeQAPIDKi].value)/256;
	Kd = (float)(core.node[Core::nodeQAPIDKd].value)/256;
	speed = (core.node[Core::nodeQAPIDSpeed].value);

	if (setPoint < targetSetPoint) {
		setPoint += speed;
		if (setPoint > targetSetPoint)
			setPoint = targetSetPoint;
	} else if (setPoint > targetSetPoint) {
		setPoint -= speed;
		if (setPoint < targetSetPoint)
			setPoint = targetSetPoint;
	}

	core.controls[Core::valueQAfeedbackSetpoint] = setPoint; 
	// Read qa position always if not running idle (or during load).
	// When running idle (or load) read is synced to RPM signal to improve signal quality

	if (core.controls[Core::valueRunMode] < ENGINE_STATE_PID_IDLE) {
		core.controls[Core::valueQAfeedbackActual] = adc.readValue_interrupt_safe(PIN_ANALOG_QA_POS);   
	}
	if (core.controls[Core::valueRunMode] == ENGINE_STATE_HIGH_LOAD_RANGE) {	
		core.controls[Core::valueQAfeedbackActual] = adc.readValueAvarage_interrupt_safe(PIN_ANALOG_QA_POS);   
	}
	
	//core.controls[Core::valueQAfeedbackActual] = analogRead(PIN_ANALOG_QA_POS);      


	// Then map it to 0..1023 range    
	currentActuatorPosition =  map(core.controls[Core::valueQAfeedbackActual],
		core.node[Core::nodeQAFeedbackMin].value,
		core.node[Core::nodeQAFeedbackMax].value,
		0,1023);

	if (currentActuatorPosition<0)
		currentActuatorPosition = 0;
	if (currentActuatorPosition > 1023) 
		currentActuatorPosition = 1023;     

	core.controls[Core::valueQAfeedbackRaw] = currentActuatorPosition;
	error = setPoint-currentActuatorPosition;

	accuracy = (accuracy+abs(error))/2;

	if (error<0)
		error = (error * ((float)core.node[Core::nodeQAPIDBias].value) / 100); 

	integral = integral + error*Ki;

/*
	if (integral>1100)
	 integral = 1100;

	if (integral<-100)
		integral = -100;
	*/
	
	if (integral>core.node[Core::nodeQAMaxPWM].value)
	 integral = core.node[Core::nodeQAMaxPWM].value;;

	if (integral<core.node[Core::nodeQAMinPWM].value)
		integral = core.node[Core::nodeQAMinPWM].value;

	derivate = (error - errorOld);

	errorOld = error;
	p = Kp* error;
	i = integral; // calculated above
	d = Kd *derivate;
	output = p+i+d;
	
	core.controls[Core::valueQAPIDPparam] = p;
	core.controls[Core::valueQAPIDIparam] = i;
	core.controls[Core::valueQAPIDDparam] = d;

	int minPwm = core.node[Core::nodeQAMinPWM].value;


	currentDutyCycle = map((int)output,0,1023,minPwm,core.node[Core::nodeQAMaxPWM].value);

	if (output > core.node[Core::nodeQAMaxPWM].value) {
		currentDutyCycle = core.node[Core::nodeQAMaxPWM].value;
	} else if (output < core.node[Core::nodeQAMinPWM].value) {
		currentDutyCycle = core.node[Core::nodeQAMinPWM].value;
	} else {
		currentDutyCycle = output; 
	}

	if (core.controls[Core::valueQADebug] == 0 && !skip) {
		// skip pwm set for test purposes
		if (setPoint>0) {
			core.controls[Core::valueQAPWMActual] = currentDutyCycle; 
			Timer3.setPwmDuty(PIN_PWM_QA,core.controls[Core::valueQAPWMActual] );
		} else {
			currentDutyCycle = 0;
			core.controls[Core::valueQAPWMActual] = currentDutyCycle; 
			Timer3.setPwmDuty(PIN_PWM_QA,core.controls[Core::valueQAPWMActual] );    
		}
		core.controls[Core::valueQAPWMActual] = currentDutyCycle; 

		Timer3.setPwmDuty(PIN_PWM_QA,core.controls[Core::valueQAPWMActual] );    
	}
	qaCalls--;
}

 void QuantityAdjuster::setPosition(int val) {
 	if (val > setPointMax)
 		setPointMax=val;
 	if (val < setPointMin)
 		setPointMin=val;

 	cli();
	core.node[Core::nodeQASetPoint].value = val;
	sei();
	
	if (val == 0) {
		targetSetPoint = 0;
		setPoint = 0;
	} else {
		targetSetPoint = val;
	}
}

void QuantityAdjuster::triggerHit() {
	statusBits = HIT_TRIGGER;
}

