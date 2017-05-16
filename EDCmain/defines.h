#ifndef DEFINES_H
#define DEFINES_H

#define VERSION_NUMBER 0x0103
#define PRODUCT_NAME "dmn-edc 201705"

/* Choose what kind of RPM sensor are you using */
//#define RPM_SENSOR_TYPE_DEFAULT 1
#define RPM_SENSOR_TYPE_CUSTOM 1

#define NUMBER_OF_CYLINDERS 5
#define NEEDLELIFTSENSOR_CYLINDER 3
#define FIRING_ORDER {4,5,3,1,2} /* Starting on with cylinder where needle sensor is */
//#define ATDC_MARK 115 /* Position of each cylinder's timing mark on flywheel/crankshaft, 115 = 11.5° after TDC. Time difference between needle lift sensor trigger and this mark is used to calculate timing */
#define BTDC_MARK 605 /* Position of each cylinder's timing mark on flywheel/crankshaft, 60.5= 60.5° before TDC. Time difference between needle lift sensor trigger and this mark is used to calculate timing */
#define FLYWHEEL_MARK_ANGLE (360 / NUMBER_OF_CYLINDERS)

#define ANALOG_INPUT_HIGH_STATE_LIMIT 975 /* if analog input is above this limit, assume no sensor is connected and internal pullup resistor feeds the almost +5v */

#define BOOST_PID_RANGE 20

#define N108_MAX_DUTY_CYCLE 242
#define N108_MIN_DUTY_CYCLE 0

#define TPS_LIMP_MODE_AMOUNT 68
#define TPS_SAFETY_BITS_IDLESW 1

/* Trigger inputs (interrupt handlers) */
#define PIN_INPUT_RPM 2 /* do not change! */ 
#define PIN_INPUT_NEEDLELIFTSENSOR 3 /* do not change! */ 

/* Note: PIN 11 & PIN 12 cannot be used for PWM (timer1 is reserved for rpm counting) */

/* Analog Inputs */
#define PIN_ANALOG_QA_POS A0
#define PIN_ANALOG_TEMP_FUEL A1
#define PIN_ANALOG_BATTERY_VOLTAGE A2
#define PIN_ANALOG_UNDEF_a A3 
#define PIN_ANALOG_UNDEF_b A4 
#define PIN_ANALOG_SERVO_POS A5
#define PIN_ANALOG_LAMBDA A6 
#define PIN_ANALOG_EGT A7 
#define PIN_ANALOG_MAP A8
#define PIN_ANALOG_TEMP_INTAKE A9
#define PIN_ANALOG_TEMP_GEARBOX A10
#define PIN_ANALOG_TEMP_COOLANT A11

/* TPS Sensor */
#define PIN_ANALOG_TPS_POS A12
#define PIN_INPUT_TPS_WOT_SW A13  // TODO: swap this with SET_SW (and use pullup ~2.5v (GND=decrease speed, +12v=set/Accel))
#define PIN_INPUT_TPS_IDLE_SW 36
#define PIN_INPUT_BRAKE_SW 38
#define PIN_INPUT_CLUTCH_SW 40
#define PIN_INPUT_CRUISE_SET_SW 42

/* FETs and/or PWM */
#define PIN_PWM_QA 5 // 3,4 or 5 (timer3)
#define PIN_PWM_TIMING_SOLENOID 6 // 6,7 or 8 (timer4)
#define PIN_PWM_AUX 7
#define PIN_PWM_BOOST_SOLENOID 8
#define PIN_PWM_SERVO 9
#define PIN_PWM_NLS_REF_VOLTAGE 10

/* servo control */
#define PIN_SERVO_DIR 37
#define PIN_INPUT_SERVO_FAULT 39 

/* relays */
#define PIN_RELAY_POWER 22
#define PIN_RELAY_FUEL_SOLENOID 23
#define PIN_RELAY_FAN1 24
#define PIN_RELAY_FAN2 25
#define PIN_RELAY_ENGINE_GLOW 26
#define PIN_RELAY_COOLANT_RELAY1 27
#define PIN_RELAY_COOLANT_RELAY2 28
#define PIN_RELAY_AUX 29

/* general outputs */
#define PIN_TACHO_OUT 11
#define PIN_GLOW_LIGHT 13

/* general inputs */
#define PIN_INPUT_POWER_ON 37
#define PIN_INPUT_VSS 19 


#define KEY_UP 'k'
#define KEY_DOWN 'j'
#define KEY_LEFT 'h'
#define KEY_RIGHT 'l'

#define DUMP_START 1
#define DUMP_INPROGRESS 2
#define DUMP_FINISHED 4
#define DUMP_SIZE 250

#define MAP_AXIS_NONE 0xFF
#define MAP_AXIS_RAW 0x00
#define MAP_AXIS_RPM 0x01
#define MAP_AXIS_TPS 0x02
#define MAP_AXIS_KPA 0x03
#define MAP_AXIS_CELSIUS 0x04
#define MAP_AXIS_VOLTAGE 0x05
#define MAP_AXIS_DUTY_CYCLE 0x06
#define MAP_AXIS_IDLERPM 0x07
#define MAP_AXIS_INJECTION_TIMING 0x08
#define MAP_AXIS_INJECTED_FUEL 0x09
#define MAP_AXIS_SECONDS 0x10
#define MAP_AXIS_FUEL_TRIM_AMOUNT 0x11
#define MAP_AXIS_PWM8 0x12


#define BOOST_MAX_CLIP 1
#define BOOST_MIN_CLIP 2
#define BOOST_OK 0

#define TACHO_TIMER_PIN_HZ 490 

#define VNT_OPEN_DELAY_ON_IDLE	240 // 0 - disable
#define FAST_START_DELAY 340


#define SENSOR_FAIL_COUNT 300

// defines for setting and clearing register bits
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

#endif
