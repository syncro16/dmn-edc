// Diagnostic trouble code(s)
//
// Stores DTC on eeprom. Only one error per power-on period is registered (unless DTC is cleared). Count of errors are stored in eeprom too.


#ifndef DTH_H
#define DTH_H

#include "Arduino.h"

#define DTC_FILE_4BYTE_ID "DT01"
#define DTC_EEPROM_OFFSET (4096 - 512)

#define DTC_NO_ERROR 0
#define DTC_INTERNAL_ERROR 1
#define DTC_CONFIGURATION_ERROR 2
#define DTC_TRAP_1 3
#define DTC_TRAP_2 4
#define DTC_TRAP_3 5
#define DTC_TRAP_4 6
#define DTC_TRAP_5 7
#define DTC_INTERNAL_UNUSED_1 8
#define DTC_STORAGE_MISMATCH 9
#define DTC_TPS_UNCONNECTED 10
#define DTC_TPS2_UNCONNECTED 11
#define DTC_MAP_UNCONNECTED 12
#define DTC_QUANTITY_ADJUSTER_UNCONNECTED 13
#define DTC_NEEDLESENSOR_UNPLAUSIBLE_SIGNAL 14
#define DTC_RPM_UNPLAUSIBLE_SIGNAL 15
#define DTC_ENGINE_TEMP_UNCONNECTED 16
#define DTC_FUEL_TEMP_UNCONNECTED 17
#define DTC_AIR_TEMP_UNCONNECTED 18
#define DTC_TPS_UNPLAUSIBLE 19

#define DTC_CONFIGURATION_MISMATCH 20


#define MAX_DTCS 64
#define MAX_DTC_LEN 60

//const char DTC_CODES[] PROGMEM = "z";

static const char DTC_CODES[MAX_DTCS][MAX_DTC_LEN] PROGMEM = 
{ 
    "No error.", //0
    "Internal error.", //1   
    "Default settings loaded.", //2
    "Diagnostic trap #1", //3
    "Diagnostic trap #2", //4
    "Diagnostic trap #3", //5
    "Diagnostic trap #4", //6
    "Diagnostic trap #5", //7
    "Unknown DTC code 8", // 8
    "DTC Storage mismatch", // 9
    "TPS#1 unconnected", // 10
    "TPS#2 unconnected", // 11
    "MAP unconnected", // 12 
    "Quantity Adjuster unconnected", // 13
    "Unplausible needle lift sensor signal", // 14
    "Unplausible RPM sensor signal", // 15
    "Engine temperature sensor unconnected", // 16
    "Fuel temperature sensor unconnected", // 17
    "Air temperature sensor unconnected", // 18
    "TPS signal unplausible", // 19
    "Configuration mismatch", // 20
    "Unknown DTC Code 21", // 21
    "Unknown DTC Code 22", // 22
    "Unknown DTC Code 23", // 23
    "Unknown DTC Code 24", // 24
    "Unknown DTC Code 25", // 25
    "Unknown DTC Code 26", // 26
    "Unknown DTC Code 27", // 27
    "Unknown DTC Code 28", // 28
    "Unknown DTC Code 29", // 29
    "Unknown DTC Code 30", // 30
    "Unknown DTC Code 31", // 31
    "Unknown DTC Code 32", // 32
    "Unknown DTC Code 33", // 33
    "Unknown DTC Code 34", // 34
    "Unknown DTC Code 35", // 35
    "Unknown DTC Code 36", // 36
    "Unknown DTC Code 37", // 37
    "Unknown DTC Code 38", // 38
    "Unknown DTC Code 39", // 39
    "Unknown DTC Code 40", // 40
    "Unknown DTC Code 41", // 41
    "Unknown DTC Code 42", // 42
    "Unknown DTC Code 43", // 43
    "Unknown DTC Code 44", // 44
    "Unknown DTC Code 45", // 45
    "Unknown DTC Code 46", // 46
    "Unknown DTC Code 47", // 47
    "Unknown DTC Code 48", // 48
    "Unknown DTC Code 49", // 49
    "Unknown DTC Code 50", // 50
    "Unknown DTC Code 51", // 51
    "Unknown DTC Code 52", // 52
    "Unknown DTC Code 53", // 53
    "Unknown DTC Code 54", // 54
    "Unknown DTC Code 55", // 55
    "Unknown DTC Code 56", // 56
    "Unknown DTC Code 57", // 57
    "Unknown DTC Code 58", // 58
    "Unknown DTC Code 59", // 59
    "Unknown DTC Code 60", // 60
    "Unknown DTC Code 61", // 61
    "Unknown DTC Code 62", // 62
    "Unknown DTC Code 63", // 63
};

class DTC {
private:
    long lastSave;
    int iterator;
    bool isTouched;
    
    unsigned char errorCodesOnStartup[MAX_DTCS];    
    unsigned char errorCodesCurrent[MAX_DTCS];
    char errorBuf[MAX_DTC_LEN];
    void saveToEEPROM();
    void load();
public:
    DTC();   
    bool seekNextError();
    char *getName();
    unsigned char getCount();
    int getIndex();
    void setError(unsigned int dtc);
    void resetAll();
    void save();
    boolean isErrorActive(unsigned int dtc);
    
};

extern DTC dtc;

#endif


