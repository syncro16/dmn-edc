#include "DTC.h"

#undef EEPROM_h
#include <EEPROM.h>
#include "utils.h"
#include "ConfEditor.h"

DTC dtc;

// private
DTC::DTC() {
    load();
    iterator = -1;
}
void DTC::save() {
    if (isTouched && millis() - lastSave >5000) {
        lastSave = millis();
        saveToEEPROM();
    }
}
void DTC::saveToEEPROM() {
    // header save
    EEPROMwriteData(DTC_EEPROM_OFFSET,DTC_FILE_4BYTE_ID,4);
    // and current codes
    EEPROMwriteData(DTC_EEPROM_OFFSET+4,(char*)&errorCodesCurrent,MAX_DTCS);
    isTouched = false;
}
void DTC::load() {
    //  header check, if fails clear DTC's & save
    EEPROMreadData(DTC_EEPROM_OFFSET,(char*)&errorBuf,4);
    errorBuf[5] = 0;
    if (strcmp(errorBuf,DTC_FILE_4BYTE_ID) == 0) {
        // file format is ok, load
        EEPROMreadData(DTC_EEPROM_OFFSET+4,(char*)&errorCodesOnStartup,MAX_DTCS);
        memcpy((char*)&errorCodesCurrent,(char*)&errorCodesOnStartup,sizeof(errorCodesOnStartup));
        isTouched = false;
    } else {
        // do not load wrong file format
        resetAll();
        setError(DTC_STORAGE_MISMATCH);
    }
}
// public:

bool DTC::seekNextError() {
    while (iterator < MAX_DTCS-1) {
        iterator++;
        if (errorCodesCurrent[iterator]>0)
            return true;
    }
    iterator = -1;
    return false;
}
char* DTC::getName() {
    if (iterator != -1) {
        strcpy(errorBuf,fetchFromFlash(DTC_CODES[iterator]));
    } else {
        errorBuf[0] = 0;
    }
    return errorBuf;
}
int DTC::getIndex() {
    return iterator;
}
unsigned char DTC::getCount() {
    if (iterator != -1)
        return errorCodesCurrent[iterator];
    return 0;
}

void DTC::setError(unsigned int dtc) {
    if (errorCodesOnStartup[dtc] < 255 
        && errorCodesCurrent[dtc] == errorCodesOnStartup[dtc]) {
        isTouched = true;
        errorCodesCurrent[dtc]++;
        confeditor.setSystemStatusMessage(fetchFromFlash(DTC_CODES[dtc]));
    }
}

void DTC::resetAll() {
    memset(errorCodesCurrent,0,sizeof(errorCodesOnStartup));
    memset(errorCodesOnStartup,0,sizeof(errorCodesOnStartup));
    isTouched = true;
    saveToEEPROM();
    confeditor.setSystemStatusMessage("DTC Reset");
}

boolean DTC::isErrorActive(unsigned int dtc) {
    return errorCodesCurrent[dtc]?true:false;;
}

