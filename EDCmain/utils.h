#ifndef UTILS_H
#define UTILS_H

#include <avr/pgmspace.h>
int EEPROMwriteData(int offset, char *ptr,int size);

int EEPROMreadData(int offset, char *ptr,int size);

void toggleHumanReadableValues();

void printValue(int i,char type);

char* fetchFromFlash(const char *str);

bool printFromFlash(const char *str);

void printPads(unsigned char n, char padChar);

void printStringWithPadding(char *str,unsigned char width,char padChar);

void printIntWithPadding(int val,unsigned char width,char padChar);

void ansiGotoXy(char x,char y);

void ansiClearScreen();

void ansiClearEolAndLf();

void ansiClearEol();

void ansiClearEos();

unsigned char mapValues(int raw,int mapMin,int mapMax);

unsigned int mapInterpolate10bit(unsigned char p1,unsigned char p2, unsigned char pos);
unsigned char mapInterpolate(unsigned char p1,unsigned char p2, unsigned char pos);
unsigned char mapLookUp(unsigned char *mapData,unsigned char x,unsigned char y);
unsigned int mapLookUp10bit(unsigned char *mapData,unsigned char x,unsigned char y);

void printMapAxis(unsigned char axisType,unsigned char mapIdx,bool verbose);

int toKpa(int raw);

int toTemperature(int rawValue);

int toVoltage(int raw);

int toRpm(int raw);

int toTps(int raw);

int tempSensorBcoefficientCalc(int raw,int bCoEfficient, int nResistance,int nTemp);

#endif
