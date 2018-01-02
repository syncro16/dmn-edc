#include "utils.h"
#undef EEPROM_h
#include <EEPROM.h>
#include <avr/pgmspace.h>
#include "Arduino.h"
#include "Core.h"
#include "defines.h"
#include "DTC.h"

// EEProm

int EEPROMwriteData(int offset, char *ptr,int size) {
  int i;
  for (i = 0; i < size; i++)
    EEPROM.write(offset++, *(ptr++));
  return i;
}

int EEPROMreadData(int offset, char*ptr,int size) {
  int i;
  for (i = 0; i < size; i++)
    *(ptr++) = EEPROM.read(offset++);
  return i;
}

bool isHumanReadable = true;

void toggleHumanReadableValues() {
    isHumanReadable=!isHumanReadable;
}

void printValue(int i,char type) {
    char buf[10];
    unsigned int fixed;    
    if (!isHumanReadable) {
        type = VALUE_INT;
    }
    // always prints 6 chars
    switch (type) {
/*        case VALUE_BAR:
            break;
        case VALUE_PERCENTAGE:
            break;*/
        case VALUE_PWM8:
            printIntWithPadding(map(i,0,255,0,100),5,' ');
            Serial.print("%");            
            break;
        case VALUE_MS:
            printIntWithPadding(i,5,' ');
            Serial.print("ms");        
            break;            
        case VALUE_KPA:
            printIntWithPadding(toKpa(i),5,' ');
            Serial.print("kPa");        
            break;

        case VALUE_CELSIUS:                   
            printIntWithPadding(toTemperature(i),5,' ');
            Serial.print("C");                            
            break;

/*        case VALUE_DEGREE:
        break;*/
        case VALUE_INJECTION_TIMING:
//            ltoa((BTDC_MARK+i),buf,10);
            ltoa((i),buf,10);
            for (unsigned char c=0;c<5-strlen(buf)-1;c++)
                Serial.print(" ");
            for (unsigned char c=0;c<strlen(buf);c++) {
                if (c == strlen(buf)-1) 
                    Serial.print(".");
                Serial.print(buf[c]);           
            }
            Serial.print("°");                                        
            break;            
        case VALUE_VOLTAGE:
            fixed = ((unsigned int)i*48);   // ~4.97v
            Serial.print(" ");            
            ltoa(fixed,buf,10);
            if (strlen(buf) == 5) {
                Serial.print(buf[0]);
                Serial.print(".");
                Serial.print(buf[1]);
                Serial.print(buf[2]);
            } else if (strlen(buf) == 4) {
                Serial.print("0.");
                Serial.print(buf[0]);
                Serial.print(buf[1]);
            }  else if (strlen(buf) == 3) {
                Serial.print("0.0");
                Serial.print(buf[0]);
            } else {
                Serial.print("0.00");
            }            
            Serial.print("v");                
            break;
        case VALUE_BATTERY_VOLTAGE:
            fixed = (float)(2.45*(float)i); // r1= 3000, r2 = 10000
            ltoa(fixed,buf,10);
            if (strlen(buf) == 4) {
                Serial.print(buf[0]);
                Serial.print(buf[1]);
                Serial.print(".");
                Serial.print(buf[2]);
                Serial.print(buf[3]);
            } else
            if (strlen(buf) == 3) {
                Serial.print(" ");
                Serial.print(buf[0]);
                Serial.print(".");
                Serial.print(buf[1]);
                Serial.print(buf[2]);
            } else if (strlen(buf) == 2) {
                Serial.print(" 0.0");
                Serial.print(buf[0]);
            } else {
                Serial.print(" 0.00");
            }            
            Serial.print("v");                
            break;            
/*
        case VALUE_FIXED_POINT_2:
            // TODO         
            break;*/
        case VALUE_BOOLEAN:
            if (i==0) {
                Serial.print("  Off");
            } else {
                Serial.print("   On");                
            }
            break;
        case VALUE_HEX:
            ltoa(i,buf,16);
            printStringWithPadding(buf,5,' ');            
            Serial.print("h");
            break;
        case VALUE_INT:
        default:
            printIntWithPadding(i,5,' ');
            Serial.print("   ");            
    }
}

// Prints string from flash storage
char flashFetchBuffer[80];

char* fetchFromFlash(const char *str) {
    strcpy_P(flashFetchBuffer, (PGM_P)str);
    if (flashFetchBuffer[0] == 0)
        return NULL;
    return flashFetchBuffer;
}

bool printFromFlash(const char *str) {
    strcpy_P(flashFetchBuffer, (PGM_P)str);   
    if (flashFetchBuffer[0] == 0) {
        return false;
    }
    Serial.print(flashFetchBuffer);
    return true;
}

void printPads(unsigned char n, char padChar) {
    memset(flashFetchBuffer,padChar,n);
    flashFetchBuffer[n] = 0;
    Serial.print(flashFetchBuffer);
}

void printIntWithPadding(int val,unsigned char width,char padChar) {
    // print enough leading zeroes!
    memset(flashFetchBuffer,padChar,30);
    // append string presentation of number to end
    itoa(val,flashFetchBuffer+30,10);
    // print string with given width
    Serial.print(flashFetchBuffer+30+strlen(flashFetchBuffer+30)-width);
}

void printStringWithPadding(char *str,unsigned char width,char padChar) {
    // print enough leading zeroes!
    memset(flashFetchBuffer,padChar,30);
    // append string presentation of number to end
    //strcpy_P(flashFetchBuffer+30, (PGM_P)str);   
    strcpy(flashFetchBuffer+30, str);
    
    // print string with given width
    Serial.print(flashFetchBuffer+30+strlen(flashFetchBuffer+30)-width);
}

// VT102/Ansi functions

const char ANSIClearScreen[] PROGMEM =  { 27,'[','2','J',27,'[','H',0};
const char ANSIClearEolAndLf[] PROGMEM = { 27,'[','K','\r','\n',0};
const char ANSIClearEol[] PROGMEM = { 27,'[','K',0};
const char ANSIClearEos[] PROGMEM = { 27,'[','J',0};
const char ANSIHideCursor[] PROGMEM = { 27,'[','?','2','5','h',0}; // doesn't work?

void ansiGotoXy(char x,char y) {
  Serial.print("\e[");
  Serial.print(y,DEC);
  Serial.print(";");
  Serial.print(x,DEC);
  Serial.print("H");
}

void ansiClearScreen() {
  printFromFlash(ANSIClearScreen);
}

void ansiClearEolAndLf() {
  printFromFlash(ANSIClearEolAndLf);
}

void ansiClearEol() {
    printFromFlash(ANSIClearEol);
}

void ansiClearEos() {
  printFromFlash(ANSIClearEos);
}

void ansiHideCursor() {
    printFromFlash(ANSIHideCursor);
}


unsigned char mapValues(int raw,int mapMin,int mapMax) {
    if (raw < mapMin)
        return 0;
    if (raw >= mapMax)
        return 0xff;
    
    return map(raw,mapMin,mapMax,0,255);
}

unsigned char mapValuesSqueeze(int raw,int mapMin,int mapMax) {
    return map(raw,0,255,mapMin,mapMax);
}

// Interpolate value between p1 and p1, given by pos (0-99)
unsigned char mapInterpolate(unsigned char p1,unsigned char p2, unsigned char pos) {
    return (p1*(100-pos)+p2*pos)/100;
}

// Interpolate value between p1 and p1 (range 0-255), given by pos (0-99)
// returned value upscaled to 0-1023 range

unsigned int mapInterpolate10bit(unsigned char p1,unsigned char p2, unsigned char pos) {
    pos=pos/2;
    unsigned int pp1=p1*4;
    unsigned int pp2=p2*4;
    return (pp1*(50-pos)+pp2*pos)/50;
}

unsigned char mapLookUp(unsigned char *mapData,unsigned char x,unsigned char y) {
    unsigned char isInterpolated = *(mapData+2+2);
    unsigned char tableSizeX = *(mapData+3+2);
    unsigned char tableSizeY = *(mapData+4+2);
    unsigned char yPos;
    int ofs = 10; // skip headers

    *(mapData+ofs+tableSizeX*tableSizeY) = x;
    *(mapData+ofs+tableSizeX*tableSizeY+1) = y;

    if (tableSizeY) {
        yPos = y / (256/(tableSizeY-1));
    } 
    else {
        yPos = 0;
    }
    unsigned char xPos = (x / (256/(tableSizeX-1)));
    
    unsigned char p1 = *(mapData+ofs+(yPos*tableSizeX)+xPos);
    unsigned char p2 = *(mapData+ofs+(yPos*tableSizeX)+(((xPos+1)>=tableSizeX)?xPos:xPos+1));
    ;
    unsigned char p3 = *(mapData+ofs+((((yPos+1)>=tableSizeX)?yPos:yPos+1)*tableSizeX)+xPos);
    unsigned char p4 = *(mapData+ofs+((((yPos+1)>=tableSizeX)?yPos:yPos+1)*tableSizeX)+(((xPos+1)>=tableSizeX)?xPos:xPos+1));
    
    unsigned char ret;
    if (isInterpolated == 'D') {
        int amountX = (x % (256/(tableSizeX-1)))*(10000/(256/(tableSizeX-1)));
        if (tableSizeY) {
            // 2D
            int amountY = (y % (256/(tableSizeY-1)))*(10000/(256/(tableSizeY-1)));
            char y1 = mapInterpolate(p1,p2,amountX /100);
            char y2 = mapInterpolate(p3,p4,amountX /100);
            ret = mapInterpolate(y1,y2,amountY /100);
        } 
        else {
            // 1D
            ret = mapInterpolate(p1,p2,amountX /100);
        }
    } 
    else {
        ret = p1;
    }
    *(mapData+ofs+tableSizeX*tableSizeY+2) = ret;
    return ret;
}

unsigned int mapLookUp10bit(unsigned char *mapData,unsigned char x,unsigned char y) {
    unsigned char isInterpolated = *(mapData+2+2);
    unsigned char tableSizeX = *(mapData+3+2);
    unsigned char tableSizeY = *(mapData+4+2);
    unsigned char yPos;
    int ofs = 10; // skip headers

    *(mapData+ofs+tableSizeX*tableSizeY) = x;
    *(mapData+ofs+tableSizeX*tableSizeY+1) = y;
    
    if (tableSizeY) {
        yPos = y / (256/(tableSizeY-1));
    } 
    else {
        yPos = 0;
    }
    unsigned char xPos = (x / (256/(tableSizeX-1)));
    
    unsigned char p1 = *(mapData+ofs+(yPos*tableSizeX)+xPos);
    unsigned char p2 = *(mapData+ofs+(yPos*tableSizeX)+(((xPos+1)>=tableSizeX)?xPos:xPos+1));
    ;
    unsigned char p3 = *(mapData+ofs+((((yPos+1)>=tableSizeX)?yPos:yPos+1)*tableSizeX)+xPos);
    unsigned char p4 = *(mapData+ofs+((((yPos+1)>=tableSizeX)?yPos:yPos+1)*tableSizeX)+(((xPos+1)>=tableSizeX)?xPos:xPos+1));
    
    unsigned int ret;
    if (isInterpolated == 'D') {
        int amountX = (x % (256/(tableSizeX-1)))*(10000/(256/(tableSizeX-1)));
        if (tableSizeY) {
            // 2D
            int amountY = (y % (256/(tableSizeY-1)))*(10000/(256/(tableSizeY-1)));
            char y1 = mapInterpolate(p1,p2,amountX /100);
            char y2 = mapInterpolate(p3,p4,amountX /100);
            ret = mapInterpolate10bit(y1,y2,amountY /100);
        } 
        else {
            // 1D
            ret = mapInterpolate10bit(p1,p2,amountX /100);
        }
    } 
    else {
        ret = p1*4;
    }
    *(mapData+ofs+tableSizeX*tableSizeY+2) = ret/4;
    *(unsigned int*)(mapData+ofs+tableSizeX*tableSizeY+3) = ret;
    return ret;
}

void printMapAxis(unsigned char axisType,unsigned char value,bool verbose) {
    switch (axisType) {
        case MAP_AXIS_NONE:
            break;
        case MAP_AXIS_RPM:
            Serial.print(toRpm(value),DEC);
            if (verbose) Serial.print(" Rpm");
            break;
        case MAP_AXIS_IDLERPM:
            Serial.print(value*4,DEC);
            if (verbose) Serial.print(" Rpm");
            break;            
        case MAP_AXIS_TPS:
            Serial.print(toTps(value),DEC);
            if (verbose) Serial.print("% TPS");
            break;
        case MAP_AXIS_KPA:
            Serial.print(toKpa(value),DEC);
            if (verbose) Serial.print(" kPa");
            break;
        case MAP_AXIS_VOLTAGE:
            //Serial.print(toVoltage(value),DEC);
            //if (verbose) Serial.print(" mV");
            printValue(value*4,VALUE_VOLTAGE);
            break;
        case MAP_AXIS_CELSIUS:
            Serial.print(toTemperature(value),DEC);
            if (verbose) Serial.print(" °C");
            break;
        case MAP_AXIS_INJECTION_TIMING:
            //Serial.print(toVoltage(value),DEC);
            //if (verbose) Serial.print(" mV");
            printValue(value*4,VALUE_INJECTION_TIMING);
            break;   
         case MAP_AXIS_INJECTED_FUEL:
            Serial.print(value,DEC);
            if (verbose) Serial.print(" IQ");
            break;
         case MAP_AXIS_FUEL_TRIM_AMOUNT:
            Serial.print((int)value-128,DEC);
            if (verbose) Serial.print(" Ftrim");
            break;                         
        default:
            Serial.print(value);
            if (verbose) Serial.print(" Raw");
    }
}

int toKpa(int raw) {
    unsigned long f = (((unsigned long)core.node[Core::nodeMAPkPa].value*256ul)*(unsigned long)raw)/65280ul;// 65536ul;
    return f;
    //return raw*1.18; // defaults for 3bar map
}

int toTemperature( int rawValue) {
    // 0..255 --> -35 .. 157

    return ((rawValue*3)/4)-35;
}

int toVoltage(int raw) {
    // mVolt
    return int(raw*19.64); // ~4.98v
}

int toRpm(int raw) {
    unsigned long f = (((unsigned long)core.node[Core::nodeControlMapScaleRPM].value*256ul)*(unsigned long)raw)/65280ul;//65536ul;
    return f;  
//    return round(((float)core.node[Core::nodeRPMMax].value/255.0)*(float)raw);
}

int toTps(int raw) {
    // percent
    return int(raw/2.55);
}

int tempSensorBcoefficientCalc(int raw,int bCoEfficient, int nResistance,int nTemp) {
    float seriesResistor = TEMP_SENSOR_SERIES_RESITOR;
    float nominalResistance = nResistance;
    float nominalTemp = nTemp;

    //convert value to resistance
    float resistance = (1023.0f / (float)raw) - 1.0f;
    resistance = seriesResistor / resistance;

    float steinhart = resistance / nominalResistance; // (R/Ro)
    steinhart = log(steinhart); // ln(R/Ro)
    steinhart = steinhart / (float)bCoEfficient; // 1/B * ln(R/Ro)
    steinhart += 1.0f / (nominalTemp + 273.15); // + (1/To)
    steinhart = 1.0f / steinhart; // Invert
    steinhart -= 273.15f; // convert to C

    // convert to internal 8bit format
    if (steinhart < -35)
        steinhart = -35;
    if (steinhart > 156)
        steinhart = 156;

    int ret = ((steinhart+35)*4)/3;
        
    return ret;
}



// a0 54, a15 69
// 
static volatile float aOutput[PIN_A15-PIN_A0+1]={-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1};
static volatile float aOversamplingFactor[PIN_A15-PIN_A0+1] = {
    PIN_ANALOG_SMOOTHING_A0,
    PIN_ANALOG_SMOOTHING_A1,
    PIN_ANALOG_SMOOTHING_A2,
    PIN_ANALOG_SMOOTHING_A3,
    PIN_ANALOG_SMOOTHING_A4,
    PIN_ANALOG_SMOOTHING_A5,
    PIN_ANALOG_SMOOTHING_A6,
    PIN_ANALOG_SMOOTHING_A7,
    PIN_ANALOG_SMOOTHING_A8,
    PIN_ANALOG_SMOOTHING_A9,
    PIN_ANALOG_SMOOTHING_A10,
    PIN_ANALOG_SMOOTHING_A11,
    PIN_ANALOG_SMOOTHING_A12,
    PIN_ANALOG_SMOOTHING_A13,
    PIN_ANALOG_SMOOTHING_A14,
    PIN_ANALOG_SMOOTHING_A15,
};

static volatile unsigned char analogReadState = false;
int safeAnalogRead(int pin) {
    analogReadState = 0;
    int input = analogRead(pin);
    // if state is set, then the interrupt handler has probably screwed up earlier conversion,    
    if (analogReadState)  {
   		dtc.setError(DTC_TRAP_2);  
        analogReadState = 0;
        // do not trust interrupted signal, use old output value instead
        return aOutput[pin-PIN_A0];
    }
    int ret = applyOversampling(pin,input);
    return ret;
}
int safeAnalogReadFromInterrupt(int pin) {
    analogReadState = 1;
//   int ret = applyOversampling(pin,analogRead(pin));
    int ret = analogRead(pin);
    return ret;
}

float applyOversampling(int pin,float input) {
    unsigned char idx = pin - PIN_A0;
    if (aOutput[idx] == -1)
        aOutput[idx] = input;
    aOutput[idx] += (input-aOutput[idx]) * aOversamplingFactor[idx];
    return round(aOutput[idx]);
//    output += ((input-output)*a);
}