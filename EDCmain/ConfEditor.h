#ifndef CONFEDITOR_H
#define CONFEDITOR_H

#include "defines.h"
#include "Arduino.h"
#include "Core.h"

class ConfEditor {
private:
    char systemStatusMessage[20];
    char keyPressed;
    char page;
    char statusIndex;
    bool statusPrinted;
    unsigned char tick;
    
    int corePageNumber;
    char activeRow,activeRowOld;
    
    struct mapEditorDataStruct {
        char cursorX,cursorXold;
        char cursorY,cursorYold;
        unsigned char lastX;
        unsigned char lastY;
        char currentMap;
        unsigned char clipboard;
    } mapEditorData;

    unsigned char mapIdx;
public:
    bool liveMode;
    bool uiEnabled;
    
private:
    void printHeader();
    void mainScreen();
    void pageDTC();
    void pageAdaptation();
    void pageMapEditor(bool compactMode=false);
    void pageOutputTests();
    void pageVisualizer();
    void pageBoostWorkBench();
    
public:
    ConfEditor();
    void handleInput(char node);
    void refresh(); 
    void setSystemStatusMessage(const char *msg); 
    void toggleStatus();
};

extern const char nodeDescription[][55] PROGMEM;

extern ConfEditor confeditor;

#endif
