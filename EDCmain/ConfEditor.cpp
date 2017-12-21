#include "ConfEditor.h"
#include "Arduino.h"
#include "utils.h"
#include "Core.h"
#include "DTC.h"

ConfEditor confeditor;

const char confEditorHeader[][80] PROGMEM = {
	PRODUCT_NAME,"Status:",
	" ",
	" ",0};

const char confEditorMainScreen[][80] PROGMEM = {
	"Global shortcuts:",
	"  <0> Main menu",
	"  <1> DTC memory",
	"  <2> Configuration/Adaptation",
	"  <3> Map Editor",
	"  <4> Output tests",
	"  <5> Visualizer",	
	"  <6> Boost Control Workbench",		
	"  <.> Toggle status indicator (Status/RPM/TPS/Map)",    
	" ",
	"Send feedback to syncro16@outlook.com or visit http://dmn.kuulalaakeri.org/",
	0};

const char confEditorDTCText[][80] PROGMEM = {"Keys: r - reset fault memory, g - generate error","DTC Name","Count",0};

const char confEditorAdaptationText[][80] PROGMEM = {
	"Keys: -/+ or </> adjust, <spc> raw/human, <cursor nodes> move, s - save",
	// 01234567890123456789012345678901234567890123456789012345678901234567890123456789
	"Item description (Page x/x)                       SetPoint  Input     Actual",
	"-----------------------------------------------------------------------------",    
	0};

const char confEditorMapCurrentOutput[] PROGMEM = "Current output (8bit / 10bit): ";
const char confEditorMapEditorHelp[] PROGMEM = "Keys: -/+ adjust, </> change, c - copy, v - paste, s - save, <cursor keys> move";
const char confEditorMapCurrentMap[] PROGMEM = "Current map:";

ConfEditor::ConfEditor() {
	liveMode = false;
	keyPressed = -1;
	mapIdx = 0;
	page = 0;
	statusPrinted = false;
	uiEnabled = false;
  setSystemStatusMessage("");  
}

void ConfEditor::printHeader() {
	char *row;
	char *status;
	char buf[16];
	
	// refrash  
	ansiGotoXy(1,1);
	ansiClearEol();
	
	row = fetchFromFlash(confEditorMainScreen[1+page])+6;
	Serial.print(row);
	
	row = fetchFromFlash(confEditorHeader[0]);
	Serial.print(" (");
	Serial.print(row);
	Serial.print(")");

	row = fetchFromFlash(confEditorHeader[1]);

	switch (statusIndex % 4) {
		default:
			status = (char*)&systemStatusMessage;
			break;
		case 1:
			itoa(core.controls[Core::valueEngineRPM],buf,10);
			memcpy(buf+strlen(buf)," RPM",5);
			status = buf;
			break;
		case 2:
			itoa(core.controls[Core::valueTPSActual],buf,10);
			memcpy(buf+strlen(buf)," TPS",5);
			status = buf;            break;
		case 3:
			itoa(core.controls[Core::valueBoostPressure],buf,10);
			memcpy(buf+strlen(buf)," kPa",5);
			status = buf;            break;            
	}
	ansiGotoXy(80-strlen(row)-strlen(status),1);        
	Serial.print(row);
	Serial.print(status);
}

void ConfEditor::mainScreen() {
	if (keyPressed == -1)
		return;
	
	char *row;
	
	unsigned char mapIdx=0;
	while ((row = fetchFromFlash(confEditorMainScreen[mapIdx]))) {
		ansiGotoXy(1,mapIdx+3);
		Serial.print(row);
		mapIdx++;
	}
}

const char confEditorOutputTestsGlow[80] PROGMEM =   {"  <Q>          Glow Plug status:"};
const char confEditorOutputTestsFan[80] PROGMEM =    {"  <A>                Fan status:"};
const char confEditorOutputPumpAdvance[80] PROGMEM = {"  t/T   Pump advance duty cycle:"};
const char confEditorOutputN75[80] PROGMEM =         {"  n/N            N75 duty cycle:"};
const char confEditorOutputLabelOn[80] PROGMEM = "ON";
const char confEditorOutputLabelOff[80] PROGMEM = "OFF";

void ConfEditor::pageOutputTests() {
	bool redrawView = false;

	core.controls[Core::valueOutputTestMode] = true;

	switch (keyPressed) {
		case -1:
			break;
		case 'Q':
		case 'q':
			core.controls[Core::valueOutputGlow] = !core.controls[Core::valueOutputGlow];
			redrawView = true;
			break;
		case 'A':
		case 'a':
			core.controls[Core::valueOutputFan] = !core.controls[Core::valueOutputFan];
			redrawView = true;
			break;
		case 't':
			core.controls[Core::valueEngineTimingDutyCycle] -= 16;
			if (core.controls[Core::valueEngineTimingDutyCycle]<0)
					core.controls[Core::valueEngineTimingDutyCycle] = 0;
			redrawView = true;				
			break;
		case 'T':
			core.controls[Core::valueEngineTimingDutyCycle] += 16;
			if (core.controls[Core::valueEngineTimingDutyCycle]>255)
					core.controls[Core::valueEngineTimingDutyCycle] = 255;
			redrawView = true;						
			break;
		case 'n':
			core.controls[Core::valueN75DutyCycle] -= 16;
			if (core.controls[Core::valueN75DutyCycle]<0)
					core.controls[Core::valueN75DutyCycle] = 0;
			redrawView = true;				
			break;
		case 'N':
			core.controls[Core::valueN75DutyCycle] += 16;
			if (core.controls[Core::valueN75DutyCycle]>255)
					core.controls[Core::valueN75DutyCycle] = 255;
			redrawView = true;						
			break;

		default:
			redrawView = true;
	} 
	if (redrawView) {
		ansiClearScreen();
		printHeader();		
		ansiGotoXy(1,5);
		Serial.print(fetchFromFlash(confEditorOutputTestsGlow));
		ansiGotoXy(35,5);
		Serial.print(core.controls[Core::valueOutputGlow]?fetchFromFlash(confEditorOutputLabelOn):fetchFromFlash(confEditorOutputLabelOff));
		ansiGotoXy(1,6);
		Serial.print(fetchFromFlash(confEditorOutputTestsFan));
		ansiGotoXy(35,6);
		Serial.print(core.controls[Core::valueOutputFan]?fetchFromFlash(confEditorOutputLabelOn):fetchFromFlash(confEditorOutputLabelOff));		
		ansiGotoXy(1,7);
		Serial.print(fetchFromFlash(confEditorOutputPumpAdvance));
		ansiGotoXy(35,7);
		Serial.print(core.controls[Core::valueEngineTimingDutyCycle]);	
		ansiGotoXy(1,8);
		Serial.print(fetchFromFlash(confEditorOutputN75));
		ansiGotoXy(35,8);
		Serial.print(core.controls[Core::valueN75DutyCycle]);						
	}
}

//const char confEditorVisualizer[80] PROGMEM =             {"A1234567890 0123456789ABCDEF0123456789ABCDEF0123456789ABCDEF0123456789ABCDEF"};

const char confEditorVisualizerTPS[80] PROGMEM =            {"        TPS 0%-------------25%-------------50%--------------75%---------100%"};
const char confEditorVisualizerFuelAmount[80] PROGMEM =     {" FuelAmount 0%-------------25%-------------50%--------------75%---------100%"};
const char confEditorVisualizerQAFB[80] PROGMEM = 			{"QA FeedBack 0%-------------25%-------------50%--------------75%---------100%"};
const char confEditorVisualizerAdvance[80] PROGMEM =        {"    Advance 0%-------------25%-------------50%--------------75%---------100%"};
const char confEditorVisualizerN75[80] PROGMEM =     		{"        N75 0%-------------25%-------------50%--------------75%---------100%"};
const char confEditorVisualizerMapActual[80] PROGMEM = 		{" MAP Actual 0%-------------25%-------------50%--------------75%---------100%"};
const char confEditorVisualizerMapRequest[80] PROGMEM = 	{"MAP Request 0%-------------25%-------------50%--------------75%---------100%"};

const char confEditorVisualizerRPM[80] PROGMEM = 			{"        RPM "};



void ConfEditor::pageVisualizer() {
	static char oldTps,oldFuelAmount,oldAdvance,oldN75,oldQAFB,oldMap,oldMapRequest;
	int oldRPM;
	char *buf = fetchFromFlash(confEditorVisualizerTPS);
	if (keyPressed != -1) {
		oldTps = -1;
		oldFuelAmount = -1;
		oldAdvance = -1;
		oldN75 = -1;
		oldQAFB = -1;
		oldMap = -1;
		oldRPM = -1;
		ansiClearScreen();
		printHeader();

		buf=fetchFromFlash(confEditorVisualizerTPS);
		ansiGotoXy(13,5);
		Serial.print(buf+12);
		ansiGotoXy(1,4);
		buf[11] = 0;
		Serial.print(buf);

		buf=fetchFromFlash(confEditorVisualizerFuelAmount);
		ansiGotoXy(13,5+2*1);
		Serial.print(buf+12);
		ansiGotoXy(1,4+2*1);
		buf[11] = 0;
		Serial.print(buf);


		buf=fetchFromFlash(confEditorVisualizerQAFB);
		ansiGotoXy(13,5+2*2);
		Serial.print(buf+12);
		ansiGotoXy(1,4+2*2);
		buf[11] = 0;
		Serial.print(buf);

		buf=fetchFromFlash(confEditorVisualizerAdvance);
		ansiGotoXy(13,5+2*3);
		Serial.print(buf+12);
		ansiGotoXy(1,4+2*3);
		buf[11] = 0;
		Serial.print(buf);

		buf=fetchFromFlash(confEditorVisualizerN75);
		ansiGotoXy(13,5+2*4);
		Serial.print(buf+12);
		ansiGotoXy(1,4+2*4);
		buf[11] = 0;
		Serial.print(buf);
		
		buf=fetchFromFlash(confEditorVisualizerMapActual);
		ansiGotoXy(13,5+2*5);
		Serial.print(buf+12);
		ansiGotoXy(1,4+2*5);
		buf[11] = 0;
		Serial.print(buf);

		buf=fetchFromFlash(confEditorVisualizerMapRequest);
		ansiGotoXy(13,5+2*6);
		Serial.print(buf+12);
		ansiGotoXy(1,4+2*6);
		buf[11] = 0;
		Serial.print(buf);

		buf=fetchFromFlash(confEditorVisualizerRPM);
		ansiGotoXy(1,4+2*7);
		Serial.print(buf);
	}
	if (core.controls[Core::valueTPSActual]/4 != oldTps) {
		oldTps = core.controls[Core::valueTPSActual]/4;
		ansiGotoXy(13,4+2*0);
		for (char i=0;i<oldTps;i++)
			Serial.print("*");
		ansiClearEol();
	}

	if (core.controls[Core::valueFuelAmount8bit]/4 != oldFuelAmount) {
		oldFuelAmount = core.controls[Core::valueFuelAmount8bit]/4;
		ansiGotoXy(13,4+2*1);
		for (char i=0;i<oldFuelAmount;i++)
			Serial.print("*");
		ansiClearEol();
	}

	if (core.controls[Core::valueQAfeedbackRaw]/16 != oldQAFB) {
		oldQAFB = core.controls[Core::valueQAfeedbackRaw]/16;
		ansiGotoXy(13,4+2*2);
		for (char i=0;i<oldQAFB;i++)
			Serial.print("*");
		ansiClearEol();	
	}

	if (core.controls[Core::valueEngineTimingDutyCycle]/4 != oldAdvance) {
		oldAdvance = core.controls[Core::valueEngineTimingDutyCycle]/4;
		ansiGotoXy(13,4+2*3);
		for (char i=0;i<oldAdvance;i++)
			Serial.print("*");
		ansiClearEol();
	}

	if (core.controls[Core::valueN75DutyCycle]/4 != oldN75) {
		oldN75 = core.controls[Core::valueN75DutyCycle]/4;
		ansiGotoXy(13,4+2*4);
		for (char i=0;i<oldN75;i++)
			Serial.print("*");
		ansiClearEol();	
	}	


	if (core.controls[Core::valueBoostPressure]/4 != oldMap) {
		oldMap = core.controls[Core::valueBoostPressure]/4;
		ansiGotoXy(13,4+2*5);
		for (char i=0;i<oldMap;i++)
			Serial.print("*");
		ansiClearEol();	
	}	
	if (core.controls[Core::valueBoostTarget]/4 != oldMapRequest) {
		oldMapRequest = core.controls[Core::valueBoostTarget]/4;
		ansiGotoXy(13,4+2*6);
		for (char i=0;i<oldMapRequest;i++)
			Serial.print("*");
		ansiClearEol();	
	}		
	if ((core.controls[Core::valueEngineRPM]/10)*10 != oldRPM) {
		oldRPM = (core.controls[Core::valueEngineRPM]/10)*10;
		ansiGotoXy(13,4+2*7);
		printIntWithPadding(oldRPM,5,' ');
		ansiClearEol();
	}
	if (keyPressed == 'P') {
		core.controls[Core::valueQADebug] = !core.controls[Core::valueQADebug];
		if (core.controls[Core::valueQADebug]) Serial.println("QA Disabled");
	}

	ansiGotoXy(13,4+2*8);
	if (core.controls[Core::valueBoostActuatorClipReason] == BOOST_MIN_CLIP) {
		Serial.print("min clip");
	} else 	if (core.controls[Core::valueBoostActuatorClipReason] == BOOST_MAX_CLIP) {
		Serial.print("MAX clip");
	} else {
		ansiClearEol();
	}


}

void ConfEditor::pageDTC() {
	switch (keyPressed) {
		case 'R':            
		case 'r':
			dtc.resetAll();
			break;
		case 'g':
			dtc.setError(DTC_TRAP_1);
			break;
	}
	if (keyPressed != -1 || tick % 32 == 0) {
		ansiClearScreen();
		printHeader();
		
		char *row;
		
		ansiGotoXy(1,3);
		
		row = fetchFromFlash(confEditorDTCText[0]);
		Serial.println(row);
		
		ansiGotoXy(1,5);
		row = fetchFromFlash(confEditorDTCText[1]);
		Serial.print(row);
		row = fetchFromFlash(confEditorDTCText[2]);
		ansiGotoXy(60,5);
		ansiGotoXy(1,6);
		printPads(70,'-');
		
		char y=7;
		while (dtc.seekNextError()) {        
			ansiGotoXy(3,y);
			
			Serial.print(dtc.getName());
			ansiGotoXy(60,y);
			
			Serial.print(dtc.getCount());
			y++;
		}

	}    
}

void ConfEditor::pageAdaptation() {
	char *row;
	bool redrawView = false;
	int i;
	const int rows=17;
	nodeStruct *item;
	
	int mapIdx = activeRow+rows*corePageNumber;
	char amount = 1;
	switch (keyPressed) {
		case 's':
			core.save();
			break;
		case KEY_LEFT:
			if (corePageNumber>0) {
				corePageNumber--;
				activeRow = 0;
			}
			redrawView = true;
			break;
		case KEY_RIGHT:
			if ((corePageNumber+1)*rows<Core::NODE_MAX) {
				corePageNumber++;
				activeRow = 0;
			}
			redrawView = true;
			break;;
		case KEY_UP:
			if (activeRow>0)
				activeRow--;
			break;
		case KEY_DOWN:
			if (activeRow<(rows-1) && mapIdx<Core::NODE_MAX)
				activeRow++;
			break;
		case '>':
			amount = 20;
		case '+':
			core.setCurrentNode(mapIdx);
			for (char i=0;i<amount;i++)
				core.incValue();
			break;
		case '<':
			amount = 20;            
		case '-':
			core.setCurrentNode(mapIdx);
			for (char i=0;i<amount;i++)
				core.decValue();
			break;
		case ' ':
			toggleHumanReadableValues();
			break;
		case -1:
			break;
			
		default:
			redrawView = true;
			break;

	}
	
	if (redrawView) {
		// refresh entire screen
		ansiClearScreen();
		printHeader();
		
		ansiGotoXy(1,3);
		
		row = fetchFromFlash(confEditorAdaptationText[0]);
		Serial.print(row);
		
		ansiGotoXy(1,5);
		printFromFlash(confEditorAdaptationText[1]);
		
		ansiGotoXy(1,6);
		printFromFlash(confEditorAdaptationText[2]);
		
		ansiGotoXy(24,5);
		Serial.print(corePageNumber+1);
		ansiGotoXy(26,5);
		Serial.print((Core::NODE_MAX+rows-1)/rows);
		
		for (i=0;i<rows;i++) {
			int mapIdx = i+rows*corePageNumber;
			if (mapIdx<=Core::NODE_MAX) {
				core.setCurrentNode(mapIdx);
				item = core.getNodeData();

                if (item->properties) {   
					ansiGotoXy(3,7+i);

					// Serial.print(item->description);
					printFromFlash(nodeDescription[mapIdx]);
					if ((item->properties & NODE_PROPERTY_LOCKED) == NODE_PROPERTY_LOCKED) {
						Serial.print(" (view only)");
					} 
				}
			}
		}

	}
	
	// current conf/sensor values
	if (redrawView || keyPressed != -1 || tick % 16 == 0) {
		for (i=0;i<rows;i++) {
			int mapIdx = i+rows*corePageNumber;        
			if (mapIdx<=Core::NODE_MAX) {
				core.setCurrentNode(mapIdx);            
				item = core.getNodeData();
				if (item->properties) {   
					ansiGotoXy(50,7+i);                
					printValue(item->value,item->type);
					
					if (item->rawValueKey != Core::valueNone) {
						ansiGotoXy(60,7+i);
						//Serial.print(controls[item->rawValueKey]);
						printValue(core.controls[item->rawValueKey],item->type);
					}
					
					if (item->actualValueKey != Core::valueNone) {
						ansiGotoXy(70,7+i);
						//Serial.print(controls[item->actualValueKey]);
						printValue(core.controls[item->actualValueKey],VALUE_PERCENTAGE);
					}
					
				}
			}        
		}
		
		// (re)Draw cursor
		
		if (activeRow != activeRowOld) {
			ansiGotoXy(1,7+activeRowOld);
			Serial.print("  ");
			ansiGotoXy(50-2,7+activeRowOld);
			Serial.print(" ");
			ansiGotoXy(58,7+activeRowOld);
			Serial.print(" ");
			activeRowOld = activeRow;
			
		}
		ansiGotoXy(1,7+activeRow);
		Serial.print(">>");
		ansiGotoXy(50-2,7+activeRow);
		Serial.print(">");
		ansiGotoXy(58,7+activeRow);
		Serial.print("<");
	}
	
}

void ConfEditor::pageMapEditor(bool compactMode = false) {    
	bool updateCell = false;
	bool updateCursor = false;
	bool redrawView = false;
	// unsigned char *mapData = editorMaps[mapIdx];
	//unsigned char *mapData = core.boostMap;
	unsigned char *mapData = core.maps[mapIdx]+2; // 2 skip file_id offset

	unsigned char tableSizeX = *(mapData+3);
	unsigned char tableSizeY = *(mapData+4);
	unsigned char axisTypeX = *(mapData+5);
	unsigned char axisTypeY = *(mapData+6);
	unsigned char axisTypeResult = *(mapData+7);
	unsigned char lastXpos = *(mapData+8+tableSizeX*tableSizeY);
	unsigned char lastYpos = *(mapData+8+tableSizeX*tableSizeY+1);
	unsigned char lastValue = *(mapData+8+tableSizeX*tableSizeY+2);
	unsigned int lastValue10b = *(unsigned int*)(mapData+8+tableSizeX*tableSizeY+3);
	
	const char xPad = 5;
	char xSpace ;

	if (tableSizeX>8) {
		xSpace = 5;
	} else {
		xSpace = 7;
	}


	const char yPad = 7;
	const char ySpace = 2;
	
	switch (keyPressed) {
		case 'c':
			mapEditorData.clipboard = *(mapData+8+mapEditorData.cursorX+mapEditorData.cursorY*tableSizeX);
			break;
		case 'v':
			(*(mapData+8+mapEditorData.cursorX+mapEditorData.cursorY*tableSizeX)) = mapEditorData.clipboard;
			updateCell = true;
			break;
		case 'h':
			ansiGotoXy(xPad+xSpace+mapEditorData.cursorX*xSpace,yPad+ySpace+mapEditorData.cursorY*ySpace);
			Serial.print(" ");
			ansiGotoXy(xPad+xSpace+mapEditorData.cursorX*xSpace+xSpace,yPad+ySpace+mapEditorData.cursorY*ySpace);
			Serial.print(" ");  
			if (mapEditorData.cursorX>0)
				mapEditorData.cursorX--;
			updateCursor = true;
			break;
		case 'l':
			ansiGotoXy(xPad+xSpace+mapEditorData.cursorX*xSpace,yPad+ySpace+mapEditorData.cursorY*ySpace);
			Serial.print(" ");
			ansiGotoXy(xPad+xSpace+mapEditorData.cursorX*xSpace+xSpace,yPad+ySpace+mapEditorData.cursorY*ySpace);
			Serial.print(" "); 
			if (mapEditorData.cursorX<tableSizeX-1)
				mapEditorData.cursorX++;
			updateCursor = true;
			break;
		case 'k':
			ansiGotoXy(xPad+xSpace+mapEditorData.cursorX*xSpace,yPad+ySpace+mapEditorData.cursorY*ySpace);
			Serial.print(" ");
			ansiGotoXy(xPad+xSpace+mapEditorData.cursorX*xSpace+xSpace,yPad+ySpace+mapEditorData.cursorY*ySpace);
			Serial.print(" ");  
			if (mapEditorData.cursorY>0)
				mapEditorData.cursorY--;
			updateCursor = true;
			break;
		case 'j':
			ansiGotoXy(xPad+xSpace+mapEditorData.cursorX*xSpace,yPad+ySpace+mapEditorData.cursorY*ySpace);
			Serial.print(" ");
			ansiGotoXy(xPad+xSpace+mapEditorData.cursorX*xSpace+xSpace,yPad+ySpace+mapEditorData.cursorY*ySpace);
			Serial.print(" ");  
			if (mapEditorData.cursorY<tableSizeY-1)
				mapEditorData.cursorY++;
			updateCursor = true;
			break;
		case '+':
			if (*(mapData+8+mapEditorData.cursorX+mapEditorData.cursorY*tableSizeX)<0xff)
				(*(mapData+8+mapEditorData.cursorX+mapEditorData.cursorY*tableSizeX))++;
			updateCell = true;            
			break;
		case '-':
			if (*(mapData+8+mapEditorData.cursorX+mapEditorData.cursorY*tableSizeX)>0)
				(*(mapData+8+mapEditorData.cursorX+mapEditorData.cursorY*tableSizeX))--;
			updateCell = true;            
			break;
		case 's':
			core.save();
			break;
		case -1:
			break;
		case '<':
			if (mapIdx > 0 ) mapIdx--;
			keyPressed = 0;
			pageMapEditor();
			return;
		case '>':
			if (mapIdx<core.numberOfMaps-1)
				mapIdx++;
			keyPressed = 0;
			pageMapEditor();
			return;
		default:
			redrawView = true;
			updateCursor = true;
			if (mapEditorData.currentMap!=mapIdx) {
				mapEditorData.cursorX=0;
				mapEditorData.cursorY=0;
				mapEditorData.currentMap = mapIdx;
			}
	}
	if (redrawView) {
		ansiClearScreen();
		printHeader();
		ansiGotoXy(0,3);
		printFromFlash(confEditorMapEditorHelp);
		
		ansiGotoXy(0,5);
		printFromFlash(confEditorMapCurrentMap);
		Serial.print(core.mapNames[mapIdx]);
		
		// Table X header
		
		for (int x=0;x<tableSizeX;x++) {
			ansiGotoXy(xPad+(1+x)*xSpace,yPad);
			int mapIdx = round((float)((255/(float)(tableSizeX-1)))*(float)x);
			printPads(1,' ');
			printMapAxis(axisTypeX,mapIdx, ((x==0||x==(tableSizeX-1))?true:false));
		}
		ansiGotoXy(xPad+xSpace,yPad+1);
		printPads(tableSizeX*xSpace,'-');
		
		// Table Y header
		
		for (int y=0;y<tableSizeY;y++) {
			ansiGotoXy(xPad-1,yPad+(1+y)*ySpace);
			int mapIdx = round((float)((255/(float)(tableSizeY-1)))*(float)y);
			
			printMapAxis(axisTypeY,mapIdx,true);
			ansiGotoXy(xPad+xSpace-1,yPad+(1+y)*ySpace);
			Serial.print("|");
			if (y<tableSizeY-1) {
				ansiGotoXy(xPad+xSpace-1,yPad+(1+y)*ySpace+1); // works for ySpace=2
				Serial.print("|");
			}
			
		}
		for (int y=0;y<tableSizeY;y++) {
			for (int x=0;x<tableSizeX;x++) {
				ansiGotoXy(xPad+(1+x)*xSpace,yPad+(1+y)*ySpace);
				printPads(1,' ');
				//Serial.print(*(mapData+8+x*y),DEC);
				printMapAxis(axisTypeResult,*(mapData+8+x+y*tableSizeX),0);
			}
		}
	}
	
	if (updateCell) {
		ansiGotoXy(xPad+xSpace+mapEditorData.cursorX*xSpace+1,yPad+ySpace+mapEditorData.cursorY*ySpace);
		printPads(xSpace-2,' ');
		ansiGotoXy(xPad+xSpace+mapEditorData.cursorX*xSpace+1,yPad+ySpace+mapEditorData.cursorY*ySpace);
		printMapAxis(axisTypeResult,*(mapData+8+mapEditorData.cursorX+mapEditorData.cursorY*tableSizeX),0);
	}

	if (updateCursor) {
		ansiGotoXy(xPad+xSpace+mapEditorData.cursorX*xSpace,yPad+ySpace+mapEditorData.cursorY*ySpace);
		Serial.print(">");
		ansiGotoXy(xPad+xSpace+mapEditorData.cursorX*xSpace+xSpace,yPad+ySpace+mapEditorData.cursorY*ySpace);
		Serial.print("<");  
	}

	// table live view (last queried X, Y and returned interpolated value (8bit or 10bit interpolated value)
	if (tick % 4 == 0) {
		ansiGotoXy(2,yPad+ySpace+round((float)mapEditorData.lastY*(float)((float)(tableSizeY-1)*(float)ySpace/255)));
		Serial.print("  ");
		ansiGotoXy(xPad+xSpace+round((float)mapEditorData.lastX*(float)((float)tableSizeX*(float)xSpace/255)),yPad-1);
		Serial.print(" ");
		
		mapEditorData.lastY = lastYpos;
		mapEditorData.lastX = lastXpos;
		
		ansiGotoXy(2,yPad+ySpace+round((float)lastYpos*(float)((float)(tableSizeY-1)*(float)ySpace/255)));
		Serial.print(">>");
		ansiGotoXy(xPad+xSpace+round((float)lastXpos*(float)((float)tableSizeX*(float)xSpace/255)),yPad-1);
		Serial.print("v"); 

		if (!compactMode) {
			ansiGotoXy(xPad+xSpace,yPad+tableSizeY*ySpace+2);
			printFromFlash(confEditorMapCurrentOutput);
			printMapAxis(axisTypeResult,lastValue,1);
			Serial.print(" / ");
			Serial.print(lastValue10b);
			ansiClearEol();
		}
		
	}

}

void ConfEditor::setSystemStatusMessage(const char *msg) {
	statusPrinted = false;
	if (strlen(msg) >= sizeof(systemStatusMessage)) {
		memcpy(systemStatusMessage,msg,sizeof(systemStatusMessage)-1);
		systemStatusMessage[sizeof(systemStatusMessage)] = 0;
	} else {
		strcpy(systemStatusMessage,msg);
	}
}

void ConfEditor::toggleStatus() {
	statusIndex++;
	statusPrinted=false;
}

void ConfEditor::handleInput(char node) {
	keyPressed = node;
	if (!uiEnabled) {
		ansiClearScreen();
		uiEnabled = true;
	}
	if (node == '.')
		toggleStatus();
	if (node>='0' && node<='9') {
		page = node-'0';
		ansiClearScreen();
		tick = 0;
		statusPrinted = false;
	}
}

void ConfEditor::refresh() {
	if (!uiEnabled)
		return;
	
	if (page>6)
		page = 0;
	
	if (!statusPrinted/* || statusIndex != 0*/) {
		printHeader();
		statusPrinted = true;
	}
	
	switch (page) {
		case 0:
			core.controls[Core::valueOutputTestMode] = false;
			mainScreen();
			break;
		case 1:
			core.controls[Core::valueOutputTestMode] = false;		
			pageDTC();
			break;
		case 2:
			core.controls[Core::valueOutputTestMode] = false;		
			pageAdaptation();
			break;
		case 3:
			core.controls[Core::valueOutputTestMode] = false;		
			pageMapEditor();
			break;
		case 4:
			core.controls[Core::valueOutputTestMode] = true;		
			pageOutputTests();
			break;
		case 5:
			core.controls[Core::valueOutputTestMode] = false;		
			pageVisualizer();
			break;	
		case 6:
			core.controls[Core::valueOutputTestMode] = false;		
			pageBoostWorkBench();
			break;									
	}
	keyPressed = -1;
	tick++;
}



const char BWBheader[] PROGMEM = "          0---------------64-------------128-------------192------------255";
//                             0123456789012345678901234567890123456789012345678901234567890123456789
const char BWBpid[] PROGMEM = "   PID:  p/P:      i/I:      d/D:                 s/S(peed)      b/B(ias)       ";
const char BWBtexts[][32] PROGMEM = {"    Base:","     PID:","   Total:","     Map:","Setpoint:","     RPM:","      IQ:","  N75 DC:"};
void ConfEditor::pageBoostWorkBench() {  

	mapIdx = Core::mapIdxTurboControlMap;
	bool redrawView = false;
	switch (keyPressed) {
		case 'p':
			core.setCurrentNode(Core::nodeBoostKp);
			if (core.node[Core::nodeBoostKp].value)
				core.node[Core::nodeBoostKp].value--;
			keyPressed=-1;
			ansiGotoXy(14,21);
			printIntWithPadding(core.getNodeData()->value,3,' ');
		break;
		case 'P':
			core.setCurrentNode(Core::nodeBoostKp);
			if (core.node[Core::nodeBoostKp].value < 255)
				core.node[Core::nodeBoostKp].value++;
			keyPressed=-1;
			ansiGotoXy(14,21);
			printIntWithPadding(core.getNodeData()->value,3,' ');
		break;
		case 'i':
			core.setCurrentNode(Core::nodeBoostKi);
			if (core.node[Core::nodeBoostKi].value)
				core.node[Core::nodeBoostKi].value--;
			keyPressed=-1;
			ansiGotoXy(24,21);
			printIntWithPadding(core.getNodeData()->value,3,' ');
		break;
		case 'I':
			core.setCurrentNode(Core::nodeBoostKi);
			if (core.node[Core::nodeBoostKi].value < 255)
				core.node[Core::nodeBoostKi].value++;
			keyPressed=-1;
			ansiGotoXy(24,21);
			printIntWithPadding(core.getNodeData()->value,3,' ');
		break;
		case 'd':
			core.setCurrentNode(Core::nodeBoostKd);
			if (core.node[Core::nodeBoostKd].value)
				core.node[Core::nodeBoostKd].value--;
			keyPressed=-1;
			ansiGotoXy(34,21);
			printIntWithPadding(core.getNodeData()->value,3,' ');
		break;
		case 'D':
			core.setCurrentNode(Core::nodeBoostKd);
			if (core.node[Core::nodeBoostKd].value < 255)
				core.node[Core::nodeBoostKd].value++;
			keyPressed=-1;
			ansiGotoXy(34,21);
			printIntWithPadding(core.getNodeData()->value,3,' ');
		break;				
		case -1:
			break;
		default:
			redrawView = true;
		
	}
	pageMapEditor(true);

	if (redrawView) {
		for (char  i=0;i<8;i++) {
			ansiGotoXy(1,24+i);
			printFromFlash(BWBtexts[i]);	
		}		
		ansiGotoXy(1,21);
		printFromFlash(BWBpid);
		ansiGotoXy(1,23);
		printFromFlash(BWBheader);
		ansiGotoXy(14,22);
		printIntWithPadding(core.node[Core::nodeBoostKp].value,4,' ');		
		ansiGotoXy(24,22);
		printIntWithPadding(core.node[Core::nodeBoostKi].value,4,' ');		
		ansiGotoXy(34,22);
		printIntWithPadding(core.node[Core::nodeBoostKd].value,4,' ');		
	}
	static int pp,ii,dd;
	if (redrawView || pp != core.controls[Core::valueBoostPIDComponentP]) {
		pp = core.controls[Core::valueBoostPIDComponentP];
		ansiGotoXy(14,22);
		printIntWithPadding(pp,4,' ');
	}
	if (redrawView || ii != core.controls[Core::valueBoostPIDComponentI]) {
		ii = core.controls[Core::valueBoostPIDComponentI];
		ansiGotoXy(24,22);
		printIntWithPadding(ii,4,' ');
	}
	if (redrawView || dd != core.controls[Core::valueBoostPIDComponentD]) {
		dd = core.controls[Core::valueBoostPIDComponentD];
		ansiGotoXy(34,22);
		printIntWithPadding(dd,4,' ');
	}

	static unsigned char oldBVDC;
	if (redrawView || core.controls[Core::valueBoostValveDutyCycle]/4 != oldBVDC) {
		oldBVDC = core.controls[Core::valueBoostValveDutyCycle]/4;
		ansiGotoXy(11,24);
		for (char i=0;i<oldBVDC;i++)
			Serial.print("*");
		ansiClearEol();
		ansiGotoXy(76,24);
		printIntWithPadding(core.controls[Core::valueBoostValveDutyCycle],4,' ');	
	}

	static int oldPid;
	if (redrawView || core.controls[Core::valueBoostPIDCorrection] /4 != oldPid) {
		oldPid = core.controls[Core::valueBoostPIDCorrection] /4 ;
		ansiGotoXy(11,25);
		ansiClearEol();	
		
		if (oldPid<0) {
			ansiGotoXy(11+32+oldPid,25);
			for (char i=0;i<(-oldPid);i++)
				Serial.print("-");
		} else {
			ansiGotoXy(11+32,25);			
			for (char i=0;i<oldPid;i++)
				Serial.print("+");
		}
		ansiGotoXy(76,25);
		printIntWithPadding(core.controls[Core::valueBoostPIDCorrection],4,' ');
	}

	static unsigned char oldBCA;
	if (redrawView || core.controls[Core::valueBoostCalculatedAmount]/4 != oldBCA) {
		oldBCA = core.controls[Core::valueBoostCalculatedAmount]/4;
		ansiGotoXy(11,26);
		for (unsigned char i=0;i<oldBCA;i++)
			Serial.print("=");
		ansiClearEol();
		ansiGotoXy(76,26);
		printIntWithPadding(core.controls[Core::valueBoostCalculatedAmount],4,' ');	
	}

	static unsigned char oldMap;
	if (redrawView || core.controls[Core::valueBoostPressure]/4 != oldMap) {
		oldMap = core.controls[Core::valueBoostPressure]/4;
		ansiGotoXy(11,27);
		for (char i=0;i<oldMap;i++)
			Serial.print("M");
		ansiClearEol();
	}


	static unsigned char oldMapSetpoint;
	if (redrawView || core.controls[Core::valueBoostTarget]/4 != oldMapSetpoint) {
		oldMapSetpoint = core.controls[Core::valueBoostTarget]/4;
		ansiGotoXy(11,28);
		for (char i=0;i<oldMapSetpoint;i++)
			Serial.print("S");
		ansiClearEol();
	}

	static unsigned char oldRPM;
	if (redrawView || (core.controls[Core::valueEngineRPM]/10)*10 != oldRPM) {
		oldRPM = (core.controls[Core::valueEngineRPM]/10)*10;
		ansiGotoXy(11,29);
		printIntWithPadding(oldRPM,5,' ');
		ansiClearEol();
	}

	static unsigned char oldIq;
	if (redrawView || core.controls[Core::valueFuelAmount8bit] != oldIq) {
		oldIq = core.controls[Core::valueFuelAmount8bit];
		ansiGotoXy(11,30);
		printIntWithPadding(oldIq,5,' ');
		ansiClearEol();
	}

	static unsigned char oldDc;
	if (redrawView || core.controls[Core::valueN75DutyCycle] != oldDc) {
		oldDc = core.controls[Core::valueN75DutyCycle];
		ansiGotoXy(11,31);
		printIntWithPadding(oldDc,5,' ');
		ansiClearEol();
	}

	/*
	p/P: xxx i/I: xxx d/D: xxx           s/S(peed) xxx b/B(ias) 
		  nn       nn       nn 

	Base:   ****************************************  nn
	PID:                              --------------  nn
	Total:  **************************                nn

	Map :    **********************                    nn (peak:0)
	SetPoint:    **********************                    nn	
	RPM:    3300
	IQ:		333
	 */
}
