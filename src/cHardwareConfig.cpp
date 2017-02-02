/*
 * cHardwareConfig.cpp
 *
 *  Created on: Jan 30, 2017
 *      Author: Mika
 */

#include "cHardwareConfig.h"

cHardwareConfig::cHardwareConfig() {
	// TODO Auto-generated constructor stub
	setDefault();
}

cHardwareConfig::~cHardwareConfig() {
	// TODO Auto-generated destructor stub
}

void cHardwareConfig::setDefault() {
	// reset everything to zero here.
	// pointers to objects have to be deleted.
	mEncoderOffset = 0;
	mEncoderCPR=0;
	mDesktopSpringGain = 0;
	mDesktopDamperGain = 0;

	mStopsSpringGain = 0;
	mStopsFrictionGain = 0;

#if 0
	if(throttlepedal != 0) {
		delete throttlepedal;
	}
	throttlepedal = 0;
	//analogAxisPinCfg[GAS]=NC;
	analogAxisPinCfg[GAS] = X11upper_2;
	if(brakepedal != 0) {
		delete brakepedal;
	}
	brakepedal = 0;
	//analogAxisPinCfg[BRAKE]=NC;
	analogAxisPinCfg[BRAKE] = X11upper_1;
	if(clutchpedal != 0) {
		delete clutchpedal;
	}
	clutchpedal = 0;
	//analogAxisPinCfg[CLUTCH]=NC;
	analogAxisPinCfg[CLUTCH] = X11upper_5;

	for(int i=0;i<3; i++) {
		analogAxisInvert[i]=0;
		analogAxisMinValue[i]=0;
		analogAxisMaxValue[i]=65535;
	}

	for(int i=0; i<32; i++) {
		if(buttonInputs[i] != 0 ) {
			delete buttonInputs[i];
			buttonInputs[i]=0;
		}
	}
#endif
}

void initHardware() {
#if 0
	// init analog axis
    if(analogAxisPinCfg[GAS]    != NC) {
    	throttlepedal = new AnalogIn(analogAxisPinCfg[GAS]);
    }
    if(analogAxisPinCfg[BRAKE]  != NC) {
    	brakepedal = new AnalogIn(analogAxisPinCfg[BRAKE]);
    }
    if(analogAxisPinCfg[CLUTCH] != NC) {
    	clutchpedal = new AnalogIn(analogAxisPinCfg[CLUTCH]);
    }

    // init button inputs
    for(int i=0; i<32; i++) {
    	if(buttonsPinCfg[i] != NC) {
    		buttonInputs[i] = new DebounceIn(buttonsPinCfg[i]);
    	}
    }
#endif
}

uint16_t cHardwareConfig::ScaleAnalogAxis(uint16_t raw, bool invert, uint16_t minvalue, uint16_t maxvalue) {
	uint16_t output = (uint16_t)(65535.0*((float)raw-(float)minvalue)/(float)(maxvalue-minvalue));
	if(invert) {
		return 65535-output;
	}
	else {
		return output;
	}
}
