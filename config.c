/*
 * config.cpp
 *
 *  Created on: 1.12.2016
 *      Author: mikat
 */


// EEPROM emulation in STM32F40x/STM32F41x microcontrollers (AN3969)

#include "config.h"
#include "simucube_io_defs.h"

void saveConfigToFlash(){

}


void readConfigFromFlash() {
	// does not read from flash now; that requires a quick test
	analogAxisPinCfg[GAS] = X11upper_2;
	analogAxisPinCfg[BRAKE] = X11upper_1;
    analogAxisPinCfg[CLUTCH] = X11upper_5;
	analogAxisInvert[GAS] = 0;
	analogAxisInvert[BRAKE] = 0;
	analogAxisInvert[CLUTCH] = 0;
	//buttonsCfg = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; // no buttons connected for demo
}
