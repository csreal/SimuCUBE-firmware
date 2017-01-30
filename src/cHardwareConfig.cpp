/*
 * cHardwareConfig.cpp
 *
 *  Created on: Jan 30, 2017
 *      Author: Mika
 */

#include "cHardwareConfig.h"

cHardwareConfig::cHardwareConfig() {
	// TODO Auto-generated constructor stub

}

cHardwareConfig::~cHardwareConfig() {
	// TODO Auto-generated destructor stub
}



uint16_t cHardwareConfig::ScaleAnalogAxis(uint16_t raw, bool invert, uint16_t minvalue, uint16_t maxvalue); {
	uint16_t output = (uint16_t)(65535.0*((float)raw-(float)minvalue)/(float)(maxvalue-minvalue));
	if(invert) {
		return 65535-output;
	}
	else {
		return output;
	}
}
