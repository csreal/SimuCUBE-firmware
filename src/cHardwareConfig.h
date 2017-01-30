/*
 * cHardwareConfig.h
 *
 *  Created on: Jan 30, 2017
 *      Author: Mika
 */

#ifndef CHARDWARECONFIG_H_
#define CHARDWARECONFIG_H_

class cHardwareConfig {
public:
	cHardwareConfig();
	virtual ~cHardwareConfig();

private:
	uint16_t ScaleAnalogAxis(uint16_t raw, bool invert, uint16_t minvalue, uint16_t maxvalue);
	}
};

#endif /* CHARDWARECONFIG_H_ */
