/*
 * cHardwareConfig.h
 *
 *  Created on: Feb 1, 2017
 *      Author: Mika
 */

#ifndef CHARDWARECONFIG_H_
#define CHARDWARECONFIG_H_

#include "types.h"
#include "../SimpleMotion/simplemotion.h"


class cHardwareConfig {
public:
	cHardwareConfig();
	virtual ~cHardwareConfig();

	smint32 mEncoderOffset;

	smint32 mEncoderCPR;
	// analog axis things
	// todo port to HAL
#if 0
	PinName analogAxisPinCfg[3];
    AnalogIn* throttlepedal;
    AnalogIn* brakepedal;
    AnalogIn* clutchpedal;
    bool analogAxisInvert[3];
    uint16_t analogAxisMinValue[3];
    uint16_t analogAxisMaxValue[3];

    PinName buttonsPinCfg[32];
    DebounceIn* buttonInputs[32];
#endif

    s8 mDesktopSpringGain;
	s8 mDesktopDamperGain;
	s8 mStopsSpringGain;
	s8 mStopsFrictionGain;

	void setDefault();

private:
	uint16_t ScaleAnalogAxis(uint16_t raw, bool invert, uint16_t minvalue, uint16_t maxvalue);
};

#endif /* CHARDWARECONFIG_H_ */
