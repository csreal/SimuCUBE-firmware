/*
 * cFFBDevice.h
 *
 *  Created on: Jan 30, 2017
 *      Author: Mika
 */

#ifndef CFFBDEVICE_H_
#define CFFBDEVICE_H_

#include "types.h"
#include "cDeviceConfig.h"
#include "../SimpleMotion/simplemotion.h"

class cFFBDevice {
public:
	cFFBDevice();
	~cFFBDevice();

	void SetDefault();

	s32 CalcTorqueCommand(s32 *readEncoderPos);

	cDeviceConfig mConfig;

	b8 mAutoCenter;
	smbus mSMBusHandle;

	s32 ConstrainEffect(s32 val);
	s32 SpringEffect(s32 err, s32 mag);
};

#endif /* CFFBDEVICE_H_ */
