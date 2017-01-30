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

class cFFBDevice {
public:
	cFFBDevice();
	virtual ~cFFBDevice();

	void SetDefault();

	s32 CalcTorqueCommand(s32 *readEncoderPos);

	cDeviceConfig mConfig;

	b8 mAutoCenter;
	smbus mSMBusHandle;

};

#endif /* CFFBDEVICE_H_ */
