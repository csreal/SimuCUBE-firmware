/*
 * cDeviceConfig.cpp
 *
 *  Created on: Jan 30, 2017
 *      Author: Mika
 */

#include "cDeviceConfig.h"

cDeviceConfig::cDeviceConfig() {
	// TODO Auto-generated constructor stub
	SetDefault();
}

cDeviceConfig::~cDeviceConfig() {
	// TODO Auto-generated destructor stub
}

void cDeviceConfig::SetDefault() { };

int *cDeviceConfig::GetProfileConfigAddr() {
	return ((int*)&profileConfig.mMaxAngle);
}

int *cDeviceConfig::GetHardwareConfigAddr() {
	return ((int*)&hardwareConfig.mEncoderOffset);
}

									// src, dest, size
void cDeviceConfig::getProfile(int *conf) {
	memcpy(GetProfileConfigAddr(), conf, sizeof(cProfileConfig));
}

void cDeviceConfig::setProfile(int *conf) {
	myMemCpy(conf, GetProfileConfigAddr(), sizeof(cProfileConfig));
}

void cDeviceConfig::getHardware(int *conf) {
	myMemCpy(GetHardwareConfigAddr(), conf, sizeof(cHardwareConfig));
}

void cDeviceConfig::setHardware(int *conf) {
	myMemCpy(conf, GetHardwareConfigAddr(), sizeof(cHardwareConfig));
}


/*	s32 mMaxAngle;							// Profile config

	u8 mFlags;
	u8 mNbTaps;

	s8 mMainGain;
	s8 mSpringGain;
	s8 mFrictionGain;
	s8 mDamperGain;
	s8 mInertiaGain;
	s8 mConstantGain;
	s8 mPeriodicGain;
	s8 mForceTableGain;

	s32 mOffset;							// Hardware config

	s8 mDesktopSpringGain;
	s8 mDesktopDamperGain;

	s8 mStopsSpringGain;
	s8 mStopsFrictionGain;

	u32 mVersion;*/
