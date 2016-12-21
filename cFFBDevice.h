#ifndef _CFFB_DEVICE_H_
#define _CFFB_DEVICE_H_

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "types.h"
#include "debug.h"
#include "Effects.h"
#include "Command.h"
#include "SimpleMotion/simplemotion.h"
#include "mbed.h"

#define GAS 0
#define BRAKE 1
#define CLUTCH 2

//void memcpy_oma() {
//}

#include "simucube_io_defs.h"
#include "DebounceIn.h"





// profile is used for quick setting values, for example
// settings between different games
_Pragma("pack(1)")
class cProfileConfig
{
public:
	cProfileConfig() { setDefault(); }
	void setDefault() {
		// reset everything to zero here.
		// pointers to objects have to be deleted.
		mMaxAngle = 0;
		mFlags = 0;
		mNbTaps = 0;
		mMainGain = 0;
		mSpringGain = 0;
		mFrictionGain = 0;
		mDamperGain = 0;
		mInertiaGain = 0;
		mConstantGain = 0;
		mPeriodicGain = 0;
		mForceTableGain = 0;
	}

	s32 mMaxAngle;

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
};






// hardware config is everything hardware related
class cHardwareConfig
{
public:
	// steering offset
	s32 mOffset;

	// analog axis things
	PinName analogAxisPinCfg[3];
    AnalogIn* throttlepedal;
    AnalogIn* brakepedal;
    AnalogIn* clutchpedal;
    bool analogAxisInvert[3];
    uint16_t analogAxisMinValue[3];
    uint16_t analogAxisMaxValue[3];

    PinName buttonsPinCfg[32];
    DebounceIn* buttonInputs[32];


    s8 mDesktopSpringGain;
	s8 mDesktopDamperGain;
	s8 mStopsSpringGain;
	s8 mStopsFrictionGain;
	cHardwareConfig() { setDefault(); }
	void setDefault() {
		// reset everything to zero here.
		// pointers to objects have to be deleted.
		mOffset = 0;
		mDesktopSpringGain = 0;
		mDesktopDamperGain = 0;

		mStopsSpringGain = 0;
		mStopsFrictionGain = 0;

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
			buttonInputs[i]=0;
		}
	}

	void initHardware() {
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
	}

	void updateAnalogAxis(uint16_t &throttlevalue, uint16_t &brakevalue, uint16_t &clutchvalue) {
		if(throttlepedal) {
			throttlevalue = ScaleAnalogAxis(throttlepedal->read_u16(), analogAxisInvert[GAS], analogAxisMinValue[GAS], analogAxisMaxValue[GAS]);
		}
		if(brakepedal) {
			brakevalue = ScaleAnalogAxis(brakepedal->read_u16(), analogAxisInvert[BRAKE], analogAxisMinValue[BRAKE], analogAxisMaxValue[BRAKE]);
		}
		if(clutchpedal) {
			clutchvalue = ScaleAnalogAxis(clutchpedal->read_u16(),   analogAxisInvert[CLUTCH], analogAxisMinValue[CLUTCH], analogAxisMaxValue[CLUTCH]);
		}
	}

	int readButtons() {
		int buttonStatus = 0;
	    for(int i = 0; i<32; i++) {
	    	if(buttonInputs[i]) {
	    		buttonStatus |= buttonInputs[i]->read() << i;
	    	}
	    }
	    return buttonStatus;
	}

	private:
	uint16_t ScaleAnalogAxis(uint16_t raw, bool invert, uint16_t minvalue, uint16_t maxvalue) {
		uint16_t output = (uint16_t)(65535.0*((float)raw-(float)minvalue)/(float)(maxvalue-minvalue));
		if(invert) {
			return 65535-output;
		}
		else {
			return output;
		}
	}
};






class cDeviceConfig
{
public:
	void SetDefault() { };
	cDeviceConfig()	{ SetDefault(); }
	
	cHardwareConfig hardwareConfig;
	cProfileConfig profileConfig;
	u8 *GetProfileConfigAddr()			{ return ((u8*)&profileConfig.mMaxAngle); } // variables listed here must be
	u8 *GetHardwareConfigAddr()			{ return ((u8*)&hardwareConfig.mOffset); }  // first in class definition.

									// src, dest, size
	void getProfile(u8 *conf)		{ myMemCpy(GetProfileConfigAddr(), conf, sizeof(cProfileConfig)); }
	void setProfile(u8 *conf)		{ myMemCpy(conf, GetProfileConfigAddr(), sizeof(cProfileConfig)); }
	void getHardware(u8 *conf)		{ myMemCpy(GetHardwareConfigAddr(), conf, sizeof(cHardwareConfig)); }
	void setHardware(u8 *conf)		{ myMemCpy(conf, GetHardwareConfigAddr(), sizeof(cHardwareConfig)); }
	

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

private:
	void myMemCpy(void *dest, void *src, size_t n)
	{
	   // Typecast src and dest addresses to (char *)
	   char *csrc = (char *)src;
	   char *cdest = (char *)dest;

	   // Copy contents of src[] to dest[]
	   for (int i=0; i<n; i++)
	       cdest[i] = csrc[i];
	}
};
_Pragma("pack()")






class cFFBDevice
{
public:
	cFFBDevice()		{ SetDefault();	}

	void SetDefault()	{ mConfig.SetDefault(); }

	s32 CalcTorqueCommand(s32 *readEncoderPos);
	
	cDeviceConfig mConfig;

	b8 mAutoCenter;
	smbus mSMBusHandle;
};

extern cFFBDevice gFFBDevice;

#endif	//_CFFB_DEVICE_H_
