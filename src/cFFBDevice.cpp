/*
 * cFFBDevice.cpp
 *
 *  Created on: Jan 30, 2017
 *      Author: Mika
 */

#include <ffbengine.h>
#include "cFFBDevice.h"
#include "command.h"

cFFBDevice::cFFBDevice() {
	// TODO Auto-generated constructor stub
	SetDefault();
}

cFFBDevice::~cFFBDevice() {
	// TODO Auto-generated destructor stub
}

void cFFBDevice::SetFFB(FfbEngine* handle) {
	ffbhandle = handle;
}
void cFFBDevice::SetDefault()	{
	mConfig.SetDefault();
}

s32 cFFBDevice::CalcTorqueCommand(s32 *readEncoderPos) {
	static s32 pos=0;
	SM_STATUS stat;
	static s32 prev_cumul_damper=0;
	s32 cumul_damper=0;
	s32 command = 0;
	{
		for (u8 id = FIRST_EID; id <= MAX_EFFECTS; id++)
		{
			volatile cEffectState* ef = ffbhandle->getEffectState(id);//gEffectStates[id];
			if (Btest(ef->state, MEffectState_Allocated | MEffectState_Playing))
			{
				s32 mag = (((s32)ef->magnitude)*((s32)ef->gain)) >> 6;
				switch (ef->type)
				{
				case USB_EFFECT_CONSTANT:
					break;
				case USB_EFFECT_RAMP:
					break;
				case USB_EFFECT_SQUARE:
					break;
				case USB_EFFECT_SINE:
					break;
				case USB_EFFECT_TRIANGLE:
					break;
				case USB_EFFECT_SAWTOOTHDOWN:
					break;
				case USB_EFFECT_SAWTOOTHUP:
					break;
				case USB_EFFECT_SPRING:
					command += constrain(SpringEffect(ef->offset - pos, (mag*mConfig.profileConfig.mSpringGain) >> 7), -(ef->negativeSaturation << 8), ef->positiveSaturation << 8);
					break;
				case USB_EFFECT_FRICTION:
					stat = smSetParameter(mSMBusHandle, 1, SMP_TORQUE_EFFECT_FRICTION, (mag*mConfig.profileConfig.mFrictionGain) >> 3);
					break;
				case USB_EFFECT_DAMPER:
					cumul_damper+=(mag*mConfig.profileConfig.mDamperGain) >> 8;
					break;
				case USB_EFFECT_INERTIA:
					stat = smSetParameter(mSMBusHandle, 1, SMP_TORQUE_EFFECT_INERTIA,(mag*mConfig.profileConfig.mInertiaGain) >> 9);
					break;
				case USB_EFFECT_CUSTOM:
					break;
				default:
					break;
				}
			}
		}
		command = (command*mConfig.profileConfig.mMainGain) >> 14;
	}
	if (mAutoCenter)//gFFBDevice.mAutoCenter)
	{
		command += SpringEffect(-pos, mConfig.hardwareConfig.mDesktopSpringGain);
		cumul_damper += mConfig.hardwareConfig.mDesktopDamperGain;
	}
	if(prev_cumul_damper!=cumul_damper)//send only if changed to avoid update rate to drop
		stat = smSetParameter(mSMBusHandle, 1, SMP_TORQUE_EFFECT_DAMPING, cumul_damper);
	prev_cumul_damper=cumul_damper;

	command += SpringEffect(-pos, 10);
	command = ConstrainEffect(command);
	/*static s16 trq=0;
	trq+=10;
	SetTorque(trq);*/
	pos=SetTorque(command);//one call consumes total of ~0.35ms. with all code and 1ms delay in main loop, the update rate seems to be currently 33Hz (other code east rest of the time).
	*readEncoderPos=pos;
	return (command);
}

s32 cFFBDevice::ConstrainEffect(s32 val)
{
	return (constrain(val, -MAX_NORM_TORQUE, MAX_NORM_TORQUE));
}

s32 cFFBDevice::SpringEffect(s32 err, s32 mag)
{
	return ((err*mag) >>1);
}


