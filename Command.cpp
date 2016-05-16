#include "mbed.h"
#include "cFFBDevice.h"

//---------------------------------------------------------------------------------------------

void InitializeTorqueCommand()
{
}


s32 encoderPos32=0;
u16 prevEncoderPos=0;
u16 encoderPos=0;

//called after drive is fully initialized to set 32 bit position from drive. needed because 16 bit incremental pos counter can roll over and it is absolute only after this is called
void resetPositionCountAt(s32 newpos)
{
	encoderPos32=newpos;
}

s32 SetTorque (s32 normalized_torque)		// between -0x10000 and 0x10000
{
	//SM_STATUS stat = smSetParameter(gFFBDevice.mSMBusHandle, 1, SMP_ABSOLUTE_SETPOINT, normalized_torque);

	s16 targetTorq;

	//limit as just to make sure its in bounds
	if(normalized_torque>16384)
		targetTorq=16384;
	else if(normalized_torque<-16384)
		targetTorq=-16384;
	else
		targetTorq=normalized_torque;

	u16 dummyVariable;

	smFastUpdateCycle(gFFBDevice.mSMBusHandle, 1, targetTorq, 0, &encoderPos, &dummyVariable);

	//calc encoder pos, unwrap it
	encoderPos32+=s16(encoderPos-prevEncoderPos);
	prevEncoderPos=encoderPos;

	return encoderPos32;
}
