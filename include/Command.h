#ifndef _COMMAND_H_
#define _COMMAND_H_

#define MAX_NORM_TORQUE		0x10000

void InitializeTorqueCommand ();

//called after drive is fully initialized to set 32 bit position from drive. needed because 16 bit incremental pos counter can roll over and it is absolute only after this is called
void resetPositionCountAt(s32 newpos);

//returns encoder counter value from drive
s32 SetTorque (s32 normalized_torque);

#endif	//_COMMAND_H_
