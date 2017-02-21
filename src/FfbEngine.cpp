/*
 * FfbEngine.cpp
 *
 *  Created on: Feb 21, 2017
 *      Author: Mika
 */

#include "FfbEngine.h"

FfbEngine::FfbEngine() {
	// TODO Auto-generated constructor stub

}

FfbEngine::~FfbEngine() {
	// TODO Auto-generated destructor stub
}

#define LEDs_SetAllLEDs(l)

volatile cEffectState* FfbEngine::getEffectState(u8 id) {
	return &gEffectStates[id];
}


void FfbEngine::SendPidStateForEffect(uint8_t eid, uint8_t effectState)
{
	pidState.effectBlockIndex = effectState;
	pidState.effectBlockIndex = 0;
}


uint8_t FfbEngine::GetNextFreeEffect(void)
{
	// Find the next free effect ID for next time
	for (u8 i = FIRST_EID; i <= MAX_EFFECTS; i++)
	{
		if (gEffectStates[i].state == 0)
		{
			gEffectStates[i].state = MEffectState_Allocated;
			return(i);
		}
	}
	return 0;
 }

void FfbEngine::StopAllEffects(void)
{
	for (uint8_t id = FIRST_EID; id <= MAX_EFFECTS; id++)
		StopEffect(id);
}

void FfbEngine::StartEffect(uint8_t id)
{
	if ((id > MAX_EFFECTS) || (gEffectStates[id].state==0))
		return;
	InitEffect(id);
	gEffectStates[id].state |= MEffectState_Playing;
}

void FfbEngine::StopEffect(uint8_t id)
{
	if ((id > MAX_EFFECTS) || (gEffectStates[id].state == 0))
		return;
	gEffectStates[id].state &= ~MEffectState_Playing;
}

void FfbEngine::FreeEffect(uint8_t id)
{
	if (id > MAX_EFFECTS)
		return;

	//debugPrint(DMid, "Free effect id %d",id);
	//LogBinaryLf(&id,1);
	gEffectStates[id].state = 0;
}

void FfbEngine::FreeAllEffects(void)
{
	//LogTextLf("Free All effects");
 	//todo: memset((void*) gEffectStates, 0, sizeof(gEffectStates));
}


// Handle incoming data from USB
bool FfbEngine::handleReceivedHIDReport(HID_REPORT report)
{
	u8 *data = report.data;
	uint8_t effectId = data[1]; // effectBlockIndex is always the second byte.

	switch (data[0])	// reportID
	{
	case 1:
		//pc.printf("got effect command\r\n");
		FfbHandle_SetEffect((USB_FFBReport_SetEffect_Output_Data_t *) data);

		break;
	case 2:
		SetEnvelope((USB_FFBReport_SetEnvelope_Output_Data_t*) data, effectId);
		break;
	case 3:
		SetCondition((USB_FFBReport_SetCondition_Output_Data_t*) data, effectId);
		/*
					LogText("Set Condition - offset : ");
					LogBinary(&((USB_FFBReport_SetCondition_Output_Data_t*)data)->cpOffset,1);
					LogText(" ,PositiveCoef : ");
					LogBinaryLf(&((USB_FFBReport_SetCondition_Output_Data_t*)data)->positiveCoefficient,1);
					*/
		break;
	case 4:
		SetPeriodic((USB_FFBReport_SetPeriodic_Output_Data_t*) data, effectId);
		//			LogTextLf("Set Periodic");
		break;
	case 5:
		SetConstantForce((USB_FFBReport_SetConstantForce_Output_Data_t*) data, effectId);
		//			LogTextLf("Set Constant Force");
		break;
	case 6:
		SetRampForce((USB_FFBReport_SetRampForce_Output_Data_t*)data, effectId);
		break;
	case 7:
		FfbHandle_SetCustomForceData((USB_FFBReport_SetCustomForceData_Output_Data_t*) data);
		break;
	case 8:
		FfbHandle_SetDownloadForceSample((USB_FFBReport_SetDownloadForceSample_Output_Data_t*) data);
		break;
	case 9:
		break;
	case 10:
		FfbHandle_EffectOperation((USB_FFBReport_EffectOperation_Output_Data_t*) data);
		break;
	case 11:
		FfbHandle_BlockFree((USB_FFBReport_BlockFree_Output_Data_t *) data);
		break;
	case 12:
		FfbHandle_DeviceControl((USB_FFBReport_DeviceControl_Output_Data_t*) data);
		break;
	case 13:
		FfbHandle_DeviceGain((USB_FFBReport_DeviceGain_Output_Data_t*) data);
		break;
	case 14:
		FfbHandle_SetCustomForce((USB_FFBReport_SetCustomForce_Output_Data_t*) data);
		break;
	case 129:
		//gFFBDevice.mConfig.setProfile(&data[1]);
		break;
	case 130:
		//gFFBDevice.mConfig.setHardware(&data[1]);
		break;
	default:
		break;
	}
	return (TRUE);
}

void FfbEngine::FfbOnCreateNewEffect (USB_FFBReport_CreateNewEffect_Feature_Data_t* inData, USB_FFBReport_PIDBlockLoad_Feature_Data_t *outData)
{
	outData->reportId = 6;
	outData->effectBlockIndex = GetNextFreeEffect();

	if (outData->effectBlockIndex == 0)
	{
		outData->loadStatus = 2;	// 1=Success,2=Full,3=Error
		//LogText("Could not create effect");
	}
	else
	{
		outData->loadStatus = 1;	// 1=Success,2=Full,3=Error

		CreateNewEffect(inData, outData->effectBlockIndex);

		//LogText("Created effect ");
		//LogBinary(&outData->effectBlockIndex,1);
		//LogText(", type ");
		//LogBinaryLf(&inData->effectType,1);
	}
	outData->ramPoolAvailable = 0xFFFF;	// =0 or 0xFFFF - don't really know what this is used for?
//	WaitMs(5);
}

void FfbEngine::FfbHandle_SetEffect(USB_FFBReport_SetEffect_Output_Data_t *data)
{
	SetEffect(data,data->effectBlockIndex);
}

void FfbEngine::FfbOnPIDPool(USB_FFBReport_PIDPool_Feature_Data_t *data)
{
	FreeAllEffects();

	data->reportId = 7;
	data->ramPoolSize = 0xFFFF;
	data->maxSimultaneousEffects = MAX_EFFECTS;
	data->memoryManagement = 3;
}

void FfbEngine::FfbHandle_SetCustomForceData(USB_FFBReport_SetCustomForceData_Output_Data_t *data)
{
	//debugPrint(DMid, "SetCustomForceData");
}

void FfbEngine::FfbHandle_SetDownloadForceSample(USB_FFBReport_SetDownloadForceSample_Output_Data_t *data)
{
	//debugPrint(DMid, "SetDownloadForceSample");
}

void FfbEngine::FfbHandle_EffectOperation(USB_FFBReport_EffectOperation_Output_Data_t *data)
{
	uint8_t eid = data->effectBlockIndex;

	if (eid == 0xFF)
		eid = 0x7F;	// All effects

	if (data->operation == 1)
	{	// Start
		//debugPrint(DLow,"Start effect id %d",eid);
		StartEffect(eid);
	}
	else if (data->operation == 2)
	{	// StartSolo
		// Stop all first
		//debugPrint(DLow,"Start solo effect id %d",eid);
		StopAllEffects();
		// Then start the given effect
		StartEffect(eid);
	}
	else if (data->operation == 3)
	{	// Stop
		//debugPrint(DLow,"Stop effect id %d",eid);
		StopEffect(eid);
	}
}


void FfbEngine::FfbHandle_BlockFree (USB_FFBReport_BlockFree_Output_Data_t *data)
{
	uint8_t eid = data->effectBlockIndex;

	if (eid == 0xFF)
	{	// all effects
		FreeAllEffects();
// 		FreeEffect(0x7f); // TODO: does this work with the wheel?
	}
	else
	{
		FreeEffect(eid);
	}
}

#define DEVICE_PAUSED			0x01
#define ACTUATORS_ENABLED		0x02
#define SAFETY_SWITCH			0x04
#define ACTUATOR_OVERRIDE		0x08
#define ACTUATOR_POWER			0x10


void FfbEngine::FfbHandle_DeviceControl(USB_FFBReport_DeviceControl_Output_Data_t *data)
{
	//	LogTextP(PSTR("Device Control: "));

	uint8_t control = data->control;
	// 1=Enable Actuators, 2=Disable Actuators, 3=Stop All Effects, 4=Reset, 5=Pause, 6=Continue

	// PID State Report:
	//	uint8_t	reportId;	// =2
	//	uint8_t	status;	// Bits: 0=Device Paused,1=Actuators Enabled,2=Safety Switch,3=Actuator Override Switch,4=Actuator Power
	//	uint8_t	effectBlockIndex;	// Bit7=Effect Playing, Bit0..7=EffectId (1..40)

	pidState.reportId = 2;
	Bset(pidState.status,SAFETY_SWITCH);
	Bset(pidState.status,ACTUATOR_POWER);
	pidState.effectBlockIndex = 0;

	switch (control)
	{
	case 0x01:
		//LogTextLf("Disable Actuators");
		Bclr(pidState.status,ACTUATORS_ENABLED);
		break;
	case 0x02:
		//LogTextLf("Enable Actuators");
		Bset(pidState.status,ACTUATORS_ENABLED);
		break;
	case 0x03:
		//LogTextLf("Stop All Effects");		// Disable auto-center spring and stop all effects
		SetAutoCenter(0);
		pidState.effectBlockIndex = 0;
		break;
	case 0x04:
		//LogTextLf("Reset");			// Reset (e.g. FFB-application out of focus)
		SetAutoCenter(1);		// Enable auto-center spring and stop all effects
//		WaitMs(75);
		FreeAllEffects();
		break;
	case 0x05:
		//LogTextLf("Pause");
		Bset(pidState.status,DEVICE_PAUSED);
		break;
	case 0x06:
		//LogTextLf("Continue");
		Bclr(pidState.status,DEVICE_PAUSED);
		break;
	default:
		//LogTextP(PSTR("Other "));
		//LogBinaryLf(&data->control,1);
		asm("nop");
	}
}

void FfbEngine::FfbHandle_DeviceGain(USB_FFBReport_DeviceGain_Output_Data_t *data)
{
	//LogTextP(PSTR("Device Gain: "));
	//LogBinaryLf(&data->gain, 1);
}

void FfbEngine::FfbHandle_SetCustomForce(USB_FFBReport_SetCustomForce_Output_Data_t *data)
{
	//LogTextLf("Set Custom Force");
//	LogBinary(&data, sizeof(USB_FFBReport_SetCustomForce_Output_Data_t));
}


//from FFB_PRO
void FfbEngine::SetAutoCenter(uint8_t enable)
{
}

// --------------------------- effect operations ---------------------------------------------------

void FfbEngine::InitEffect(uint8_t effectId)
{
	volatile cEffectState* effect = &gEffectStates[effectId];
	effect->counter = 0;
	effect->fcounter = 0;
	effect->fade_start_time = effect->duration - effect->fadeTime;
}

// modify operations ---------------------------------------------------------

void FfbEngine::ModifyDuration(uint8_t effectId, uint16_t duration)
{
	volatile cEffectState* effect = &gEffectStates[effectId];
	effect->duration = duration;
	effect->counter = 0;
	effect->fade_start_time = duration - effect->fadeTime;

	//debugPrint(DMid,"ModifyDuration eid=%d %d %d %d",effectId,effect->duration, effect->counter, effect->fade_start_time);
}

void FfbEngine::SetEnvelope (USB_FFBReport_SetEnvelope_Output_Data_t* data, int effectId)
{
	volatile cEffectState* effect=&gEffectStates[effectId];
	effect->attackLevel = data->attackLevel;
	effect->fadeLevel = data->fadeLevel;
	effect->attackTime = data->attackTime;
	effect->fadeTime = data->fadeTime;
	//debugPrint(DMid,"SetEnvelope eid=%d %d %d %d %d",effectId,data->attackLevel,data->fadeLevel,data->attackTime,data->fadeTime);
}

void FfbEngine::SetCondition (USB_FFBReport_SetCondition_Output_Data_t* data, int effectId)
{
	volatile cEffectState* effect=&gEffectStates[effectId];

	effect->magnitude = (s16)data->positiveCoefficient;
	effect->offset = data->cpOffset;
	effect->positiveSaturation = data->positiveSaturation;
	effect->negativeSaturation = data->negativeSaturation;

	//debugPrint(DMid,"SetCondition eid=%d %d %d %d %d",effectId,effect->magnitude, effect->offset, effect->positiveSaturation, effect->negativeSaturation);
}

void FfbEngine::SetPeriodic (USB_FFBReport_SetPeriodic_Output_Data_t* data, int effectId)
{
	volatile cEffectState* effect=&gEffectStates[effectId];

	effect->magnitude = (s16)data->magnitude;
	effect->offset = data->offset;
	effect->phase = (s16)data->phase;
	effect->period = (u16)data->period;
	effect->freq = 0.0f;

	//debugPrint(DMid,"SetPeriodic eid=%d %d %d %d %d %d",effectId,effect->magnitude, effect->offset, effect->phase, effect->period, effect->freq);
}

void FfbEngine::SetConstantForce (USB_FFBReport_SetConstantForce_Output_Data_t* data, int effectId)
{
	volatile cEffectState* effect=&gEffectStates[effectId];

	effect->magnitude = data->magnitude;

	//pc.printf("SetConstantForce eid=%d", effectId);
	//pc.printf(" magnitude: %ld", effect->magnitude);
	//pc.printf("\r\n");
	//debugPrint(DMid,"SetConstantForce eid=%d %d",effectId,effect->magnitude);
}

void FfbEngine::SetRampForce (USB_FFBReport_SetRampForce_Output_Data_t* data, int effectId )
{
	volatile cEffectState* effect=&gEffectStates[effectId];

	effect->start_mag = data->start;
	effect->end_mag= data->end;

	//debugPrint(DMid,"SetRampForce eid=%d %d %d",effectId,effect->start_mag, effect->end_mag);
}

void FfbEngine::SetEffect (USB_FFBReport_SetEffect_Output_Data_t *data, int effectId )
{
	volatile cEffectState* effect=&gEffectStates[effectId];

	uint8_t eid = data->effectBlockIndex;
	effect->type = data->effectType;
	effect->gain = data->gain;
	ModifyDuration(eid,data->duration); // 0..32767 ms

//	bool is_periodic = false;

	char const *type;

	// Fill in the effect type specific data
	switch (data->effectType)
	{
		case USB_EFFECT_SQUARE:
			//pc.printf("square effect\r\n");
			type="SQUARE";
		break;
		case USB_EFFECT_SINE:
			//pc.printf("sine effect\r\n");
			type="SINE";
		break;
		case USB_EFFECT_TRIANGLE:
			//pc.printf("triangle effect\r\n");
			type="TRIANGLE";
		break;

		case USB_EFFECT_SAWTOOTHDOWN:
			//pc.printf("sawtooth down effect\r\n");
			type="SAWTOOTH";
		break;

		case USB_EFFECT_SAWTOOTHUP:
			//pc.printf("sawtooth up effect\r\n");
			type="SAWTOOTHUP";
		break;
//			is_periodic = true;
		case USB_EFFECT_CONSTANT:
			//pc.printf("constant effect\r\n");
			type="CONSTANT";
		break;

		case USB_EFFECT_RAMP:
			//pc.printf("ramp effect\r\n");
			type="RAMP";
		break;

		case USB_EFFECT_SPRING:
			//pc.printf("spring effect\r\n");
			type="SRPING";
		break;

		case USB_EFFECT_DAMPER:
			//pc.printf("damper effect\r\n");
			type="DAMPER";
		break;
		case USB_EFFECT_INERTIA:
			//pc.printf("inertia effect\r\n");
			type="INERTIA";
		break;

		case USB_EFFECT_FRICTION:
			//pc.printf("friction effect\r\n");
			type="FRICTION";
		break;

		case USB_EFFECT_CUSTOM:
			effect->period = data->samplePeriod;	// 0..32767 ms
			//pc.printf("custom effect\r\n");
			type="CUSTOM";

		break;

		default:
			type="UNKNOWN EFFECT";
			//pc.printf("unknown effect\r\n");
		break;
	}
	//debugPrint(DMid,"SetEffect eid=%d %s gain %d duration %d",effectId, type,data->gain,data->duration);
}

void FfbEngine::CreateNewEffect(USB_FFBReport_CreateNewEffect_Feature_Data_t* inData, int effectId )
{
	volatile cEffectState* effect=&gEffectStates[effectId];

	effect->type = inData->effectType;
	effect->gain = 0xFF;
	effect->attackLevel = 0xFF;
	effect->fadeLevel = 0xFF;

	effect->positiveSaturation = 0;
	effect->negativeSaturation = 0;
	effect->magnitude = 0;
	effect->phase = 0;
	effect->offset = 0;
	effect->period = 100;
	effect->duration = USB_DURATION_INFINITE;
	effect->fadeTime = USB_DURATION_INFINITE;

	//bdebugPrint(DHigh,"CreateNewEffect eid=%d",effectId);
}


// ----------------------------------------------
// Debug and other settings
// ----------------------------------------------

#ifdef _DEBUG

uint8_t FfbDebugListEffects(uint8_t *index)
	{
	if (*index == 0)
		*index = 2;

//	if (*index >= nextEID)
	if (*index >= MAX_EFFECTS)
		return 0;

	cEffectState *e = (cEffectState*) &gEffectStates[*index];

	LogBinary(index, 1);
	if (e->state == MEffectState_Allocated)
		LogTextP(PSTR(" Allocated"));
	else if (e->state == MEffectState_Playing)
		LogTextP(PSTR(" Playing\n"));
	else
		LogTextP(PSTR(" Free"));

	if (e->state)
		{
		LogTextP(PSTR("  duration="));
		LogBinary(&e->duration, 2);
		LogTextP(PSTR("\n  fadeTime="));
		LogBinary(&e->fadeTime, 2);
		LogTextP(PSTR("\n  gain="));
		LogBinary(&e->gain, 1);
		}

	*index = *index + 1;

	return 1;
}
#endif
