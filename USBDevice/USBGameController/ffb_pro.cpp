/*
  Force Feedback Joystick

  Copyright 2012  Tero Loimuneva (tloimu [at] gmail [dot] com)
  Copyright 2013  Saku Kekkonen
  Copyright 2016  Etienne Saint-Paul (esaintpaul [at] gameseed [dot] fr)

  Permission to use, copy, modify, distribute, and sell this
  software and its documentation for any purpose is hereby granted
  without fee, provided that the above copyright notice appear in
  all copies and that both that the copyright notice and this
  permission notice and warranty disclaimer appear in supporting
  documentation, and that the name of the author not be used in
  advertising or publicity pertaining to distribution of the
  software without specific, written prior permission.

  The author disclaim all warranties with regard to this
  software, including all implied warranties of merchantability
  and fitness.  In no event shall the author be liable for any
  special, indirect or consequential damages or any damages
  whatsoever resulting from loss of use, data or profits, whether
  in an action of contract, negligence or other tortious action,
  arising out of or in connection with the use or performance of
  this software.
*/

#include "stdint.h"
#include "types.h"
#include "ffb.h"
#include "ffb_pro.h"
#include "debug.h"
#include "USBGameController.h"

//--------------------------------------------------------------------------------------------------

void SetAutoCenter(uint8_t enable)
{
}

// --------------------------- effect operations ---------------------------------------------------

void InitEffect(uint8_t effectId)
{
	volatile cEffectState* effect = &gEffectStates[effectId];
	effect->counter = 0;
	effect->fcounter = 0;
	effect->fade_start_time = effect->duration - effect->fadeTime;
}

// modify operations ---------------------------------------------------------

void ModifyDuration(uint8_t effectId, uint16_t duration)
{
	volatile cEffectState* effect = &gEffectStates[effectId];
	effect->duration = duration;
	effect->counter = 0;
	effect->fade_start_time = duration - effect->fadeTime;

	debugPrint(DMid,"ModifyDuration eid=%d %d %d %d",effectId,effect->duration, effect->counter, effect->fade_start_time);
}

void SetEnvelope (USB_FFBReport_SetEnvelope_Output_Data_t* data, int effectId)
{
	volatile cEffectState* effect=&gEffectStates[effectId];
	effect->attackLevel = data->attackLevel;
	effect->fadeLevel = data->fadeLevel;
	effect->attackTime = data->attackTime;
	effect->fadeTime = data->fadeTime;
	debugPrint(DMid,"SetEnvelope eid=%d %d %d %d %d",effectId,data->attackLevel,data->fadeLevel,data->attackTime,data->fadeTime);
}

void SetCondition (USB_FFBReport_SetCondition_Output_Data_t* data, int effectId)
{
	volatile cEffectState* effect=&gEffectStates[effectId];

	effect->magnitude = (s16)data->positiveCoefficient;
	effect->offset = data->cpOffset;
	effect->positiveSaturation = data->positiveSaturation;
	effect->negativeSaturation = data->negativeSaturation;

	debugPrint(DMid,"SetCondition eid=%d %d %d %d %d",effectId,effect->magnitude, effect->offset, effect->positiveSaturation, effect->negativeSaturation);
}

void SetPeriodic (USB_FFBReport_SetPeriodic_Output_Data_t* data, int effectId)
{
	volatile cEffectState* effect=&gEffectStates[effectId];

	effect->magnitude = (s16)data->magnitude;
	effect->offset = data->offset;
	effect->phase = (s16)data->phase;
	effect->period = (u16)data->period;
	effect->freq = 0.0f;

	debugPrint(DMid,"SetPeriodic eid=%d %d %d %d %d %d",effectId,effect->magnitude, effect->offset, effect->phase, effect->period, effect->freq);
}

void SetConstantForce (USB_FFBReport_SetConstantForce_Output_Data_t* data, int effectId)
{
	volatile cEffectState* effect=&gEffectStates[effectId];

	effect->magnitude = data->magnitude;

	debugPrint(DMid,"SetConstantForce eid=%d %d",effectId,effect->magnitude);
}

void SetRampForce (USB_FFBReport_SetRampForce_Output_Data_t* data, int effectId )
{
	volatile cEffectState* effect=&gEffectStates[effectId];

	effect->start_mag = data->start;
	effect->end_mag= data->end;

	debugPrint(DMid,"SetRampForce eid=%d %d %d",effectId,effect->start_mag, effect->end_mag);
}

void SetEffect (USB_FFBReport_SetEffect_Output_Data_t *data, int effectId )
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
			type="SQUARE";
		break;
		case USB_EFFECT_SINE:
			type="SINE";
		break;
		case USB_EFFECT_TRIANGLE:
			type="TRIANGLE";
		break;

		case USB_EFFECT_SAWTOOTHDOWN:
			type="SAWTOOTH";
		break;

		case USB_EFFECT_SAWTOOTHUP:
			type="SAWTOOTHUP";
		break;
//			is_periodic = true;
		case USB_EFFECT_CONSTANT:
			type="CONSTANT";
		break;

		case USB_EFFECT_RAMP:
			type="RAMP";
		break;
	
		case USB_EFFECT_SPRING:
			type="SRPING";
		break;

		case USB_EFFECT_DAMPER:
			type="DAMPER";
		break;
		case USB_EFFECT_INERTIA:
			type="INERTIA";
		break;
		
		case USB_EFFECT_FRICTION:
			type="FRICTION";
		break;
		
		case USB_EFFECT_CUSTOM:	
			effect->period = data->samplePeriod;	// 0..32767 ms
			type="CUSTOM";

		break;
		
		default:
			type="UNKNOWN EFFECT";
		break;
	}
	debugPrint(DMid,"SetEffect eid=%d %s gain %d duration %d",effectId, type,data->gain,data->duration);
}

void CreateNewEffect(USB_FFBReport_CreateNewEffect_Feature_Data_t* inData, int effectId )
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

	debugPrint(DHigh,"CreateNewEffect eid=%d",effectId);
}
