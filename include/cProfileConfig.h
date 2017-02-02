/*
 * cProfileConfig.h
 *
 *  Created on: Feb 1, 2017
 *      Author: Mika
 */

#ifndef CPROFILECONFIG_H_
#define CPROFILECONFIG_H_

#include "../SimpleMotion/simplemotion.h"
#include "types.h"

class cProfileConfig {
public:
	cProfileConfig();
	virtual ~cProfileConfig();
	void SetDefault();


	float mMaxAngle;

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

#endif /* CPROFILECONFIG_H_ */
