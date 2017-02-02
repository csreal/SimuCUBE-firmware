/*
 * cProfileConfig.cpp
 *
 *  Created on: Feb 1, 2017
 *      Author: Mika
 */

#include "cProfileConfig.h"

cProfileConfig::cProfileConfig() {
	// TODO Auto-generated constructor stub
	SetDefault();
}

cProfileConfig::~cProfileConfig() {
	// TODO Auto-generated destructor stub
}

void cProfileConfig::SetDefault() {
	// reset everything to zero here.
	// pointers to objects have to be deleted.
	mMaxAngle = 360.0;
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
