/*
 * cDeviceConfig.h
 *
 *  Created on: Jan 30, 2017
 *      Author: Mika
 */

#ifndef CDEVICECONFIG_H_
#define CDEVICECONFIG_H_

class cDeviceConfig {
public:
	cDeviceConfig();
	virtual ~cDeviceConfig();

	void SetDefault();

	cHardwareConfig hardwareConfig;
	cProfileConfig profileConfig;

	int *GetProfileConfigAddr();
	int *GetHardwareConfigAddr();


	void getProfile(int *conf);
	void setProfile(int *conf);
	void getHardware(int *conf);
	void setHardware(int *conf);

};

#endif /* CDEVICECONFIG_H_ */
