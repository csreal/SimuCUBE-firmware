/*
 * Config.h
 *
 *  Created on: 30.9.2016
 *      Author: mikat
 */

#ifndef CONFIG_H_
#define CONFIG_H_


// some basic parameters, consider moving into a data structure for saving/loading to/from flash
int steeringDegrees		= 270;
smint32 steeringEncoderOffset = 0;

// todo: pedal and buttons mapping, like this:
// map pin numbers to to table when configured, 0 if not configured
int analogAxis [3] = {0, 0, 0};
// more pedal setting values into similar table.
int buttons[32] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

void saveConfigToFlash();
void readConfigFromFlash();

#endif /* CONFIG_H_ */
