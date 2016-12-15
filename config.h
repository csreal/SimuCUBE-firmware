/*
 * config.h
 *
 *  Created on: 30.9.2016
 *      Author: mikat
 */

#ifndef CONFIG_H_
#define CONFIG_H_

#include "SimpleMotion\simplemotion.h"
#include "simucube_io_defs.h"

#define GAS 0
#define BRAKE 1
#define CLUTCH 2


// some basic parameters, consider moving into a data structure for saving/loading to/from flash
int steeringDegrees		= 270;
smint32 steeringEncoderOffset = 0;



// pedal and buttons mapping
// map pin numbers to to table when configured, 0 if not configured
PinName analogAxisPinCfg [3];
int analogAxisInvert[3];

// filtering for these analog inputs
int analogAxisFiltering[3];

int buttonsCfg[32];

// EEPROM emulation in STM32F40x/STM32F41x microcontrollers (AN3969)
void saveConfigToFlash();
void readConfigFromFlash();

#endif /* CONFIG_H_ */
