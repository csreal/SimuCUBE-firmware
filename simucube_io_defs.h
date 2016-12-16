/*
 * simucube_io_defs.h
 *
 *  Created on: 1.12.2016
 *      Author: mikat
 */

#ifndef SIMUCUBE_IO_DEFS_H_
#define SIMUCUBE_IO_DEFS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx.h"
#include "PinNames.h"

// this file should include all io port definitions on the SimuCUBE board
// so that e.g. physical connectors X11-X12 port and pin names can be used

#define X11upper_1 PB_1
#define X11upper_2 PB_0
#define X11upper_3 PC_5
#define X11upper_5 PC_4
#define X11upper_6 PA_7
#define X11upper_7 PE_12

#define X11lower_1 PC15
#define X11lower_2 PC3
#define X11lower_3 PC3
#define X11lower_5 PA4
#define X11lower_6 PA5
#define X11lower_7 PA6

#define X12upper_1 PC11
#define X12upper_2 PC6
#define X12upper_3 PC7
#define X12upper_4 PC8
#define X12upper_5 PC9
#define X12upper_6 PE5
#define X12upper_7 PC10

#define X12lower_1 PE2
#define X12lower_2 PE3
#define X12lower_3 PE1
#define X12lower_4 PE0
#define X12lower_5 PC12
#define X12lower_6 PC13
#define X12lower_7 PC14


#ifdef __cplusplus
}
#endif

#endif /* SIMUCUBE_IO_DEFS_H_ */
