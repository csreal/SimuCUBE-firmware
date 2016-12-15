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
// todo: list all pins here

#define X11upper_1 PB_1
#define X11upper_2 PB_0
#define X11upper_3 PC_5
#define X11upper_5 PC_4
#define X11upper_6 PA_7
#define X11upper_7 PE_12


#ifdef __cplusplus
}
#endif

#endif /* SIMUCUBE_IO_DEFS_H_ */
