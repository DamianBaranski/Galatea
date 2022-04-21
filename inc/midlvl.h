/*
 * midlvl.h
 *
 *  Created on: 1 mar 2016
 *      Author: zajec
 */

#ifndef APPLICATION_USER_MIDLVL_H_
#define APPLICATION_USER_MIDLVL_H_

#include "hardware.h"

float linearIR(int channel, int value); //ADC to mm //channel 1-4 //pomiar liczymy od środka robota dla bocznych i od przodu dla przednich
char walls(); //00000001 - front, 00000010 - rigth, 00000100 - left;

#endif /* APPLICATION_USER_MIDLVL_H_ */
