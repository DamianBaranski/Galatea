/*
 * hardware.h
 *
 *  Created on: 30 sty 2016
 *      Author: zajec
 */

#ifndef APPLICATION_USER_HARDWARE_H_
#define APPLICATION_USER_HARDWARE_H_

#include "floodfill.h"

#define min(a,b) (((a)<(b))?(a):(b))
#define max(a,b) (((a)>(b))?(a):(b))
#define PI 3.141592

//S in mm (or degrees for turn), V in mm/s, type: 'F' - front, 'R' - right, 'L' - left, 'T' - turn (in turn positive S means robot turn left), 'B' - test move to go back, 'E' wait move
void profiler_perform(float S, float V_init, float V_end, char type);

int GetADC(int kanal); //ADC_CHANNEL_x 0 - 4092
short int GetEncoderR();//wynik zwraca w tickach
short int GetEncoderL();
void MotorSetPWM(int L, int R);//0-1000
void MotorStart();
void MotorSetDir(int L, int R);/*1-przod, -1-tyl, 0-nic*/
void MotorSetPWM2(int L, int R);
int distMeasure(int channel);//1-left,2-right,3-frontright,4-frontleft
int blindMeasure(int channel);

#endif /* APPLICATION_USER_HARDWARE_H_ */
