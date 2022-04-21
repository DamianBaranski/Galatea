/*
 * midlvl.c
 *
 *  Created on: 1 mar 2016
 *      Author: zajec
 */

#include "midlvl.h"
#include <math.h>
#define IRTABLESIZE 31
#define ILOSC_ANALIZOWANYCH 30

static const  float irTable[4][IRTABLESIZE] = {
		1,13,31,47,64,77,89,99,106,113,118,123,128,132,137,139,141,144,148,151,153,155,156,158,159,161,161,162,164,164,166,
		/*1,11,27,41,56,67,77,86,92,98,103,107,111,115,119,121,123,125,129,131,133,135,136,137,138,140,140,141,143,143,144,*/
		7,21,36,52,65,76,83,92,98,106,111,115,119,124,125,129,133,136,138,140,143,144,147,148,149,150,151,152,153,154,154,
		/*0,10,27,39,52,61,69,75,82,86, 91, 95, 98,102,105,107,109,112,113,115,118,118,119,120,121,121,122,123,123,125,125,*/
		  0,17,34,46,60,69,78,85,92,98,103,109,114,119,123,128,131,135,139,142,145,149,150,154,156,159,162,164,166,169,171,
		/*0, 3,20,34,45,57,66,74,82,88,95,101,107,112,117,123,128,133,138,143,147,153,158,163,168,174,180,186,192,199,205*/
		  0,12,31,40,52,63,71,80,87,93,97,105,112,117,213,127,132,128,143,149,154,159,162,167,171,177,182,187,193,199,203};

/*static const float linFcnParam[4][7]=
		{{42,0.0175,-45,-0.021,13,-19000,3135},
		{25,0.019,-40,-0.03,9,-17000,3010},
		{40,0.026,-40,-0.023,24,-41000,2430},
		{46,0.017,-40,-0.028,15,-17000,3310}};


float linearIR(int channel, int value) {//return mm
	if (value>linFcnParam[channel-1][6]-100) {
		value=linFcnParam[channel-1][6]-100;
	}
	float tmp=linFcnParam[channel-1][0]+linFcnParam[channel-1][1]*value+linFcnParam[channel-1][2]*exp(linFcnParam[channel-1][3]*(value-linFcnParam[channel-1][4]))+linFcnParam[channel-1][5]/(value-linFcnParam[channel-1][6]);
	if (tmp<0) {
		tmp=0;
	}
	return tmp;
}*/

float linearIR(int channel, int value) {//return mm //channel 1-4
	int i=0;
	float out;
	if (value < irTable[channel-1][0]) {
		out = 0.0;
	} else {
		for (i=0;i<IRTABLESIZE-1;i++) {
			if (irTable[channel-1][i] <= value && irTable[channel-1][i+1] > value) {
				out = 10.0 * (i + (value-irTable[channel-1][i])/(irTable[channel-1][i+1]-(irTable[channel-1][i])));
				break;
			}
		}
		if (i==IRTABLESIZE-1) {
			return IRTABLESIZE*10.0;//1000.0;
		}
	}
	if (channel == 1 || channel == 2) {
		return out+10.0;
	}
	return out;
}

extern const float tresholdFront;
extern const float tresholdSide;

extern float opticMeasure[4][ILOSC_ANALIZOWANYCH]; //pomiary podczas jednego ruchu
extern unsigned int numer_pomiaru;

char walls() { //00000001 - front, 00000010 - right, 00000100 - left;
	int np=numer_pomiaru;
	int i=np-5;
	float front=0.0;
	float right=0.0;
	float left=0.0;
	char out=0;
	if (i<0) {
		i=ILOSC_ANALIZOWANYCH+i;
		while(i>ILOSC_ANALIZOWANYCH-1-5 || i < np) {
			front += opticMeasure[2][i]+opticMeasure[3][i];
			right += opticMeasure[1][i];
			left  += opticMeasure[0][i];
			i++;
			if (i==ILOSC_ANALIZOWANYCH)
				i=0;
		}
		front /= 10.0;
		right /= 5.0;
		left  /= 5.0;
	} else {
		for(;i<np;i++) {
			front += opticMeasure[2][i]+opticMeasure[3][i];
			right += opticMeasure[1][i];
			left  += opticMeasure[0][i];
		}
		front /= 10.0;
		right /= 5.0;
		left  /= 5.0;
	}
	if (front < tresholdFront) {
		out |= 1;
	}
	if (right < tresholdSide) {
		out |= 2;
	}
	if (left < tresholdSide) {
		out |= 4;
	}
	return out;
}

