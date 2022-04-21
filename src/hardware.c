/*
 * hardware.c
 *
 *  Created on: 30 sty 2016
 *      Author: zajec
 */
#include "hardware.h"
#include "stm32f3xx_hal.h"
#include "cmsis_os.h"
#include "math.h"
#include "stdio.h"
#include "midlvl.h"

struct __FILE {
	int dummy;
};
FILE __stdout;
extern UART_HandleTypeDef huart3;
extern float MeasureIR[2];

extern TIM_HandleTypeDef htim2;
extern ADC_HandleTypeDef hadc1;
/*extern struct
{
	float P;    //Nstawy sterownika
	float I;
	float D;

	float zadVl;  //Zadana prędkość koła Lewego
	float zadVr;  //Zadana prędkość koła prawego
	float pomVl;  //Prędkość koła Lewego
	float pomVr;  //Prędkość koła prawego
	//Chronione potrzebne do pid-a
	float Il; //Warotość całki
	float Ir;
	float Dl; //Warotość pochodnej
	float Dr;
	float sterL;
	float sterR;
	float errorL;
	float errorR;
} PID;*/

extern struct
{
	float P;    //Nstawy sterownika
	float I;
	float D;

	double zadSl; //zadana droga koła lewego
	double zadSr; //zadana droga koła prawego
	float zadVl;  //Zadana prędkość koła Lewego
	float zadVr;  //Zadana prędkość koła prawego
	double pomSl;  //droga koła Lewego
	double pomSr;  //droga koła prawego
	//Chronione potrzebne do pid-a
	float Il; //Warotość całki
	float Ir;
	float Dl; //Warotość pochodnej
	float Dr;
	float sterL;
	float sterR;
	float errorL;
	float errorR;
} PID;

short int GetEncoderR()
{
	return -TIM4->CNT;
}
short int GetEncoderL()
{
	return TIM1->CNT;
}


int GetADC(int kanal)
{
	ADC_ChannelConfTypeDef sConfig;
	sConfig.Channel = kanal;
	sConfig.Rank = 1;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	HAL_ADC_ConfigChannel(&hadc1, &sConfig);

	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1,1000);
	int value = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);
	return value;
}

void MotorSetPWM(int L, int R)
{
	TIM_OC_InitTypeDef sConfigOC;

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = L;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1);

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = R;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2);

}

void MotorStart() {
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
}

void MotorSetDir(int L, int R) {
	if (L==-1) {
		HAL_GPIO_WritePin(L_MOTOR_OUT1_GPIO_Port,L_MOTOR_OUT1_Pin,1);
		HAL_GPIO_WritePin(L_MOTOR_OUT1_GPIO_Port,L_MOTOR_OUT2_Pin,0);
	}
	if (R==-1) {
		HAL_GPIO_WritePin(R_MOTOR_OUT1_GPIO_Port,R_MOTOR_OUT1_Pin,1);
		HAL_GPIO_WritePin(R_MOTOR_OUT1_GPIO_Port,R_MOTOR_OUT2_Pin,0);
	}
	if (L==1) {
		HAL_GPIO_WritePin(L_MOTOR_OUT1_GPIO_Port,L_MOTOR_OUT1_Pin,0);
		HAL_GPIO_WritePin(L_MOTOR_OUT1_GPIO_Port,L_MOTOR_OUT2_Pin,1);
	}
	if (R==1) {
		HAL_GPIO_WritePin(R_MOTOR_OUT1_GPIO_Port,R_MOTOR_OUT1_Pin,0);
		HAL_GPIO_WritePin(R_MOTOR_OUT1_GPIO_Port,R_MOTOR_OUT2_Pin,1);
	}
	if (L==0) {
		HAL_GPIO_WritePin(L_MOTOR_OUT1_GPIO_Port,L_MOTOR_OUT1_Pin,0);
		HAL_GPIO_WritePin(L_MOTOR_OUT1_GPIO_Port,L_MOTOR_OUT2_Pin,0);
	}
	if (R==0) {
		HAL_GPIO_WritePin(R_MOTOR_OUT1_GPIO_Port,R_MOTOR_OUT1_Pin,0);
		HAL_GPIO_WritePin(R_MOTOR_OUT1_GPIO_Port,R_MOTOR_OUT2_Pin,0);
	}
}

void MotorSetPWM2(int L, int R)
{
	TIM_OC_InitTypeDef sConfigOC;

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = (L>0)?L:-L;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1);

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = (R>0)?R:-R;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2);

	if (L<0)
	{
		HAL_GPIO_WritePin(L_MOTOR_OUT1_GPIO_Port,L_MOTOR_OUT1_Pin,1);
		HAL_GPIO_WritePin(L_MOTOR_OUT1_GPIO_Port,L_MOTOR_OUT2_Pin,0);
	}
	else if (L>0)
	{
		HAL_GPIO_WritePin(L_MOTOR_OUT1_GPIO_Port,L_MOTOR_OUT1_Pin,0);
		HAL_GPIO_WritePin(L_MOTOR_OUT1_GPIO_Port,L_MOTOR_OUT2_Pin,1);
	}

	if (R<0)
	{
		HAL_GPIO_WritePin(R_MOTOR_OUT1_GPIO_Port,R_MOTOR_OUT1_Pin,1);
		HAL_GPIO_WritePin(R_MOTOR_OUT1_GPIO_Port,R_MOTOR_OUT2_Pin,0);
	}
	else if (R>0)
	{
		HAL_GPIO_WritePin(R_MOTOR_OUT1_GPIO_Port,R_MOTOR_OUT1_Pin,0);
		HAL_GPIO_WritePin(R_MOTOR_OUT1_GPIO_Port,R_MOTOR_OUT2_Pin,1);
	}

	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
}

void SetSpeed(float Vl, float Vr)
{
	PID.zadVr=Vl;
	PID.zadVl=Vr;
}


extern const float tresholdSide;
extern float V_max; //Prędkość maksymalna
extern float V_max_curve;
extern float a;	//Przyspieszenia
extern float turn_a;
extern float curve_a;
extern const float rozstaw_kol;//rozstaw kól

extern char out;

float pIR=-0.0001;
float dIR=-0.00015;
extern float mid;
const float turn180=105;//98.91;

//Zmienne potrzebne dalej
float S_min;		 //Minimalna droga aby uzyskać V_max
float S_max;		 //Droga odniesienia dla turn (bo turn przyjmuje też ujemną)
float S_l, S_r;		 //Droga każdego z kół przy jeździe po łuku
float Rr;			 //promien jazdy po luku
float Vl_zad_po_luku;
float Vr_zad_po_luku;
float S_initToEnd;   //minimalna droga potrzebna do zmiany prędkości z V_init do V_end
float T_a;		//czas przyspieszania
float T_vnorm;		//czas jazdy z max predkoscia
float T_h;          //czas hamowania
float Sa;			//droga przyspieszenia
float Sh;			//droga hamowania
float T_end;        //czas calosci ruchu
float al, ar;		//Przyspieszenia dla kół z osobna
float Vl, Vr;		//Aktualne prędkości dla kół z osobna

float err=0;
float prvErr=0;

float t=0;
float corr=0;//correction

extern const float dT;

//funkcja ma dziurę - w przypadku gdy S jest za małe żeby z zadanym a przejsc z V_inti do V_end //edit: już nie
//jazda po łuku ma na stałe wpisany łuk o promieniu 80, z prędkością poczatkową 95 (inaczej sie nie da, z resztą to ruch po klotoidzie)
//tab btw gdzieś są ;ozamieniane L z R więc tu też :P
//S in mm (or degrees for turn), V in mm/s, type: 'F' - front, 'R' - right, 'L' - left, 'T' - turn (in turn positive S means robot turn left), 'B' - test move to go back, 'E' wait move
void profiler_perform(float S, float V_init, float V_end, char type) {

	err=0;
	prvErr=0;

	t=0;
	corr=0;//correction

	if (type == 'F') {
		//sprawdzamy czy rozpedzi sie fo V_max
		S_min = 0.5 / a * (2 * V_max * V_max - V_init * V_init - V_end * V_end);
		if (S>S_min) {//profil trapez
			T_a = (V_max-V_init)/a;
			T_h = (V_max-V_end)/a;
			Sa = 0.5*(V_init+V_max)*T_a;
			Sh = 0.5*(V_end+V_max)*T_h;
			T_vnorm = (S-Sa-Sh)/V_max;
			al=a;
			ar=a;
		} else {//profil trojkat
			S_initToEnd = 0.5 / a * fabs(V_init*V_init - V_end*V_end);
			if (S<=S_initToEnd) {
				al = 0.5 / S * fabs(V_init*V_init - V_end*V_end);
				ar = 0.5 / S * fabs(V_init*V_init - V_end*V_end);
				T_vnorm = 0;
				if (V_init > V_end) {
					T_h = 2*S/(V_init+V_end);
					T_a = 0;
				} else {
					T_a = 2*S/(V_init+V_end);
					T_h = 0;
				}
			} else {
				T_a = (-max(V_end,V_init)+sqrt(max(V_end,V_init)*max(V_end,V_init) + a * (S-S_initToEnd)) )/a;
				T_h = T_a;
				if (V_init == V_end) {

				} else if (V_init > V_end) {
					T_h += 2*S_initToEnd/(V_init+V_end);
				} else {
					T_a += 2*S_initToEnd/(V_init+V_end);
				}
				T_vnorm=0;
				al=a;
				ar=a;
			}
		}
		T_end = T_a+T_h+T_vnorm;

		//Obliczam przyspieszenia dla każdego z kół
		//al=Sl/(T_a*(T_a+T_vnorm));
		//ar=Sr/(T_a*(T_a+T_vnorm));
		//a nie lepiej tak?
		//al=Sl/S_max*a;
		//ar=Sr/S_max*a;



		corr=0;//correction
		for (t=0.0;t<T_a; t+=(dT)) {
			Vl = V_init + al*t + al*dT/2.0;
			Vr = V_init + ar*t + ar*dT/2.0;
			if (MeasureIR[0]<tresholdSide &&  MeasureIR[1]<tresholdSide && (out & 6)) {
				err=(MeasureIR[0]-MeasureIR[1]);
			} else if (MeasureIR[0]<tresholdSide && (out & 4)) {
				err=2*(MeasureIR[0]-mid);
			} else if (MeasureIR[1]<tresholdSide && (out & 2)) {
				err=2*(mid-MeasureIR[1]);
			} else {
				err=0;
			}
			corr = pIR*(Vl+Vr)*err+dIR*(Vl+Vr)*prvErr;
			prvErr=err;
			SetSpeed(Vl-corr,Vr+corr);
			osDelay(dT*1000);
		}
		for(;t<(T_vnorm+T_a); t+=(dT)) {
			Vl = V_max;
			Vr = V_max;
			if (MeasureIR[0]<tresholdSide &&  MeasureIR[1]<tresholdSide && (out & 6)) {
				err=(MeasureIR[0]-MeasureIR[1]);
			} else if (MeasureIR[0]<tresholdSide && (out & 4)) {
				err=2*(MeasureIR[0]-mid);
			} else if (MeasureIR[1]<tresholdSide && (out & 2)) {
				err=2*(mid-MeasureIR[1]);
			} else {
				err=0;
			}
			corr = pIR*(Vl+Vr)*err+dIR*(Vl+Vr)*prvErr;
			prvErr=err;
			SetSpeed(Vl-corr,Vr+corr);
			osDelay(dT*1000);
		}
		for(;t<T_end;t+=(dT)) {
			Vl = V_end + al*(T_end-t) - al*dT/2.0;
			Vr = V_end + ar*(T_end-t) - ar*dT/2.0;
			if (MeasureIR[0]<tresholdSide &&  MeasureIR[1]<tresholdSide && (out & 6) ) {
				err=(MeasureIR[0]-MeasureIR[1]);
			} else if (MeasureIR[0]<tresholdSide && (out & 4) ) {
				err=2*(MeasureIR[0]-mid);
			} else if (MeasureIR[1]<tresholdSide && (out & 2) ) {
				err=2*(mid-MeasureIR[1]);
			} else {
				err=0;
			}
			corr = pIR*(Vl+Vr)*err+dIR*(Vl+Vr)*prvErr;
			prvErr=err;
			SetSpeed(Vl-corr,Vr+corr);
			osDelay(dT*1000);
		}
		SetSpeed(V_end,V_end);
	} else if (type == 'T') {//turn, ignoruje V_init i V_end
		S=turn180*(S/180.0);
		S_max = fabs(S);

		//sprawdzamy czy rozpedzi sie fo V_max
		S_min = ( V_max * V_max)/turn_a;
		if (S_max>S_min) {//profil trapez
			T_a = (V_max)/turn_a;
			T_h = (V_max)/turn_a;
			T_vnorm = (S_max-S_min)/V_max;
		} else {//profil trojkat
			T_a = sqrt(S_max/turn_a);
			T_h = T_a;
			T_vnorm=0;
		}
		T_end = T_a+T_h+T_vnorm;

		//Obliczam przyspieszenia dla każdego z kół
		//al=Sl/(T_a*(T_a+T_vnorm));
		//ar=Sr/(T_a*(T_a+T_vnorm));
		//a nie lepiej tak?
		//al=Sl/S_max*a;
		//ar=Sr/S_max*a;

		al=(S/S_max)*turn_a;
		ar=-(S/S_max)*turn_a;

		for (t=0.0;t<T_a; t+=(dT)) {
			Vl = al*t + al*dT/2.0;
			Vr = ar*t + ar*dT/2.0;
			SetSpeed(Vl,Vr);
			osDelay(dT*1000);
		}
		for(;t<(T_vnorm+T_a); t+=(dT)) {
			Vl = (S/S_max)*V_max;
			Vr = -(S/S_max)*V_max;
			SetSpeed(Vl,Vr);
			osDelay(dT*1000);
		}
		for(;t<T_end;t+=(dT)) {
			Vl = al*(T_end-t) - al*dT/2.0;
			Vr = ar*(T_end-t) - ar*dT/2.0;
			SetSpeed(Vl,Vr);
			osDelay(dT*1000);
		}
		SetSpeed(0,0);
	} else if (type == 'L') {//zakladamy że V_init i V_end są takie same i mają utrzymywać się na stałym poziomie
		/*Rr = 2*S/PI;
		S_l = 0.5 * PI * (Rr + rozstaw_kol);
		S_r = 0.5 * PI * (Rr - rozstaw_kol);
		T_end = S / V_init;
		Vl_zad_po_luku = S_l / T_end;
		Vr_zad_po_luku = S_r / T_end;

		T_a = max(fabs(Vl_zad_po_luku-V_init),fabs(Vr_zad_po_luku-V_init))/a;
		T_h = T_a;
		T_vnorm = T_end-T_h-T_a;*/

		T_a=1.0/sqrt(2.0)*sqrt(95/V_max_curve);
		T_h=T_a;
		T_vnorm=0.0;
		T_end=T_a+T_vnorm+T_h;

		al = curve_a;
		ar = -curve_a;
		for (t=0.0;t<T_a;t+=(dT)) {
			Vl = al * t + V_init + al*dT/2.0;
			Vr = ar * t + V_init + ar*dT/2.0;
			//Vl = min(Vl,Vl_zad_po_luku);
			//Vr = max(Vr,Vr_zad_po_luku);
			SetSpeed(Vl,Vr);
			osDelay(dT*1000);
		}
		for (;t<T_a+T_vnorm;t+=(dT)) {
			Vl = Vl_zad_po_luku;
			Vr = Vr_zad_po_luku;
			SetSpeed(Vl,Vr);
			osDelay(dT*1000);
		}
		for(;t<T_end;t+=(dT)) {
			Vl = al*(T_end-t) + V_init - al*dT/2.0;
			Vr = ar*(T_end-t) + V_init - ar*dT/2.0;
			//Vl = min(Vl,Vl_zad_po_luku);
			//Vr = max(Vr,Vr_zad_po_luku);
			SetSpeed(Vl,Vr);
			osDelay(dT*1000);
		}
		SetSpeed(V_init,V_init);
	} else if (type == 'R') {//zakladamy że V_init i V_end są takie same i mają utrzymywać się na stałym poziomie
		/*Rr = 2*S/PI;
		S_l = 0.5 * PI * (Rr - rozstaw_kol);
		S_r = 0.5 * PI * (Rr + rozstaw_kol);
		T_end = S / V_init;
		Vl_zad_po_luku = S_l / T_end;
		Vr_zad_po_luku = S_r / T_end;

		T_a = max(fabs(Vl_zad_po_luku-V_init),fabs(Vr_zad_po_luku-V_init))/a;
		T_h = T_a;
		T_vnorm = T_end-T_h-T_a;*/

		T_a=1.0/sqrt(2.0)*sqrt(95/V_max_curve);
		T_h=T_a;
		T_vnorm=0.0;
		T_end=T_a+T_vnorm+T_h;

		al = -curve_a;
		ar = curve_a;
		for (t=0.0;t<T_a;t+=(dT)) {
			Vl = al * t + V_init + al*dT/2.0;
			Vr = ar * t + V_init + ar*dT/2.0;
			//Vl = max(Vl,Vl_zad_po_luku);
			//Vr = min(Vr,Vr_zad_po_luku);
			SetSpeed(Vl,Vr);
			osDelay(dT*1000);
		}
		for (;t<T_a+T_vnorm;t+=(dT)) {
			Vl = Vl_zad_po_luku;
			Vr = Vr_zad_po_luku;
			SetSpeed(Vl,Vr);
			osDelay(dT*1000);
		}
		for(;t<T_end;t+=(dT)) {
			Vl = al*(T_end-t) + V_init - al*dT/2.0;
			Vr = ar*(T_end-t) + V_init - ar*dT/2.0;
			// = max(Vl,Vl_zad_po_luku);
			//Vr = min(Vr,Vr_zad_po_luku);
			SetSpeed(Vl,Vr);
			osDelay(dT*1000);
		}
		SetSpeed(V_init,V_init);
	} else if (type == 'B') {
		al = 50;
		ar = 50;
		T_a = sqrt(S/al);
		T_h = sqrt(S/al);
		T_end = T_a+T_h;
		for (t=0.0;t<T_a;t+=(dT)) {
			SetSpeed(-al*t,-ar*t);
			osDelay(dT*1000);
		}
		for (;t<T_end;t+=(dT)) {
			SetSpeed(-al*(T_end-t),-ar*(T_end-t));
			osDelay(dT*1000);
		}
		SetSpeed(0.0,0.0);
	} else if (type == 'E') {
		SetSpeed(0.0,0.0);
		osDelay(5000);
	}
}

int distMeasure(int channel) {
int result;
switch (channel) {
case 1: {
	HAL_GPIO_WritePin(IR_LED1_OUT_GPIO_Port,IR_LED1_OUT_Pin,1);
	osDelay(1);
	result=GetADC(ADC_CHANNEL_1);
	HAL_GPIO_WritePin(IR_LED1_OUT_GPIO_Port,IR_LED1_OUT_Pin,0);
};break;
case 2: {
	HAL_GPIO_WritePin(IR_LED2_OUT_GPIO_Port,IR_LED2_OUT_Pin,1);
	osDelay(1);
	result=GetADC(ADC_CHANNEL_2);
	HAL_GPIO_WritePin(IR_LED2_OUT_GPIO_Port,IR_LED2_OUT_Pin,0);
};break;
case 3: {
	HAL_GPIO_WritePin(IR_LED3_OUT_GPIO_Port,IR_LED3_OUT_Pin,1);
	osDelay(1);
	result=GetADC(ADC_CHANNEL_3);
	HAL_GPIO_WritePin(IR_LED3_OUT_GPIO_Port,IR_LED3_OUT_Pin,0);
};break;
case 4: {
	HAL_GPIO_WritePin(IR_LED4_OUT_GPIO_Port,IR_LED4_OUT_Pin,1);
	osDelay(1);
	result=GetADC(ADC_CHANNEL_4);
	HAL_GPIO_WritePin(IR_LED4_OUT_GPIO_Port,IR_LED4_OUT_Pin,0);
};break;
}
return result;
}

int blindMeasure(int channel) {
int result;
switch (channel) {
case 1: {
	result=GetADC(ADC_CHANNEL_1);
	HAL_GPIO_WritePin(IR_LED1_OUT_GPIO_Port,IR_LED1_OUT_Pin,0);
};break;
case 2: {
	result=GetADC(ADC_CHANNEL_2);
	HAL_GPIO_WritePin(IR_LED2_OUT_GPIO_Port,IR_LED2_OUT_Pin,0);
};break;
case 3: {
	result=GetADC(ADC_CHANNEL_3);
	HAL_GPIO_WritePin(IR_LED3_OUT_GPIO_Port,IR_LED3_OUT_Pin,0);
};break;
case 4: {
	result=GetADC(ADC_CHANNEL_4);
	HAL_GPIO_WritePin(IR_LED4_OUT_GPIO_Port,IR_LED4_OUT_Pin,0);
};break;
}
return result;
}
