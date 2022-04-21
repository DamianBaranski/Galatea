#include <stdio.h>
#include <math.h>
#include <print.h>
#include <stdlib.h>
#include "stm32f3xx_hal.h"

extern UART_HandleTypeDef huart3;

struct RingBuff
{
	char buff[BUFFSIZE];
	char *read_ptr;
	char *write_ptr;
} in, out;

char Received;

char *Max(char *A, char *B)
{
	return (A>B)?B:A;
}

void initUart()
{
	in.read_ptr=in.buff;
	in.write_ptr=in.buff;
	out.read_ptr=out.buff;
	out.write_ptr=out.buff;
	HAL_UART_Receive_IT(&huart3, &Received, 1);
}

char empty(struct RingBuff* buff)
{
	return buff->write_ptr==buff->read_ptr;
}

char full(struct RingBuff* buff)
{
	char *next=buff->buff+(buff->write_ptr-buff->buff+1)%BUFFSIZE;
	return next==buff->read_ptr;

}

void push(struct RingBuff* buff, char character)
{
	char *next=buff->buff+(buff->write_ptr-buff->buff+1)%BUFFSIZE;
	if(!full(buff))
	{
		(*buff->write_ptr)=character;
		buff->write_ptr=next;
	}
}

char pop(struct RingBuff* buff)
{
	if(!empty(buff))
	{
		char character=*buff->read_ptr;
		buff->read_ptr=buff->buff+(buff->read_ptr-buff->buff+1)%BUFFSIZE;
		return character;
	}
	return '\n';
}

void transmit()
{
	//Jeśli buffor nie jest pusty i akurat uart ma wolne
	if(!empty(&out) && ((huart3.State == HAL_UART_STATE_READY) || (huart3.State == HAL_UART_STATE_BUSY_RX)))
	{
		char *end_ptr;
		if(out.read_ptr>out.write_ptr)
			end_ptr=out.buff+BUFFSIZE;
		else
			end_ptr=out.write_ptr;

		HAL_UART_Transmit_IT(&huart3,(unsigned char*)out.read_ptr,end_ptr-out.read_ptr);
		if(out.read_ptr>out.write_ptr)
			out.read_ptr=out.buff;
		else
			out.read_ptr=out.write_ptr;
	}
}

void print(char *text)
{
	while(*text)
	{
		push(&out,*text);
		text++;
	}
	transmit();
}

void printI(int number)
{
	int power=pow(10,(int)(log10(abs(number))));
	if(number==0) power=1;

	if(number<0)
	{
		number=-number;
		push(&out,'-');
	}

	while(power!=0)
	{
		push(&out,number/power+48);
		number=number%power;
		power=power/10;
	}
	transmit();
}

void printF(float real, int precision)
{
	printI((int)real);
	push(&out,'.');
	int power=pow(10,precision);
	printI((int)abs(real*power)%power);
	transmit();
}

char getChar()
{
	return pop(&in);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	transmit();
	HAL_GPIO_TogglePin(LED_G_GPIO_Port,LED_G_Pin);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	push(&in,Received);
	HAL_GPIO_TogglePin(LED_Y_GPIO_Port,LED_Y_Pin);
	HAL_UART_Receive_IT(&huart3, &Received, 1);

}
