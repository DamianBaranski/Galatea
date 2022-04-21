/*
 * usart.h
 *
 *  Created on: 17 paź 2016
 *      Author: damian
 */

#ifndef APPLICATION_USER_PRINT_H_
#define APPLICATION_USER_PRINT_H_

#define BUFFSIZE 32
//Inicializacja UARTu
void initUart();
//Wysyła tekst do uart-a
void print(char *text);
//Wysłanie liczby przez uart
void printI(int number);
//Wysłanie liczby zmiennoprzecinkowej przez uart
void printF(float real, int precision);

//Pobiera jeden znak
char getChar();
//ToDo Funkcje pobierające tekst i liczby z uart-u
//char scan(char *buff);
//char scanI(int *number);
//char scanF(float *real);
#endif /* APPLICATION_USER_PRINT_H_ */
