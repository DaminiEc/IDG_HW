/*
 * u_UART.h
 *
 *  Created on: Mar 8, 2024
 *      Author: Damini
 */

#ifndef INC_U_UART_H_
#define INC_U_UART_H_

extern unsigned char userInput;

void InitUART(void);
void put_char(int ch);
void USART2_IRQHandler (void);
void transmitString( char * buffer );

#endif /* INC_U_UART_H_ */
