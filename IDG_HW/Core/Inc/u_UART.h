/*
 * u_UART.h
 *
 *  Created on: Mar 8, 2024
 *      Author: Damini
 */

#ifndef INC_U_UART_H_
#define INC_U_UART_H_

#define RX_BUF_SIZE 4

extern char rxBuf[RX_BUF_SIZE];
extern volatile uint8_t rxIndex;
extern volatile uint8_t uartRxEvent;

void InitUART(void);
void put_char(int ch);
void USART2_IRQHandler (void);
void transmitString( char * buffer );

#endif /* INC_U_UART_H_ */
