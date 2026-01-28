/*
 * u_UART.c
 *
 *  Created on: Mar 8, 2024
 *      Author: Damini
 */

#include "u_Main.h"

unsigned char userInput=0;
extern volatile char uartEvent;

/*FUNCTION**********************************************************************
 *
 * Function Name : initUART
 * Description   : Initializes UART-2
 * Parameters: NONE
 * Return value: NONE
 *END**************************************************************************/
void InitUART(void) {

   //clock must be enable to it.
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

   //Pins Output Type: Push Pull
   //So that its state can be pulled up (logic-1), pulled down (logic-0).
    GPIOA->OTYPER	&= ~(1<<2);
    GPIOA->OTYPER   &= ~(1<<3);

   //Set Pins speed to medium.
    GPIOA->OSPEEDR	|= (1<<4);//PA2
    GPIOA->OSPEEDR  |= (1<<6);//PA3

    //Initial Pin State: High i.e. idle state is high
    GPIOA->PUPDR	|= (1<<4);//PA2
    GPIOA->PUPDR	|= (1<<6);//PA3

    //Configure GPIO Pins as alternate function
    GPIOA->MODER   |= (2<<4);   // Bits (5:4)= 1:0 --> Alternate Function for Pin PA2
    GPIOA->MODER   |= (2<<6);   // Bits (7:6)= 1:0 --> Alternate Function for Pin PA3

    //Connect Pin to USART. AF7

    GPIOA->AFR[0]  |= (7<<8);   // Bites (11:10:9:8) = 0:1:1:1  --> AF7 Alternate function for USART2 at Pin PA2
    GPIOA->AFR[0]  |= (7<<12);   // Bites (15:14:13:12) = 0:1:1:1  --> AF7 Alternate function for USART2 at Pin PA3


    RCC->APB1ENR	|= RCC_APB1ENR_USART2EN;//clock enable for UART2

    USART2->BRR = 0x683; // for 9600

    USART2->CR1 = 0x00;   // Clear ALL
    USART2->CR1 |= (1<<13);   // UE = 1... Enable USART
    USART2->CR1 &= ~(1<<12); //1-start bit, 8-bit data
    USART2->CR1 &= ~(1<<10); //1-start bit, 8-bit data

    USART2->CR1 |= (1<<2); // RE=1.. Enable the Receiver
    USART2->CR1 |= (1<<3);  // TE=1.. Enable Transmitter

    //enable Rx interrupt
    USART2->CR1 |= (1<<5);

    //Disable Hardware Flow Control
    USART2->CR2 &= ~(1<<8);
    USART2->CR2 &= ~(1<<9);

    //1-stop bit
    USART2->CR2 &= ~(1<<12);
    USART2->CR2 &= ~(1<<13);

   //Allow NVIC to acknowledge USART2 interrupt
   NVIC_EnableIRQ(USART2_IRQn);

}

/*FUNCTION**********************************************************************
 *
 * Function Name : put_char
 * Description   : Puts character out of UART
 * Parameters: NONE
 * Return value: NONE
 *END**************************************************************************/
void put_char(int ch) {

  USART2->DR = ch;

  //Wait until the data is transmitted
  while (!(USART2->SR & (1<<6)));

}

/*FUNCTION**********************************************************************
 *
 * Function Name : USART2_IRQHandler
 * Description   : Handles UART 2 RX interrupt
 * Parameters: NONE
 * Return value: NONE
 *END**************************************************************************/
void USART2_IRQHandler (void) {

  uartEvent =1; // signal to main loop
  userInput = USART2->DR;
  //read contents from USART2 data register and transmit it back
  put_char(USART2->DR);

}

/*FUNCTION**********************************************************************
 *
 * Function Name : transmitString
 * Description   : Transmits string
 * Parameters: NONE
 * Return value: NONE
 *END**************************************************************************/
void transmitString( char * buffer ) {

    volatile int i = 0;

    while ( buffer[i] != '\0' ) {
      put_char(buffer[i]);
      ++i;
    }
}
