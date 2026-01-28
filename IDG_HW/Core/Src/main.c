/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "u_Main.h"
#include "u_UART.h"
#include "u_PWM.h"
#include "u_ADC.h"
#include <stdio.h>

/* System state */
uint8_t ledState = 0;                 // 0 = OFF, 1 = ON
uint8_t currentBrightness = 0;        // Applied PWM duty
uint8_t targetBrightness = 0;         // Desired brightness

/* Input flags */
volatile char uartEvent = 0;
volatile char adcEvent = 0;
volatile char buttonEvent = 0;

/* Priority tracking */
typedef enum {
    INPUT_NONE,
    INPUT_UART,
    INPUT_ADC,
    INPUT_BUTTON
} InputSource;

InputSource lastInput = INPUT_NONE;
int counter =0; // PWM Duty cycle

void u_ButtonInit(void);
void u_EXTI0_Init(void);
void delay(int usecs);

int main(void)
{

	InitUART();
	InitPWM();
	InitADC();
	ADC_StartConversion();
	u_ButtonInit();
	u_EXTI0_Init();

	/* Initial state */
    ledState = 0;
    currentBrightness = 0;
    targetBrightness = 0;
	//Infinite loop
	transmitString("IDG Homework Assignment\r\n");

	while (1)
	{
        /* -------- Input Processing (Priority Based) -------- */

        if (uartEvent)
        {
            /* Process UART command */
            if (userInput == '0') 
			{
				ledState = 0;   // OFF
			}
            else if (userInput == '1')
			{
				ledState = 1;   // ON
                targetBrightness = 100;
			} 
            else if (userInput >= 0 && userInput <= 100)
        	{
				ledState = 1;
				targetBrightness = userInput; // Direct duty assignment
        	}

            lastInput = INPUT_UART;
            uartEvent = 0;
        }
        else if (adcEvent)
        {
            /* Read ADC and map to brightness */
            targetBrightness = (adcValue * 100) / 4095;
            lastInput = INPUT_ADC;
            adcEvent = 0;
        }
        else if (buttonEvent)
        {
            /* Toggle LED ON/OFF */
            ledState ^= 1;
            lastInput = INPUT_BUTTON;
            buttonEvent = 0;
        }

		/* -------- LED State Handling -------- */

        if (ledState == 0)
        {
            currentBrightness = 0;
            TIM4->CCR4 = 0;
            continue;
        }

		/* -------- Brightness Ramp Control -------- */

        if (currentBrightness < targetBrightness)
        {
            currentBrightness++;
        }
        else if (currentBrightness > targetBrightness)
        {
            currentBrightness--;
        }

        /* Update PWM output */
        TIM4->CCR4 = currentBrightness;

        /* Small delay for smooth transition */
        delay(5000);//5ms
	}

}

/**
  * @brief  This function is used to Initialize Button
  *
  * @retval None
  */
void u_ButtonInit(void){
	  /* Configure User push button*/
	  RCC->AHB1ENR |= (1 << 0); // enable clock to PORTA

	  GPIOA->MODER &= ~(3<<0); // PA0 as Input
	  GPIOA->PUPDR |= (2<<0); // PA0 with pulldown
}

/**
  * @brief  This function is used to Initialize Button
  *
  * @retval None
  */
void u_EXTI0_Init(void){
	RCC->APB2ENR |= (1<<14);      // Enable System configuration controller
	SYSCFG->EXTICR[0] &= ~(1<<0); // Select Port A as source, EXTIx = 0b0000
	SYSCFG->EXTICR[0] &= ~(1<<1); // Select Port A as source, EXTIx = 0b0000
	SYSCFG->EXTICR[0] &= ~(1<<2); // Select Port A as source, EXTIx = 0b0000
	SYSCFG->EXTICR[0] &= ~(1<<3); // Select Port A as source, EXTIx = 0b0000
	EXTI->IMR |= (1<<0);         // Disable interrupt request mask on EXTI line 0
	EXTI->FTSR &= ~(1<<0);        // Enable EXTI on Falling edge
	EXTI->RTSR |= (1<<0);       // enable EXTI on Rising edge

	RCC->APB2ENR &= ~(1<<14);     // Disable System configuration controller

	NVIC->IP[EXTI0_IRQn] =  (1 << 4);    // Step 3: Set priority to 1
	NVIC_EnableIRQ(EXTI0_IRQn); // Step 4: Enable interrupt
}

/**
  * @brief  This function is Handles Button interrupt
  *
  * @retval None
  */
void EXTI0_IRQHandler(void){

	buttonEvent = 1;      // Signal main loop
	EXTI->PR |= (1<<0); // Clear PR to re-enable EXTI interrupt
}

void delay(int usecs){
	int count = (usecs * 48) / 4;
	for (int i = 0; i < count; ++i){
	    count--;
	}
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
