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
#include <string.h>
#include <stdlib.h>

#define POT_DEADBAND 10   // minimum change to trigger update

volatile char buttonEvent = 0;

/* Priority tracking */
typedef enum {
	INPUT_NONE,
	INPUT_UART,
	INPUT_POT,
	INPUT_BUTTON
} InputSource;

InputSource lastInput = INPUT_NONE;
// LED state
typedef enum {LED_OFF=0, LED_ON=1} LED_State;
volatile LED_State ledState = LED_OFF;
/* System state */
uint8_t currentBrightness = 0;        // Applied PWM duty
uint8_t targetBrightness = 0;         // Desired brightness

volatile uint32_t prevPotValue = 0;    // previous ADC value for comparison

void u_ButtonInit(void);
void u_EXTI0_Init(void);
void delay(int usecs);

void HandleUART(void)
{
	if (uartRxEvent)
	{
		if (strcasecmp((char *)rxBuf, "ON") == 0)
		{
			lastInput = INPUT_POT;
			return;
		}
		char *endPtr;
		int value = strtol((char *)rxBuf, &endPtr, 10);

		if (*endPtr == '\0') // valid number
		{
			if (value == 0)
			{
				ledState = LED_OFF;
				targetBrightness = 0;
			}
			else if (value >= 1 && value <= 100)
			{
				ledState = LED_ON;
				targetBrightness = value;
			}
		}
		// clear UART event
		uartRxEvent = 0;
		lastInput = INPUT_UART;
	}
}

void HandleButton(void)
{
	if (buttonEvent)
	{
		buttonEvent = 0;
		ledState ^= 1; // toggle

		if (ledState == LED_OFF)
			targetBrightness = 0;
		else
			targetBrightness = 100; // default full brightness

		lastInput = INPUT_BUTTON;
	}
}

void LED_UpdateBrightness(void)
{
	if (currentBrightness < targetBrightness)
		currentBrightness++;
	else if (currentBrightness > targetBrightness)
		currentBrightness--;

	TIM4->CCR4 = currentBrightness / 100; // update PWM Duty cycle scaled to 100
}

void PollPotentiometer(void)
{
	uint32_t adcRaw;

	//    // Only act if UART and Button haven't been the last input
	if (lastInput != INPUT_UART && lastInput != INPUT_BUTTON)
	{
		// Start ADC conversion
		ADC1->CR2 |= ADC_CR2_SWSTART;

		if(!(ADC1->SR & ADC_SR_EOC))
		{
			adcRaw = ADC1->DR;

			int32_t delta = adcRaw - prevPotValue;

			// Only act if change is significant (deadband)
			if(delta >= POT_DEADBAND || delta <= -POT_DEADBAND)
			{
				prevPotValue = adcRaw;

				// Map 0–255 to 4 levels → 0–3
				uint8_t level = adcRaw / 64;   // 256/4 = 64
				if(level > 3) level = 3;       // clamp max

				// Set LED brightness: 0%, 25%, 50%, 75%, 100%
				targetBrightness = (level) * 25;

				ledState = (targetBrightness > 0) ? LED_ON : LED_OFF;

				// Treat pot as last input
				lastInput = INPUT_POT;
			}
		}

		ADC1->CR2 &= ~ADC_CR2_SWSTART;
	}
}

int main(void)
{

	InitUART();
	InitPWM();
	InitADC();
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
		HandleUART();
		HandleButton();
		PollPotentiometer();

		// Smoothly update LED brightness
		LED_UpdateBrightness();

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
