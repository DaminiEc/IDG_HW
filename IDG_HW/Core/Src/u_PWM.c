/*
 * u_PWM.c
 *
 *  Created on: Mar 10, 2024
 *      Author: Damini
 */


#include "stm32f411xe.h"

#define PERIOD 100
#define DUTY 60

unsigned char dutyCycle = 0;

void InitPWM(void){

	// Enable clocks for GPIOD and TIMER-4
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
	RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;

	// Set alternate function on PD15
	GPIOD->MODER  |= GPIO_MODER_MODE15_1;
	GPIOD->AFR[1] |= GPIO_AFRH_AFSEL15_1;

	// Set CC4 channel to output mode
	TIM4->CCMR2 &= ~TIM_CCMR2_CC4S;

	// Set polarity to active high
	TIM4->CCER &= ~TIM_CCER_CC4P;

	// Set PWM mode 1
	TIM4->CCMR2 |= TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1;

	// Set period and duty cycle
	TIM4->PSC = 0;//No prescaler, so the timer runs at 16 MHz directly.
	TIM4->ARR = 1599; //For a 10 kHz PWM signal. i.e. Period/PWM period = 16M/10K
	TIM4->CCR4 = 800;

	// Set preload bit and auto-reload bit
	TIM4->CCMR2 |= TIM_CCMR2_OC4PE;
	TIM4->CR1  |= TIM_CR1_ARPE;

	// Set upcounter mode
	TIM4->CCER |= TIM_CCER_CC4E;
	TIM4->CR1 |= TIM_CR1_CEN; // Enable count
}
