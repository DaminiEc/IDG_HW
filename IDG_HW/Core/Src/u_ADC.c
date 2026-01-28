/*
 * u_ADC.c
 *
 *  Created on: Jan 26, 2026
 *      Author: Damini
 */

#include "stm32f411xe.h"

#define GPIOA_EN   (1U << 0)
#define ADC1_EN    (1U << 8)

/* Global variable updated in ISR */
volatile uint16_t adcValue = 0;
extern volatile char adcEvent;

void InitADC(void)
{
    /* 1. Enable clocks */
    RCC->AHB1ENR |= GPIOA_EN;
    RCC->APB2ENR |= ADC1_EN;

    /* 2. Configure PA1 as analog */
    GPIOA->MODER |= (3U << (1 * 2));   // Analog mode
    GPIOA->PUPDR &= ~(3U << (1 * 2));  // No pull-up/down

    /* 3. ADC common configuration */
    ADC->CCR &= ~(3U << 16);           // PCLK2 / 2

    /* 4. ADC1 configuration */
    ADC1->CR1 |= ADC_CR1_EOCIE;        // Enable EOC interrupt
    ADC1->CR2 |= ADC_CR2_CONT;         // Continuous conversion (optional)
    ADC1->CR2 |= ADC_CR2_ADON;          // Enable ADC

    /* 5. Channel configuration */
    ADC1->SQR1 = 0;                    // 1 conversion
    ADC1->SQR3 = 1;                    // Channel 1 (PA1)

    /* 6. Sampling time */
    ADC1->SMPR2 |= (3U << 3);           // Channel 1 sample time

    /* 7. Enable ADC interrupt in NVIC */
    NVIC_EnableIRQ(ADC_IRQn);
}

void ADC_StartConversion(void)
{
    ADC1->CR2 |= ADC_CR2_SWSTART;
}

void ADC_IRQHandler(void)
{
    adcEvent =1;
    if (ADC1->SR & ADC_SR_EOC)
    {
        adcValue = ADC1->DR;   // Reading DR clears EOC
    }
}