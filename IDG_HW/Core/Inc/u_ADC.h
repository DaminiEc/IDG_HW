/*
 * u_ADC.h
 *
 *  Created on: Jan 26, 2026
 *      Author: Damini
 */

#ifndef INC_U_ADC_H_
#define INC_U_ADC_H_

#include <stdint.h>

void InitADC(void);
void ADC_StartConversion(void);

extern volatile uint16_t adcValue;

#endif /* INC_U_ADC_H_ */
