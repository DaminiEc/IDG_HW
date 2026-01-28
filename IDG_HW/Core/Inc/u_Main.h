/*
 * u_Main.h
 *
 *  Created on: Mar 8, 2024
 *      Author: Damini
 */

#ifndef INC_U_MAIN_H_
#define INC_U_MAIN_H_

#include "stm32f411xe.h"


#define       __setbit(___reg, ___bit)      ((___reg) |= (1U << (___bit)))
#define       __clearbit(___reg, ___bit)    ((___reg) &= (~(1U << (___bit))))
#define       __togglebit(___reg, ___bit)   ((___reg) ^= (1U << (___bit)))
#define       __getbit(___reg, ___bit)      (((___reg) & (1U << (___bit))) >> (___bit))

#endif /* INC_U_MAIN_H_ */
