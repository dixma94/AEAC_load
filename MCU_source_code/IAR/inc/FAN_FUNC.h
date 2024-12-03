#ifndef FAN_FUNC_H
#define FAN_FUNC_H

#include "main.h"

/* ##################### -= Основные определения для FAN =- ##################### */

#define MIN_FAN_SPEED           32
#define MAX_FAN_SPEED           100
#define FAN_STOP                0 

/* --------------------- -= Прототипы функций =- --------------------- */

void FAN_Init(void);
void FAN_PWM_Set(uint8_t SetSpd);
void FAN_Stop(void);
uint8_t FAN_Spd_Calc(uint16_t SetCurrent);

#endif /* FAN_FUNC_H */
