/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __IWDG_H
#define __IWDG_H

#include "stm32f2xx_hal.h"

/* ------------------ -= Прототипы =- ------------------ */

void IWDG_Init(uint8_t prescaler, uint16_t reload);
void IWDG_Reload(void);

#endif /* __IWDG_H */