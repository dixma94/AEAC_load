#include "main.h"
#include "stm32f2xx_it.h"


void SysTick_Handler(void)
{
  HAL_IncTick();
}