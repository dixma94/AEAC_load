//***************************************************************************
//  File........: IWDG.c
//  Author(s)...: KotyshkovEJ
//  Target(s)...: stm32f207
//  Compiler....: IAR
//  Description.: Инициализация IWDG
//  Date........: 30.11.2020
//***************************************************************************

#include "IWDG.h"

//Структура HAL для работы с Independent watchdog
IWDG_HandleTypeDef hiwdg;

/**************************************************************************
*   Function name: IWDG_Init
*   Returns      : void
*   Parameters   : int prescaler(max 3 bits), int reload(max 11 bits)
*   Purpose      : Инициализация структуры hiwdg
****************************************************************************/
void IWDG_Init(uint8_t prescaler, uint16_t reload)
{
  //Указатель на периферию
  hiwdg.Instance = IWDG;
  //F(iwdg) = LSI_VALUE(32kHz) / Prescaler
  hiwdg.Init.Prescaler = prescaler;
  //Количество тиков до перезагрузки
  hiwdg.Init.Reload = reload;
  
  //the LSI oscillator will be enabled by hardware
  HAL_IWDG_Init(&hiwdg);
}

/**************************************************************************
*   Function name: IWDG_Reload
*   Returns      : void
*   Parameters   : void
*   Purpose      : Сбрасывает счётчик WatchDog
****************************************************************************/
void IWDG_Reload(void)
{
  __HAL_IWDG_RELOAD_COUNTER(&hiwdg);
}
