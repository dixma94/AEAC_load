//***************************************************************************
//  File........: MODBUS_TIMEOUT.c
//  Author(s)...: KotyshkovEJ
//  Target(s)...: stm32f207
//  Compiler....: IAR
//  Description.: Функции измерения импеданса
//  Date........: 23.04.2021
//***************************************************************************
#include "MODBUS_TIMEOUT.h"
#include "BoardConfig.h"
#include "Alg_FUNC.h"

COMM_CONTROL_PARAM CommCtrlParam = { FALSE,//CtrlCounterEn
                                       0   //CtrlCounterVal
                                           };

extern LOAD_MAIN Main_Struct;

/**************************************************************************
*   Function name: MODBUS_TIMEOUT_Init
*   Returns      : void
*   Parameters   : void
*   Purpose      : Настройка таймера для подсчёта таймаута
****************************************************************************/
void MODBUS_TIMEOUT_Init(void)
{
  // Запускаем таймер для генерации меандра
  // TIM5 находится на шине APB1 и тактируется 60 МГц
  // Расчет таймера (60МГц/6000)/1000=10Гц(0.1s)
  // Включаем тактирование таймера
  
  MODBUS_TIMEOUT_TIM_CLK_ENABLE();
  
  // Конфигурируем таймер подсчёта таймаута
  TIM_HandleTypeDef TIM_HTD ={0};
  TIM_HTD.Instance = TIM5;
  TIM_HTD.Init.Prescaler = 6000 - 1;
  TIM_HTD.Init.CounterMode = TIM_COUNTERMODE_UP;
  TIM_HTD.Init.Period = 1000 - 1;
  TIM_HTD.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  TIM_HTD.Init.RepetitionCounter = TIM_AUTORELOAD_PRELOAD_ENABLE;
  HAL_TIM_Base_Init(&TIM_HTD);
  
  // Разрешаем прерываение по обновлению
  TIM5->DIER |= TIM_DIER_UIE;
  
  // Разрешаем прерывание TIM2
  HAL_NVIC_EnableIRQ(TIM5_IRQn);
  HAL_NVIC_SetPriority(TIM5_IRQn, 3, 0);
  
  // Очищаем флаг прерывания
  TIM5->SR &= ~TIM_SR_UIF;
}

/**************************************************************************
*   Function name: MODBUS_TIMEOUT_En
*   Returns      : void
*   Parameters   : BOOL Enable
*   Purpose      : Запуск таймера для подсчёта таймаута
****************************************************************************/
void MODBUS_TIMEOUT_En(BOOL Enable)
{
  if(Enable)
  {
    CommCtrlParam.CtrlCounterEn = TRUE;
    CommCtrlParam.CtrlCounterVal = 0;
    // Запускаем таймер таймаута
    TIM5->CR1 |= TIM_CR1_CEN;
  }else
  {
    CommCtrlParam.CtrlCounterEn = FALSE;
    // Запускаем таймер таймаута
    TIM5->CR1 &= ~TIM_CR1_CEN;
  }
  
}

/**************************************************************************
*   Function name: TIM5_IRQHandler
*   Returns      : void
*   Parameters   : void
*   Purpose      : Обработчик прерывания таймера TIM5 для подсчёта таймаута связи
****************************************************************************/
void TIM5_IRQHandler()
{
  // Если это прерываение по обновлению
  if((TIM5->SR & TIM_SR_UIF) != 0)
  {
    // Очищаем флаг прерывания
    TIM5->SR &= ~TIM_SR_UIF;
     // Счетчик таймаута потери связи
    if (CommCtrlParam.CtrlCounterEn)
    {
      // Если связи нет N секунд, 
      if (CommCtrlParam.CtrlCounterVal < LOST_COMM_TIMEOUT)
      {
        CommCtrlParam.CtrlCounterVal++;
      }
      else
      {
        // Останавливаем нагрузку
        Main_Struct.Load_Mode = LOAD_OFF;
        Main_Struct.Refrash_Flag = TRUE;
        MODBUS_TIMEOUT_En(FALSE);
      }
    }
  }
}