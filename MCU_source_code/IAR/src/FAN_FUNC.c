//***************************************************************************
//  File........: FAN_FUNC.c
//  Author(s)...: KotyshkovEJ
//  Target(s)...: stm32f207
//  Compiler....: IAR
//  Description.: Инициализация и управление пинами вентиляторов
//  Date........: 13.04.2021
//***************************************************************************

#include "FAN_FUNC.h"
#include "BoardConfig.h"
#include "Alg_FUNC.h"

extern LOAD_VERSION_LIMITS Load_limits;

static TIM_HandleTypeDef TIM9_HTD = {0};

//Прототипы функций
void FAN_GPIO_Init(void);
void FAN_TIM_Init(void);

/**************************************************************************
*   Function name: FAN_Init
*   Returns      : void
*   Parameters   : void
*   Purpose      : Инициализация управления вентиляторами
****************************************************************************/
void FAN_Init()
{
  //Настройка таймера на работу в режиме генерации ШИМ
  FAN_TIM_Init();

  // Настройка портов в/в
  FAN_GPIO_Init();
  
  //Запуск генерации ШИМ
  HAL_TIM_PWM_Start(&TIM9_HTD, TIM_CHANNEL_2);
}

/**************************************************************************
*   Function name: FAN_PWM_Set
*   Returns      : void
*   Parameters   : uint8_t SetSpd(Значение тока)
*   Purpose      : Установка скорости вентилятора
****************************************************************************/
void FAN_PWM_Set(uint8_t SetSpd)
{
  if(SetSpd < MIN_FAN_SPEED) SetSpd = MIN_FAN_SPEED;
  if(SetSpd > MAX_FAN_SPEED) SetSpd = MAX_FAN_SPEED;
  TIM9->CCR2 = SetSpd;
}

/**************************************************************************
*   Function name: FAN_STOP
*   Returns      : void
*   Parameters   : void
*   Purpose      : Остановка вентилятора
****************************************************************************/
void FAN_Stop()
{
  TIM9->CCR2 = FAN_STOP;
}

/**************************************************************************
*   Function name: FAN_Spd_Calc
*   Returns      : uint8_t(Значение скорости вентилятора в %)
*   Parameters   : uint16_t SetCurrent(Значение тока)
*   Purpose      : Инициализация управления вентиляторами
****************************************************************************/
uint8_t FAN_Spd_Calc(uint16_t SetCurrent)
{
  uint16_t FAN_Spd = 0;
  
  //Рассчёт скорости вентилятора
  FAN_Spd = SetCurrent/10;
  FAN_Spd *= Load_limits.FanCoeff;
  FAN_Spd /= 100;
  FAN_Spd += 30;
  
  //Валидация
  if(FAN_Spd > 100) FAN_Spd = 100;
  
  return (uint8_t)FAN_Spd;
}

/**************************************************************************
*   Function name: FAN_GPIO_Init
*   Returns      : void
*   Parameters   : void
*   Purpose      : Инициализация пина управления вентиляторами
****************************************************************************/
void FAN_GPIO_Init(void)
{
  FAN_PORT_RCC_ENABLE();
  
  GPIO_InitTypeDef GPIO_ITD;
    GPIO_ITD.Pin = FAN_PIN;
    GPIO_ITD.Mode = GPIO_MODE_AF_PP;
    GPIO_ITD.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_ITD.Pull = GPIO_NOPULL;
    GPIO_ITD.Alternate = GPIO_AF3_TIM9;
  HAL_GPIO_Init(FAN_PORT, &GPIO_ITD);
}


/**************************************************************************
*   Function name: FAN_TIM_Init
*   Returns      : void
*   Parameters   : void
*   Purpose      : Инициализация таймера для генерации ШИМ
****************************************************************************/
void FAN_TIM_Init(void)
{
  // Запускаем таймер на 30 Гц для генерации ШИМ
  // TIM9 находится на шине APB2 и тактируется 120 МГц
  // Расчет таймера (120МГц/40000)/100= 30Гц
  
  // Включаем тактирование таймера
  TIM_FAN_CLK_ENABLE();

  TIM_OC_InitTypeDef sConfigOC = {0}; 

  TIM9_HTD.Instance = TIM9;
  TIM9_HTD.Init.Prescaler = 40000;
  TIM9_HTD.Init.CounterMode = TIM_COUNTERMODE_UP;
  TIM9_HTD.Init.Period = 100-1;
  TIM9_HTD.Init.ClockDivision = TIM_CLOCKDIVISION_DIV4;
  TIM9_HTD.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  
  HAL_TIM_PWM_Init(&TIM9_HTD);
  
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  
  HAL_TIM_PWM_ConfigChannel(&TIM9_HTD, &sConfigOC, TIM_CHANNEL_2);
}