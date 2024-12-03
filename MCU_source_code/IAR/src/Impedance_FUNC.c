//***************************************************************************
//  File........: Impedance_FUNC.c
//  Author(s)...: KotyshkovEJ
//  Target(s)...: stm32f207
//  Compiler....: IAR
//  Description.: Функции измерения импеданса
//  Date........: 16.04.2021
//***************************************************************************

#include "Impedance_FUNC.h"
#include "BoardConfig.h"
#include "CurrentSet_FUNC.h"
#include "math.h"
#include "Alg_FUNC.h"

IMPEDANCE_DATA ImpedanceData;

extern CONFIG_DATA ConfigData;
extern DEVICE_REGISTER DeviceRegister;
extern LOAD_MAIN Main_Struct;

//Прототипы функций
void Impedance_TIM_Init();
void GAIN_GPIO_Init();
void IMP_ADC_GPIO_Init();
void PGA_Gain_Select(PGA_GAIN Gain);
void Impedance_ADC_Init();
void Resp_Calc();

/**************************************************************************
*   Function name: Impedance_Periph_Init
*   Returns      : void
*   Parameters   : void
*   Purpose      : Инициализация периферии для измерения импеданса
****************************************************************************/
void Impedance_Periph_Init()
{
  //Инициализурем структуру для измерения импеданса
  ImpedanceData.Step = CHECK_STEP;
  
  ImpedanceData.Current.Range = RANGE_1;
  ImpedanceData.Current.Val_Pos = CURR_LOW;
  
  ImpedanceData.Measure.Gain = GAIN_128;
  ImpedanceData.Measure.Samples_Count_HI = 0;
  ImpedanceData.Measure.Samples_Count_LO = 0;
  ImpedanceData.Measure.Samples_Sum_HI = 0;
  ImpedanceData.Measure.Samples_Sum_LO = 0;
  ImpedanceData.Measure.V_Response = 0;
  
  //Выставляем флаг измерения импеданса
  DeviceRegister.ro_region.DeviceState |= IMPEDANCE_MEASURE_FLAG;
  
  IMP_ADC_GPIO_Init();
  
  GAIN_GPIO_Init();
  
  Impedance_ADC_Init();
  
  Impedance_TIM_Init();
}

/**************************************************************************
*   Function name: Impedance_TIM_Init
*   Returns      : void
*   Parameters   : void
*   Purpose      : Настройка таймера для генерации меандра
****************************************************************************/
void Impedance_TIM_Init(void)
{
    // Запускаем таймер для генерации меандра
    // TIM2 находится на шине APB1 и тактируется 60 МГц
    // Расчет таймера (60МГц/6000)/50=200Гц(5ms)
    
    // Включаем тактирование таймера
    IMPEDANCE_TIM_GEN_CLK_ENABLE();
    
    // Конфигурируем таймер генерации меандра
    TIM_HandleTypeDef TIM_HTD ={0};
    TIM_HTD.Instance = TIM2;
    TIM_HTD.Init.Prescaler = 6000-1;
    TIM_HTD.Init.CounterMode = TIM_COUNTERMODE_UP;
    TIM_HTD.Init.Period = 50 - 1;
    TIM_HTD.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    TIM_HTD.Init.RepetitionCounter = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_Base_Init(&TIM_HTD);
    
    // Запускаем таймер для измерения напржения
    // TIM4 находится на шине APB1 и тактируется 60 МГц
    // Расчет таймера (60МГц/300)/5=40000Гц(25us)
    
    // Включаем тактирование таймера
    IMPEDANCE_TIM_MEAS_CLK_ENABLE();
    
    // Конфигурируем таймер измерителя напряжения
    TIM_HTD.Instance = TIM4;
    TIM_HTD.Init.Prescaler = 300-1;
    TIM_HTD.Init.CounterMode = TIM_COUNTERMODE_UP;
    TIM_HTD.Init.Period = 5 - 1;
    TIM_HTD.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    TIM_HTD.Init.RepetitionCounter = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_Base_Init(&TIM_HTD);
    
    // Разрешаем прерываение по обновлению
    TIM2->DIER |= TIM_DIER_UIE;
    TIM4->DIER |= TIM_DIER_UIE;
    
    // Разрешаем прерывание TIM2
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
    HAL_NVIC_SetPriority(TIM2_IRQn, 3, 0);
    
     // Разрешаем прерывание TIM4
    HAL_NVIC_EnableIRQ(TIM4_IRQn);
    HAL_NVIC_SetPriority(TIM4_IRQn, 0, 0);
    
    // Очищаем флаг прерывания
    TIM2->SR &= ~TIM_SR_UIF;
    
    // Запускаем таймер генерации
    TIM2->CR1 |= TIM_CR1_CEN;
}

/**************************************************************************
*   Function name: Impedance_Measure_STOP
*   Returns      : void
*   Parameters   : void
*   Purpose      : Остановка генерации меандра
****************************************************************************/
void Impedance_Measure_STOP(void)
{      
    // Запрещаем прерывание TIM2
    HAL_NVIC_DisableIRQ(TIM2_IRQn);
    HAL_NVIC_DisableIRQ(TIM4_IRQn);
    
    // Останавливаем таймеры
    TIM2->CR1 &= ~TIM_CR1_CEN;
    TIM4->CR1 &= ~TIM_CR1_CEN;
    
    // Очищаем флаг прерывания
    TIM2->SR &= ~TIM_SR_UIF;
    TIM4->SR &= ~TIM_SR_UIF;
    
    // Выключаем тактирование таймеров
    IMPEDANCE_TIM_GEN_CLK_ENABLE();
    IMPEDANCE_TIM_MEAS_CLK_DISABLE();
    
    //Выходим из режима измерения
    Main_Struct.Load_Mode = LOAD_OFF;
    //Выставляем флаг обновления регистров
    Main_Struct.Refrash_Flag = TRUE;
}

/**************************************************************************
*   Function name: DAC_GPIO_Init
*   Returns      : void
*   Parameters   : void
*   Purpose      : Инициализация GPIO для выбора усиления на AD8231
****************************************************************************/
void GAIN_GPIO_Init()
{
  GAIN_PORT_PIN_RCC_ENABLE();
  
  GPIO_InitTypeDef GPIO_ITD;
  GPIO_ITD.Pin = PGA_GAIN_PIN_BIT_2 | PGA_GAIN_PIN_BIT_1 | PGA_GAIN_PIN_BIT_0;
  GPIO_ITD.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_ITD.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_ITD.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PGA_GAIN_PORT, &GPIO_ITD);
}

/**************************************************************************
*   Function name: IMP_ADC_GPIO_Init
*   Returns      : void
*   Parameters   : void
*   Purpose      : Инициализация GPIO для работы АЦП
****************************************************************************/
void IMP_ADC_GPIO_Init()
{
  IMP_ADC_PORT_PIN_RCC_ENABLE();
  
  GPIO_InitTypeDef GPIO_ITD;
  GPIO_ITD.Pin = IMP_ADC_PIN;
  GPIO_ITD.Mode = GPIO_MODE_ANALOG;
  GPIO_ITD.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_ITD.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IMP_ADC_PORT, &GPIO_ITD);
}

/**************************************************************************
*   Function name: Gain_Select
*   Returns      : void
*   Parameters   : PGA_GAIN Gain(Усиление, описание типа находится в заголовочном файле)
*   Purpose      : Выбор усиления на AD8231
****************************************************************************/
void PGA_Gain_Select(PGA_GAIN Gain)
{  
  switch (Gain)
  {
  case GAIN_1:
    HAL_GPIO_WritePin(PGA_GAIN_PORT, PGA_GAIN_PIN_BIT_2, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(PGA_GAIN_PORT, PGA_GAIN_PIN_BIT_1, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(PGA_GAIN_PORT, PGA_GAIN_PIN_BIT_0, GPIO_PIN_RESET);
    break;
    
  case GAIN_2:
    HAL_GPIO_WritePin(PGA_GAIN_PORT, PGA_GAIN_PIN_BIT_2, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(PGA_GAIN_PORT, PGA_GAIN_PIN_BIT_1, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(PGA_GAIN_PORT, PGA_GAIN_PIN_BIT_0, GPIO_PIN_SET);
    break;
    
  case GAIN_4:
    HAL_GPIO_WritePin(PGA_GAIN_PORT, PGA_GAIN_PIN_BIT_2, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(PGA_GAIN_PORT, PGA_GAIN_PIN_BIT_1, GPIO_PIN_SET);
    HAL_GPIO_WritePin(PGA_GAIN_PORT, PGA_GAIN_PIN_BIT_0, GPIO_PIN_RESET);
    break;
    
  case GAIN_8:
    HAL_GPIO_WritePin(PGA_GAIN_PORT, PGA_GAIN_PIN_BIT_2, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(PGA_GAIN_PORT, PGA_GAIN_PIN_BIT_1, GPIO_PIN_SET);
    HAL_GPIO_WritePin(PGA_GAIN_PORT, PGA_GAIN_PIN_BIT_0, GPIO_PIN_SET);
    break;
    
  case GAIN_16:
    HAL_GPIO_WritePin(PGA_GAIN_PORT, PGA_GAIN_PIN_BIT_2, GPIO_PIN_SET);
    HAL_GPIO_WritePin(PGA_GAIN_PORT, PGA_GAIN_PIN_BIT_1, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(PGA_GAIN_PORT, PGA_GAIN_PIN_BIT_0, GPIO_PIN_RESET);
    break;
    
  case GAIN_32:
    HAL_GPIO_WritePin(PGA_GAIN_PORT, PGA_GAIN_PIN_BIT_2, GPIO_PIN_SET);
    HAL_GPIO_WritePin(PGA_GAIN_PORT, PGA_GAIN_PIN_BIT_1, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(PGA_GAIN_PORT, PGA_GAIN_PIN_BIT_0, GPIO_PIN_SET);
    break;
    
  case GAIN_64:
    HAL_GPIO_WritePin(PGA_GAIN_PORT, PGA_GAIN_PIN_BIT_2, GPIO_PIN_SET);
    HAL_GPIO_WritePin(PGA_GAIN_PORT, PGA_GAIN_PIN_BIT_1, GPIO_PIN_SET);
    HAL_GPIO_WritePin(PGA_GAIN_PORT, PGA_GAIN_PIN_BIT_0, GPIO_PIN_RESET);
    break;
    
  case GAIN_128:
    HAL_GPIO_WritePin(PGA_GAIN_PORT, PGA_GAIN_PIN_BIT_2, GPIO_PIN_SET);
    HAL_GPIO_WritePin(PGA_GAIN_PORT, PGA_GAIN_PIN_BIT_1, GPIO_PIN_SET);
    HAL_GPIO_WritePin(PGA_GAIN_PORT, PGA_GAIN_PIN_BIT_0, GPIO_PIN_SET);
    break;
  }
}
/**************************************************************************
*   Function name: Current_Select
*   Returns      : uint16_t(значение тока для установки)
*   Parameters   : IMP_CURR_STRUCT Current_Struct(Структура о данных тока, описание типа находится в заголовочном файле)
*   Purpose      : Выбор тока
****************************************************************************/
uint16_t Current_Select(IMP_CURR_STRUCT* Current_Struct)
{  
  switch(Current_Struct->Range)
  {
  case RANGE_1:
    if(ImpedanceData.Current.Val_Pos == CURR_LOW) return CURR_RANGE_1_LO;
    else return CURR_RANGE_1_HI;
    break;
  case RANGE_2:
    if(ImpedanceData.Current.Val_Pos == CURR_LOW) return CURR_RANGE_2_LO;
    else return CURR_RANGE_2_HI;
    break;
  case RANGE_3:
    if(ImpedanceData.Current.Val_Pos == CURR_LOW) return CURR_RANGE_3_LO;
    else return CURR_RANGE_3_HI;
    break;
  case RANGE_4:
    if(ImpedanceData.Current.Val_Pos == CURR_LOW) return CURR_RANGE_4_LO;
    else return CURR_RANGE_4_HI;
    break;
  }
  return 0;
}


/**************************************************************************
*   Function name: Impedance_ADC_Init
*   Returns      : void
*   Parameters   : void
*   Purpose      : Инициализация АЦП для измерения импеданса
****************************************************************************/
void Impedance_ADC_Init()
{
  IMPEDANCE_ADC_CLK_ENABLE();
  // ADC Voltage, ADC Current
  ADC_ChannelConfTypeDef sConfig = {0};
  //Настройка АЦП
  ADC_HandleTypeDef ADC_ITD;
  ADC_ITD.Instance = IMPEDANCE_ADC;
  ADC_ITD.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  ADC_ITD.Init.Resolution = ADC_RESOLUTION_12B;
  ADC_ITD.Init.ScanConvMode = DISABLE;
  ADC_ITD.Init.ContinuousConvMode = DISABLE;
  ADC_ITD.Init.DiscontinuousConvMode = DISABLE;
  ADC_ITD.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  ADC_ITD.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  ADC_ITD.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  ADC_ITD.Init.NbrOfConversion = 1;
  ADC_ITD.Init.DMAContinuousRequests = DISABLE;
  ADC_ITD.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  HAL_ADC_Init(&ADC_ITD);

  //Настройка канала измерения напряжения
  sConfig.Channel = IMPEDANCE_V_CHAN;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  HAL_ADC_ConfigChannel(&ADC_ITD, &sConfig);
}

/**************************************************************************
*   Function name: Resp_Calc
*   Returns      : void
*   Parameters   : void
*   Purpose      : Расчёт отклика напряжения
****************************************************************************/
void Resp_Calc()
{
  //Расчитываем среднее
  uint32_t ADC_VAL_HI = ImpedanceData.Measure.Samples_Sum_HI / ImpedanceData.Measure.Samples_Count_HI;
  uint32_t ADC_VAL_LO = ImpedanceData.Measure.Samples_Sum_LO / ImpedanceData.Measure.Samples_Count_LO;
  
  //Зануляем буфера
  ImpedanceData.Measure.Samples_Count_HI = 0;
  ImpedanceData.Measure.Samples_Sum_HI = 0;
  ImpedanceData.Measure.Samples_Count_LO = 0;
  ImpedanceData.Measure.Samples_Sum_LO = 0;
  ImpedanceData.Measure.Period_Count = 0;
  
  //Отклик
  ImpedanceData.Measure.V_Response = ADC_VAL_LO - ADC_VAL_HI;
}

/**************************************************************************
*   Function name: Resist_Calc
*   Returns      : void
*   Parameters   : void
*   Purpose      : Расчёт сопротивления
****************************************************************************/
void Resist_Calc()
{
  uint64_t Resist = 250000;
  Resist /= 4096;
  Resist *= ImpedanceData.Measure.V_Response;
  Resist *= 10;
  Resist /= ImpedanceData.Measure.Gain;
  switch(ImpedanceData.Current.Range)
  {
  case RANGE_1:
    Resist /= (CURR_RANGE_1_HI - CURR_RANGE_1_LO)/10;
    break;
  case RANGE_2:
    Resist /= (CURR_RANGE_2_HI - CURR_RANGE_2_LO)/10;
    break;
  case RANGE_3:
    Resist /= (CURR_RANGE_3_HI - CURR_RANGE_3_LO)/10;
    break;
  case RANGE_4:
    Resist /= (CURR_RANGE_4_HI - CURR_RANGE_4_LO)/10;
    break;
  }
  
  Resist *= 10000;
  Resist /= ConfigData.ConfigRegion->messImpCoeff;
  
  uint16_t Resist_result;
  if ((Resist/100) > 6000)
  {
    Resist_result = 0x00;
  }
  else if ((Resist/100) < 99)
  {
    Resist_result = ((uint16_t)Resist | 0x8000);
  }
  else if ((Resist/100) < 999)
  {
    Resist_result = (((uint16_t)(Resist / 10)) | 0x4000);
  }
  else
  {
    Resist_result = (uint16_t)(Resist/100);
  }
  ImpedanceData.Measure.Resistance = Resist_result;
}


/**************************************************************************
*   Function name: TIM2_IRQHandler
*   Returns      : void
*   Parameters   : void
*   Purpose      : Обработчик прерывания таймера TIM2 для генерации меандра
****************************************************************************/
void TIM2_IRQHandler()
{
  // Если это прерываение по обновлению
  if((TIM2->SR & TIM_SR_UIF) != 0)
  {
    // Очищаем флаг прерывания
    TIM2->SR &= ~TIM_SR_UIF;
    
    // Останавливаем таймер на время настройки
    TIM4->CR1 &= ~TIM_CR1_CEN;
    
    //Если необходимо выбрать предел
    if(ImpedanceData.Step == RANGE_SEL_STEP)
    {
      if(ImpedanceData.Measure.V_Response < MIN_RESP_VAL && ImpedanceData.Current.Range < RANGE_4)
      {
        ImpedanceData.Current.Range++;
        ImpedanceData.Step++;
      }else if(ImpedanceData.Measure.V_Response > MAX_RESP_VAL && ImpedanceData.Measure.Gain > 1)
      {
        ImpedanceData.Measure.Gain /= 2;
        ImpedanceData.Step++;
      }else 
      { 
        ImpedanceData.Measure.Resistance = 0;
        ImpedanceData.Step = DONE;
      }// Если варианты закончились выходим из измерения
    }
    //Переключаем на противоположное значение тока
    if(ImpedanceData.Current.Val_Pos == CURR_LOW)
    {
      ImpedanceData.Current.Val_Pos = CURR_HIGH;
    }else {ImpedanceData.Current.Val_Pos = CURR_LOW;}
    
    //Выбераем ток в зависимости от предела
    uint16_t Curr_Sel = Current_Select(&ImpedanceData.Current);
    //Выставляем ток на выходе
    SetCurrent(Curr_Sel);
    
    switch(ImpedanceData.Step)
    {
    case CHECK_STEP:
      //Ждём необходимого количества периодов
      if(ImpedanceData.Measure.Period_Count == CHECK_PERIOD_COUNT)
      {
        //Расчёт отклика напряжения
        Resp_Calc();
        //Если отклик попадает в допустимые границы, запускаем цикл измерения, иначе возвращаемся к выбору предела
        if(ImpedanceData.Measure.V_Response > MIN_RESP_VAL && ImpedanceData.Measure.V_Response < MAX_RESP_VAL)
        {
          ImpedanceData.Step++;
        }else
        {
          ImpedanceData.Step--;
        }
      }
      break;
    case MEASURE_STEP:
      //Ждём необходимого количества периодов
      if(ImpedanceData.Measure.Period_Count == MEASURE_PERIOD_COUNT)
      {
        //Расчёт отклика напряжения
        Resp_Calc();
        //Расчёт сопротивления
        Resist_Calc();
        //Конец измерения
        ImpedanceData.Step++;
      }
      break;
    case DONE:
      //Записываем результат в регистр MODBUS
      DeviceRegister.ro_region.DeviceImpedance = ImpedanceData.Measure.Resistance;
      //Убираем флаг измерения импеданса в регистре MODBUS
      DeviceRegister.ro_region.DeviceState &= ~IMPEDANCE_MEASURE_FLAG;
      //Останавливаем таймеры и заканчиваем измерение
      Impedance_Measure_STOP();
      break;
    }
    
    //Запускаем АЦП и таймер только если находимся либо в режиме проверки предела, либо в режим измерения
    if(ImpedanceData.Step == CHECK_STEP || ImpedanceData.Step == MEASURE_STEP)
    {
      //Запускаем измеритель
      //Выставляем усиление
      PGA_Gain_Select(ImpedanceData.Measure.Gain);
      
      ImpedanceData.Measure.DelayToStart = DELAY_TO_START_MEAS;
      ImpedanceData.Measure.Measure_Count = MEAS_COUNT;
      //Включаем АЦП
      IMPEDANCE_ADC->CR2 |= ADC_CR2_ADON;
      //Запускаем первое преобразование АЦП
      IMPEDANCE_ADC->CR2 |= ADC_CR2_SWSTART;
      // Очищаем флаг прерывания
      TIM4->SR &= ~TIM_SR_UIF;
      // Запускаем таймер измерения
      TIM4->CR1 |= TIM_CR1_CEN;
      
      //Если верхний ток добавляем период
      if(ImpedanceData.Current.Val_Pos == CURR_LOW)
      {
        ImpedanceData.Measure.Period_Count++;
      }
    }
  }
}

/**************************************************************************
*   Function name: TIM4_IRQHandler
*   Returns      : void
*   Parameters   : void
*   Purpose      : Обработчик прерывания таймера TIM4 для измерения напряжения
****************************************************************************/
void TIM4_IRQHandler()
{
  // Если это прерываение по обновлению
  if((TIM4->SR & TIM_SR_UIF) != 0)
  {
    // Очищаем флаг прерывания
    TIM4->SR &= ~TIM_SR_UIF;
    if(ImpedanceData.Measure.DelayToStart)
    {
      ImpedanceData.Measure.DelayToStart--;
    }else
    {
      if(ImpedanceData.Measure.Measure_Count)
      {
         ImpedanceData.Measure.Measure_Count--;
         uint32_t ADC_VAL = IMPEDANCE_ADC->DR;
         
         //Выбираем необходимый буфер
         if(ImpedanceData.Measure.Period_Count >= MEAS_START_PERIOD)
         {
            switch(ImpedanceData.Current.Val_Pos)
            {
               case CURR_LOW:
                ImpedanceData.Measure.Samples_Sum_LO += ADC_VAL;
                ImpedanceData.Measure.Samples_Count_LO++;
                break;
              case CURR_HIGH:
                ImpedanceData.Measure.Samples_Sum_HI += ADC_VAL;
                ImpedanceData.Measure.Samples_Count_HI++;
                break;
            }
         }    
         //Запускаем новое преобразование
         IMPEDANCE_ADC->CR2 |= ADC_CR2_SWSTART; 
      }
    }
  }
}
