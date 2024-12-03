//***************************************************************************
//  File........: CurrentSet_FUNC.c
//  Author(s)...: KotyshkovEJ
//  Target(s)...: stm32f207
//  Compiler....: IAR
//  Description.: Инициализация переферии (DAC)
//  Date........: 13.04.2021
//***************************************************************************

#include "CurrentSet_FUNC.h"
#include "BoardConfig.h"
#include "Alg_FUNC.h"
#include "Measure_FUNC.h"

static  DAC_HandleTypeDef hdac = {0};

extern CONFIG_DATA ConfigData;
extern DEVICE_REGISTER DeviceRegister;
extern double CalibCurrOutCoeff         [3];
extern uint8_t fCalibrated;
extern uint8_t ModbusConfigState;
extern LOAD_VERSION_LIMITS Load_limits;

//Прототипы функций
void DAC_GPIO_Init(void);
void DAC_Init(void);
uint16_t CurrCoeff_Calc(uint16_t Value);

/**************************************************************************
*   Function name: DAC_Init
*   Returns      : void
*   Parameters   : void
*   Purpose      : Инициализация GPIO для работы ЦАП
****************************************************************************/
void CurrentSet_Init()
{
  // Настройка портов в/в
  DAC_GPIO_Init();
  
  // Настройка ЦАП
  DAC_Init();
  
  
}

/**************************************************************************
*   Function name: SetCurrent
*   Returns      : void
*   Parameters   : uint16_t Value(Значение, которое запишется в ЦАП)
*   Purpose      : Функция устанавливает выходное значение на ЦАП
****************************************************************************/
void SetCurrent(uint16_t Value)
{
   //если нормальный режим
   if (fCalibrated && (ConfigData.ModbusConfigState != MB_DEVICE_CONF_STATE))
   {
        if(Value <= Load_limits.Max_Current && Value > 0)
        {
           DAC->DHR12R2 = CurrCoeff_Calc(Value*10);
        }
        else
        {
           DAC->DHR12R2 = 0;
        }
   }
   else
   {
     //если режим конфигурации
      DAC->DHR12R2 = Value;
   }
}

/**************************************************************************
*   Function name: CurrCoeff_Calc
*   Returns      : uint16_t(Рассчётное значение для ЦАП)
*   Parameters   : uint16_t Value(Значение в абсолютных единицах тока)
*   Purpose      : Функция рассчитывает значение для ЦАП на основе калибровочных коэффициентов
****************************************************************************/
uint16_t CurrCoeff_Calc(uint16_t Value)
{
  uint32_t Calc_Curr = 0;
  Calc_Curr = GetRezult ((double*)&CalibCurrOutCoeff, (uint16_t)Value);
  return Calc_Curr;
}

/**************************************************************************
*   Function name: DAC_GPIO_Init
*   Returns      : void
*   Parameters   : void
*   Purpose      : Инициализация GPIO для работы ЦАП
****************************************************************************/
void DAC_GPIO_Init()
{
  DAC_PORT_PIN_RCC_ENABLE();
  
  GPIO_InitTypeDef GPIO_ITD;
    GPIO_ITD.Pin = DAC_PIN;
    GPIO_ITD.Mode = GPIO_MODE_ANALOG;
    GPIO_ITD.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_ITD.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DAC_PIN_PORT, &GPIO_ITD);
}

/**************************************************************************
*   Function name: DAC_Init
*   Returns      : void
*   Parameters   : void
*   Purpose      : Инициализация самого ЦАП
****************************************************************************/
void DAC_Init()
{  
  DAC_RCC_ENABLE();
  //Настройки DAC
  hdac.Instance = DAC;
  //Инициализация DAC
  HAL_DAC_Init(&hdac);
  //Настройка канала DAC
  DAC_ChannelConfTypeDef hchandac;
  
  hchandac.DAC_OutputBuffer = DAC_OUTPUTBUFFER_DISABLE;
  hchandac.DAC_Trigger = DAC_TRIGGER_NONE;
  
  HAL_DAC_ConfigChannel(&hdac, &hchandac,DAC_CHAN);
  
  //Записываем нулевое значение DAC, чтобы сбросить выход в 0
  DAC->DHR12R2 = 0;
  
  HAL_DAC_Start(&hdac, DAC_CHAN);
  
  
}
