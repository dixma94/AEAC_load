//***************************************************************************
//  File........: CurrentSet_FUNC.c
//  Author(s)...: KotyshkovEJ
//  Target(s)...: stm32f207
//  Compiler....: IAR
//  Description.: ������������� ��������� (DAC)
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

//��������� �������
void DAC_GPIO_Init(void);
void DAC_Init(void);
uint16_t CurrCoeff_Calc(uint16_t Value);

/**************************************************************************
*   Function name: DAC_Init
*   Returns      : void
*   Parameters   : void
*   Purpose      : ������������� GPIO ��� ������ ���
****************************************************************************/
void CurrentSet_Init()
{
  // ��������� ������ �/�
  DAC_GPIO_Init();
  
  // ��������� ���
  DAC_Init();
  
  
}

/**************************************************************************
*   Function name: SetCurrent
*   Returns      : void
*   Parameters   : uint16_t Value(��������, ������� ��������� � ���)
*   Purpose      : ������� ������������� �������� �������� �� ���
****************************************************************************/
void SetCurrent(uint16_t Value)
{
   //���� ���������� �����
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
     //���� ����� ������������
      DAC->DHR12R2 = Value;
   }
}

/**************************************************************************
*   Function name: CurrCoeff_Calc
*   Returns      : uint16_t(���������� �������� ��� ���)
*   Parameters   : uint16_t Value(�������� � ���������� �������� ����)
*   Purpose      : ������� ������������ �������� ��� ��� �� ������ ������������� �������������
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
*   Purpose      : ������������� GPIO ��� ������ ���
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
*   Purpose      : ������������� ������ ���
****************************************************************************/
void DAC_Init()
{  
  DAC_RCC_ENABLE();
  //��������� DAC
  hdac.Instance = DAC;
  //������������� DAC
  HAL_DAC_Init(&hdac);
  //��������� ������ DAC
  DAC_ChannelConfTypeDef hchandac;
  
  hchandac.DAC_OutputBuffer = DAC_OUTPUTBUFFER_DISABLE;
  hchandac.DAC_Trigger = DAC_TRIGGER_NONE;
  
  HAL_DAC_ConfigChannel(&hdac, &hchandac,DAC_CHAN);
  
  //���������� ������� �������� DAC, ����� �������� ����� � 0
  DAC->DHR12R2 = 0;
  
  HAL_DAC_Start(&hdac, DAC_CHAN);
  
  
}
