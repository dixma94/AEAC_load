//***************************************************************************
//  File........: MODBUS_TIMEOUT.c
//  Author(s)...: KotyshkovEJ
//  Target(s)...: stm32f207
//  Compiler....: IAR
//  Description.: ������� ��������� ���������
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
*   Purpose      : ��������� ������� ��� �������� ��������
****************************************************************************/
void MODBUS_TIMEOUT_Init(void)
{
  // ��������� ������ ��� ��������� �������
  // TIM5 ��������� �� ���� APB1 � ����������� 60 ���
  // ������ ������� (60���/6000)/1000=10��(0.1s)
  // �������� ������������ �������
  
  MODBUS_TIMEOUT_TIM_CLK_ENABLE();
  
  // ������������� ������ �������� ��������
  TIM_HandleTypeDef TIM_HTD ={0};
  TIM_HTD.Instance = TIM5;
  TIM_HTD.Init.Prescaler = 6000 - 1;
  TIM_HTD.Init.CounterMode = TIM_COUNTERMODE_UP;
  TIM_HTD.Init.Period = 1000 - 1;
  TIM_HTD.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  TIM_HTD.Init.RepetitionCounter = TIM_AUTORELOAD_PRELOAD_ENABLE;
  HAL_TIM_Base_Init(&TIM_HTD);
  
  // ��������� ����������� �� ����������
  TIM5->DIER |= TIM_DIER_UIE;
  
  // ��������� ���������� TIM2
  HAL_NVIC_EnableIRQ(TIM5_IRQn);
  HAL_NVIC_SetPriority(TIM5_IRQn, 3, 0);
  
  // ������� ���� ����������
  TIM5->SR &= ~TIM_SR_UIF;
}

/**************************************************************************
*   Function name: MODBUS_TIMEOUT_En
*   Returns      : void
*   Parameters   : BOOL Enable
*   Purpose      : ������ ������� ��� �������� ��������
****************************************************************************/
void MODBUS_TIMEOUT_En(BOOL Enable)
{
  if(Enable)
  {
    CommCtrlParam.CtrlCounterEn = TRUE;
    CommCtrlParam.CtrlCounterVal = 0;
    // ��������� ������ ��������
    TIM5->CR1 |= TIM_CR1_CEN;
  }else
  {
    CommCtrlParam.CtrlCounterEn = FALSE;
    // ��������� ������ ��������
    TIM5->CR1 &= ~TIM_CR1_CEN;
  }
  
}

/**************************************************************************
*   Function name: TIM5_IRQHandler
*   Returns      : void
*   Parameters   : void
*   Purpose      : ���������� ���������� ������� TIM5 ��� �������� �������� �����
****************************************************************************/
void TIM5_IRQHandler()
{
  // ���� ��� ����������� �� ����������
  if((TIM5->SR & TIM_SR_UIF) != 0)
  {
    // ������� ���� ����������
    TIM5->SR &= ~TIM_SR_UIF;
     // ������� �������� ������ �����
    if (CommCtrlParam.CtrlCounterEn)
    {
      // ���� ����� ��� N ������, 
      if (CommCtrlParam.CtrlCounterVal < LOST_COMM_TIMEOUT)
      {
        CommCtrlParam.CtrlCounterVal++;
      }
      else
      {
        // ������������� ��������
        Main_Struct.Load_Mode = LOAD_OFF;
        Main_Struct.Refrash_Flag = TRUE;
        MODBUS_TIMEOUT_En(FALSE);
      }
    }
  }
}