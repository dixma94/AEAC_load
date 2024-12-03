//***************************************************************************
//  File........: Impedance_FUNC.c
//  Author(s)...: KotyshkovEJ
//  Target(s)...: stm32f207
//  Compiler....: IAR
//  Description.: ������� ��������� ���������
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

//��������� �������
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
*   Purpose      : ������������� ��������� ��� ��������� ���������
****************************************************************************/
void Impedance_Periph_Init()
{
  //������������� ��������� ��� ��������� ���������
  ImpedanceData.Step = CHECK_STEP;
  
  ImpedanceData.Current.Range = RANGE_1;
  ImpedanceData.Current.Val_Pos = CURR_LOW;
  
  ImpedanceData.Measure.Gain = GAIN_128;
  ImpedanceData.Measure.Samples_Count_HI = 0;
  ImpedanceData.Measure.Samples_Count_LO = 0;
  ImpedanceData.Measure.Samples_Sum_HI = 0;
  ImpedanceData.Measure.Samples_Sum_LO = 0;
  ImpedanceData.Measure.V_Response = 0;
  
  //���������� ���� ��������� ���������
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
*   Purpose      : ��������� ������� ��� ��������� �������
****************************************************************************/
void Impedance_TIM_Init(void)
{
    // ��������� ������ ��� ��������� �������
    // TIM2 ��������� �� ���� APB1 � ����������� 60 ���
    // ������ ������� (60���/6000)/50=200��(5ms)
    
    // �������� ������������ �������
    IMPEDANCE_TIM_GEN_CLK_ENABLE();
    
    // ������������� ������ ��������� �������
    TIM_HandleTypeDef TIM_HTD ={0};
    TIM_HTD.Instance = TIM2;
    TIM_HTD.Init.Prescaler = 6000-1;
    TIM_HTD.Init.CounterMode = TIM_COUNTERMODE_UP;
    TIM_HTD.Init.Period = 50 - 1;
    TIM_HTD.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    TIM_HTD.Init.RepetitionCounter = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_Base_Init(&TIM_HTD);
    
    // ��������� ������ ��� ��������� ���������
    // TIM4 ��������� �� ���� APB1 � ����������� 60 ���
    // ������ ������� (60���/300)/5=40000��(25us)
    
    // �������� ������������ �������
    IMPEDANCE_TIM_MEAS_CLK_ENABLE();
    
    // ������������� ������ ���������� ����������
    TIM_HTD.Instance = TIM4;
    TIM_HTD.Init.Prescaler = 300-1;
    TIM_HTD.Init.CounterMode = TIM_COUNTERMODE_UP;
    TIM_HTD.Init.Period = 5 - 1;
    TIM_HTD.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    TIM_HTD.Init.RepetitionCounter = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_Base_Init(&TIM_HTD);
    
    // ��������� ����������� �� ����������
    TIM2->DIER |= TIM_DIER_UIE;
    TIM4->DIER |= TIM_DIER_UIE;
    
    // ��������� ���������� TIM2
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
    HAL_NVIC_SetPriority(TIM2_IRQn, 3, 0);
    
     // ��������� ���������� TIM4
    HAL_NVIC_EnableIRQ(TIM4_IRQn);
    HAL_NVIC_SetPriority(TIM4_IRQn, 0, 0);
    
    // ������� ���� ����������
    TIM2->SR &= ~TIM_SR_UIF;
    
    // ��������� ������ ���������
    TIM2->CR1 |= TIM_CR1_CEN;
}

/**************************************************************************
*   Function name: Impedance_Measure_STOP
*   Returns      : void
*   Parameters   : void
*   Purpose      : ��������� ��������� �������
****************************************************************************/
void Impedance_Measure_STOP(void)
{      
    // ��������� ���������� TIM2
    HAL_NVIC_DisableIRQ(TIM2_IRQn);
    HAL_NVIC_DisableIRQ(TIM4_IRQn);
    
    // ������������� �������
    TIM2->CR1 &= ~TIM_CR1_CEN;
    TIM4->CR1 &= ~TIM_CR1_CEN;
    
    // ������� ���� ����������
    TIM2->SR &= ~TIM_SR_UIF;
    TIM4->SR &= ~TIM_SR_UIF;
    
    // ��������� ������������ ��������
    IMPEDANCE_TIM_GEN_CLK_ENABLE();
    IMPEDANCE_TIM_MEAS_CLK_DISABLE();
    
    //������� �� ������ ���������
    Main_Struct.Load_Mode = LOAD_OFF;
    //���������� ���� ���������� ���������
    Main_Struct.Refrash_Flag = TRUE;
}

/**************************************************************************
*   Function name: DAC_GPIO_Init
*   Returns      : void
*   Parameters   : void
*   Purpose      : ������������� GPIO ��� ������ �������� �� AD8231
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
*   Purpose      : ������������� GPIO ��� ������ ���
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
*   Parameters   : PGA_GAIN Gain(��������, �������� ���� ��������� � ������������ �����)
*   Purpose      : ����� �������� �� AD8231
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
*   Returns      : uint16_t(�������� ���� ��� ���������)
*   Parameters   : IMP_CURR_STRUCT Current_Struct(��������� � ������ ����, �������� ���� ��������� � ������������ �����)
*   Purpose      : ����� ����
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
*   Purpose      : ������������� ��� ��� ��������� ���������
****************************************************************************/
void Impedance_ADC_Init()
{
  IMPEDANCE_ADC_CLK_ENABLE();
  // ADC Voltage, ADC Current
  ADC_ChannelConfTypeDef sConfig = {0};
  //��������� ���
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

  //��������� ������ ��������� ����������
  sConfig.Channel = IMPEDANCE_V_CHAN;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  HAL_ADC_ConfigChannel(&ADC_ITD, &sConfig);
}

/**************************************************************************
*   Function name: Resp_Calc
*   Returns      : void
*   Parameters   : void
*   Purpose      : ������ ������� ����������
****************************************************************************/
void Resp_Calc()
{
  //����������� �������
  uint32_t ADC_VAL_HI = ImpedanceData.Measure.Samples_Sum_HI / ImpedanceData.Measure.Samples_Count_HI;
  uint32_t ADC_VAL_LO = ImpedanceData.Measure.Samples_Sum_LO / ImpedanceData.Measure.Samples_Count_LO;
  
  //�������� ������
  ImpedanceData.Measure.Samples_Count_HI = 0;
  ImpedanceData.Measure.Samples_Sum_HI = 0;
  ImpedanceData.Measure.Samples_Count_LO = 0;
  ImpedanceData.Measure.Samples_Sum_LO = 0;
  ImpedanceData.Measure.Period_Count = 0;
  
  //������
  ImpedanceData.Measure.V_Response = ADC_VAL_LO - ADC_VAL_HI;
}

/**************************************************************************
*   Function name: Resist_Calc
*   Returns      : void
*   Parameters   : void
*   Purpose      : ������ �������������
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
*   Purpose      : ���������� ���������� ������� TIM2 ��� ��������� �������
****************************************************************************/
void TIM2_IRQHandler()
{
  // ���� ��� ����������� �� ����������
  if((TIM2->SR & TIM_SR_UIF) != 0)
  {
    // ������� ���� ����������
    TIM2->SR &= ~TIM_SR_UIF;
    
    // ������������� ������ �� ����� ���������
    TIM4->CR1 &= ~TIM_CR1_CEN;
    
    //���� ���������� ������� ������
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
      }// ���� �������� ����������� ������� �� ���������
    }
    //����������� �� ��������������� �������� ����
    if(ImpedanceData.Current.Val_Pos == CURR_LOW)
    {
      ImpedanceData.Current.Val_Pos = CURR_HIGH;
    }else {ImpedanceData.Current.Val_Pos = CURR_LOW;}
    
    //�������� ��� � ����������� �� �������
    uint16_t Curr_Sel = Current_Select(&ImpedanceData.Current);
    //���������� ��� �� ������
    SetCurrent(Curr_Sel);
    
    switch(ImpedanceData.Step)
    {
    case CHECK_STEP:
      //��� ������������ ���������� ��������
      if(ImpedanceData.Measure.Period_Count == CHECK_PERIOD_COUNT)
      {
        //������ ������� ����������
        Resp_Calc();
        //���� ������ �������� � ���������� �������, ��������� ���� ���������, ����� ������������ � ������ �������
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
      //��� ������������ ���������� ��������
      if(ImpedanceData.Measure.Period_Count == MEASURE_PERIOD_COUNT)
      {
        //������ ������� ����������
        Resp_Calc();
        //������ �������������
        Resist_Calc();
        //����� ���������
        ImpedanceData.Step++;
      }
      break;
    case DONE:
      //���������� ��������� � ������� MODBUS
      DeviceRegister.ro_region.DeviceImpedance = ImpedanceData.Measure.Resistance;
      //������� ���� ��������� ��������� � �������� MODBUS
      DeviceRegister.ro_region.DeviceState &= ~IMPEDANCE_MEASURE_FLAG;
      //������������� ������� � ����������� ���������
      Impedance_Measure_STOP();
      break;
    }
    
    //��������� ��� � ������ ������ ���� ��������� ���� � ������ �������� �������, ���� � ����� ���������
    if(ImpedanceData.Step == CHECK_STEP || ImpedanceData.Step == MEASURE_STEP)
    {
      //��������� ����������
      //���������� ��������
      PGA_Gain_Select(ImpedanceData.Measure.Gain);
      
      ImpedanceData.Measure.DelayToStart = DELAY_TO_START_MEAS;
      ImpedanceData.Measure.Measure_Count = MEAS_COUNT;
      //�������� ���
      IMPEDANCE_ADC->CR2 |= ADC_CR2_ADON;
      //��������� ������ �������������� ���
      IMPEDANCE_ADC->CR2 |= ADC_CR2_SWSTART;
      // ������� ���� ����������
      TIM4->SR &= ~TIM_SR_UIF;
      // ��������� ������ ���������
      TIM4->CR1 |= TIM_CR1_CEN;
      
      //���� ������� ��� ��������� ������
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
*   Purpose      : ���������� ���������� ������� TIM4 ��� ��������� ����������
****************************************************************************/
void TIM4_IRQHandler()
{
  // ���� ��� ����������� �� ����������
  if((TIM4->SR & TIM_SR_UIF) != 0)
  {
    // ������� ���� ����������
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
         
         //�������� ����������� �����
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
         //��������� ����� ��������������
         IMPEDANCE_ADC->CR2 |= ADC_CR2_SWSTART; 
      }
    }
  }
}
