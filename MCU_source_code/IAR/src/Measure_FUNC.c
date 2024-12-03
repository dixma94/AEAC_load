//***************************************************************************
//  File........: Measure_FUNC.c
//  Author(s)...: KotyshkovEJ
//  Target(s)...: stm32f207
//  Compiler....: IAR
//  Description.: ������������� ��������� (GPIO, ADC, DMA, TIM)
//  Date........: 13.04.2021
//***************************************************************************

#include "Measure_FUNC.h"
#include "BoardConfig.h"
#include "math.h"
#include "Alg_FUNC.h"

uint16_t ADC_Arr[2] = {0};
double CalibCurrOutCoeff         [3];
double CalibVoltMeasCoeff        [3];
double CalibCurrMeasCoeff        [3];
uint8_t fCalibrated = 0;


ADC_HandleTypeDef ADC_ITD;
DMA_HandleTypeDef hdma_ADC;

static MEASURE_FILT Measure_Filter = {0};

extern DEVICE_REGISTER DeviceRegister;
extern CONFIG_DATA ConfigData;
extern LOAD_VERSION_LIMITS Load_limits;

//��������� �������
void Measure_GPIO_Init();
void Measure_ADC_Init();
void Measure_DMA_Init();
void Measure_TIM_Init();


/**************************************************************************
*   Function name: Measure_Init
*   Returns      : void
*   Parameters   : void
*   Purpose      : ������� �������������� ��������� ��� ��������� (GPIO, ADC, DMA, TIM)
****************************************************************************/
void Measure_Init()
{
  // ��������� ������ �/�
  Measure_GPIO_Init();
  
  // ��������� DMA
  Measure_DMA_Init();
  
  // ��������� ���
  Measure_ADC_Init();
  
  // ��������� �������
  Measure_TIM_Init();
  
  //������ DMA c ��������� �������
  HAL_ADC_Start_DMA(&ADC_ITD, (uint32_t*)ADC_Arr, 2);
}

/**************************************************************************
*   Function name: Measure_GPIO_Init
*   Returns      : void
*   Parameters   : void
*   Purpose      : ������������� GPIO ��� ������ ���
****************************************************************************/
void Measure_GPIO_Init()
{
  MEASURE_GPIO_CLK_ENABLE();
  
  GPIO_InitTypeDef GPIO_ITD;
  
  // ADC Voltage, ADC Current
  GPIO_ITD.Pin = MEASURE_V_PIN| MEASURE_I_PIN;
  GPIO_ITD.Mode = GPIO_MODE_ANALOG;
  GPIO_ITD.Pull = GPIO_NOPULL;
  GPIO_ITD.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(MEASURE_PORT, &GPIO_ITD);
}

/**************************************************************************
*   Function name: Measure_DMA_Init
*   Returns      : void
*   Parameters   : void
*   Purpose      : ������������� DMA ��� ������ ���
****************************************************************************/
void Measure_DMA_Init(void)
{
  DMA_ADC_CLK_ENABLE();
  
  hdma_ADC.Instance = DMA_ADC_Stream;
  hdma_ADC.Init.Channel = DMA_CHANNEL_0;
  hdma_ADC.Init.Direction = DMA_PERIPH_TO_MEMORY;
  hdma_ADC.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_ADC.Init.MemInc = DMA_MINC_ENABLE;
  hdma_ADC.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  hdma_ADC.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
  hdma_ADC.Init.Mode = DMA_CIRCULAR;
  hdma_ADC.Init.Priority = DMA_PRIORITY_HIGH;
  hdma_ADC.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
  HAL_DMA_Init(&hdma_ADC);
  
  ADC_ITD.DMA_Handle = &hdma_ADC;
  hdma_ADC.Parent = &ADC_ITD; 

  HAL_NVIC_SetPriority(DMA_ADC_IRQn, 0, 1);
  HAL_NVIC_EnableIRQ(DMA_ADC_IRQn);
}

/**************************************************************************
*   Function name: Measure_ADC_Init
*   Returns      : void
*   Parameters   : void
*   Purpose      : ������������� ������� ��� ������ ��� � ������ ��������� ���������
****************************************************************************/
void Measure_TIM_Init()
{
  // ��������� ������ �� 10 ��� ��� ��������� ���������� � ����
  // TIM3 ��������� �� ���� APB1 � ����������� 60 ���
  // ������ ������� (60���/600)/10 = 10���
  
  // �������� ������������ �������
  __HAL_RCC_TIM3_CLK_ENABLE();
  
  // ������������� ������
  TIM_HandleTypeDef TIM3_HTD ={0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  
  TIM3_HTD.Instance = TIM3;
  TIM3_HTD.Init.Prescaler = 600-1;
  TIM3_HTD.Init.CounterMode = TIM_COUNTERMODE_UP;
  TIM3_HTD.Init.Period = 10-1;
  TIM3_HTD.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  TIM3_HTD.Init.RepetitionCounter = TIM_AUTORELOAD_PRELOAD_ENABLE;
  HAL_TIM_Base_Init(&TIM3_HTD);
  
  // TIM3 TRGO selection /
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
  HAL_TIMEx_MasterConfigSynchronization(&TIM3_HTD, &sMasterConfig);
  // ��������� ����������� �� ����������
  TIM3->DIER |= TIM_DIER_UIE;
  
  // ��������� ���������� TIM6
  HAL_NVIC_EnableIRQ(TIM3_IRQn);
  HAL_NVIC_SetPriority(TIM3_IRQn, 1, 1);
  
  // ������� ���� ����������
  TIM3->SR &= ~TIM_SR_UIF;
  
  // ��������� ������
  TIM3->CR1 |= TIM_CR1_CEN;
  
}

/**************************************************************************
*   Function name: Measure_ADC_Init
*   Returns      : void
*   Parameters   : void
*   Purpose      : ������������� ������ ���
****************************************************************************/
void Measure_ADC_Init()
{
  MEASURE_ADC_CLK_ENABLE();
  
  // ADC Voltage, ADC Current
  ADC_ChannelConfTypeDef sConfig = {0};

  //��������� ���
  ADC_ITD.Instance = MEASURE_ADC;
  ADC_ITD.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  ADC_ITD.Init.Resolution = ADC_RESOLUTION_12B;
  ADC_ITD.Init.ScanConvMode = ENABLE;
  ADC_ITD.Init.ContinuousConvMode = DISABLE;
  ADC_ITD.Init.DiscontinuousConvMode = DISABLE;
  ADC_ITD.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  ADC_ITD.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
  ADC_ITD.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  ADC_ITD.Init.NbrOfConversion = 2;
  ADC_ITD.Init.DMAContinuousRequests = ENABLE;
  ADC_ITD.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  HAL_ADC_Init(&ADC_ITD);

  //��������� ������ ��������� ����
  sConfig.Channel = MEASURE_I_CHAN;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  HAL_ADC_ConfigChannel(&ADC_ITD, &sConfig);
  
  //��������� ������ ��������� ����������
  sConfig.Channel = MEASURE_V_CHAN;
  sConfig.Rank = 2;
  HAL_ADC_ConfigChannel(&ADC_ITD, &sConfig);
}

/**************************************************************************
*   Function name: DMA_ADC_IRQHandler
*   Returns      : void
*   Parameters   : void
*   Purpose      : ���������� ���������� �� ���������� �������� ������ DMA
****************************************************************************/
void DMA_ADC_IRQHandler()
{
  // ���� ���������� �� ��������� ��������
  if ((DMA_ADC->LISR & DMA_ADC_IF_TCIF) != 0)
  {
    // ���������� ���� ���������� �� ��������� �������� DMA
    DMA_ADC->LIFCR |= DMA_ADC_IFC_TCIF;
    // ���������� ���� ���������� �� �������� �������� DMA
    DMA_ADC->LIFCR |= DMA_ADC_IFC_HTIF;
    if (Measure_Filter.Point_cnt++ >= 1024)
    {
      Measure_Filter.Point_cnt = 0;
      
      uint32_t VoltRMS = (uint16_t)sqrt((uint32_t)(Measure_Filter.Voltage_Squares_Sum >> 10));
      uint32_t CurrRMS = (uint16_t)sqrt((uint32_t)(Measure_Filter.Current_Squares_Sum >> 10));
      
       if (fCalibrated && (ConfigData.ModbusConfigState != MB_DEVICE_CONF_STATE))
       { 
          VoltRMS = GetRezult ((double*)&CalibVoltMeasCoeff, (uint16_t)VoltRMS)/10;
          CurrRMS = GetRezult ((double*)&CalibCurrMeasCoeff, (uint16_t)CurrRMS)/10;
          
          
         // ���������� �������
           if ((VoltRMS > Load_limits.Max_Voltage) || (VoltRMS < 20))
           {
              SW_Pin_Rem(FALSE);
           }else 
           {
              SW_Pin_Rem(TRUE);
           }
       }

      DeviceRegister.ro_region.DeviceVoltageRMS = VoltRMS;
      DeviceRegister.ro_region.DeviceCurrentRMS = CurrRMS;
      Measure_Filter.Voltage_Squares_Sum = 0;
      Measure_Filter.Current_Squares_Sum = 0;
    }else{
      Measure_Filter.Voltage_Squares_Sum += (ADC_Arr[1] * ADC_Arr[1]);
      Measure_Filter.Current_Squares_Sum += (ADC_Arr[0] * ADC_Arr[0]);
    }
  }
}

/**************************************************************************
*   Function name: CalibMakeCoeff
*   Returns      : void
*   Parameters   : double *,uint16_t *,uint16_t *
*   Purpose      : ������ ������� �������������
****************************************************************************/
void CalibMakeCoeff (double * dest_mass, uint16_t * CalibrPoint, uint16_t * Point)
{
    double u1=(double)*(CalibrPoint);
    double u2=(double)*(CalibrPoint + 1);
    double u3=(double)*(CalibrPoint + 2);
    double x1 = (double)*(Point);
    double x2 = (double)*(Point + 1);
    double x3 = (double)*(Point + 2);
    
    double denom = (-x3+x2)*(-x2*x1+x2*x3+x1*x1-x3*x1);
    *(dest_mass)    = (x2*x2*u1*x3-x2*x2*x1*u3-x3*x3*u1*x2+u3*x1*x1*x2+x3*x3*x1*u2-u2*x1*x1*x3)/denom;
    *(dest_mass + 1)=-(-x2*x2*u3+x2*x2*u1-x3*x3*u1-u2*x1*x1+u3*x1*x1+u2*x3*x3)/denom;
    *(dest_mass + 2)=(x1*u3-x1*u2+x3*u2-u3*x2-u1*x3+u1*x2)/(-x1*x1*x3+x1*x1*x2-x1*x2*x2+x1*x3*x3-x3*x3*x2+x3*x2*x2);
}

/**************************************************************************
*   Function name: GetRezult
*   Returns      : uint16_t
*   Parameters   : double *,uint16_t
*   Purpose      : ������ �������� �� �������������
****************************************************************************/
uint16_t GetRezult (double * coeff, uint16_t value)
{
    double x=(double)value;
    
    double rez = ((*(coeff + 2)*x*x+(*(coeff + 1))*x+(*(coeff))) + 0.5);
    
    // ���� ��������� �������������, �� ������� "0"
    if (rez < 0) rez = 0;
    
    // ����������� ���������, ������ ���������� �� ������ ( + 0.5 )
    return  (uint16_t)rez;
}