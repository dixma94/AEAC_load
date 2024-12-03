//***************************************************************************
//  File........: USART_Driver.c
//  Author(s)...: KotyshkovEJ
//  Target(s)...: stm32f207
//  Compiler....: IAR
//  Description.: ������������� ��������� (GPIO, USART, DMA, TIMER)
//  Date........: 09.04.2021
//***************************************************************************

#include "UART_Driver.h"
#include "BoardConfig.h"
#include "MODBUS_TIMEOUT.h"

uint8_t RxTxFrame[MB_BUF_LEN];
volatile uint8_t RxTxLength = 0;

extern CONFIG_DATA ConfigData;
extern COMM_CONTROL_PARAM CommCtrlParam;
extern DEVICE_REGISTER DeviceRegister;

const uint16_t DelayTimeMass[3] = {0, 150, 400}; // ������ ��������, ��� �������� � ����� ������������

//��������� �������
void UART_GPIO_Init(void);
void UART_DMA_Init(void);
void UART_TIM_Init(void);



/**************************************************************************
*   Function name: UART_Init
*   Returns      : void
*   Parameters   : void
*   Purpose      : ������������� ���� ���������
****************************************************************************/
void UART_Init(void)
{
    // ��������� ������ �/�
    UART_GPIO_Init();
    
    // ��������� DMA
    UART_DMA_Init();
    
    // ��������� �������
    UART_TIM_Init();
	
    // �������� ������������
    UART_CLK_ENABLE();
    
    // ������������� UART
    UART_HandleTypeDef UART_HTD;
    UART_HTD.Instance = MB_UART;
    
    UART_HTD.Init.BaudRate = UART_BRATE;
    UART_HTD.Init.WordLength = UART_WORDLENGTH_8B;
    UART_HTD.Init.StopBits = UART_STOPBITS_1;
    UART_HTD.Init.Parity = UART_PARITY_NONE;
    UART_HTD.Init.Mode = UART_MODE_TX_RX;
    HAL_UART_Init(&UART_HTD);
        
   
    // ��������� ������ � DMA
    MB_UART->CR3 |= USART_CR3_DMAT;            // ��������� �������� UART ����� DMA
    MB_UART->CR3 |= USART_CR3_DMAR;            // ��������� ����� UART ����� DMA 
    
    // ���������/��������� ������ DMA
    DMA_RX_Stream->CR  |= DMA_SxCR_EN;         // ��������� ������ ������ ��������
    DMA_TX_Stream->CR  &= ~DMA_SxCR_EN;        // ��������� ������ ������ �����������
	
    // ��������� ���������� USART
    HAL_NVIC_EnableIRQ(MB_IRQn);
    HAL_NVIC_SetPriority(MB_IRQn, 2, 0);
    // �������� USART
    MB_UART->CR1 |= USART_CR1_UE;
}


/**************************************************************************
*   Function name: UART_GPIO_Init
*   Returns      : void
*   Parameters   : void
*   Purpose      : ��������� ������ �/�
****************************************************************************/
void UART_GPIO_Init(void)
{
    MB_GPIO_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_ITD;

    // TX, RX
	GPIO_ITD.Pin = UART_TX| UART_RX;
	GPIO_ITD.Mode = GPIO_MODE_AF_PP;
	GPIO_ITD.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_ITD.Pull = GPIO_PULLUP;
        GPIO_ITD.Alternate = UART_AF;
      HAL_GPIO_Init(UART_Port, &GPIO_ITD);
    
    // RW
  	GPIO_ITD.Pin = MB_RW_Pin;
	GPIO_ITD.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_ITD.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_ITD.Pull = GPIO_PULLDOWN;
  	HAL_GPIO_Init(MB_Port, &GPIO_ITD);
}


/**************************************************************************
*   Function name: UART_DMA_Init
*   Returns      : void
*   Parameters   : void
*   Purpose      : ��������� DMA ��� USART
****************************************************************************/
void UART_DMA_Init(void)
{
  // �������� ������������ DMA
  DMA_UART_CLK_ENABLE();
  
  DMA_HandleTypeDef DMA_HTD;
  // ��������� DMA ��� TX
  DMA_HTD.Instance = DMA_TX_Stream;
  
  DMA_HTD.Init.Channel = DMA_CHANNEL_4;
  DMA_HTD.Init.Direction = DMA_MEMORY_TO_PERIPH;
  DMA_HTD.Init.PeriphInc = DMA_PINC_DISABLE;
  DMA_HTD.Init.MemInc = DMA_MINC_ENABLE;
  DMA_HTD.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  DMA_HTD.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  DMA_HTD.Init.Mode = DMA_NORMAL;
  DMA_HTD.Init.Priority = DMA_PRIORITY_HIGH;
  DMA_HTD.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
  HAL_DMA_Init(&DMA_HTD);
  
  DMA_TX_Stream->M0AR = (uint32_t) RxTxFrame;     //����� ������ �����������
  DMA_TX_Stream->PAR = (uint32_t) &MB_UART->DR;              //����� �������� ������ �����������
  DMA_TX_Stream->CR |= DMA_SxCR_TCIE;                         //��������� ���������� �� ���������� ������
  
  // ��������� DMA ��� RX
  DMA_HTD.Instance = DMA_RX_Stream;
  
  DMA_HTD.Init.Direction = DMA_PERIPH_TO_MEMORY;
  DMA_HTD.Init.Priority = DMA_PRIORITY_VERY_HIGH;
  HAL_DMA_Init(&DMA_HTD);
  
  DMA_RX_Stream->M0AR = (uint32_t) RxTxFrame;                  //����� ������ ���������
  DMA_RX_Stream->PAR = (uint32_t) &MB_UART->DR;              //����� �������� ������ ���������
  DMA_RX_Stream->NDTR = MB_BUF_LEN;                           //������ ������ ���������
  
  // ��������� ���������� �� ��������� ��������
  HAL_NVIC_EnableIRQ(DMA_TX_IRQn);
  HAL_NVIC_SetPriority(DMA_TX_IRQn, 1, 0);
}


/**************************************************************************
*   Function name: UART_TIM_Init
*   Returns      : void
*   Parameters   : void
*   Purpose      : ��������� ������� ��� ����������� ��������� ����� MODBUS
****************************************************************************/
void UART_TIM_Init(void)
{
    // ��������� ������ ������� (0,5 �������)
	// ������������ ������� �� UART - 11 ���
	// (�����-���, 8 ��� ������, ��� ��������, ����-���)
    // ��� �������� 9600: 0.5*11/9600=0.0006c.(0.6ms)
    // TIM6 ��������� �� ���� APB1 � ����������� 60 ���
    // ������ ������� (60���/300)/120=1666��(0.6ms)
    
    // �������� ������������ �������
    TIM_UART_CLK_ENABLE();
    
    // ������������� ������
    TIM_HandleTypeDef TIM6_HTD ={0};
    TIM6_HTD.Instance = TIM6;
    TIM6_HTD.Init.Prescaler = 300-1;
    TIM6_HTD.Init.CounterMode = TIM_COUNTERMODE_UP;
    TIM6_HTD.Init.Period = 120 - 1;
    TIM6_HTD.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    TIM6_HTD.Init.RepetitionCounter = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_Base_Init(&TIM6_HTD);
    
    // ��������� ����������� �� ����������
    TIM6->DIER |= TIM_DIER_UIE;
    
    // ��������� ���������� TIM6
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
    HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 3, 0);
    
    // ������� ���� ����������
    TIM6->SR &= ~TIM_SR_UIF;
    
    // ��������� ������
    TIM6->CR1 |= TIM_CR1_CEN;
}

/**************************************************************************
*   Function name: TIM6_DAC_IRQHandler
*   Returns      : void
*   Parameters   : void
*   Purpose      : ���������� �� ������� �������� �������� ����� ��������
****************************************************************************/
void TIM6_DAC_IRQHandler(void)
{
  static uint8_t LastRecByteCnt = 0;
  static uint8_t  tmp_silence_cnt = 0;
  
  // ���� ��� ����������� �� ����������
  if((TIM6->SR & TIM_SR_UIF) != 0)
  {
    // ������� ���� ����������
    TIM6->SR &= ~TIM_SR_UIF;
    //������������ ����� ����� ������ �� ���� � ����� ���������� 30 �����
    if((HAL_GetTick() - ConfigData.ModbusTempCount) < MB_CONF_STATE_MAX_TIME)
    {
      //���� � ������� 4 ������ �� ��������� ��������� ���������� ����� ���������� � ����
      if ((HAL_GetTick() - ConfigData.ModbusTempCount) > MB_CONF_ENTER_MAX_TIME)
      {
        ConfigData.Receive_Bytes = 0;
        ConfigData.ModbusConfigState = MB_DEVICE_NOT_CONF_STATE;
        DeviceRegister.ro_region.DeviceState &= ~CONGIG_STATE_FLAG;
        //��������� ���������, ����������� �� ���������������� ���������
        HAL_GPIO_WritePin(DEBUG_LED_PORT, DEBUG_LED_PIN, GPIO_PIN_RESET);
      }
    }
    //��������� ���������� �������� ����
    uint8_t RecByteCnt = MB_BUF_LEN - DMA_RX_Stream->NDTR;    
    if (RecByteCnt)
    {          
      if (RecByteCnt == LastRecByteCnt)
      {            
        if (++tmp_silence_cnt > MB_SILENCE)
        {
          LastRecByteCnt = 0;
          tmp_silence_cnt = 0;
          
          DMA_RX_Stream->CR &= ~DMA_SxCR_EN;
          // ��������� ����� ������ ���������
          DMA_RX_Stream->M0AR = (uint32_t)RxTxFrame;  
          DMA_RX_Stream->NDTR = MB_BUF_LEN;         
          // ���������� ���� ���������� �� ��������� �������� �����(������������ ����� ���������� ������ DMA)
          DMA_TX_RX->LIFCR |= DMA_LIFCR_CTCIF1;
          // �������� ����� ��������
          DMA_RX_Stream->CR |=  DMA_SxCR_EN;
          RxTxLength = RecByteCnt;
          if (RecByteCnt == 1 && (HAL_GetTick() - ConfigData.ModbusTempCount) < MB_CONF_STATE_MAX_TIME)
          {
            if (ConfigData.Receive_Bytes == 0)
            {
              ConfigData.Receive_Bytes++;
            } else 
            {
              if ((HAL_GetTick() - ConfigData.ModbusTempCount > (DelayTimeMass[ConfigData.Receive_Bytes] - MB_DELAY_TIME_ACCURACY))&&
                  (HAL_GetTick() - ConfigData.ModbusTempCount < (DelayTimeMass[ConfigData.Receive_Bytes] + MB_DELAY_TIME_ACCURACY)))
              {
                ConfigData.Receive_Bytes++;
                if(ConfigData.Receive_Bytes == 3)
                {
                  //���������� ���� ������ ���������
                  ConfigData.ModbusConfigState = MB_DEVICE_CONF_STATE;
                  DeviceRegister.ro_region.DeviceState |= CONGIG_STATE_FLAG;
                  //�������� ���������, ����������� �� ���������������� ���������
                  HAL_GPIO_WritePin(DEBUG_LED_PORT, DEBUG_LED_PIN, GPIO_PIN_SET);
                } 
              }
            }
            ConfigData.ModbusTempCount = HAL_GetTick();
          } else
          {
            ConfigData.ModbusTempCount = HAL_GetTick();
            //���� �������� �������� �������, ���������� �������, ��� ��� �������� ���������
            if(CommCtrlParam.CtrlCounterEn)
            {
              CommCtrlParam.CtrlCounterVal = 0;
            }
            uint8_t responseLen = ModbusPacketProc (RxTxFrame, RecByteCnt);
            
            if (responseLen)
            {
              MB_SendFrame(responseLen);
              return;
            }     
          }  
          //-------------------------------------------------------------
        }  
        //=============================================================
      } else
      {
        
        tmp_silence_cnt = 0;
        
        LastRecByteCnt = RecByteCnt;
      }
    } 
  }    
}


/**************************************************************************
*   Function name: MB_IRQHandler
*   Returns      : void
*   Parameters   : void
*   Purpose      : ���������� ���������� �� �������� ���������� ����� USART
****************************************************************************/
void MB_IRQHandler(void)
{
  // ���� ���������� �� ��������� �������� �����
  if ((MB_UART->SR & USART_SR_TC) != 0)
  {
    //��������� ��������� � ������ �������� �����
    HAL_GPIO_WritePin(MODBUS_LED_PORT, MODBUS_LED_PIN, GPIO_PIN_RESET);
    
    // ���������� ���� ���������� �� ������ ����� USART
    MB_UART->SR &= ~USART_SR_TC;
    
    // ��������� ���������� �� ��������� �������� �����
    MB_UART->CR1 &= ~USART_CR1_TCIE;
    
    // ����������� ������� RS-485 �� �����
    HAL_GPIO_WritePin(MB_Port, MB_RW_Pin, MB_READ);
  }

}

/**************************************************************************
*   Function name: DMA_TX_IRQHandler
*   Returns      : void
*   Parameters   : void
*   Purpose      : ���������� ���������� �� ���������� �������� ����� � USART
****************************************************************************/
void DMA_TX_IRQHandler(void)
{
    // ���� ���������� �� ��������� �������� �����
    if ((DMA_TX_RX->LISR & DMA_TX_IF_TCIF) != 0)
    {
        // ���������� ���� ���������� �� ��������� �������� �����
        DMA_TX_RX->LIFCR |= DMA_TX_IFC_TCIF;
        
        // ���������� ���� ���������� �� ��������� �������� USART
        MB_UART->SR &= ~USART_SR_TC;
        
        // ��������� ���������� �� ��������� �������� ����� USART
        MB_UART->CR1 |= USART_CR1_TCIE; 
    }
}


/**************************************************************************
*   Function name: MB_SendFrame
*   Returns      : void
*   Parameters   : void
*   Purpose      : ����� ��������� �����
****************************************************************************/
void MB_SendFrame(uint8_t Length)
{
    // ����������� ������� RS-485 �� ��������
    HAL_GPIO_WritePin(MB_Port, MB_RW_Pin, MB_WRITE);
    //�������� ��������� � ������ �������� �����
    HAL_GPIO_WritePin(MODBUS_LED_PORT, MODBUS_LED_PIN, GPIO_PIN_SET);
    
    DMA_TX_Stream->M0AR = (uint32_t)RxTxFrame;
    DMA_TX_Stream->NDTR = Length;
    
    DMA_TX_Stream->CR |= DMA_SxCR_EN;           // ��������� ������ ������
}
