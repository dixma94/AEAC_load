//***************************************************************************
//  File........: USART_Driver.c
//  Author(s)...: KotyshkovEJ
//  Target(s)...: stm32f207
//  Compiler....: IAR
//  Description.: Инициализация переферии (GPIO, USART, DMA, TIMER)
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

const uint16_t DelayTimeMass[3] = {0, 150, 400}; // Массив задержек, для перехода в режим конфигурации

//Прототипы функций
void UART_GPIO_Init(void);
void UART_DMA_Init(void);
void UART_TIM_Init(void);



/**************************************************************************
*   Function name: UART_Init
*   Returns      : void
*   Parameters   : void
*   Purpose      : Инициализация всей периферии
****************************************************************************/
void UART_Init(void)
{
    // Настройка портов в/в
    UART_GPIO_Init();
    
    // Настройка DMA
    UART_DMA_Init();
    
    // Настройка таймера
    UART_TIM_Init();
	
    // Включаем тактирование
    UART_CLK_ENABLE();
    
    // Конфигурируем UART
    UART_HandleTypeDef UART_HTD;
    UART_HTD.Instance = MB_UART;
    
    UART_HTD.Init.BaudRate = UART_BRATE;
    UART_HTD.Init.WordLength = UART_WORDLENGTH_8B;
    UART_HTD.Init.StopBits = UART_STOPBITS_1;
    UART_HTD.Init.Parity = UART_PARITY_NONE;
    UART_HTD.Init.Mode = UART_MODE_TX_RX;
    HAL_UART_Init(&UART_HTD);
        
   
    // Разрешаем работу с DMA
    MB_UART->CR3 |= USART_CR3_DMAT;            // разрешить передачу UART через DMA
    MB_UART->CR3 |= USART_CR3_DMAR;            // разрешить прием UART через DMA 
    
    // Разрешаем/запрещаем работу DMA
    DMA_RX_Stream->CR  |= DMA_SxCR_EN;         // разрешить работу канала приёмника
    DMA_TX_Stream->CR  &= ~DMA_SxCR_EN;        // запретить работу канала передатчика
	
    // Разрешаем прерывания USART
    HAL_NVIC_EnableIRQ(MB_IRQn);
    HAL_NVIC_SetPriority(MB_IRQn, 2, 0);
    // Включаем USART
    MB_UART->CR1 |= USART_CR1_UE;
}


/**************************************************************************
*   Function name: UART_GPIO_Init
*   Returns      : void
*   Parameters   : void
*   Purpose      : Настройка портов в/в
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
*   Purpose      : Настройка DMA для USART
****************************************************************************/
void UART_DMA_Init(void)
{
  // Включаем тактирование DMA
  DMA_UART_CLK_ENABLE();
  
  DMA_HandleTypeDef DMA_HTD;
  // Настройка DMA для TX
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
  
  DMA_TX_Stream->M0AR = (uint32_t) RxTxFrame;     //адрес буфера передатчика
  DMA_TX_Stream->PAR = (uint32_t) &MB_UART->DR;              //адрес регистра данных передатчика
  DMA_TX_Stream->CR |= DMA_SxCR_TCIE;                         //разрешить прерывание по завершении обмена
  
  // Настройка DMA для RX
  DMA_HTD.Instance = DMA_RX_Stream;
  
  DMA_HTD.Init.Direction = DMA_PERIPH_TO_MEMORY;
  DMA_HTD.Init.Priority = DMA_PRIORITY_VERY_HIGH;
  HAL_DMA_Init(&DMA_HTD);
  
  DMA_RX_Stream->M0AR = (uint32_t) RxTxFrame;                  //адрес буфера приемника
  DMA_RX_Stream->PAR = (uint32_t) &MB_UART->DR;              //адрес регистра данных приемника
  DMA_RX_Stream->NDTR = MB_BUF_LEN;                           //размер буфера приемника
  
  // Разрешаем прерывание по окончанию передачи
  HAL_NVIC_EnableIRQ(DMA_TX_IRQn);
  HAL_NVIC_SetPriority(DMA_TX_IRQn, 1, 0);
}


/**************************************************************************
*   Function name: UART_TIM_Init
*   Returns      : void
*   Parameters   : void
*   Purpose      : Натсройка таймера для определения окончания кадра MODBUS
****************************************************************************/
void UART_TIM_Init(void)
{
    // Запускаем таймер отсчета (0,5 символа)
	// Длительность символа по UART - 11 бит
	// (старт-бит, 8 бит данных, бит четности, стоп-бит)
    // Для скорости 9600: 0.5*11/9600=0.0006c.(0.6ms)
    // TIM6 находится на шине APB1 и тактируется 60 МГц
    // Расчет таймера (60МГц/300)/120=1666Гц(0.6ms)
    
    // Включаем тактирование таймера
    TIM_UART_CLK_ENABLE();
    
    // Конфигурируем таймер
    TIM_HandleTypeDef TIM6_HTD ={0};
    TIM6_HTD.Instance = TIM6;
    TIM6_HTD.Init.Prescaler = 300-1;
    TIM6_HTD.Init.CounterMode = TIM_COUNTERMODE_UP;
    TIM6_HTD.Init.Period = 120 - 1;
    TIM6_HTD.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    TIM6_HTD.Init.RepetitionCounter = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_Base_Init(&TIM6_HTD);
    
    // Разрешаем прерываение по обновлению
    TIM6->DIER |= TIM_DIER_UIE;
    
    // Разрешаем прерывание TIM6
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
    HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 3, 0);
    
    // Очищаем флаг прерывания
    TIM6->SR &= ~TIM_SR_UIF;
    
    // Запускаем таймер
    TIM6->CR1 |= TIM_CR1_CEN;
}

/**************************************************************************
*   Function name: TIM6_DAC_IRQHandler
*   Returns      : void
*   Parameters   : void
*   Purpose      : Прерывания от таймера подсчета таймаута между пакетами
****************************************************************************/
void TIM6_DAC_IRQHandler(void)
{
  static uint8_t LastRecByteCnt = 0;
  static uint8_t  tmp_silence_cnt = 0;
  
  // Если это прерываение по обновлению
  if((TIM6->SR & TIM_SR_UIF) != 0)
  {
    // Очищаем флаг прерывания
    TIM6->SR &= ~TIM_SR_UIF;
    //Максимальное время после старта на вход в режим калибровки 30 минут
    if((HAL_GetTick() - ConfigData.ModbusTempCount) < MB_CONF_STATE_MAX_TIME)
    {
      //Если в течении 4 секунд не поступало сообщений сбрасываем режим калибровки в ноль
      if ((HAL_GetTick() - ConfigData.ModbusTempCount) > MB_CONF_ENTER_MAX_TIME)
      {
        ConfigData.Receive_Bytes = 0;
        ConfigData.ModbusConfigState = MB_DEVICE_NOT_CONF_STATE;
        DeviceRegister.ro_region.DeviceState &= ~CONGIG_STATE_FLAG;
        //Выключаем светодиод, указывающий на конфигурационное состояние
        HAL_GPIO_WritePin(DEBUG_LED_PORT, DEBUG_LED_PIN, GPIO_PIN_RESET);
      }
    }
    //Вычисляем количество принятых байт
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
          // обновляем адрес буфера приемника
          DMA_RX_Stream->M0AR = (uint32_t)RxTxFrame;  
          DMA_RX_Stream->NDTR = MB_BUF_LEN;         
          // Сбрасываем флаг прерывания по окончании передачи кадра(выставляется после отключения потока DMA)
          DMA_TX_RX->LIFCR |= DMA_LIFCR_CTCIF1;
          // Включаем канал приёмника
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
                  //Выставляем флаг режима настройки
                  ConfigData.ModbusConfigState = MB_DEVICE_CONF_STATE;
                  DeviceRegister.ro_region.DeviceState |= CONGIG_STATE_FLAG;
                  //Включаем светодиод, указывающий на конфигурационное состояние
                  HAL_GPIO_WritePin(DEBUG_LED_PORT, DEBUG_LED_PIN, GPIO_PIN_SET);
                } 
              }
            }
            ConfigData.ModbusTempCount = HAL_GetTick();
          } else
          {
            ConfigData.ModbusTempCount = HAL_GetTick();
            //Если контроль таймаута включен, сбрасываем счётчик, так как получили сообщение
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
*   Purpose      : Обработчик прерываний по передаче последнего байта USART
****************************************************************************/
void MB_IRQHandler(void)
{
  // Если прерывание по окончании передачи байта
  if ((MB_UART->SR & USART_SR_TC) != 0)
  {
    //Выключаем светодиод в начале отпарвки кадра
    HAL_GPIO_WritePin(MODBUS_LED_PORT, MODBUS_LED_PIN, GPIO_PIN_RESET);
    
    // Сбрасываем флаг прерывания по приему байта USART
    MB_UART->SR &= ~USART_SR_TC;
    
    // Запрещаем прерывания по окончании передачи байта
    MB_UART->CR1 &= ~USART_CR1_TCIE;
    
    // Переключаем драйвер RS-485 на прием
    HAL_GPIO_WritePin(MB_Port, MB_RW_Pin, MB_READ);
  }

}

/**************************************************************************
*   Function name: DMA_TX_IRQHandler
*   Returns      : void
*   Parameters   : void
*   Purpose      : Обработчик прерывания по завершению передачи кадра в USART
****************************************************************************/
void DMA_TX_IRQHandler(void)
{
    // Если прерывание по окончании передачи кадра
    if ((DMA_TX_RX->LISR & DMA_TX_IF_TCIF) != 0)
    {
        // Сбрасываем флаг прерывания по окончании передачи кадра
        DMA_TX_RX->LIFCR |= DMA_TX_IFC_TCIF;
        
        // Сбрасываем флаг прерывания по окончанию передачи USART
        MB_UART->SR &= ~USART_SR_TC;
        
        // Разрешаем прерывания по окончании передачи байта USART
        MB_UART->CR1 |= USART_CR1_TCIE; 
    }
}


/**************************************************************************
*   Function name: MB_SendFrame
*   Returns      : void
*   Parameters   : void
*   Purpose      : Старт прередачи кадра
****************************************************************************/
void MB_SendFrame(uint8_t Length)
{
    // Переключаем драйвер RS-485 на передачу
    HAL_GPIO_WritePin(MB_Port, MB_RW_Pin, MB_WRITE);
    //Включаем светодиод в начале отпарвки кадра
    HAL_GPIO_WritePin(MODBUS_LED_PORT, MODBUS_LED_PIN, GPIO_PIN_SET);
    
    DMA_TX_Stream->M0AR = (uint32_t)RxTxFrame;
    DMA_TX_Stream->NDTR = Length;
    
    DMA_TX_Stream->CR |= DMA_SxCR_EN;           // разрешить работу канала
}
