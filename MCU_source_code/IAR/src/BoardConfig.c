//***************************************************************************
//  File........: BoardConfig.c
//  Author(s)...: KotyshkovEJ
//  Target(s)...: stm32f207
//  Compiler....: IAR
//  Description.: Конфигурация периферии
//  Date........: 30.11.2020
//***************************************************************************

#include "BoardConfig.h"
#include "UART_Driver.h"
#include "Measure_FUNC.h"
#include "CurrentSet_FUNC.h"
#include "SaveReadConfigurationToFlashMCU.h"
#include "FAN_FUNC.h"
#include "MODBUS_TIMEOUT.h"

extern DEVICE_REGISTER DeviceRegister;
extern CONFIG_DATA ConfigData;

//Прототипы функций
void DEGUG_LED_GPIO_Init(void);
void MODBUS_LED_GPIO_Init(void);
void SW_GPIO_Init(void);

/**************************************************************************
*   Function name: BoardConfig
*   Returns      : void
*   Parameters   : void
*   Purpose      : Функция вызывается в начале функции main для инициализации некоторой периферии 
****************************************************************************/
void BoardConfig(void)
{
  //Инициализация независимого WatchDog
//  IWDG_Init(IWDG_PRESCALER_256 ,IWDG_RELOAD);
  ReadConfiguratioStruct();
  ConfigData.ConfigRegion = (DEVICE_CONFIG_REGION*)EEPROM_START_ADDRESS;
  DEGUG_LED_GPIO_Init();
  MODBUS_LED_GPIO_Init();
  SW_GPIO_Init();
  UART_Init();
  MODBUS_TIMEOUT_Init();
  Measure_Init();
  FAN_Init();
  CurrentSet_Init();
}

/**************************************************************************
*   Function name: SW_Pin_Rem
*   Returns      : void
*   Parameters   : void
*   Purpose      : Функция управления ключами
****************************************************************************/
void SW_Pin_Rem(BOOL Value)
{
  HAL_GPIO_WritePin(SW_PORT, SW_PIN, (GPIO_PinState)Value);
}

/**************************************************************************
*   Function name: DEGUG_LED_GPIO_Init
*   Returns      : void
*   Parameters   : void
*   Purpose      : Инициализация пина управления светодиодом отладки/конфигурации
****************************************************************************/
void DEGUG_LED_GPIO_Init(void)
{
  DEBUG_LED_RCC_ENABLE();
  
  GPIO_InitTypeDef GPIO_ITD;
    GPIO_ITD.Pin = DEBUG_LED_PIN;
    GPIO_ITD.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_ITD.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_ITD.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DEBUG_LED_PORT, &GPIO_ITD);
}

/**************************************************************************
*   Function name: MODBUS_LED_GPIO_Init
*   Returns      : void
*   Parameters   : void
*   Purpose      : Инициализация пина управления светодиодом передачи по MODBUS
****************************************************************************/
void MODBUS_LED_GPIO_Init(void)
{
  MODBUS_LED_RCC_ENABLE();
  
  GPIO_InitTypeDef GPIO_ITD;
    GPIO_ITD.Pin = MODBUS_LED_PIN;
    GPIO_ITD.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_ITD.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_ITD.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MODBUS_LED_PORT, &GPIO_ITD);
}

/**************************************************************************
*   Function name: MODBUS_LED_GPIO_Init
*   Returns      : void
*   Parameters   : void
*   Purpose      : Инициализация пина управления ключами
****************************************************************************/
void SW_GPIO_Init(void)
{
  SW_PORT_RCC_ENABLE();
  
  GPIO_InitTypeDef GPIO_ITD;
    GPIO_ITD.Pin = SW_PIN;
    GPIO_ITD.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_ITD.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_ITD.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SW_PORT, &GPIO_ITD);
}

