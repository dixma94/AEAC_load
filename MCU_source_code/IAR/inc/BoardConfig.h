#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

#include "main.h"

/* ################## -= Определения для IWDG =- ################## */
#define IWDG_RELOAD             128// 125гЦ / 2500 = 0.05 Гц = 20 с(Максимальное значение регистра 4096)

/* ################## -= Определения для светодиодов =- ################## */
#define DEBUG_LED_PORT          GPIOC
#define DEBUG_LED_PIN           GPIO_PIN_12

#define DEBUG_LED_RCC_ENABLE  __HAL_RCC_GPIOC_CLK_ENABLE

#define MODBUS_LED_PORT          GPIOD
#define MODBUS_LED_PIN           GPIO_PIN_4

#define MODBUS_LED_RCC_ENABLE  __HAL_RCC_GPIOD_CLK_ENABLE

/* ################## -= Определния для управления ключами =- ################## */
#define SW_PORT               GPIOD
#define SW_PIN                GPIO_PIN_8

#define SW_PORT_RCC_ENABLE  __HAL_RCC_GPIOD_CLK_ENABLE

/* ################## -= Определния для управления ключами =- ################## */
#define FAN_PORT               GPIOE
#define FAN_PIN                GPIO_PIN_6

#define FAN_PORT_RCC_ENABLE  __HAL_RCC_GPIOE_CLK_ENABLE

#define TIM_FAN_CLK_ENABLE   __HAL_RCC_TIM9_CLK_ENABLE


/* ##################### -= Определения для USART_Driver =- ##################### */
   
#define UART_Port          GPIOB
#define UART_TX            GPIO_PIN_10
#define UART_RX            GPIO_PIN_11
#define UART_AF	           GPIO_AF7_USART3
#define MB_GPIO_CLK_ENABLE  __HAL_RCC_GPIOB_CLK_ENABLE

#define MB_UART            USART3
#define UART_CLK_ENABLE    __HAL_RCC_USART3_CLK_ENABLE
#define MB_IRQn		        USART3_IRQn
#define MB_IRQHandler	    USART3_IRQHandler

#define MB_Port             GPIOB
#define MB_RW_Pin           GPIO_PIN_9
#define MB_READ             GPIO_PIN_RESET
#define MB_WRITE            GPIO_PIN_SET

#define DMA_TX_RX           DMA1
#define DMA_UART_CLK_ENABLE __HAL_RCC_DMA1_CLK_ENABLE
#define DMA_TX_IF_TCIF      DMA_LISR_TCIF3      // Interrupt flag for DMA1_Stream3
#define DMA_TX_IFC_TCIF     DMA_LIFCR_CTCIF3    // Clear flag for interrupt DMA1_Stream3

#define DMA_TX_Stream       DMA1_Stream3   
#define DMA_RX_Stream       DMA1_Stream1  
#define DMA_TX_IRQn         DMA1_Stream3_IRQn
#define DMA_TX_IRQHandler   DMA1_Stream3_IRQHandler

#define TIM_UART_CLK_ENABLE __HAL_RCC_TIM6_CLK_ENABLE

/* ##################### -= Определения для Mesure_FUNC =- ##################### */
#define MEASURE_PORT             GPIOA
#define MEASURE_V_PIN            GPIO_PIN_2
#define MEASURE_I_PIN            GPIO_PIN_0
#define MEASURE_GPIO_CLK_ENABLE  __HAL_RCC_GPIOA_CLK_ENABLE

#define MEASURE_ADC_CLK_ENABLE   __HAL_RCC_ADC1_CLK_ENABLE

#define MEASURE_ADC               ADC1

#define MEASURE_V_CHAN          ADC_CHANNEL_2
#define MEASURE_I_CHAN          ADC_CHANNEL_0

#define DMA_ADC                 DMA2
#define DMA_ADC_CLK_ENABLE      __HAL_RCC_DMA2_CLK_ENABLE
#define DMA_ADC_Stream          DMA2_Stream0
#define DMA_ADC_IRQn            DMA2_Stream0_IRQn
#define DMA_ADC_IRQHandler      DMA2_Stream0_IRQHandler
#define DMA_ADC_IF_TCIF         DMA_LISR_TCIF0      // Interrupt flag for DMA2_Stream0
#define DMA_ADC_IFC_TCIF        DMA_LIFCR_CTCIF0    // Clear flag for interrupt DMA2_Stream0
#define DMA_ADC_IFC_HTIF        DMA_LIFCR_CHTIF0    // Clear flag for interrupt DMA2_Stream0

/* ##################### -= Определения для CurrentSet_FUNC =- ##################### */
#define DAC_PORT_PIN_RCC_ENABLE    __HAL_RCC_GPIOA_CLK_ENABLE
#define DAC_RCC_ENABLE             __HAL_RCC_DAC_CLK_ENABLE
#define DAC_PIN_PORT               GPIOA
#define DAC_PIN                    GPIO_PIN_5

#define DAC_CHAN                   DAC_CHANNEL_2

/* ##################### -= Определения для измерителя импеданса =- ##################### */
#define GAIN_PORT_PIN_RCC_ENABLE    __HAL_RCC_GPIOA_CLK_ENABLE

#define PGA_GAIN_PORT               GPIOA
#define PGA_GAIN_PIN_BIT_0          GPIO_PIN_11
#define PGA_GAIN_PIN_BIT_1          GPIO_PIN_10
#define PGA_GAIN_PIN_BIT_2          GPIO_PIN_9

#define IMP_ADC_PORT_PIN_RCC_ENABLE    __HAL_RCC_GPIOA_CLK_ENABLE

#define IMP_ADC_PORT                GPIOA
#define IMP_ADC_PIN                 GPIO_PIN_1

#define IMPEDANCE_TIM_GEN_CLK_ENABLE    __HAL_RCC_TIM2_CLK_ENABLE
#define IMPEDANCE_TIM_MEAS_CLK_ENABLE    __HAL_RCC_TIM4_CLK_ENABLE
#define IMPEDANCE_TIM_GEN_CLK_DISABLE   __HAL_RCC_TIM2_CLK_DISABLE
#define IMPEDANCE_TIM_MEAS_CLK_DISABLE   __HAL_RCC_TIM4_CLK_DISABLE

#define IMPEDANCE_ADC               ADC2

#define IMPEDANCE_ADC_CLK_ENABLE    __HAL_RCC_ADC2_CLK_ENABLE
#define IMPEDANCE_V_CHAN            ADC_CHANNEL_1

/* ##################### -= Определения для измерения импеданса =- ##################### */
#define MODBUS_TIMEOUT_TIM_CLK_ENABLE    __HAL_RCC_TIM5_CLK_ENABLE


/* ################## -= Прототипы функций =- ################## */

void BoardConfig(void);
void SW_Pin_Rem(BOOL Value);

#endif /* BOARD_CONFIG_H */