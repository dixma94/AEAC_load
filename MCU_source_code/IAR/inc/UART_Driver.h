#ifndef USART_DRIVER_H
#define USART_DRIVER_H

#include "main.h"

#define UART_BRATE                     9600

#define MB_BUF_LEN                      256


#define MB_TIMEOUT                      1000
#define MB_ERROR_COUNT                  10
#define MB_BROADCAST_TIMEOUT            667
#define MB_SILENCE                      7

/* --------------------- -= Флаги статуса ModBus =- --------------------- */
   
#define MB_TX_COMPLETE                  0x01            // Кадр данных был успешно передан
#define MB_RX_COMPLETE                  0x02            // Ответ на кадр был успешно принят
#define MB_CRC_ERR                      0x04            // Ошибка CRC при приеме кадра
#define MB_3_5_SILENCE                  0x08            // Тишина на линии 3.5t или больше
#define MB_1_5_SILENCE                  0x10            // Тишина на линии более 1.5t
#define MB_ERROR                        0x20            // Нет связи по шине
#define MB_BROADCAST                    0x40            // Текущая функция является широковещательной
#define ANY_FLAG                        0x80


/* --------------------- -= Прототипы функций =- --------------------- */
   
void UART_Init(void);
void MB_SendFrame(uint8_t Length);

#endif /* USART_DRIVER_H */

