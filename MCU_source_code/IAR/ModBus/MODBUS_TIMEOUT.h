#ifndef MODBUS_TIMEOUT_H
#define MODBUS_TIMEOUT_H

#include "main.h"

#define LOST_COMM_TIMEOUT    200     // 200 * 0.1 секунд (20 сек)

// Структура содержащая флаг управления и счетный регистр таймера 
// структура применяется для отсчета времени после потери связи
typedef struct COMM_CONTROL_PARAM{
    BOOL CtrlCounterEn;              // включить счетчик
    uint8_t CtrlCounterVal;         // значение счетчика
}COMM_CONTROL_PARAM;

/* --------------------- -= Прототипы функций =- --------------------- */

void MODBUS_TIMEOUT_Init(void);
void MODBUS_TIMEOUT_En(BOOL Enable);

#endif /* MODBUS_TIMEOUT_H */
