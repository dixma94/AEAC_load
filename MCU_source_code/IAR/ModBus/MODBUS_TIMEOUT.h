#ifndef MODBUS_TIMEOUT_H
#define MODBUS_TIMEOUT_H

#include "main.h"

#define LOST_COMM_TIMEOUT    200     // 200 * 0.1 ������ (20 ���)

// ��������� ���������� ���� ���������� � ������� ������� ������� 
// ��������� ����������� ��� ������� ������� ����� ������ �����
typedef struct COMM_CONTROL_PARAM{
    BOOL CtrlCounterEn;              // �������� �������
    uint8_t CtrlCounterVal;         // �������� ��������
}COMM_CONTROL_PARAM;

/* --------------------- -= ��������� ������� =- --------------------- */

void MODBUS_TIMEOUT_Init(void);
void MODBUS_TIMEOUT_En(BOOL Enable);

#endif /* MODBUS_TIMEOUT_H */
