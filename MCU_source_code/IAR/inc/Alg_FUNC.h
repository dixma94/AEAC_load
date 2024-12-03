#ifndef ALG_FUNC_H
#define ALG_FUNC_H

#include "main.h"

/* ##################### -= �������� ����������� ��������� =- ##################### */


#define HW_VER_20V      20// 20 ��������� ������
#define HW_VER_60V      21// 60 ��������� ������
#define HW_VER_35V      23// 35 ��������� ������ 45A

#define FAN_STOP_DELAY       60000//�������� � �������������
#define FAN_STARTUP_DELAY    30000 //������ ������������ ����� ���������


/* ##################### -= �������� ����� ������ =- ##################### */

typedef enum{
  LOAD_OFF = 0,
  LOAD_ON = 1,
  LOAD_IMP_MEASURE = 5
}LOAD_MODE;



typedef enum{
  FAN_AUTO = 0,
  FAN_MANUAL = 1,
}FAN_MODE;


typedef struct{
  LOAD_MODE  Load_Mode;
  FAN_MODE   FAN_Mode;
  BOOL       Refrash_Flag;
  uint8_t    Shadow_FANSpd;
  uint64_t   LOAD_StopTime;
  uint64_t   LOAD_FAN_StartupDelay;
}LOAD_MAIN;

typedef struct{
  uint16_t  Max_Current;
  uint16_t  Max_Voltage;
  uint32_t  Max_Power;
  uint16_t  FanCoeff;
}LOAD_VERSION_LIMITS;


/* --------------------- -= ��������� ������� =- --------------------- */

void MODBUS_DeviceControl_Proc(void);

#endif /* ALG_FUNC_H */
