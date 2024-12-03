#ifndef IMPEDANCE_FUNC_H
#define IMPEDANCE_FUNC_H

#include "main.h"

/* ##################### -= �������� ����������� =- ##################### */

#define MIN_RESP_VAL     500
#define MAX_RESP_VAL     3500

#define DELAY_TO_START_MEAS  80
#define MEAS_START_PERIOD    3
#define MEAS_COUNT           110 //���������� ��������� �� ������

#define  CURR_RANGE_1_LO     10
#define  CURR_RANGE_1_HI     50
#define  CURR_RANGE_2_LO     20
#define  CURR_RANGE_2_HI     100
#define  CURR_RANGE_3_LO     40
#define  CURR_RANGE_3_HI     200
#define  CURR_RANGE_4_LO     80
#define  CURR_RANGE_4_HI     400

/* ##################### -= ����������� ���������� �������� ��� ��������� =- ##################### */
#define  CHECK_PERIOD_COUNT 50
#define  MEASURE_PERIOD_COUNT  400

typedef enum{
  GAIN_1 = 1,
  GAIN_2 = 2,
  GAIN_4 = 4,
  GAIN_8 = 8,
  GAIN_16 = 16,
  GAIN_32 = 32,
  GAIN_64 = 64,
  GAIN_128 = 128
}PGA_GAIN; //����� �������� ��� AD8231

typedef enum{
  RANGE_1 = 1,
  RANGE_2 = 2,
  RANGE_3 = 3,
  RANGE_4 = 4
}CURR_RANGE; // ������� ��������� ����

typedef enum{
  RANGE_SEL_STEP = 0,//��� ������ �������
  CHECK_STEP = 1,//�������� ���������� �������
  MEASURE_STEP = 2,//���� ���
  DONE = 3
}IMP_STEP; // ���� ��������� �������������

typedef enum{
  CURR_LOW = 0,
  CURR_HIGH = 1
}CURR_POS;

typedef struct{
  CURR_POS Val_Pos;//������� ��������� ����
  CURR_RANGE Range;//������� ������ ����
}IMP_CURR_STRUCT;

typedef struct{
  uint64_t Samples_Sum_HI;//����� ��������� ������� ��� �������� �������� ����
  uint32_t Samples_Count_HI;//���������� ������� � ����� HI
  uint64_t Samples_Sum_LO;//����� ��������� ������� ��� ������� �������� ����
  uint32_t Samples_Count_LO;//���������� ������� � ����� LO
  uint16_t Period_Count;//���������� ���������� ��������
  uint16_t Measure_Count;//���������� ���������� � �������
  uint16_t DelayToStart;//�������� ����� ������� ���������
  uint16_t V_Response;//������ ����������
  uint16_t Resistance;//��������� ���������
  PGA_GAIN Gain;//������� �������� ��������
}IMP_MEASURE_STRUCT;

typedef struct{
  IMP_CURR_STRUCT Current;
  IMP_MEASURE_STRUCT Measure;
  IMP_STEP Step;
}IMPEDANCE_DATA;

/* --------------------- -= ��������� ������� =- --------------------- */

void Impedance_Periph_Init(void);
void Impedance_Measure_STOP(void);

#endif /* IMPEDANCE_FUNC_H */