#ifndef MEASURE_FUNC_H
#define MEASURE_FUNC_H

#include "main.h"

/* --------------------- -= �������� �������� =- --------------------- */

typedef struct{
  uint64_t Voltage_Squares_Sum; //����� ��������� ����������
  uint64_t Current_Squares_Sum; //����� ��������� �����
  uint16_t Point_cnt; //����������� ������������
}MEASURE_FILT;

/* --------------------- -= ��������� ������� =- --------------------- */
   
void Measure_Init(void);
void CalibMakeCoeff (double * dest_mass, uint16_t * CalibrPoint, uint16_t * Point);
uint16_t GetRezult (double * coeff, uint16_t value);

#endif /* MEASURE_FUNC_H */
