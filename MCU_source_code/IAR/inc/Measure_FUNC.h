#ifndef MEASURE_FUNC_H
#define MEASURE_FUNC_H

#include "main.h"

/* --------------------- -= Описания структур =- --------------------- */

typedef struct{
  uint64_t Voltage_Squares_Sum; //Сумма квадратов напряжений
  uint64_t Current_Squares_Sum; //Сумма квадратов токов
  uint16_t Point_cnt; //Количечство суммирований
}MEASURE_FILT;

/* --------------------- -= Прототипы функций =- --------------------- */
   
void Measure_Init(void);
void CalibMakeCoeff (double * dest_mass, uint16_t * CalibrPoint, uint16_t * Point);
uint16_t GetRezult (double * coeff, uint16_t value);

#endif /* MEASURE_FUNC_H */
