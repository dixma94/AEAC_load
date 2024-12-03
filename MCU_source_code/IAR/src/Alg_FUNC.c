//***************************************************************************
//  File........: Alg_FUNC.c
//  Author(s)...: KotyshkovEJ
//  Target(s)...: stm32f207
//  Compiler....: IAR
//  Description.: Основные функции работы нагрузки
//  Date........: 13.04.2021
//***************************************************************************

#include "Alg_FUNC.h"
#include "BoardConfig.h"
#include "CurrentSet_FUNC.h"
#include "FAN_FUNC.h"
#include "Impedance_FUNC.h"
#include "MODBUS_TIMEOUT.h"
#include "Measure_FUNC.h"
#include "MODBUS.h"

LOAD_MAIN Main_Struct = {//Начальные состояния
                          LOAD_OFF, //Load_Mode;
                          FAN_AUTO, //FAN_Mode;
                          FALSE,    //Refrash_Flag
                          0,        //Shadow_FANSpd
                          0         //LOAD_StopTime
                                   };



extern DEVICE_REGISTER DeviceRegister;
extern uint8_t fCalibrated;
extern CONFIG_DATA ConfigData;
LOAD_VERSION_LIMITS Load_limits;

/**************************************************************************
*   Function name: TIM3_IRQHandler
*   Returns      : void
*   Parameters   : void
*   Purpose      : Прерывание в котором обрабатывается основной алгоритм работы.
****************************************************************************/
void TIM3_IRQHandler()
{
  if((TIM3->SR & TIM_SR_UIF) != 0)
  {
    // Очищаем флаг прерывания
    TIM3->SR &= ~TIM_SR_UIF;
   
    //Выключаем стартовый запуск вентиляторов если истекло время
    if(Main_Struct.LOAD_FAN_StartupDelay)
    {
      Main_Struct.LOAD_FAN_StartupDelay--;
      if(Main_Struct.LOAD_FAN_StartupDelay==0)
      {
         FAN_Stop();
      }        
    }
    
    //Если ПРЕВЫШАЕМ мощность ИЛИ максимальное напряжение отключаем нагрузку
    if(DeviceRegister.rw_region.DeviceCurrentSet * DeviceRegister.ro_region.DeviceVoltageRMS >= Load_limits.Max_Power || DeviceRegister.ro_region.DeviceVoltageRMS>= Load_limits.Max_Voltage)
    {
        if (fCalibrated && (ConfigData.ModbusConfigState != MB_DEVICE_CONF_STATE))
        {
         Main_Struct.Refrash_Flag = TRUE;
         Main_Struct.Load_Mode = LOAD_OFF;
         //Выставляем флаг ошибки
         DeviceRegister.ro_region.DeviceState |= HIGH_VOLTAGE_OR_VOLTAGE_FLAG;
        }
    }
    
    //Если выставлен флаг обнолвения, пишем новые значения в регистры
    if(Main_Struct.Refrash_Flag)
    {
      switch(Main_Struct.Load_Mode)
      {
      //Включение нормального режима электронной нагрузки
      case LOAD_ON:
        //если не в режиме калибровки
        if (fCalibrated && (ConfigData.ModbusConfigState != MB_DEVICE_CONF_STATE))
        {
           //Если переданный ток больше максимального тока нагрузки, то зануляем значение
           if(DeviceRegister.rw_region.DeviceCurrentSet > Load_limits.Max_Current)
           {
              DeviceRegister.rw_region.DeviceCurrentSet = 0;
           }
          
          //Если НЕ ПРЕВЫШАЕМ мощность И максимальное напряжение включаем вентиляторы и ток
          if(DeviceRegister.rw_region.DeviceCurrentSet * DeviceRegister.ro_region.DeviceVoltageRMS <= Load_limits.Max_Power && DeviceRegister.ro_region.DeviceVoltageRMS<=Load_limits.Max_Voltage)
          {
            //сбрасываем флаг ошибки
            DeviceRegister.ro_region.DeviceState &= ~HIGH_VOLTAGE_OR_VOLTAGE_FLAG;
            //Рассчитываем скорость вентилятора
            Main_Struct.Shadow_FANSpd = FAN_Spd_Calc(DeviceRegister.rw_region.DeviceCurrentSet);
            FAN_PWM_Set(Main_Struct.Shadow_FANSpd);
            SetCurrent(DeviceRegister.rw_region.DeviceCurrentSet);
            DeviceRegister.ro_region.DeviceState &= ~FAN_MANUAL_FLAG;    
          }
          
        }
        else
        {
            //сбрасываем флаг ошибки
            DeviceRegister.ro_region.DeviceState &= ~HIGH_VOLTAGE_OR_VOLTAGE_FLAG;
            //Рассчитываем скорость вентилятора
            Main_Struct.Shadow_FANSpd = FAN_Spd_Calc(DeviceRegister.rw_region.DeviceCurrentSet);
            FAN_PWM_Set(Main_Struct.Shadow_FANSpd);
            SetCurrent(DeviceRegister.rw_region.DeviceCurrentSet);
            DeviceRegister.ro_region.DeviceState &= ~FAN_MANUAL_FLAG;
        }
        
        break;
      case LOAD_IMP_MEASURE:
        Main_Struct.Shadow_FANSpd = FAN_Spd_Calc(500);
        FAN_PWM_Set(Main_Struct.Shadow_FANSpd);
        //Начинаем измерение импеданса(в конце измерения 
        Impedance_Periph_Init();
        break;
      //Во всех остальных случаях отключаем нагрузку
      default:
        SetCurrent(0);
        if(Main_Struct.FAN_Mode != FAN_MANUAL)
        {
          Main_Struct.LOAD_StopTime = HAL_GetTick();
        }
        Main_Struct.Load_Mode = LOAD_OFF;
        break;
      }
      //Если выбран мануальный режим работы вентиляторов
      if(Main_Struct.FAN_Mode == FAN_MANUAL)
      {
        if(DeviceRegister.rw_region.FanPwmSet >= Main_Struct.Shadow_FANSpd)
        {
          DeviceRegister.ro_region.DeviceState |= FAN_MANUAL_FLAG;
          FAN_PWM_Set(DeviceRegister.rw_region.FanPwmSet);
        }
      }
      Main_Struct.Refrash_Flag = FALSE;
    }
    
       
    
    //Проверяем время остановки, если не равно нулю, значит выключаем вентиляторы с задержкой
    if(Main_Struct.LOAD_StopTime && Main_Struct.Load_Mode == LOAD_OFF)
    {
      if(HAL_GetTick() -Main_Struct.LOAD_StopTime  >= FAN_STOP_DELAY)
      {
        //Останавливаем вентилятор
        FAN_Stop();
        //Обнуляем, чтобы не попасть снова
        Main_Struct.LOAD_StopTime = 0;
      }
    }
  }
}

/**************************************************************************
*   Function name: MODBUS_DeviceControl_Proc
*   Returns      : void
*   Parameters   : void
*   Purpose      : Обработка регистра управления нагрузкой
****************************************************************************/
void MODBUS_DeviceControl_Proc()
{
  // Исполняем команду        
  if(DeviceRegister.ro_region.DeviceControl & LOAD_STATE)
  {
    //Включаем нагрузку
    Main_Struct.Load_Mode = LOAD_ON;
    //Выставляем флаг обновления
    Main_Struct.Refrash_Flag = TRUE;
    //Включаем отслеживание таймаута MODBUS
    MODBUS_TIMEOUT_En(TRUE);
  }
  else 
  {
    //Задаём скорость вентилятора равной нулю
    Main_Struct.Shadow_FANSpd = 0;
    //Выключаем нагрузку
    Main_Struct.Load_Mode = LOAD_OFF;
    //Выставляем флаг обновления
    Main_Struct.Refrash_Flag = TRUE;
    //Выключаем отслеживание таймаута MODBUS
    MODBUS_TIMEOUT_En(FALSE);
  }
  if (DeviceRegister.ro_region.DeviceControl & FAN_STATE)
  {
    //Мануальный режим работы вентилятора
    Main_Struct.FAN_Mode = FAN_MANUAL;
    //Выставляем флаг обновления
    Main_Struct.Refrash_Flag = TRUE;
  }
  else
  {
    //Автоматический режим работы вентилятора
    Main_Struct.FAN_Mode = FAN_AUTO;
    //Выставляем флаг обновления
    Main_Struct.Refrash_Flag = TRUE;
  }
  if (DeviceRegister.ro_region.DeviceControl & IMPEDANCE_MEASURE)
  {
    //Режим измерение имеданса
    Main_Struct.Load_Mode = LOAD_IMP_MEASURE;
    //Выставляем флаг обновления
    Main_Struct.Refrash_Flag = TRUE;
  }      
}