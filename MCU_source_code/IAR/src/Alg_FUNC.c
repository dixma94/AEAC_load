//***************************************************************************
//  File........: Alg_FUNC.c
//  Author(s)...: KotyshkovEJ
//  Target(s)...: stm32f207
//  Compiler....: IAR
//  Description.: �������� ������� ������ ��������
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

LOAD_MAIN Main_Struct = {//��������� ���������
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
*   Purpose      : ���������� � ������� �������������� �������� �������� ������.
****************************************************************************/
void TIM3_IRQHandler()
{
  if((TIM3->SR & TIM_SR_UIF) != 0)
  {
    // ������� ���� ����������
    TIM3->SR &= ~TIM_SR_UIF;
   
    //��������� ��������� ������ ������������ ���� ������� �����
    if(Main_Struct.LOAD_FAN_StartupDelay)
    {
      Main_Struct.LOAD_FAN_StartupDelay--;
      if(Main_Struct.LOAD_FAN_StartupDelay==0)
      {
         FAN_Stop();
      }        
    }
    
    //���� ��������� �������� ��� ������������ ���������� ��������� ��������
    if(DeviceRegister.rw_region.DeviceCurrentSet * DeviceRegister.ro_region.DeviceVoltageRMS >= Load_limits.Max_Power || DeviceRegister.ro_region.DeviceVoltageRMS>= Load_limits.Max_Voltage)
    {
        if (fCalibrated && (ConfigData.ModbusConfigState != MB_DEVICE_CONF_STATE))
        {
         Main_Struct.Refrash_Flag = TRUE;
         Main_Struct.Load_Mode = LOAD_OFF;
         //���������� ���� ������
         DeviceRegister.ro_region.DeviceState |= HIGH_VOLTAGE_OR_VOLTAGE_FLAG;
        }
    }
    
    //���� ��������� ���� ����������, ����� ����� �������� � ��������
    if(Main_Struct.Refrash_Flag)
    {
      switch(Main_Struct.Load_Mode)
      {
      //��������� ����������� ������ ����������� ��������
      case LOAD_ON:
        //���� �� � ������ ����������
        if (fCalibrated && (ConfigData.ModbusConfigState != MB_DEVICE_CONF_STATE))
        {
           //���� ���������� ��� ������ ������������� ���� ��������, �� �������� ��������
           if(DeviceRegister.rw_region.DeviceCurrentSet > Load_limits.Max_Current)
           {
              DeviceRegister.rw_region.DeviceCurrentSet = 0;
           }
          
          //���� �� ��������� �������� � ������������ ���������� �������� ����������� � ���
          if(DeviceRegister.rw_region.DeviceCurrentSet * DeviceRegister.ro_region.DeviceVoltageRMS <= Load_limits.Max_Power && DeviceRegister.ro_region.DeviceVoltageRMS<=Load_limits.Max_Voltage)
          {
            //���������� ���� ������
            DeviceRegister.ro_region.DeviceState &= ~HIGH_VOLTAGE_OR_VOLTAGE_FLAG;
            //������������ �������� �����������
            Main_Struct.Shadow_FANSpd = FAN_Spd_Calc(DeviceRegister.rw_region.DeviceCurrentSet);
            FAN_PWM_Set(Main_Struct.Shadow_FANSpd);
            SetCurrent(DeviceRegister.rw_region.DeviceCurrentSet);
            DeviceRegister.ro_region.DeviceState &= ~FAN_MANUAL_FLAG;    
          }
          
        }
        else
        {
            //���������� ���� ������
            DeviceRegister.ro_region.DeviceState &= ~HIGH_VOLTAGE_OR_VOLTAGE_FLAG;
            //������������ �������� �����������
            Main_Struct.Shadow_FANSpd = FAN_Spd_Calc(DeviceRegister.rw_region.DeviceCurrentSet);
            FAN_PWM_Set(Main_Struct.Shadow_FANSpd);
            SetCurrent(DeviceRegister.rw_region.DeviceCurrentSet);
            DeviceRegister.ro_region.DeviceState &= ~FAN_MANUAL_FLAG;
        }
        
        break;
      case LOAD_IMP_MEASURE:
        Main_Struct.Shadow_FANSpd = FAN_Spd_Calc(500);
        FAN_PWM_Set(Main_Struct.Shadow_FANSpd);
        //�������� ��������� ���������(� ����� ��������� 
        Impedance_Periph_Init();
        break;
      //�� ���� ��������� ������� ��������� ��������
      default:
        SetCurrent(0);
        if(Main_Struct.FAN_Mode != FAN_MANUAL)
        {
          Main_Struct.LOAD_StopTime = HAL_GetTick();
        }
        Main_Struct.Load_Mode = LOAD_OFF;
        break;
      }
      //���� ������ ���������� ����� ������ ������������
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
    
       
    
    //��������� ����� ���������, ���� �� ����� ����, ������ ��������� ����������� � ���������
    if(Main_Struct.LOAD_StopTime && Main_Struct.Load_Mode == LOAD_OFF)
    {
      if(HAL_GetTick() -Main_Struct.LOAD_StopTime  >= FAN_STOP_DELAY)
      {
        //������������� ����������
        FAN_Stop();
        //��������, ����� �� ������� �����
        Main_Struct.LOAD_StopTime = 0;
      }
    }
  }
}

/**************************************************************************
*   Function name: MODBUS_DeviceControl_Proc
*   Returns      : void
*   Parameters   : void
*   Purpose      : ��������� �������� ���������� ���������
****************************************************************************/
void MODBUS_DeviceControl_Proc()
{
  // ��������� �������        
  if(DeviceRegister.ro_region.DeviceControl & LOAD_STATE)
  {
    //�������� ��������
    Main_Struct.Load_Mode = LOAD_ON;
    //���������� ���� ����������
    Main_Struct.Refrash_Flag = TRUE;
    //�������� ������������ �������� MODBUS
    MODBUS_TIMEOUT_En(TRUE);
  }
  else 
  {
    //����� �������� ����������� ������ ����
    Main_Struct.Shadow_FANSpd = 0;
    //��������� ��������
    Main_Struct.Load_Mode = LOAD_OFF;
    //���������� ���� ����������
    Main_Struct.Refrash_Flag = TRUE;
    //��������� ������������ �������� MODBUS
    MODBUS_TIMEOUT_En(FALSE);
  }
  if (DeviceRegister.ro_region.DeviceControl & FAN_STATE)
  {
    //���������� ����� ������ �����������
    Main_Struct.FAN_Mode = FAN_MANUAL;
    //���������� ���� ����������
    Main_Struct.Refrash_Flag = TRUE;
  }
  else
  {
    //�������������� ����� ������ �����������
    Main_Struct.FAN_Mode = FAN_AUTO;
    //���������� ���� ����������
    Main_Struct.Refrash_Flag = TRUE;
  }
  if (DeviceRegister.ro_region.DeviceControl & IMPEDANCE_MEASURE)
  {
    //����� ��������� ��������
    Main_Struct.Load_Mode = LOAD_IMP_MEASURE;
    //���������� ���� ����������
    Main_Struct.Refrash_Flag = TRUE;
  }      
}