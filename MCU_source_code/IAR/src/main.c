#include "main.h"
#include "BoardConfig.h"
#include "SaveReadConfigurationToFlashMCU.h"
#include "Alg_FUNC.h"
#include "Measure_FUNC.h"
#include "FAN_FUNC.h"

extern DEVICE_REGISTER DeviceRegister;
extern CONFIG_DATA ConfigData;
extern LOAD_MAIN Main_Struct;
extern LOAD_VERSION_LIMITS Load_limits;
extern double CalibCurrOutCoeff         [3];
extern double CalibVoltMeasCoeff        [3];
extern double CalibCurrMeasCoeff        [3];
extern uint8_t fCalibrated;

int main()
{
  HAL_Init();
  //Ставим указатель на структуру во FLASH
  ConfigData.ConfigRegion = (DEVICE_CONFIG_REGION*)EEPROM_START_ADDRESS;
  //Считываем структуру из FLASH в MODBUS регистры 
  ReadConfiguratioStruct();
  
  //проверяем флеш на мусор
  if (DeviceRegister.DeviceConfigRegion.flagDeviceConfigured != 0x5555)
  {        
    //устанавливаем флаг настройки
    DeviceRegister.DeviceConfigRegion.flagDeviceConfigured = 0x5555;
    
    
    DeviceRegister.DeviceConfigRegion.DeviceType           = EL_LOAD;
    DeviceRegister.DeviceConfigRegion.hwVersion            = HW_VER_20V;
    DeviceRegister.DeviceConfigRegion.swVersion            = 200;
    DeviceRegister.ro_region.LoadVersion = HW_VER_20V;
     DeviceRegister.ro_region.LoadVersion1 = 200;
    
    DeviceRegister.DeviceConfigRegion.messImpCoeff         = 9970;
    
    DeviceRegister.DeviceConfigRegion.ModBusADDR           = 0;
    DeviceRegister.DeviceConfigRegion.baudRate             = 0;
    DeviceRegister.DeviceConfigRegion.parity               = 0;
    DeviceRegister.DeviceConfigRegion.stopBit              = 0;
    // стандартные настроечные данные
    DeviceRegister.DeviceCalibRegion.IMeasP1               = 142;
    DeviceRegister.DeviceCalibRegion.IMeasP2               = 1488;
    DeviceRegister.DeviceCalibRegion.IMeasP3               = 2918;
    DeviceRegister.DeviceCalibRegion.VMeasP1               = 211;
    DeviceRegister.DeviceCalibRegion.VMeasP2               = 2026;
    DeviceRegister.DeviceCalibRegion.VMeasP3               = 4040;
    DeviceRegister.DeviceCalibRegion.ISetP1                = 138;
    DeviceRegister.DeviceCalibRegion.ISetP2                = 1497;
    DeviceRegister.DeviceCalibRegion.ISetP3                = 2937;
    
    DeviceRegister.DeviceCalibRegion.CalibIMeasP1          = 969;
    DeviceRegister.DeviceCalibRegion.CalibIMeasP2          = 14600;
    DeviceRegister.DeviceCalibRegion.CalibIMeasP3          = 29045;
    DeviceRegister.DeviceCalibRegion.CalibVMeasP1          = 1004;
    DeviceRegister.DeviceCalibRegion.CalibVMeasP2          = 10007;
    DeviceRegister.DeviceCalibRegion.CalibVMeasP3          = 20007;
    DeviceRegister.DeviceCalibRegion.CalibISetP1           = 958;
    DeviceRegister.DeviceCalibRegion.CalibISetP2           = 14595;
    DeviceRegister.DeviceCalibRegion.CalibISetP3           = 29030;

    
    //В резевервные записываем нули
    DeviceRegister.DeviceConfigRegion.DefaultCoefFlag      = 0x5555;
    DeviceRegister.DeviceConfigRegion.ReservedDC9          = 0;
    DeviceRegister.DeviceConfigRegion.ReservedDC10         = 0;
    
    // Записываем стандартную конфигурацию
    WriteConfiguratioStruct(&DeviceRegister.DeviceConfigRegion,&DeviceRegister.DeviceCalibRegion);

  }
  
  fCalibrated = TRUE;
  if (DeviceRegister.DeviceCalibRegion.IMeasP1 == 0) fCalibrated = FALSE;
    
  if (fCalibrated)
  {   
       CalibMakeCoeff((double*)&CalibCurrOutCoeff,  &DeviceRegister.DeviceCalibRegion.ISetP1, &DeviceRegister.DeviceCalibRegion.CalibISetP1); 
       CalibMakeCoeff((double*)&CalibVoltMeasCoeff, &DeviceRegister.DeviceCalibRegion.CalibVMeasP1, &DeviceRegister.DeviceCalibRegion.VMeasP1);
       CalibMakeCoeff((double*)&CalibCurrMeasCoeff, &DeviceRegister.DeviceCalibRegion.CalibIMeasP1, &DeviceRegister.DeviceCalibRegion.IMeasP1);
        
  }
  //Установка ограничений по току и напряжению в зависимости от версии
  switch (ConfigData.ConfigRegion->hwVersion)
  {
    //20 вольтовая версия 36A
    case 20:
      Load_limits.Max_Current = 3600;
      Load_limits.Max_Voltage = 2000;
      Load_limits.Max_Power = 6500000;
      Load_limits.FanCoeff = 28; 
    break;
     //60 вольтовая версия 15A
    case 21:
      Load_limits.Max_Current = 1500;
      Load_limits.Max_Voltage = 6500;
      Load_limits.Max_Power = 7500000;
       Load_limits.FanCoeff = 90; 
    break;
     //35 вольтовая версия 45A
    case 23:
      Load_limits.Max_Current = 4800;
      Load_limits.Max_Voltage = 3500;
      Load_limits.Max_Power = 8000000;
       Load_limits.FanCoeff = 24; 
    break;   
    default:
      Load_limits.Max_Current = 3600;
      Load_limits.Max_Voltage = 2000;
      Load_limits.Max_Power = 6500000;
       Load_limits.FanCoeff = 28; 
  }
   
  //Настройка переферии устройства
  BoardConfig();
  
  // Проверка вентиляторов 
  FAN_PWM_Set(70);
  DeviceRegister.ro_region.DeviceState &= ~FAN_MANUAL_FLAG;
  Main_Struct.LOAD_FAN_StartupDelay = FAN_STARTUP_DELAY;
    
  DeviceRegister.ro_region.LoadVersion = DeviceRegister.DeviceConfigRegion.hwVersion;
  DeviceRegister.ro_region.LoadVersion1 = DeviceRegister.DeviceConfigRegion.swVersion;
    
  while(1)
  {
   
    //Нормальный режим
    if(ConfigData.ModbusConfigState == MB_DEVICE_NOT_CONF_STATE)
    {
      //Навсякий случай обновляем указатель
     // ConfigData.ConfigRegion = (DEVICE_CONFIG_REGION*)EEPROM_START_ADDRESS;
      //Сбрасываем WatchDog
      IWDG_Reload();
    }
    //Режим настройки
    else if(ConfigData.ModbusConfigState == MB_DEVICE_CONF_STATE)
    {
      //В режиме настройки висим в бесконечном цикле
      while(ConfigData.ModbusConfigState == MB_DEVICE_CONF_STATE)
      {
        //Ставим указатель на структуру с настроечными коэффициентами
        ConfigData.ConfigRegion = &DeviceRegister.DeviceConfigRegion;
        //Сбрасываем WatchDog
        IWDG_Reload();
      }
      //Если были переданы новые коэффициенты, устанавливаем их и записываем во флеш
      if(DeviceRegister.DeviceConfigRegion.flagDeviceConfigured == 0x5445)
      {
        DeviceRegister.DeviceConfigRegion.flagDeviceConfigured = 0x5555;
        DeviceRegister.DeviceConfigRegion.DefaultCoefFlag = 0;
        
        DeviceRegister.ro_region.LoadVersion = DeviceRegister.DeviceConfigRegion.hwVersion;
        DeviceRegister.ro_region.LoadVersion1 = DeviceRegister.DeviceConfigRegion.swVersion;
        
        //Сохраняем новую структуру во FLASH
        WriteConfiguratioStruct(&DeviceRegister.DeviceConfigRegion,&DeviceRegister.DeviceCalibRegion);
        //Мигаем светодиодом, если коэффициенты были записаны
        HAL_GPIO_WritePin(DEBUG_LED_PORT, DEBUG_LED_PIN, GPIO_PIN_RESET);
        HAL_Delay(100);
        HAL_GPIO_WritePin(DEBUG_LED_PORT, DEBUG_LED_PIN, GPIO_PIN_SET);
      }
      //Зависаем, чтобы перезагрузиться
      while(1);
    }
  }
}
