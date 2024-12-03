#ifndef MODBUS_H
#define MODBUS_H

/******************************************************************************
*    Заголовочный файл для MODBUS.c
******************************************************************************/

#define MB_ADDRESS_BROADCAST     0

#define MB_CONF_STATE_MAX_TIME   1800000          //
#define MB_CONF_ENTER_MAX_TIME   4000          //       
#define MB_DELAY_TIME_ACCURACY   20              //  

//=============================================================================
// Тип устройства
#define POWER_SOURCE             1
#define EL_LOAD                  2

//=============================================================================
// Регистр управления устройством DeviceControl, описание битов
#define LOAD_STATE             0x1
#define FAN_STATE              0x2
#define IMPEDANCE_MEASURE      0x4

// Регистр DeviceState
#define ST_LOW_VOLTAGE                                     1
#define ST_TIMEOUT_EXPIRED                                 2
#define FAN_MANUAL_FLAG                                 0x10
#define HIGH_VOLTAGE_OR_VOLTAGE_FLAG                    0x20
#define IMPEDANCE_MEASURE_FLAG                          0x80
#define CONGIG_STATE_FLAG                               0x8000;


//=============================================================================

// Структура Read/Write
typedef struct RW_REGION{
    uint16_t DeviceCurrentSet;     // 0  (0x00) // RW (Read/Write only)
    uint16_t DeviceVoltageSet;     // 1  (0x01) // RW (Read/Write only)
    uint16_t FanPwmSet;            // 2  (0x02) // RW (Read/Write only)
    uint16_t ReservedRW1;          // 3  (0x03) // Reserved (Read/Write only) 
    uint16_t ReservedRW2;          // 4  (0x04) // Reserved (Read/Write only) 
    uint16_t ReservedRW3;          // 5  (0x05) // Reserved (Read/Write only) 
    uint16_t ReservedRW4;          // 6  (0x06) // Reserved (Read/Write only) 
    uint16_t ReservedRW5;          // 7  (0x07) // Reserved (Read/Write only) 
    uint16_t ReservedRW6;          // 8  (0x08) // Reserved (Read/Write only) 
    uint16_t ReservedRW7;          // 9  (0x09) // Reserved (Read/Write only) 
    uint16_t ReservedRW8;          // 10  (0x0A) // Reserved (Read/Write only) 
    uint16_t ReservedRW9;          // 11  (0x0B) // Reserved (Read/Write only)  
    uint16_t ReservedRW10;         // 12  (0x0C) // Reserved (Read/Write only)  
}RW_REGION;

// Конфигурационные и настроечные данные 
typedef struct DEVICE_CONFIG_REGION{
    uint16_t flagDeviceConfigured; // 13  (0x0D) // Флаг, что устройство сконфигурировано 
    uint16_t DeviceType;           // 14  (0x0E) // Тип устройства 
    uint16_t hwVersion;            // 15  (0x0F) // Версия железа
    uint16_t swVersion;            // 16  (0x10) // Версия програмного обеспечения
    uint16_t ModBusADDR;           // 17  (0x11) // Сетевой адрес 
    uint16_t baudRate;             // 18  (0x12) // BaudRate USART
    uint16_t stopBit;              // 19  (0x13) // USART
    uint16_t parity;               // 20  (0x14) // USART
    // КАЛИБРОВОЧНЫЕ ДАННЫЕ
    uint16_t messVoltOffset;       // 21  (0x15) // Измерение напряжения, смещение нуля
    uint16_t messVoltCoeff;        // 22  (0x16) // Измерение напряжения, установочный коэффициент
    uint16_t messCurrOffset;       // 23  (0x17) // Измерение тока, смещение нуля
    uint16_t messCurrCoeff;        // 24  (0x18) // Измерение тока, установочный коэффициент
    uint16_t setCurrOffset;        // 25  (0x19) // Установка тока, смещение нуля
    uint16_t setCurrCoeff;         // 26  (0x1A) // Установка тока, установочный коэффициент
    uint16_t messImpCoeff;         // 27  (0x1B) // Измерение импеданса, установочный коэффициент
    uint16_t DefaultCoefFlag;      // 28  (0x1C) // Указывает, что записаны стандартные коэффициенты
    uint16_t ReservedDC9;          // 29  (0x1D) // Reserved
    uint16_t ReservedDC10;         // 30  (0x1E) // Reserved
}DEVICE_CONFIG_REGION;

// Структура Read only
typedef struct RO_REGION{
    uint16_t DeviceControl;        // 31 (0x1F) // Регистр управления блоком, и его состояния (обязательно располагается в начале структуры!!!)
    uint16_t DeviceState;          // 32 (0x20) // RO Состояние тестовых сигналов блока   
    uint16_t DeviceCurrentRMS;     // 33 (0x21) // RO (Read only)
    uint16_t DeviceVoltageRMS;     // 34 (0x22) // RO (Read only)
    uint16_t DevicePower;          // 35 (0x23) // RO (Read only)
    uint16_t DeviceImpedance;      // 36 (0x24)
    uint16_t LoadVersion;          // 37 (0x25) // Aa?ney aeiea
    uint16_t LoadVersion1;         // 38 (0x26) // Aa?ney aeiea
    uint16_t LoadVersion2;         // 39 (0x27) // Aa?ney aeiea
}RO_REGION;

typedef struct DEVICE_CALIB_REGION{
  uint16_t VMeasP1;             // 40 (0x28) //Измереннное напряжение, точка 1
  uint16_t VMeasP2;             // 41 (0x29) //Измереннное напряжение, точка 2
  uint16_t VMeasP3;             // 42 (0x2A) //Измереннное напряжение, точка 3
  uint16_t ISetP1;              // 43 (0x2B) //Выходной ток, точка 1
  uint16_t ISetP2;              // 44 (0x2C) //Выходной ток, точка 2
  uint16_t ISetP3;              // 45 (0x2D) //Выходной ток, точка 3
  uint16_t IMeasP1;             // 46 (0x2E) //Измеренный ток, точка 1
  uint16_t IMeasP2;             // 47 (0x2F) //Измеренный ток, точка 2
  uint16_t IMeasP3;             // 48 (0x30) //Измеренный ток, точка 3
  
  uint16_t CalibVMeasP1;        // 49 (0x31) //Измереннное напряжение мультиметр, точка 1
  uint16_t CalibVMeasP2;        // 50 (0x32) //Измереннное напряжение мультиметр, точка 2
  uint16_t CalibVMeasP3;        // 51 (0x33) //Измереннное напряжение мультиметр, точка 3
  uint16_t CalibISetP1;         // 52 (0x34) //Выходной ток мультиметр, точка 1
  uint16_t CalibISetP2;         // 53 (0x35) //Выходной ток мультиметр, точка 2
  uint16_t CalibISetP3;         // 54 (0x36) //Выходной ток мультиметр, точка 3
  uint16_t CalibIMeasP1;        // 55 (0x37) //Измеренный ток мультиметр, точка 1
  uint16_t CalibIMeasP2;        // 56 (0x38) //Измеренный ток мультиметр, точка 2
  uint16_t CalibIMeasP3;        // 57 (0x39) //Измеренный ток мультиметр, точка 3
  }DEVICE_CALIB_REGION;
/*
===============================================================================
 Структура регистров состоит из областей R/W, RO.
 
                      ______________________________________________       
                     |    Register    |    MODBUS     |  Register   |
                     |    region 1    | CONFIGURATION |  region 2   |
                      ----------------------------------------------
                     |                |                             |
                     |                |                             |
 normal mode         |<- READ/WRITE ->|<-         READ ONLY       ->|
                     |                                              |
 programm mode       |<-         READ/WRITE         ->|<-         ->|  

    В нормальном режиме, Region 1 доступен как для чтения, так и для записи.
Регионы MODBUS_CONFIGURATION и Register region 2 доступны только для чтения.
В режиме программирования, область доступная для записи, расширяется до начала 
области Register region 2. Таким образом, становится доступной для записи область 
MODBUS CONFIGURATION, в которой хранятся настройки: адрес устройства и параметры 
интерфейса.
-------------------------------------------------------------------------------
*/

// Структура регистров
typedef struct {
    struct RW_REGION              rw_region;   
    struct DEVICE_CONFIG_REGION   DeviceConfigRegion;
    struct RO_REGION              ro_region;
    struct DEVICE_CALIB_REGION           DeviceCalibRegion;
} DEVICE_REGISTER;

//=============================================================================

// Описание типа состояния конфигурации
typedef enum{
  MB_DEVICE_NOT_CONF_STATE = 0, //Конфигурация запрещена.
  MB_DEVICE_CONF_STATE = 3 //Конфигурация разрещена.
} MODBUS_CONFIG_STATE;

// Рабочие данные конфигурации
typedef struct {
  MODBUS_CONFIG_STATE ModbusConfigState; // Текущее состояние конфигурации 
  uint8_t Receive_Bytes; // Счётчик принятых байтов
  DEVICE_CONFIG_REGION* ConfigRegion; //Указатеь на текующую конфигурацию(необходим для переключения в режим настройки)
  uint32_t ModbusTempCount; //Счётчик сброса разрешения на калибровку
} CONFIG_DATA;

//=============================================================================

uint8_t ModbusPacketProc (uint8_t * packet, uint8_t packetLen);

#endif /* MODBUS_H */
































