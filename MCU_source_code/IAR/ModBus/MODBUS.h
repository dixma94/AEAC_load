#ifndef MODBUS_H
#define MODBUS_H

/******************************************************************************
*    ������������ ���� ��� MODBUS.c
******************************************************************************/

#define MB_ADDRESS_BROADCAST     0

#define MB_CONF_STATE_MAX_TIME   1800000          //
#define MB_CONF_ENTER_MAX_TIME   4000          //       
#define MB_DELAY_TIME_ACCURACY   20              //  

//=============================================================================
// ��� ����������
#define POWER_SOURCE             1
#define EL_LOAD                  2

//=============================================================================
// ������� ���������� ����������� DeviceControl, �������� �����
#define LOAD_STATE             0x1
#define FAN_STATE              0x2
#define IMPEDANCE_MEASURE      0x4

// ������� DeviceState
#define ST_LOW_VOLTAGE                                     1
#define ST_TIMEOUT_EXPIRED                                 2
#define FAN_MANUAL_FLAG                                 0x10
#define HIGH_VOLTAGE_OR_VOLTAGE_FLAG                    0x20
#define IMPEDANCE_MEASURE_FLAG                          0x80
#define CONGIG_STATE_FLAG                               0x8000;


//=============================================================================

// ��������� Read/Write
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

// ���������������� � ����������� ������ 
typedef struct DEVICE_CONFIG_REGION{
    uint16_t flagDeviceConfigured; // 13  (0x0D) // ����, ��� ���������� ���������������� 
    uint16_t DeviceType;           // 14  (0x0E) // ��� ���������� 
    uint16_t hwVersion;            // 15  (0x0F) // ������ ������
    uint16_t swVersion;            // 16  (0x10) // ������ ����������� �����������
    uint16_t ModBusADDR;           // 17  (0x11) // ������� ����� 
    uint16_t baudRate;             // 18  (0x12) // BaudRate USART
    uint16_t stopBit;              // 19  (0x13) // USART
    uint16_t parity;               // 20  (0x14) // USART
    // ������������� ������
    uint16_t messVoltOffset;       // 21  (0x15) // ��������� ����������, �������� ����
    uint16_t messVoltCoeff;        // 22  (0x16) // ��������� ����������, ������������ �����������
    uint16_t messCurrOffset;       // 23  (0x17) // ��������� ����, �������� ����
    uint16_t messCurrCoeff;        // 24  (0x18) // ��������� ����, ������������ �����������
    uint16_t setCurrOffset;        // 25  (0x19) // ��������� ����, �������� ����
    uint16_t setCurrCoeff;         // 26  (0x1A) // ��������� ����, ������������ �����������
    uint16_t messImpCoeff;         // 27  (0x1B) // ��������� ���������, ������������ �����������
    uint16_t DefaultCoefFlag;      // 28  (0x1C) // ���������, ��� �������� ����������� ������������
    uint16_t ReservedDC9;          // 29  (0x1D) // Reserved
    uint16_t ReservedDC10;         // 30  (0x1E) // Reserved
}DEVICE_CONFIG_REGION;

// ��������� Read only
typedef struct RO_REGION{
    uint16_t DeviceControl;        // 31 (0x1F) // ������� ���������� ������, � ��� ��������� (����������� ������������� � ������ ���������!!!)
    uint16_t DeviceState;          // 32 (0x20) // RO ��������� �������� �������� �����   
    uint16_t DeviceCurrentRMS;     // 33 (0x21) // RO (Read only)
    uint16_t DeviceVoltageRMS;     // 34 (0x22) // RO (Read only)
    uint16_t DevicePower;          // 35 (0x23) // RO (Read only)
    uint16_t DeviceImpedance;      // 36 (0x24)
    uint16_t LoadVersion;          // 37 (0x25) // Aa?ney aeiea
    uint16_t LoadVersion1;         // 38 (0x26) // Aa?ney aeiea
    uint16_t LoadVersion2;         // 39 (0x27) // Aa?ney aeiea
}RO_REGION;

typedef struct DEVICE_CALIB_REGION{
  uint16_t VMeasP1;             // 40 (0x28) //����������� ����������, ����� 1
  uint16_t VMeasP2;             // 41 (0x29) //����������� ����������, ����� 2
  uint16_t VMeasP3;             // 42 (0x2A) //����������� ����������, ����� 3
  uint16_t ISetP1;              // 43 (0x2B) //�������� ���, ����� 1
  uint16_t ISetP2;              // 44 (0x2C) //�������� ���, ����� 2
  uint16_t ISetP3;              // 45 (0x2D) //�������� ���, ����� 3
  uint16_t IMeasP1;             // 46 (0x2E) //���������� ���, ����� 1
  uint16_t IMeasP2;             // 47 (0x2F) //���������� ���, ����� 2
  uint16_t IMeasP3;             // 48 (0x30) //���������� ���, ����� 3
  
  uint16_t CalibVMeasP1;        // 49 (0x31) //����������� ���������� ����������, ����� 1
  uint16_t CalibVMeasP2;        // 50 (0x32) //����������� ���������� ����������, ����� 2
  uint16_t CalibVMeasP3;        // 51 (0x33) //����������� ���������� ����������, ����� 3
  uint16_t CalibISetP1;         // 52 (0x34) //�������� ��� ����������, ����� 1
  uint16_t CalibISetP2;         // 53 (0x35) //�������� ��� ����������, ����� 2
  uint16_t CalibISetP3;         // 54 (0x36) //�������� ��� ����������, ����� 3
  uint16_t CalibIMeasP1;        // 55 (0x37) //���������� ��� ����������, ����� 1
  uint16_t CalibIMeasP2;        // 56 (0x38) //���������� ��� ����������, ����� 2
  uint16_t CalibIMeasP3;        // 57 (0x39) //���������� ��� ����������, ����� 3
  }DEVICE_CALIB_REGION;
/*
===============================================================================
 ��������� ��������� ������� �� �������� R/W, RO.
 
                      ______________________________________________       
                     |    Register    |    MODBUS     |  Register   |
                     |    region 1    | CONFIGURATION |  region 2   |
                      ----------------------------------------------
                     |                |                             |
                     |                |                             |
 normal mode         |<- READ/WRITE ->|<-         READ ONLY       ->|
                     |                                              |
 programm mode       |<-         READ/WRITE         ->|<-         ->|  

    � ���������� ������, Region 1 �������� ��� ��� ������, ��� � ��� ������.
������� MODBUS_CONFIGURATION � Register region 2 �������� ������ ��� ������.
� ������ ����������������, ������� ��������� ��� ������, ����������� �� ������ 
������� Register region 2. ����� �������, ���������� ��������� ��� ������ ������� 
MODBUS CONFIGURATION, � ������� �������� ���������: ����� ���������� � ��������� 
����������.
-------------------------------------------------------------------------------
*/

// ��������� ���������
typedef struct {
    struct RW_REGION              rw_region;   
    struct DEVICE_CONFIG_REGION   DeviceConfigRegion;
    struct RO_REGION              ro_region;
    struct DEVICE_CALIB_REGION           DeviceCalibRegion;
} DEVICE_REGISTER;

//=============================================================================

// �������� ���� ��������� ������������
typedef enum{
  MB_DEVICE_NOT_CONF_STATE = 0, //������������ ���������.
  MB_DEVICE_CONF_STATE = 3 //������������ ���������.
} MODBUS_CONFIG_STATE;

// ������� ������ ������������
typedef struct {
  MODBUS_CONFIG_STATE ModbusConfigState; // ������� ��������� ������������ 
  uint8_t Receive_Bytes; // ������� �������� ������
  DEVICE_CONFIG_REGION* ConfigRegion; //�������� �� �������� ������������(��������� ��� ������������ � ����� ���������)
  uint32_t ModbusTempCount; //������� ������ ���������� �� ����������
} CONFIG_DATA;

//=============================================================================

uint8_t ModbusPacketProc (uint8_t * packet, uint8_t packetLen);

#endif /* MODBUS_H */
































