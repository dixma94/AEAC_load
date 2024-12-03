/******************************************************************************
*    ��������� ������ modbus
******************************************************************************/

#include "stm32f2xx.h"
//#include "main.h"
#include "MODBUS.h"
#include "MODBUS_CRC.h"
#include "MODBUS_FUNC.h"


DEVICE_REGISTER DeviceRegister;
CONFIG_DATA ConfigData;


/*
*******************************************************************************
*    ��������� Modbus ������
*     ���������� ����� ��������������� ������
*     � ���� ������ ����������� �����, ���������� 0
*******************************************************************************
*/
uint8_t ModbusPacketProc (uint8_t * packet, uint8_t packetLen)
{   
  uint8_t retPacketLen = 0;
  
  // ���� ������ �� ��� ��� �� �����������������, �� ���������� ������
  if ((packet[0] != EL_LOAD) &&
      (packet[0] != MB_ADDRESS_BROADCAST))
  {
    return 0;
  }
  
  // ��������� ����������� �����    
  if (usMBCRC16(packet, packetLen))
  {
    // ����������� ����� �����������
    return 0;
  }
  
  // ������������ �������� �����
  switch (*(packet+1))
  {
  case READ_COILS:
    retPacketLen = mb_ErrorMessage(packet,ILLEGAL_FUNCTION);    //
    break;
  case READ_DISCRETE_INPUTS:
    retPacketLen = mb_ErrorMessage(packet,ILLEGAL_FUNCTION);    //
    break;
  case READ_HOLDING_REGISTERS:
    retPacketLen = mb_ReadHoldingRegisters(packet,packetLen);
    break;
  case READ_INPUT_REGISTERS:
    retPacketLen = mb_ErrorMessage(packet,ILLEGAL_FUNCTION);    //
    break;
  case WRITE_SINGLE_COIL:
    retPacketLen = mb_ErrorMessage(packet,ILLEGAL_FUNCTION);    //
    //retPacketLen = mb_WriteSingleCoil (packet, packetLen);   
    break;
  case WRITE_SINGLE_REGISTER:
    retPacketLen = mb_WriteSingleRegister(packet,packetLen);
    break;
  case WRITE_MULTIPLE_COIL:
    retPacketLen = mb_ErrorMessage(packet,ILLEGAL_FUNCTION);    //
    break;
  case WRITE_MULTIPLE_REGISTER:
    retPacketLen = mb_WriteMultipleRegister(packet,packetLen);
    break;
  case WRITE_ELLOAD_CONTROL_REGISTER:                             // USER defined function
    retPacketLen = mb_WriteLoadControlRegister(packet, packetLen);
    break;
  default:
    retPacketLen = mb_ErrorMessage(packet,ILLEGAL_FUNCTION);    //
    break;
  }
  
  
  return retPacketLen;
}











































