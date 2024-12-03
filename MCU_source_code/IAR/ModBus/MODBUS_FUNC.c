/******************************************************************************
*    ������� Modbus
******************************************************************************/
#include "stm32f2xx.h"
//#include "main.h"
#include "MODBUS.h"
#include "MODBUS_CRC.h"
#include "MODBUS_FUNC.h"
#include "CurrentSet_FUNC.h"
#include "Alg_FUNC.h"



extern DEVICE_REGISTER DeviceRegister;
extern CONFIG_DATA ConfigData;
extern LOAD_MAIN Main_Struct;

/*
*******************************************************************************
*    ��������� �� �������
*******************************************************************************
*/
uint8_t mb_ErrorMessage (uint8_t * packet, uint8_t error)
{
    // � ���� ������ ���������� 0x80
    *(packet + 1) |= 0x80;
    // ��� ������
    *(packet + 2)  = error;
    
    // Calculate CRC16 checksum for Modbus-Serial-Line-PDU.
    uint16_t usCRC16 = usMBCRC16((uint8_t*)packet, 3);
    *(packet + 3) = (uint8_t)( usCRC16 & 0xFF );
    *(packet + 4) = (uint8_t)( usCRC16 >> 8 );
    
    return 5;
}


//=============================================================================
//                          ����������� �������
//=============================================================================


/*
*******************************************************************************
*    0x03 READ_HOLDING_REGISTERS
*     ���������� ����� ������ � ��������
*     BROADCAST ������ �� ��������������
*******************************************************************************
*/
uint8_t mb_ReadHoldingRegisters (uint8_t * packet, uint8_t packetLen)
{
    uint16_t RegAddr = 0;
    uint16_t RegNumb = 0;
    uint16_t * DevRegistr = (uint16_t*)&DeviceRegister;
    uint8_t  * pBuff = (packet + 3);
    
    // ���������, ���� ������ �����������������, �� �� ������������ ���
    if (packet[0] == MB_ADDRESS_BROADCAST) return 0;
    
    // �������� ��������� �����
    RegAddr  = (uint16_t)( packet[2] << 8 );
    RegAddr |= (uint16_t)( packet[3] );  
   
    // �������� ���������� ���������
    RegNumb  = (uint16_t)( packet[4] << 8 );
    RegNumb |= (uint16_t)( packet[5] );
    
    
    //���������, �� ������� �� ������� �� ������� ��������� ��� ������ �������:
    if ((RegAddr + RegNumb) >= ((sizeof (RW_REGION) + sizeof (DEVICE_CONFIG_REGION) + sizeof(RO_REGION)+ sizeof(DEVICE_CALIB_REGION)) / 2) + 1)
    {
        return packetLen = mb_ErrorMessage(packet,ILLEGAL_DATA_ADDRESS);
    }    
    
    
    //��������� �����: �.�. �������� 16 ���, ������ � ������ = *2;
    packet[2] = (RegNumb * 2);
    
    // ������ ��������
    for (uint8_t i = 0; i < RegNumb; i++)
    {
        *pBuff++ = (uint8_t)((DevRegistr[RegAddr + i] >> 8) & 0xFF);
        *pBuff++ = (uint8_t)(DevRegistr[RegAddr + i] & 0xFF);
    }
    
    // ������� ����������� �����
    uint16_t usCRC16 = usMBCRC16((uint8_t*)packet, (pBuff - packet));
    *pBuff++ = (uint8_t)( usCRC16 & 0xFF );
    *pBuff++   = (uint8_t)( usCRC16 >> 8 );
    
    packetLen = pBuff - packet;
    
    return packetLen;
}


/*
*******************************************************************************
*    0x05 - WRITE_SINGLE_COIL
*     ���������� ����� ������ � ��������
*******************************************************************************
*/
uint8_t mb_WriteSingleCoil (uint8_t * packet, uint8_t packetLen)
{
    uint16_t CoilAddr  = 0;
    uint16_t CoilValue = 0;
    
    // �������� ����� �����
    CoilAddr  = (uint16_t)( packet[2] << 8 );
    CoilAddr |= (uint16_t)( packet[3] );
    // �������� �������� �����
    CoilValue  = (uint16_t)( packet[4] << 8 );
    CoilValue |= (uint16_t)( packet[5] );    
    
//    if (CoilAddr > sizeof(
    
    
    return packetLen;
}


/*
*******************************************************************************
*    0x06 - WRITE_SINGLE_REGISTER
*     ���������� ����� ������ � ��������
*******************************************************************************
*/
uint8_t mb_WriteSingleRegister (uint8_t * packet, uint8_t packetLen)
{
  
    uint16_t RegAddr = 0;
    uint16_t * DevRegistr = (uint16_t*)&DeviceRegister;
    uint8_t SizeRWConfig = 0;
    uint8_t SizeRWConfigRO = 0;
    uint8_t SizeRWConfigROCalib = 0;
    
    
    // �������� ����� ��������
    RegAddr  = (uint16_t)( packet[2] << 8 );
    RegAddr |= (uint16_t)( packet[3] );   
    
      // ���������, ����� �� ���������� � ������ �������
    if (ConfigData.ModbusConfigState == MB_DEVICE_CONF_STATE)
    {
        // ���� ���������� ��������� � ������ ������������, �� �������, 
        // ����������� ��� ������, �����������:
        SizeRWConfig = ((sizeof (RW_REGION) + sizeof (DEVICE_CONFIG_REGION)) / 2);
        SizeRWConfigRO = ((sizeof (RW_REGION) + sizeof (DEVICE_CONFIG_REGION) + sizeof (RO_REGION)) / 2);
        SizeRWConfigROCalib =((sizeof (RW_REGION) + sizeof (DEVICE_CONFIG_REGION) + sizeof (RO_REGION)+ sizeof (DEVICE_CALIB_REGION)) / 2);
         
        if ((RegAddr < SizeRWConfig) || ( (RegAddr < SizeRWConfigROCalib) && (RegAddr >= SizeRWConfigRO) ) )
        {
          // ���������� � �������
          DevRegistr[RegAddr] = (uint16_t)( packet[4] << 8 );
          DevRegistr[RegAddr]|= (uint16_t)( packet[5] );   
          
          //���������� ���� ��� ������
          DeviceRegister.DeviceConfigRegion.flagDeviceConfigured = 0x5445;
        } else packetLen = mb_ErrorMessage(packet,ILLEGAL_DATA_ADDRESS);
    } 
    else
    {
      if (RegAddr < (sizeof (RW_REGION) / 2))
      {
          // ���������� � �������
        DevRegistr[RegAddr] = (uint16_t)( packet[4] << 8 );
        DevRegistr[RegAddr]|= (uint16_t)( packet[5] );   
      } else packetLen = mb_ErrorMessage(packet,ILLEGAL_DATA_ADDRESS);
    } 
    
    return packetLen;
}


/*
*******************************************************************************
*    0x10 - WRITE_MULTIPLE_REGISTER
*     ���������� ����� ������ � ��������
*******************************************************************************
*/
uint8_t mb_WriteMultipleRegister (uint8_t * packet, uint8_t packetLen)
{
    uint16_t RegAddr = 0;
    uint16_t RegNumb = 0;
    uint16_t * DevRegistr = (uint16_t*)&DeviceRegister;
    uint8_t  * pBuff = (packet + 7);
    uint8_t SizeRWConfig = 0;
    uint8_t SizeRWConfigRO = 0;
    uint8_t SizeRWConfigROCalib = 0;
    uint8_t Size = 0;
  
    // �������� ��������� �����
    RegAddr  = (uint16_t)( packet[2] << 8 );
    RegAddr |= (uint16_t)( packet[3] );  
   
    // �������� ���������� ���������
    RegNumb  = (uint16_t)( packet[4] << 8 );
    RegNumb |= (uint16_t)( packet[5] );
    

    // ���������, ����� �� ������ � ������ �������� �������
    // ���������, ����� �� ���������� � ������ �������
    if (ConfigData.ModbusConfigState == MB_DEVICE_CONF_STATE)
    {
        // ���� ���������� ��������� � ������ ������������, �� �������, 
        // ����������� ��� ������, �����������:
        Size= sizeof (RW_REGION);
        SizeRWConfig = ((sizeof (RW_REGION) + sizeof (DEVICE_CONFIG_REGION)) / 2);
        SizeRWConfigRO = ((sizeof (RW_REGION) + sizeof (DEVICE_CONFIG_REGION) + sizeof (RO_REGION)) / 2);
        SizeRWConfigROCalib =((sizeof (RW_REGION) + sizeof (DEVICE_CONFIG_REGION) + sizeof (RO_REGION)+ sizeof (DEVICE_CALIB_REGION)) / 2);
        
        if ((((RegAddr + RegNumb)-1) < SizeRWConfig) || ( (((RegAddr + RegNumb)-1) < SizeRWConfigROCalib) && (((RegAddr + RegNumb)-1) >= SizeRWConfigRO) ) )
        {    
            // ����� � ��������
            for (uint8_t i = 0; i < RegNumb; i++)
            {
                DevRegistr[RegAddr + i] = (*pBuff++ << 8);
                DevRegistr[RegAddr + i] |= *pBuff++;
            }
        } else {
            return packetLen = mb_ErrorMessage(packet,ILLEGAL_DATA_ADDRESS);
        }
        
    } 
    else
    {
      if (RegAddr < (sizeof (RW_REGION) / 2))
      {
            // ����� � ��������
            for (uint8_t i = 0; i < RegNumb; i++)
            {
                DevRegistr[RegAddr + i] = (*pBuff++ << 8);
                DevRegistr[RegAddr + i] |= *pBuff++;
            }
      } else packetLen = mb_ErrorMessage(packet,ILLEGAL_DATA_ADDRESS);
    } 
    
   
    
    
    // ������ ��������� �����
    pBuff = (packet + 6);
    // ������� ����������� �����
    uint16_t usCRC16 = usMBCRC16((uint8_t*)packet, (pBuff - packet));
    *pBuff++ = (uint8_t)( usCRC16 & 0xFF );
    *pBuff++   = (uint8_t)( usCRC16 >> 8 );
    
    return (pBuff - packet);

}


//=============================================================================
//                        ���������������� �������
//=============================================================================


/*
*******************************************************************************
*    0x65 - WRITE_ELLOAD_CONTROL_REGISTER 
*     ������ ������� ������������� ������ ��� ���������� ��������� �����
*     ������� ����������
*******************************************************************************
*/
uint8_t mb_WriteLoadControlRegister (uint8_t * packet, uint8_t packetLen)
{
    uint16_t RegAddr = 0;
    uint16_t RegVal  = 0;
    uint16_t * DevRegistr = (uint16_t*)&DeviceRegister;
      
    // �������� ����� ��������
    RegAddr  = (uint16_t)( packet[2] << 8 );
    RegAddr |= (uint16_t)( packet[3] );   
      
    // ���� ����� �������� �� ��������� � ������� �������� ����������, ���������� ������
    if (RegAddr != ((&DeviceRegister.ro_region.DeviceControl - DevRegistr)))
    {
        return packetLen = mb_ErrorMessage(packet,ILLEGAL_DATA_ADDRESS);
    }    
    
    RegVal  = (uint16_t)( packet[4] << 8 );
    RegVal |= (uint16_t)( packet[5] );    
    
    // ��������� ������. ������� ����� ��������� ������ ���� ����������� ���
    if (!(RegVal & 0xF8))
    {
      //�� ���������, ������ ������ �� ������
      //���������� � ������� ����������
      DevRegistr[RegAddr] = (uint16_t)( packet[4] << 8 );
      DevRegistr[RegAddr]|= (uint16_t)( packet[5] ); 
      
      //������������ �������� ������
      MODBUS_DeviceControl_Proc();
      
    } else packetLen = mb_ErrorMessage(packet,ILLEGAL_DATA_VALUE);
  
    return packetLen;
}



















































