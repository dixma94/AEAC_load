/******************************************************************************
*    Функции Modbus
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
*    Сообщение об ошибках
*******************************************************************************
*/
uint8_t mb_ErrorMessage (uint8_t * packet, uint8_t error)
{
    // К коду функци прибавляем 0x80
    *(packet + 1) |= 0x80;
    // код ошибки
    *(packet + 2)  = error;
    
    // Calculate CRC16 checksum for Modbus-Serial-Line-PDU.
    uint16_t usCRC16 = usMBCRC16((uint8_t*)packet, 3);
    *(packet + 3) = (uint8_t)( usCRC16 & 0xFF );
    *(packet + 4) = (uint8_t)( usCRC16 >> 8 );
    
    return 5;
}


//=============================================================================
//                          СТАНДАРТНЫЕ ФУНКЦИИ
//=============================================================================


/*
*******************************************************************************
*    0x03 READ_HOLDING_REGISTERS
*     Возвращаем длину пакета к передаче
*     BROADCAST запрос не поддерживается
*******************************************************************************
*/
uint8_t mb_ReadHoldingRegisters (uint8_t * packet, uint8_t packetLen)
{
    uint16_t RegAddr = 0;
    uint16_t RegNumb = 0;
    uint16_t * DevRegistr = (uint16_t*)&DeviceRegister;
    uint8_t  * pBuff = (packet + 3);
    
    // Проверяем, если запрос широковещательный, то не поддерживаем его
    if (packet[0] == MB_ADDRESS_BROADCAST) return 0;
    
    // Получаем начальный адрес
    RegAddr  = (uint16_t)( packet[2] << 8 );
    RegAddr |= (uint16_t)( packet[3] );  
   
    // Получаем количество регистров
    RegNumb  = (uint16_t)( packet[4] << 8 );
    RegNumb |= (uint16_t)( packet[5] );
    
    
    //Проверяем, не выходит ли команда за пределы доступной для чтения области:
    if ((RegAddr + RegNumb) >= ((sizeof (RW_REGION) + sizeof (DEVICE_CONFIG_REGION) + sizeof(RO_REGION)+ sizeof(DEVICE_CALIB_REGION)) / 2) + 1)
    {
        return packetLen = mb_ErrorMessage(packet,ILLEGAL_DATA_ADDRESS);
    }    
    
    
    //Формируем ответ: т.к. регистры 16 бит, значит в байтах = *2;
    packet[2] = (RegNumb * 2);
    
    // Читаем регистры
    for (uint8_t i = 0; i < RegNumb; i++)
    {
        *pBuff++ = (uint8_t)((DevRegistr[RegAddr + i] >> 8) & 0xFF);
        *pBuff++ = (uint8_t)(DevRegistr[RegAddr + i] & 0xFF);
    }
    
    // Считаем контрольную сумму
    uint16_t usCRC16 = usMBCRC16((uint8_t*)packet, (pBuff - packet));
    *pBuff++ = (uint8_t)( usCRC16 & 0xFF );
    *pBuff++   = (uint8_t)( usCRC16 >> 8 );
    
    packetLen = pBuff - packet;
    
    return packetLen;
}


/*
*******************************************************************************
*    0x05 - WRITE_SINGLE_COIL
*     Возвращаем длину пакета к передаче
*******************************************************************************
*/
uint8_t mb_WriteSingleCoil (uint8_t * packet, uint8_t packetLen)
{
    uint16_t CoilAddr  = 0;
    uint16_t CoilValue = 0;
    
    // Получаем адрес флага
    CoilAddr  = (uint16_t)( packet[2] << 8 );
    CoilAddr |= (uint16_t)( packet[3] );
    // Получаем значение флага
    CoilValue  = (uint16_t)( packet[4] << 8 );
    CoilValue |= (uint16_t)( packet[5] );    
    
//    if (CoilAddr > sizeof(
    
    
    return packetLen;
}


/*
*******************************************************************************
*    0x06 - WRITE_SINGLE_REGISTER
*     Возвращаем длину пакета к передаче
*******************************************************************************
*/
uint8_t mb_WriteSingleRegister (uint8_t * packet, uint8_t packetLen)
{
  
    uint16_t RegAddr = 0;
    uint16_t * DevRegistr = (uint16_t*)&DeviceRegister;
    uint8_t SizeRWConfig = 0;
    uint8_t SizeRWConfigRO = 0;
    uint8_t SizeRWConfigROCalib = 0;
    
    
    // Получаем адрес регистра
    RegAddr  = (uint16_t)( packet[2] << 8 );
    RegAddr |= (uint16_t)( packet[3] );   
    
      // Проверяем, можно ли записывать в данный регистр
    if (ConfigData.ModbusConfigState == MB_DEVICE_CONF_STATE)
    {
        // Если устройство находится в режиме конфигурации, то область, 
        // разрешенная для записи, расширяется:
        SizeRWConfig = ((sizeof (RW_REGION) + sizeof (DEVICE_CONFIG_REGION)) / 2);
        SizeRWConfigRO = ((sizeof (RW_REGION) + sizeof (DEVICE_CONFIG_REGION) + sizeof (RO_REGION)) / 2);
        SizeRWConfigROCalib =((sizeof (RW_REGION) + sizeof (DEVICE_CONFIG_REGION) + sizeof (RO_REGION)+ sizeof (DEVICE_CALIB_REGION)) / 2);
         
        if ((RegAddr < SizeRWConfig) || ( (RegAddr < SizeRWConfigROCalib) && (RegAddr >= SizeRWConfigRO) ) )
        {
          // Записываем в регистр
          DevRegistr[RegAddr] = (uint16_t)( packet[4] << 8 );
          DevRegistr[RegAddr]|= (uint16_t)( packet[5] );   
          
          //выставляем флаг для записи
          DeviceRegister.DeviceConfigRegion.flagDeviceConfigured = 0x5445;
        } else packetLen = mb_ErrorMessage(packet,ILLEGAL_DATA_ADDRESS);
    } 
    else
    {
      if (RegAddr < (sizeof (RW_REGION) / 2))
      {
          // Записываем в регистр
        DevRegistr[RegAddr] = (uint16_t)( packet[4] << 8 );
        DevRegistr[RegAddr]|= (uint16_t)( packet[5] );   
      } else packetLen = mb_ErrorMessage(packet,ILLEGAL_DATA_ADDRESS);
    } 
    
    return packetLen;
}


/*
*******************************************************************************
*    0x10 - WRITE_MULTIPLE_REGISTER
*     Возвращаем длину пакета к передаче
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
  
    // Получаем начальный адрес
    RegAddr  = (uint16_t)( packet[2] << 8 );
    RegAddr |= (uint16_t)( packet[3] );  
   
    // Получаем количество регистров
    RegNumb  = (uint16_t)( packet[4] << 8 );
    RegNumb |= (uint16_t)( packet[5] );
    

    // Проверяем, модно ли писать в данный диапазон адресов
    // Проверяем, можно ли записывать в данный регистр
    if (ConfigData.ModbusConfigState == MB_DEVICE_CONF_STATE)
    {
        // Если устройство находится в режиме конфигурации, то область, 
        // разрешенная для записи, расширяется:
        Size= sizeof (RW_REGION);
        SizeRWConfig = ((sizeof (RW_REGION) + sizeof (DEVICE_CONFIG_REGION)) / 2);
        SizeRWConfigRO = ((sizeof (RW_REGION) + sizeof (DEVICE_CONFIG_REGION) + sizeof (RO_REGION)) / 2);
        SizeRWConfigROCalib =((sizeof (RW_REGION) + sizeof (DEVICE_CONFIG_REGION) + sizeof (RO_REGION)+ sizeof (DEVICE_CALIB_REGION)) / 2);
        
        if ((((RegAddr + RegNumb)-1) < SizeRWConfig) || ( (((RegAddr + RegNumb)-1) < SizeRWConfigROCalib) && (((RegAddr + RegNumb)-1) >= SizeRWConfigRO) ) )
        {    
            // Пишем в регистры
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
            // Пишем в регистры
            for (uint8_t i = 0; i < RegNumb; i++)
            {
                DevRegistr[RegAddr + i] = (*pBuff++ << 8);
                DevRegistr[RegAddr + i] |= *pBuff++;
            }
      } else packetLen = mb_ErrorMessage(packet,ILLEGAL_DATA_ADDRESS);
    } 
    
   
    
    
    // Теперь формируем ответ
    pBuff = (packet + 6);
    // Считаем контрольную сумму
    uint16_t usCRC16 = usMBCRC16((uint8_t*)packet, (pBuff - packet));
    *pBuff++ = (uint8_t)( usCRC16 & 0xFF );
    *pBuff++   = (uint8_t)( usCRC16 >> 8 );
    
    return (pBuff - packet);

}


//=============================================================================
//                        ПОЛЬЗОВАТЕЛЬСКИЕ ФУНКЦИИ
//=============================================================================


/*
*******************************************************************************
*    0x65 - WRITE_ELLOAD_CONTROL_REGISTER 
*     Данная функция предназначена только для управления нагрузкой через
*     регистр управления
*******************************************************************************
*/
uint8_t mb_WriteLoadControlRegister (uint8_t * packet, uint8_t packetLen)
{
    uint16_t RegAddr = 0;
    uint16_t RegVal  = 0;
    uint16_t * DevRegistr = (uint16_t*)&DeviceRegister;
      
    // Получаем адрес регистра
    RegAddr  = (uint16_t)( packet[2] << 8 );
    RegAddr |= (uint16_t)( packet[3] );   
      
    // Если адрес регистра не совпадает с адресом регистра управления, возвращаем ошибку
    if (RegAddr != ((&DeviceRegister.ro_region.DeviceControl - DevRegistr)))
    {
        return packetLen = mb_ErrorMessage(packet,ILLEGAL_DATA_ADDRESS);
    }    
    
    RegVal  = (uint16_t)( packet[4] << 8 );
    RegVal |= (uint16_t)( packet[5] );    
    
    // Проверяем данные. Регистр может содержать только один управляющий бит
    if (!(RegVal & 0xF8))
    {
      //Всё нормально, данные похожи на правду
      //Записываем в регистр управления
      DevRegistr[RegAddr] = (uint16_t)( packet[4] << 8 );
      DevRegistr[RegAddr]|= (uint16_t)( packet[5] ); 
      
      //Обрабатываем принятые данные
      MODBUS_DeviceControl_Proc();
      
    } else packetLen = mb_ErrorMessage(packet,ILLEGAL_DATA_VALUE);
  
    return packetLen;
}



















































