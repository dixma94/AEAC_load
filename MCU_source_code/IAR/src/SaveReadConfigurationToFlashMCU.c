//***************************************************************************
//  File........: Measure_FUNC.c
//  Author(s)...: KotyshkovEJ
//  Target(s)...: stm32f207
//  Compiler....: IAR
//  Description.: ���������� � �������� ���������������� � ����������� ������ �� 
//                ���������� ���� ����������������
//  Date........: 13.04.2021
//***************************************************************************

#include "SaveReadConfigurationToFlashMCU.h"


extern DEVICE_REGISTER DeviceRegister;


/**************************************************************************
*   Function name: WriteConfiguratioStruct
*   Returns      : void
*   Parameters   : DEVICE_CONFIG_REGION* DeviceConfigRegion(��������� �� ���������,
                   ������� ����� �������� �� ����)
*   Purpose      : ������ ������������ �� ����
****************************************************************************/
void WriteConfiguratioStruct(DEVICE_CONFIG_REGION* DeviceConfigRegion,DEVICE_CALIB_REGION* DeviceCalibRegion)
{
    //FLASH_Status FLASHStatus;
    uint16_t * pStruct;
    uint32_t Address = 0x00;
  
    // ����� ��������� �� ������ ���������
    pStruct = (uint16_t *)DeviceConfigRegion;

    // Unlock the Flash Program Erase controller 
    HAL_FLASH_Unlock();

    // ������� ��� ����� 
    FLASH->SR &=~ (FLASH_SR_EOP | FLASH_SR_WRPERR | FLASH_SR_PGAERR | 
				   FLASH_SR_PGPERR | FLASH_SR_PGSERR);

    // ����� ������� ���������� ������� ������
    FLASH_Erase_Sector(FLASH_SECTOR_2, FLASH_VOLTAGE_RANGE_3);


    Address = EEPROM_START_ADDRESS;
    while ((Address < (EEPROM_START_ADDRESS + sizeof(DEVICE_CONFIG_REGION))))
    {
        
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, Address, *(uint16_t *)pStruct++);
        Address += 2;
    }
    
     //����� ��������� �� ������ ��������� 
    pStruct = (uint16_t *)DeviceCalibRegion;
    while ((Address < (EEPROM_START_ADDRESS + sizeof(DEVICE_CONFIG_REGION)+ sizeof(DEVICE_CALIB_REGION))))
    {
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, Address, *(uint16_t *)pStruct++);
        Address += 2;
    }
    
   

    //FLASH_Lock
    HAL_FLASH_Lock();
}


/**************************************************************************
*   Function name: ReadConfiguratioStruct
*   Returns      : void (������������, ��������� �� ����)
*   Parameters   : void
*   Purpose      : ������ ������������ �� ����
****************************************************************************/
void ReadConfiguratioStruct(void)
{
  uint16_t * pStruct;
    uint32_t Address = 0x00;

    // ?????? ?????????? ?? ?????? ?????????
    pStruct = (uint16_t *)&DeviceRegister.DeviceConfigRegion;

    Address = EEPROM_START_ADDRESS;
    while (Address < (EEPROM_START_ADDRESS + sizeof(DEVICE_CONFIG_REGION)))
    {
        *pStruct++ = *(__IO uint16_t*)Address;
        Address += 2;
    }
    // ?????? ?????????? ?? ?????? ?????????
    pStruct = (uint16_t *)&DeviceRegister.DeviceCalibRegion;
    while (Address < (EEPROM_START_ADDRESS + sizeof(DEVICE_CONFIG_REGION)+ sizeof(DEVICE_CALIB_REGION)))
    {
        *pStruct++ = *(__IO uint16_t*)Address;
        Address += 2;
    }
}

/**************************************************************************
*   Function name: ReadConfiguratioStruct
*   Returns      : void
*   Parameters   : void
*   Purpose      : ������ ������������ �� ����
****************************************************************************/
void  ReadCalibrationStruct(void)
{
  uint16_t * pStruct;
  uint32_t Address = 0x00;
  
  // ����� ���������� �� ������ ���������
  
    pStruct = (uint16_t *)&DeviceRegister.DeviceCalibRegion;
    while (Address < (EEPROM_START_ADDRESS + sizeof(DEVICE_CONFIG_REGION)+ sizeof(DEVICE_CALIB_REGION)))
    {
        *pStruct++ = *(__IO uint16_t*)Address;
        Address += 2;
    }
}





























































