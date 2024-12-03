#ifndef SAVEREADCONFIGURATIONTOFLASHMCU_H
#define SAVEREADCONFIGURATIONTOFLASHMCU_H

#include "main.h"

/* EEPROM start address in Flash */
#define EEPROM_START_ADDRESS    ((uint32_t)0x08008000 ) // EEPROM emulation start address:

/* --------------------- -= Прототипы функций =- --------------------- */
   
void WriteConfiguratioStruct(DEVICE_CONFIG_REGION* DeviceConfigRegion,DEVICE_CALIB_REGION* DeviceCalibRegion);
void ReadConfiguratioStruct(void);
void ReadCalibrationStruct(void);

#endif /* SAVEREADCONFIGURATIONTOFLASHMCU_H */












