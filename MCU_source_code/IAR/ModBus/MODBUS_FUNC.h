/******************************************************************************
*    Заголовочник к Modbus
******************************************************************************/


//=============================================================================
// Адреса в пакете
//=============================================================================


//=============================================================================
// Modbus функции
//=============================================================================
// Стандартые функции
#define READ_COILS               0x01   // Чтение нескольких регистров флагов, чтение состояния дискретных выходов (например реле, если "1" - обмотка реле запитана "0" - обесточена)
#define READ_DISCRETE_INPUTS     0x02   // Чтение нескольких дискретных входов
#define READ_HOLDING_REGISTERS   0x03   // Чтение нескольких регистров хранения
#define READ_INPUT_REGISTERS     0x04   // Чтение состояния аналоговых входов (например регистры результата преобразования ADC) 
#define WRITE_SINGLE_COIL        0x05   // Запись значения одного флага
#define WRITE_SINGLE_REGISTER    0x06   // Записть значения в один регистр хранения
#define WRITE_MULTIPLE_COIL      0x0F
#define WRITE_MULTIPLE_REGISTER  0x10   // Запись нескольких регистров
// Пользовательские функции
#define WRITE_SOURCE_CONTROL_REGISTER   0x64   // Запись регистра управления источником
#define WRITE_ELLOAD_CONTROL_REGISTER   0x65   // Запись регистра управления электронной нагрузкой 

//=============================================================================
// Сообщение об ошибках
//=============================================================================
#define ILLEGAL_FUNCTION           0x01    // недопустимая функция
#define ILLEGAL_DATA_ADDRESS       0x02    // недопустимый адрес данных
#define ILLEGAL_DATA_VALUE         0x03    // недопустимое значение данных
#define SLAVE_DEVICE_FAILURE       0x04    // сбой ведомого устройства
#define ACKNOWLEDGE                0x05
#define SLAVE_DEVICE_BUSY          0x06 
#define MEMORY_PARITY_ERROR        0x08  
#define GATEWAY_PATH_UNAVAILABLE   0x0A
#define GATEWAY_TARGET_DEVICE_FAILED_TO_RESPOND 0x0B


//=============================================================================
// Прототыпы функций
//=============================================================================
uint8_t mb_ErrorMessage (uint8_t * packet, uint8_t error);             
uint8_t mb_ReadHoldingRegisters (uint8_t * packet, uint8_t packetLen);       // 0x03
uint8_t mb_WriteSingleCoil (uint8_t * packet, uint8_t packetLen);            // 0x05
uint8_t mb_WriteSingleRegister (uint8_t * packet, uint8_t packetLen);        // 0x06
uint8_t mb_WriteMultipleRegister (uint8_t * packet, uint8_t packetLen);      // 0x10
uint8_t mb_WriteLoadControlRegister (uint8_t * packet, uint8_t packetLen);   // 0x64























