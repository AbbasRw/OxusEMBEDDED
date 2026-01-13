/*
 * 24C256N_EEPROM.h
 *
 *  Created on: 23 Ara 2025
 *      Author: abbas.raimkulov
 */

#ifndef INC_24C256N_EEPROM_H_
#define INC_24C256N_EEPROM_H_

#include "stm32h7xx_hal.h"

#define EEPROM_I2C_ADDR     (0x50 << 1)   // HAL uses 8-bit address
#define EEPROM_PAGE_SIZE    64
#define EEPROM_TIMEOUT      100

HAL_StatusTypeDef EEPROM_WriteByte(uint16_t memAddr, uint8_t data);
HAL_StatusTypeDef EEPROM_ReadByte(uint16_t memAddr, uint8_t *data);

HAL_StatusTypeDef EEPROM_WriteBuffer(uint16_t memAddr, uint8_t *data, uint16_t len);
HAL_StatusTypeDef EEPROM_ReadBuffer(uint16_t memAddr, uint8_t *data, uint16_t len);


#endif /* INC_24C256N_EEPROM_H_ */
