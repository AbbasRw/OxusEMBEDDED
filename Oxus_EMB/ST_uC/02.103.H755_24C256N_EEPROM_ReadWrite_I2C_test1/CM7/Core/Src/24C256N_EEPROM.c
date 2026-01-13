/*
 * 24C256N_EEPROM.c
 *
 *  Created on: 23 Ara 2025
 *      Author: abbas.raimkulov
 */


#include "24C256N_EEPROM.h"

extern I2C_HandleTypeDef hi2c2;

/* ===== Internal wait ===== */
static void EEPROM_WaitReady(void)
{
    while (HAL_I2C_IsDeviceReady(&hi2c2, EEPROM_I2C_ADDR, 10, EEPROM_TIMEOUT) != HAL_OK);
}

/* ===== Single byte write ===== */
HAL_StatusTypeDef EEPROM_WriteByte(uint16_t memAddr, uint8_t data)
{
    HAL_StatusTypeDef ret;

    ret = HAL_I2C_Mem_Write(
        &hi2c2,
        EEPROM_I2C_ADDR,
        memAddr,
        I2C_MEMADD_SIZE_16BIT,
        &data,
        1,
        EEPROM_TIMEOUT
    );

    EEPROM_WaitReady();
    return ret;
}

/* ===== Single byte read ===== */
HAL_StatusTypeDef EEPROM_ReadByte(uint16_t memAddr, uint8_t *data)
{
    return HAL_I2C_Mem_Read(&hi2c2,
        EEPROM_I2C_ADDR,
        memAddr,
        I2C_MEMADD_SIZE_16BIT,
        data,
        1,
        EEPROM_TIMEOUT);
}

/* ===== Buffer write (page-safe) ===== */
HAL_StatusTypeDef EEPROM_WriteBuffer(uint16_t memAddr, uint8_t *data, uint16_t len)
{
    HAL_StatusTypeDef ret;
    uint16_t bytesToWrite;

    while (len > 0)
    {
        bytesToWrite = EEPROM_PAGE_SIZE - (memAddr % EEPROM_PAGE_SIZE);
        if (bytesToWrite > len)
            bytesToWrite = len;

        ret = HAL_I2C_Mem_Write(
            &hi2c2,
            EEPROM_I2C_ADDR,
            memAddr,
            I2C_MEMADD_SIZE_16BIT,
            data,
            bytesToWrite,
            EEPROM_TIMEOUT
        );

        if (ret != HAL_OK)
            return ret;

        EEPROM_WaitReady();

        memAddr += bytesToWrite;
        data    += bytesToWrite;
        len     -= bytesToWrite;
    }

    return HAL_OK;
}

/* ===== Buffer read ===== */
HAL_StatusTypeDef EEPROM_ReadBuffer(uint16_t memAddr, uint8_t *data, uint16_t len)
{
    return HAL_I2C_Mem_Read(
        &hi2c2,
        EEPROM_I2C_ADDR,
        memAddr,
        I2C_MEMADD_SIZE_16BIT,
        data,
        len,
        EEPROM_TIMEOUT
    );
}
