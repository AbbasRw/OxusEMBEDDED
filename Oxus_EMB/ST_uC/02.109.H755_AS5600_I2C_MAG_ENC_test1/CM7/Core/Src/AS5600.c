/*
 * AS5600.c
 *
 *  Created on: 5 Şub 2026
 *      Author: abbas.raimkulov
 */

#include "AS5600.h"




HAL_StatusTypeDef AS5600_Read8(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *val)
{
    return HAL_I2C_Mem_Read(hi2c,
                            AS5600_I2C_ADDR,
                            reg,
                            I2C_MEMADD_SIZE_8BIT,
                            val,
                            1,
                            20);
}

HAL_StatusTypeDef AS5600_Write8(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t val)
{
    return HAL_I2C_Mem_Write(hi2c,
                             AS5600_I2C_ADDR,
                             reg,
                             I2C_MEMADD_SIZE_8BIT,
                             &val,
                             1,
                             20);
}

/* Read 12-bit value stored as:
   reg_high: bits(11:8) in low nibble
   reg_low : bits(7:0)
*/
HAL_StatusTypeDef AS5600_Read12(I2C_HandleTypeDef *hi2c, uint8_t reg_high, uint16_t *val12)
{
    uint8_t buf[2];
    HAL_StatusTypeDef st = HAL_I2C_Mem_Read(hi2c,
                                           AS5600_I2C_ADDR,
                                           reg_high,
                                           I2C_MEMADD_SIZE_8BIT,
                                           buf,
                                           2,
                                           20);
    if (st != HAL_OK) return st;

    uint8_t high_nibble = (uint8_t)(buf[0] & 0x0Fu);       // keep only bits(3:0)
    uint8_t low_byte    = buf[1];

    *val12 = (uint16_t)(((uint16_t)high_nibble << 8) | low_byte); // 12-bit in [11:0]
    return HAL_OK;
}

/* Write 12-bit value into reg_high/reg_low with datasheet layout */
HAL_StatusTypeDef AS5600_Write12(I2C_HandleTypeDef *hi2c, uint8_t reg_high, uint16_t val12)
{
    uint16_t v = AS5600_Mask12(val12);

    uint8_t buf[2];
    buf[0] = (uint8_t)((v >> 8) & 0x0Fu);   // only low nibble used in high register
    buf[1] = (uint8_t)(v & 0xFFu);          // full low byte

    return HAL_I2C_Mem_Write(hi2c,
                             AS5600_I2C_ADDR,
                             reg_high,
                             I2C_MEMADD_SIZE_8BIT,
                             buf,
                             2,
                             20);
}

/* Generic 16-bit read/write (CONF etc.) where full bytes are used */
HAL_StatusTypeDef AS5600_Read16(I2C_HandleTypeDef *hi2c, uint8_t reg_high, uint16_t *val16)
{
    uint8_t buf[2];
    HAL_StatusTypeDef st = HAL_I2C_Mem_Read(hi2c,
                                           AS5600_I2C_ADDR,
                                           reg_high,
                                           I2C_MEMADD_SIZE_8BIT,
                                           buf,
                                           2,
                                           20);
    if (st != HAL_OK) return st;

    *val16 = (uint16_t)(((uint16_t)buf[0] << 8) | buf[1]);
    return HAL_OK;
}

HAL_StatusTypeDef AS5600_Write16(I2C_HandleTypeDef *hi2c, uint8_t reg_high, uint16_t val16)
{
    uint8_t buf[2];
    buf[0] = (uint8_t)(val16 >> 8);
    buf[1] = (uint8_t)(val16 & 0xFFu);

    return HAL_I2C_Mem_Write(hi2c,
                             AS5600_I2C_ADDR,
                             reg_high,
                             I2C_MEMADD_SIZE_8BIT,
                             buf,
                             2,
                             20);
}

/* -------- High-level 12-bit setters -------- */

HAL_StatusTypeDef AS5600_SetZeroPosition(I2C_HandleTypeDef *hi2c, uint16_t zpos_12b)
{
    return AS5600_Write12(hi2c, AS5600_REG_ZPOS_H, zpos_12b);
}

HAL_StatusTypeDef AS5600_SetMaxPosition(I2C_HandleTypeDef *hi2c, uint16_t mpos_12b)
{
    return AS5600_Write12(hi2c, AS5600_REG_MPOS_H, mpos_12b);
}

HAL_StatusTypeDef AS5600_SetMaxAngle(I2C_HandleTypeDef *hi2c, uint16_t mang_12b)
{
    return AS5600_Write12(hi2c, AS5600_REG_MANG_H, mang_12b);
}

/* -------- High-level 12-bit getters -------- */

HAL_StatusTypeDef AS5600_GetAngle12(I2C_HandleTypeDef *hi2c, uint16_t *angle_12b)
{
    return AS5600_Read12(hi2c, AS5600_REG_ANGLE_H, angle_12b);
}

HAL_StatusTypeDef AS5600_GetRawAngle12(I2C_HandleTypeDef *hi2c, uint16_t *raw_angle_12b)
{
    return AS5600_Read12(hi2c, AS5600_REG_RAW_ANGLE_H, raw_angle_12b);
}

/* -------- Diagnostics -------- */

HAL_StatusTypeDef AS5600_GetStatus(I2C_HandleTypeDef *hi2c, uint8_t *status)
{
    return AS5600_Read8(hi2c, AS5600_REG_STATUS, status);
}

HAL_StatusTypeDef AS5600_GetAGC(I2C_HandleTypeDef *hi2c, uint8_t *agc)
{
    return AS5600_Read8(hi2c, AS5600_REG_AGC, agc);
}

HAL_StatusTypeDef AS5600_GetMagnitude12(I2C_HandleTypeDef *hi2c, uint16_t *mag_12b)
{
    /* Magnitude in datasheet is 12-bit across 0x1B (11:8) and 0x1C (7:0) */
    return AS5600_Read12(hi2c, AS5600_REG_MAGNITUDE_H, mag_12b);
}

HAL_StatusTypeDef AS5600_GetCONF16(I2C_HandleTypeDef *hi2c, uint16_t *mag_12b)
{
    /* Magnitude in datasheet is 12-bit across 0x1B (11:8) and 0x1C (7:0) */
    return AS5600_Read16(hi2c, AS5600_REG_CONF_H, mag_12b);
}
