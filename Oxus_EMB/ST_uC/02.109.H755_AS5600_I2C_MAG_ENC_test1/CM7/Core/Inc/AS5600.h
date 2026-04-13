/*
 * AS5600.h
 *
 *  Created on: 5 Şub 2026
 *      Author: abbas.raimkulov
 */

#ifndef INC_AS5600_H_
#define INC_AS5600_H_

#include "stm32h7xx_hal.h"
#include <stdint.h>
#include <stdbool.h>


#define AS5600_I2C_ADDR     	(0x36 << 1)
/* Register map (per datasheet image) */
#define AS5600_REG_ZMCO          0x00u

#define AS5600_REG_ZPOS_H        0x01u
#define AS5600_REG_ZPOS_L        0x02u

#define AS5600_REG_MPOS_H        0x03u
#define AS5600_REG_MPOS_L        0x04u

#define AS5600_REG_MANG_H        0x05u
#define AS5600_REG_MANG_L        0x06u

#define AS5600_REG_CONF_H        0x07u
#define AS5600_REG_CONF_L        0x08u

#define AS5600_REG_STATUS        0x0Bu
#define AS5600_REG_RAW_ANGLE_H   0x0Cu
#define AS5600_REG_RAW_ANGLE_L   0x0Du
#define AS5600_REG_ANGLE_H       0x0Eu
#define AS5600_REG_ANGLE_L       0x0Fu

#define AS5600_REG_AGC           0x1Au
#define AS5600_REG_MAGNITUDE_H   0x1Bu
#define AS5600_REG_MAGNITUDE_L   0x1Cu

#define AS5600_REG_BURN          0xFFu  // NOT used here (OTP)

/* Status bits in STATUS register (0x0B) */
#define AS5600_STATUS_MD         (1u << 5)  // Magnet detected
#define AS5600_STATUS_ML         (1u << 4)  // Magnet too weak
#define AS5600_STATUS_MH         (1u << 3)  // Magnet too strong


typedef struct
{
    uint16_t raw_angle;   // 0–4095
    float angle_deg;      // 0–360°
    uint8_t comm_ok;
    uint8_t magnet_detected;
    uint8_t magnet_too_weak;
    uint8_t magnet_too_strong;

    uint8_t agc_value_read;
    uint16_t mag_value_read;

    uint8_t	conf_PM;
    uint8_t	conf_HYST;
    uint8_t	conf_OUTS;
    uint8_t	conf_PWMF;
    uint8_t	conf_SF;
    uint8_t	conf_FTH;
    uint8_t conf_WD;


} as5600_t;



/* Low-level helpers */
HAL_StatusTypeDef AS5600_Read8 (I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *val);
HAL_StatusTypeDef AS5600_Write8(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t val);

HAL_StatusTypeDef AS5600_Read12(I2C_HandleTypeDef *hi2c, uint8_t reg_high, uint16_t *val12);
HAL_StatusTypeDef AS5600_Write12(I2C_HandleTypeDef *hi2c, uint8_t reg_high, uint16_t val12);

HAL_StatusTypeDef AS5600_Read16(I2C_HandleTypeDef *hi2c, uint8_t reg_high, uint16_t *val16);
HAL_StatusTypeDef AS5600_Write16(I2C_HandleTypeDef *hi2c, uint8_t reg_high, uint16_t val16);

/* High-level setters (12-bit) */
HAL_StatusTypeDef AS5600_SetZeroPosition(I2C_HandleTypeDef *hi2c, uint16_t zpos_12b);
HAL_StatusTypeDef AS5600_SetMaxPosition (I2C_HandleTypeDef *hi2c, uint16_t mpos_12b);
HAL_StatusTypeDef AS5600_SetMaxAngle    (I2C_HandleTypeDef *hi2c, uint16_t mang_12b);

/* High-level getters (12-bit) */
HAL_StatusTypeDef AS5600_GetAngle12     (I2C_HandleTypeDef *hi2c, uint16_t *angle_12b);
HAL_StatusTypeDef AS5600_GetRawAngle12  (I2C_HandleTypeDef *hi2c, uint16_t *raw_angle_12b);

/* Diagnostics */
HAL_StatusTypeDef AS5600_GetStatus      (I2C_HandleTypeDef *hi2c, uint8_t *status);
HAL_StatusTypeDef AS5600_GetAGC         (I2C_HandleTypeDef *hi2c, uint8_t *agc);
HAL_StatusTypeDef AS5600_GetMagnitude12 (I2C_HandleTypeDef *hi2c, uint16_t *mag_12b);
HAL_StatusTypeDef AS5600_GetCONF16		(I2C_HandleTypeDef *hi2c, uint16_t *mag_12b);
/* Utility */
static inline uint16_t AS5600_Mask12(uint16_t v) { return (uint16_t)(v & 0x0FFFu); }


#endif /* INC_AS5600_H_ */
