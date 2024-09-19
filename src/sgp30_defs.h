/**
* Copyright (c) 2024 Ian Cohee
*
* BSD-3-Clause
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its
*    contributors may be used to endorse or promote products derived from
*    this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
* IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
* @file    : sgp30_defs.h
* @author  : Ian Cohee
* @date    : 2024-30-09
* @version : v0.1.0
* @brief   : SGP30 Gas Sensor Module Header
*
* Header file containing declarations for the SGP30 module.
* The Gas Sensor module provides functionality to measure potentially 
* harmful gas levels in the air.
* 
* https://www.mouser.com/pdfdocs/Sensirion_Gas_Sensors_SGP30_Datasheet_EN-1148053.pdf
* 
*/

#ifndef __SGP30_DEFS_HEADER__
#define __SGP30_DEFS_HEADER__

#ifdef __KERNEL__
#include <linux/types.h>
#include <linux/kernel.h>
#else
#include <stdint.h>
#include <stddef.h>
#endif

/*****************/
/* Common Macros */
/*****************/
#ifdef __KERNEL__
#if !defined(UINT8_C) && !defined(INT8_C)
#define INT8_C(x)    S8_C(x)
#define UINT8_C(x)   U8_C(x)
#endif

#if !defined(UINT16_C) && !defined(INT16_C)
#define INT16_C(x)   S16_C(x)
#define UINT16_C(x)  U16_C(x)
#endif

#if !defined(INT32_C) && !defined(UINT32_C)
#define INT32_C(x)   S32_C(x)
#define UINT32_C(x)  U32_C(x)
#endif

#if !defined(INT64_C) && !defined(UINT64_C)
#define INT64_C(x)   S64_C(x)
#define UINT64_C(x)  U64_C(x)
#endif
#endif

/* C standard macros */
#ifndef NULL
#ifdef __cplusplus
#define NULL         0
#else
#define NULL         ((void *) 0)
#endif
#endif

/**
 * @brief I2C addresses for this device.
*/
#define SGP30_I2C_ADDR                              UINT8_C(0x58)
#define SGP30_I2C_ADDR_READ                         UINT8_C((0x58 << 1) | 0x01)
#define SGP30_I2C_ADDR_WRITE                        UINT8_C((0x58 << 1) | 0x00)

/**
 * @brief Misc Constants
*/
#define SGP30_FEATURESET                            UINT16_C(0x0020)
#define SGP30_CRC8_POLYNOMIAL                       UINT8_C(0x31)
#define SGP30_CRC8_INIT                             UINT8_C(0xFF)
#define SGP30_WORD_LEN                              UINT8_C(2)
#define SGP30_DEFAULT_TIMEOUT                       UINT8_C(0X0A)

/**
 * @brief Get Serial ID
*/
#define SPG30_GET_SERIAL_ID_CMD                     UINT16_C(0x3682)
#define SPG30_GET_SERIAL_ID_CMD_LEN                 UINT8_C(0x02)
#define SGP30_GET_SERIAL_ID_RESP_LEN                UINT8_C(0x09)

/**
 * @brief Init_air_quality command
*/
#define SGP30_INIT_AIR_QUALITY_CMD                  UINT16_C(0x2003)
#define SGP30_INIT_AIR_QUALITY_CMD_LEN              UINT8_C(0x02)
#define SGP30_INIT_AIR_QUALITY_MAX_MS               SGP30_DEFAULT_TIMEOUT

/**
 * @brief Measure_air_quality command
*/
#define SGP30_MEASURE_AIR_QUALITY_CMD               UINT16_C(0x2008)
#define SGP30_MEASURE_AIR_QUALITY_CMD_LEN           UINT8_C(0x02)
#define SGP30_MEASURE_AIR_QUALITY_RESP_LEN          UINT8_C(0x06)
#define SGP30_MEASURE_AIR_QUALITY_MAX_MS            UINT8_C(0x0C)

/**
 * @brief Get_baseline command
*/
#define SGP30_GET_BASELINE_CMD                      UINT16_C(0x2015)
#define SGP30_GET_BASELINE_CMD_LEN                  UINT8_C(0x02)
#define SGP30_GET_BASELINE_CMD_BYTE_1               UINT8_C(0x20)
#define SGP30_GET_BASELINE_CMD_BYTE_2               UINT8_C(0x15)
#define SGP30_GET_BASELINE_CMD_RESP_LEN             UINT8_C(0x06)
#define SGP30_GET_BASELINE_CMD_MAX_MS               SGP30_DEFAULT_TIMEOUT

/**
 * @brief Set_humidity command
 */
#define SGP30_SET_HUMIDITY_CMD                      UINT16_C(0x2061)
#define SGP30_SET_HUMIDITY_CMD_LEN                  UINT8_C(0x02)
#define SGP30_SET_HUMIDITY_PARAM_LEN                UINT8_C(0x03)
#define SGP30_SET_HUMIDITY_CMD_MAX_MS               SGP30_DEFAULT_TIMEOUT

/**
 * @brief Measure_test command
*/
#define SGP30_MEASURE_TEST_CMD                      UINT16_C(0x2032)
#define SGP30_MEASURE_TEST_CMD_LEN                  UINT8_C(0x02)
#define SGP30_MEASURE_TEST_RESP_LEN                 UINT8_C(0x03)
#define SGP30_MEASURE_TEST_MAX_MS                   UINT8_C(0xDC)
#define SGP30_MEASURE_TEST_PASS_VAL                 UINT16_C(0xD400)

/**
 * @brief "Soft reset" is implemented using General Call
 */
#define SGP30_SOFT_REST_CMD_GEN_CALL_ADDR           UINT16_C(0x0006)
#define SGP30_SOFT_REST_CMD_GEN_CALL_ADDR_LEN       UINT8_C(0x02)

/* Generic return code definitions */
#define SGP30_OK                                    INT8_C(0)

/* Error code definitions */
/* NULL ptr passed */
#define SGP30_E_NULL_PTR                            INT8_C(-1)

/* Malloc */
#define SGP30_E_NOMEM                               INT8_C(-2)

/* Communication errors */
#define SGP30_E_READ                                INT8_C(-3)
#define SGP30_E_WRITE                               INT8_C(-4)

/* Device not found */
#define SGP30_E_DEV_NOT_FOUND                       INT8_C(-5)

/* Self-test failed */
#define SGP30_E_SELF_TEST                           INT8_C(-6)

/* CRC mismatch */
#define SGP30_E_CRC_1                               INT8_C(-7)
#define SGP30_E_CRC_2                               INT8_C(-8)
#define SGP30_E_CRC_3                               INT8_C(-9)

/* TRX - transmit then receive */
#define SGP30_E_TRX                                 INT8_C(-10)

/* Function pointer setter functions may use this */
#define SGP30_E_NULL_FPTR                           INT8_C(-11)
#define SGP30_E_NULL_DATA_PTR                       INT8_C(-12)

/**
 * @brief Interface return types and definitions
 */
#define SGP30_INTF_RET_TYPE                         int8_t
#define SGP30_INTF_RET_SUCCESS                      INT8_C(0)

/********************************************************* */
/**               Function Pointers                       */
/********************************************************* */

/**
 * @brief Bus communication function pointer which should be mapped to
 * the platform specific read functions of the user
 *
 * @param[in]     reg_addr : 8bit register address of the sensor
 * @param[out]    reg_data : Data from the specified address
 * @param[in]     length   : Length of the reg_data array
 * @param[in,out] intf_ptr : Void pointer that can enable the linking of descriptors
 *                           for interface related callbacks
 * 
 * @return Success of read operation
 * @retval 0 for Success
 * @retval Non-zero for Failure
 */
typedef SGP30_INTF_RET_TYPE (*sgp30_read_fptr_t)(uint8_t reg_addr, 
                                                 uint8_t *reg_data,
                                                 uint32_t length,
                                                 void *intf_ptr);

/**
 * @brief Bus communication function pointer which should be mapped to
 * the platform specific write functions of the user
 *
 * @param[in]     reg_addr : 8bit register address of the sensor
 * @param[out]    reg_data : Data to the specified address
 * @param[in]     length   : Length of the reg_data array
 * @param[in,out] intf_ptr : Void pointer that can enable the linking of descriptors
 *                           for interface related callbacks
 * @retval 0 for Success
 * @retval Non-zero for Failure
 *
 */
typedef SGP30_INTF_RET_TYPE (*sgp30_write_fptr_t)(uint8_t reg_addr,
                                                  const uint8_t *reg_data,
                                                  uint32_t length,
                                                  void *intf_ptr);

/**
 * @brief Delay function pointer which should be mapped to
 * delay function of the user
 *
 * @param period - The time period in microseconds
 * @param[in,out] intf_ptr : Void pointer that can enable the linking of descriptors
 *                           for interface related callbacks
 */
typedef void (*sgp30_delay_us_fptr_t)(uint32_t period, void *intf_ptr);

/**
 * @brief Sgp30Dev definition
*/
typedef struct sgp30_dev {
    /* Chip Id */
    uint64_t chip_id;

    /**
     * The interface pointer is used to enable the user
     * to link their interface descriptors for reference during the
     * implementation of the read and write interfaces to the
     * hardware.
     */
    void *intf_ptr;

    /* Read function pointer */
    sgp30_read_fptr_t read;

    /* Write function pointer */
    sgp30_write_fptr_t write;

    /* us delay function pointer */
    sgp30_delay_us_fptr_t delay_us;

    /* Generic I2C address */
    uint8_t i2c_addr;

    /* Read I2C address */
    uint8_t i2c_addr_read;

    /* Write I2C address*/
    uint8_t i2c_addr_write;
} Sgp30Dev;

/**
 * @brief A type for returning CO2 and TVOC readings.
*/
typedef struct sgp30_readings {
    /* Raw reading */
    uint64_t raw;

    /* CO2 and TVOC reading */
    uint32_t co2_tvoc;

    /* CO2 reading from co2_tvoc */
    uint16_t co2;

    /* TVOC reading from co2_tvoc */
    uint16_t tvoc;
} Sgp30DevReadings;

#endif // __SGP30_DEFS_HEADER__
