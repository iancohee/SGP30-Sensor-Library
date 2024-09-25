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
* @file    : sgp30.h
* @author  : Ian Cohee
* @date    : 2024-30-09
* @version : v0.1.0
* @brief   : SGP30 Gas Sensor Module Header
*
* Header file containing declarations for the SGP30 module.
* The Gas Sensor module provides functionality to measure potentially 
* harmful gas levels in the air.
*/

#ifndef __SGP30_HEADER__
#define __SGP30_HEADER__

#include "sgp30_defs.h"

#include <stdlib.h>


/* CPP guard */
#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Allocate a Sgp30 instance.
 *
 * @retval Pointer to Sgp30 instance.
 */
Sgp30Dev* sgp30_alloc(void);

/**
 * @brief Set device read function pointer
 * 
 * @param[in]  dev   : Pointer to a sgp30dev device
 * @param[in]  fptr  : Function pointer to read function
 * 
 * @return Success settings read function
 * @retval 0 on success
 * @retval < 0 on failure
 */
int8_t sgp30_set_read_fptr(Sgp30Dev* dev, sgp30_read_fptr_t fptr);

/**
 * @brief Set device write function pointer
 * 
 * @param[in]  dev   : Pointer to a sgp30dev device
 * @param[in]  fptr  : Function pointer to write function
 * 
 * @return Success settings write function
 * @retval 0 on success
 * @retval < 0 on failure
 */
int8_t sgp30_set_write_fptr(Sgp30Dev* dev, sgp30_write_fptr_t fptr);

/** 
 * @brief Set device delay (in us) function pointer
 * 
 * @param[in]  dev   : Pointer to a sgp30dev device
 * @param[in]  fptr  : Function pointer to delay function
 * 
 * @return Success settings delay function
 * @retval 0 on success
 * @retval < 0 on failure
 */
int8_t sgp30_set_delay_us_fptr(Sgp30Dev* dev, sgp30_delay_us_fptr_t fptr);

/**
 * @brief Set I2C read address
 * 
 * Default is as defined in the datasheet.
 * 
 * @param[in] addr_read : Byte address of the new read register
 * 
 * @return Success of setting register value
 * @retval 0 on success
 * @retval < 0 on failure
 */
int8_t sgp30_set_i2c_addr_read(Sgp30Dev* dev, uint8_t addr_read);

/**
 * @brief Set I2C write address
 * 
 * Default is as defined in the datasheet.
 * 
 * @param[in] addr_write : Byte address of the new read register
 * 
 * @return Success of setting register value
 * @retval 0 on success
 * @retval < 0 on failure
 */
int8_t sgp30_set_i2c_addr_write(Sgp30Dev* dev, uint8_t addr_write);

/**
 * @brief Initialize Sgp30 instance.
 * 
 * Read Chip ID and initialize measurement data.
 * 
 * @param[in]  dev : Pointer to a sgp30dev instance
 * 
 * @return Success of initialization
 * @retval 0 on success
 * @retval < 0 on failure
 */
int8_t sgp30_init(Sgp30Dev* dev);

/**
 * @brief Free a Sgp30dev instance.
 *
 * @param[in]  dev : Pointer to a Sgp30Dev pointer
 *
 * @return Void.
 */
void sgp30_free(Sgp30Dev** dev);

/**
 * @brief Read data from Sgp30 instance using I2C.
 * 
 * CRC checksum is not calculated for bytes read.
 *
 * @param[in]   dev      : Pointer to a Sgp30Dev
 * @param[out]  data     : Pointer to byte array to read data into
 * @param[in]   data_len : Number of bytes to read
 *
 * @return Success of device read
 * @retval 0 on success
 * @retval < 0 on failure
 */
int8_t sgp30_read(Sgp30Dev* dev, uint8_t* data, uint32_t data_len);

/**
 * @brief Read a word (16-bit) from sensor and check the CRC (third byte).
 * 
 * sgp30_read_word only reads 24-bits: 1 16-bit word + 1 8-bit CRC
 * 
 * @param[in]   dev      : Pointer to Sgp30 instance.
 * @param[out]  data     : Data buffer to read into.
 * 
 * @return Success of device read
 * @retval 0 on success
 * @retval < 0 on failure
 * */
int8_t sgp30_read_word(Sgp30Dev* dev, uint8_t* data);

/**
 * @brief Read two words (16-bit each) from sensor and check the CRCs (bytes 2 & 5).
 * 
 * @param[in]   dev      : Pointer to Sgp30 instance.
 * @param[out]  data     : Data buffer to read into.
 * 
 * @return Success of device read
 * @retval 0 on success
 * @retval < 0 on failure
*/
int8_t sgp30_read_double_word(Sgp30Dev* dev, uint8_t* data);

/**
 * @brief Write command to SGP30 instance using I2C.
 *
 * @param[in]  dev      : Pointer to a Sgp30 instance
 * @param[in]  command  : Pointer to byte array containing command bytes
 * @param[in]  data_len : Number of bytes to read
 *
 * @return Success of device read
 * @retval 0 on success
 * @retval < 0 on failure
 */
int8_t sgp30_write(Sgp30Dev* dev, const uint8_t* command, uint32_t data_len);

/**
 * @brief Write command to SGP30 instance and receive output.
 * 
 * A 0.5ms pause occurs between write and read, per the datasheet. 
 * 
 * This function uses a single buffer, it is the caller's responsibility to make
 * sure the buffer is big enough for MAX(command, response) length.
 * 
 * The command is overwritten by the response.
 *
 * @param[in]      dev         : Pointer to a Sgp30 instance
 * @param[in,out]  data        : Pointer to byte array containing command bytes
 * @param[in]      command_len : Length of the command being sent
 * @param[in]      data_len    : Number of bytes to read after sending the command
 *
 * @return Success of device read
 * @retval 0 on success
 * @retval < 0 on failure
 */
int8_t sgp30_trx(Sgp30Dev* dev,
                 uint8_t* data,
                 uint32_t command_len,
                 uint32_t data_len);

/**
 * @brief Get ID from the chip.
 * 
 * Result is stored in sgp30->chip_id, and can be accessed via sgp30_get_id()
 * 
 * This implementation was take from 
 * https://github.com/adafruit/Adafruit_SGP30/blob/master/Adafruit_SGP30.cpp
 *
 * @param[in]  dev : Pointer to sgp30dev instance.
 *
 * @return Success of device read
 * @retval 0 on success
 * @retval < 0 on failure
 */
int8_t sgp30_read_chip_id(Sgp30Dev* dev);

/**
 * @brief Get ID from instance.
 *
 * @param[in]  dev : Pointer to sgp30dev instance.
 *
 * @return 48-bit chip ID as uint64_t.
 */
uint64_t sgp30_chip_id(Sgp30Dev* dev);

/**
 * @brief Initiate a self test.
 * 
 * Run a built-in test, the result of which should be 0xD400.
 * If the self test does not return this value, this function
 * return false.
 * 
 * This implementation was take from 
 * https://github.com/adafruit/Adafruit_SGP30/blob/master/Adafruit_SGP30.cpp
 *
 * @param[in]  dev : Pointer to Sgp30dev instance.
 *
 * @return Success of device read
 * @retval 0 on success
 * @retval < 0 on failure
 */
int8_t sgp30_measure_test(Sgp30Dev* dev);

/**
 * @brief Get_baseline configuration.
 * 
 * @param[in]      dev  : Pointer to Sgp30Dev instance.
 * @param[in,out]  data : Pointer to uint8_t array to store data 
 * 
 * @return Success of device read
 * @retval 0 on success
 * @retval < 0 on failure
 */
int8_t sgp30_get_baseline(Sgp30Dev* dev, uint8_t* data);

/**
 * @brief Measure air quality.
 * 
 * @param[in]     dev      : Pointer to Sgp30Dev instance.
 * @param[in,out] readings : Pointer to Sgp30DevReadings instance.
 * 
 * @return Success of air quality measurement
 * @retval 0 on success
 * @retval < 0 on failure
*/
int8_t sgp30_measure_air_quality(Sgp30Dev* dev, Sgp30DevReadings* readings);

/**
 * @brief Soft reset the chip.
 * 
 * @param[in]  dev : Pointer to Sgp30 instance;
 * 
 * @return soft reset command set successfully
 * @retval 0 on success
 * @retval < 0 on failure
*/
int8_t sgp30_soft_reset(Sgp30Dev* dev);

/**
 * @brief Generate CRC checksum for given data.
 * 
 * This device expects each command to include a CRC byte.
 * Exceptions are the word (2-byte) commands that already
 * include a 3-bit checksum.
 * 
 * This implementation was take from 
 * https://github.com/adafruit/Adafruit_SGP30/blob/master/Adafruit_SGP30.cpp
 *
 * @param[in]  data    : Pointer to byte array to checksum.
 * @param[in]  datalen : Length of byte array to use in CRC
 *
 * @return CRC Checksum byte
 * @retval checksum byte value on success
 * @retval 0 on error
 */
uint8_t sgp30_generate_crc(uint8_t* data, uint8_t datalen);

#ifdef __cplusplus
}
#endif

#endif // __SGP30_HEADER__
