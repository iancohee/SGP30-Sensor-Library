/**
* Copyright (c) 2024 Ian cohee
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
* @file    : sgp30.c
* @author  : Ian Cohee
* @date    : 2024-30-09
* @version : v0.1.0
* @brief   : SGP30 Gas Sensor Module Implementation
* 
* Code file containing implementations for the GY-SGP30 module.
* The Gas Sensor module provides functionality to measure potentially harmful
* gas levels in the air.
*/

#include "sgp30.h"

#ifdef __KERNEL__
#include <linux/string.h>
#else
#include <string.h>
#endif

#define TAG "sgp30"

/**
 * static declarations
 */

/* Internal API to check SGP30 for NULL pointers */
static int8_t sgp30_null_ptr_check(const Sgp30Dev *dev);

/* Internal API to delay for period useconds */
static int8_t sgp30_delay_us(Sgp30Dev *dev, uint32_t period);

/**
 * public API
 */

/* Allocate SGP30 object and return it or NULL */
Sgp30Dev* sgp30_alloc(void)
{
    Sgp30Dev *dev = malloc(sizeof(Sgp30Dev));
    if (dev == NULL) {
        return NULL;
    }
    dev->i2c_addr = SGP30_I2C_ADDR;
    dev->i2c_addr_read = SGP30_I2C_ADDR_READ;
    dev->i2c_addr_write = SGP30_I2C_ADDR_WRITE;
    return dev;
}

/**
 * @brief Set device read function pointer
 * 
 * @param[in]  dev  : Pointer to a sgp30dev device
 * @param[in]  fptr : Function pointer to read function
 * 
 * @return Success settings read function
 * @retval 0 on success
 * @retval < 0 on failure
 */
uint8_t sgp30_set_read_fptr(Sgp30Dev *dev, sgp30_read_fptr_t fptr)
{
    if (sgp30_null_ptr_check(dev) == SGP30_E_NULL_PTR || fptr == NULL) {
        return SGP30_E_NULL_PTR;
    }
    dev->read = fptr;
    return SGP30_OK;
}

/**
 * @brief Set device write function pointer
 * 
 * @param[in]  dev  : Pointer to a sgp30dev device
 * @param[in]  fptr : Function pointer to write function
 * 
 * @return Success settings write function
 * @retval 0 on success
 * @retval < 0 on failure
 */
uint8_t sgp30_set_write_fptr(Sgp30Dev *dev, sgp30_write_fptr_t fptr)
{
    if (sgp30_null_ptr_check(dev) == SGP30_E_NULL_PTR || fptr == NULL) {
        return SGP30_E_NULL_PTR;
    }
    dev->write = fptr;
    return SGP30_OK;
}

/** 
 * @brief Set device delay (in us) function pointer
 * 
 * @param[in]  dev  : Pointer to a sgp30dev device
 * @param[in]  fptr : Function pointer to delay function
 * 
 * @return Success settings delay function
 * @retval 0 on success
 * @retval < 0 on failure
 */
uint8_t sgp30_set_delay_us_fptr(Sgp30Dev *dev, sgp30_delay_us_fptr_t fptr)
{
    if (sgp30_null_ptr_check(dev) == SGP30_E_NULL_PTR || fptr == NULL) {
        return SGP30_E_NULL_PTR;
    }
    dev->delay_us = fptr;
    return SGP30_OK;
}

/**
 * @brief Initialize Sgp30 instance.
 * Read Chip ID and initialize measurement data.
 * 
 * @param[in]  dev : Pointer to a sgp30dev instance
 * 
 * @return Success of initialization
 * @retval 0 on success
 * @retval < 0 on failure
 */
uint8_t sgp30_init(Sgp30Dev *dev)
{
    if (dev == NULL) {
        return SGP30_E_NULL_PTR;
    }

    // chip takes a max. of 0.6ms to start after power up
    sgp30_delay_us(dev, 600);
    if(sgp30_read_chip_id(dev) < 0) {
        return SGP30_E_DEV_NOT_FOUND;
    }

    if(sgp30_measure_test(dev) < 0) {
        return SGP30_E_SELF_TEST;
    }

    uint8_t data[2];
    data[0] = (uint8_t)((SGP30_INIT_AIR_QUALITY_CMD & 0xFF00) >> 8);
    data[1] = (uint8_t)(SGP30_INIT_AIR_QUALITY_CMD & 0x00FF);
    if(sgp30_write(dev, data, SGP30_INIT_AIR_QUALITY_CMD_LEN) < 0) {
        return SGP30_E_WRITE;
    }

    return SGP30_OK;
}

/**
 * @brief Free a Sgp30 instance.
 *
 * @param[in]  dev  : Pointer to a Sgp30 instance
 *
 * @return Void.
 */
void sgp30_free(Sgp30Dev *dev)
{
    if (sgp30_null_ptr_check(dev) == SGP30_E_NULL_PTR) {
        return;
    }
    dev->intf_ptr = NULL;
    free(dev);
}

/**
 * @brief Read data from Sgp30 instance using I2C.
 * 
 * CRC checksum is not calculated for bytes read.
 *
 * @param[in]   dev      : Pointer to a Sgp30 instance
 * @param[out]  data     : Pointer to byte array to read data into
 * @param[in]   data_len : Number of bytes to read
 *
 * @return Success of device read
 * @retval 0 on success
 * @retval < 0 on failure
 */
int8_t sgp30_read(Sgp30Dev *dev, uint8_t *data, uint32_t data_len)
{
    if ((sgp30_null_ptr_check(dev) == SGP30_E_NULL_PTR) || (data == NULL)) {
        return SGP30_E_NULL_PTR;
    }
    return dev->read(dev->i2c_addr_read, data, data_len, dev->intf_ptr);
}

/**
 * @brief Read a word (16-bit) from sensor and check the CRC (third byte).
 * 
 * sgp30_read_word only reads 24-bits: 1 16-bit word + 1 8-bit CRC
 * 
 * @param[in]   dev     : Pointer to Sgp30 instance.
 * @param[out]  data    : Data buffer to read into.
 * 
 * @return True if read succeeds and CRC matches, false on failure.
*/
int8_t sgp30_read_word(Sgp30Dev *dev, uint8_t *data)
{
    if ((sgp30_null_ptr_check(dev) == SGP30_E_NULL_PTR) || (data == NULL)) {
        return SGP30_E_NULL_PTR;
    }
    if(sgp30_read(dev, data, SGP30_WORD_LEN + 1) != SGP30_OK) {
        return SGP30_E_READ;
    }
    uint8_t crc = sgp30_generate_crc(data, SGP30_WORD_LEN);
    if(crc != data[2]) {
        return SGP30_E_CRC_1;
    }
    return SGP30_OK;
}

/**
 * @brief Read two words (16-bit each) from sensor and check the CRCs (bytes 2 & 5).
 * 
 * @param[in]   dev     : Pointer to Sgp30 instance.
 * @param[out]  data    : Data buffer to read into.
 * 
 * @return True if read succeeds and CRC matches, false on failure.
*/
int8_t sgp30_read_double_word(Sgp30Dev *dev, uint8_t *data)
{
    if ((sgp30_null_ptr_check(dev) == SGP30_E_NULL_PTR) || (data == NULL)) {
        return SGP30_E_NULL_PTR;
    }
    if(sgp30_read(dev, data, (SGP30_WORD_LEN * 2) + 2) < 0) {
        return SGP30_E_READ;
    }
    uint8_t crc = sgp30_generate_crc(data, SGP30_WORD_LEN);
    if(crc != data[2]) {
        return SGP30_E_CRC_1;
    }
    crc = sgp30_generate_crc(&data[3], SGP30_WORD_LEN);
    if(crc != data[5]) {
        return SGP30_E_CRC_2;
    }
    return SGP30_OK;
}

/**
 * @brief Write command to SGP30 instance using I2C.
 *
 * @param[in]  dev      : Pointer to a Sgp30 instance
 * @param[in]  command  : Pointer to byte array containing command bytes
 * @param[in]  data_len : Number of bytes to read
 *
 * @return True on success, false on failure.
 */
int8_t sgp30_write(Sgp30Dev *dev, const uint8_t *command, uint32_t data_len)
{
    if ((sgp30_null_ptr_check(dev) == SGP30_E_NULL_PTR) || (command == NULL)) {
        return SGP30_E_NULL_PTR;
    }
    return dev->write(dev->i2c_addr_write, command, data_len, dev->intf_ptr);
}

/**
 * @brief Write command to SGP30 instance and receive output.
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
 * @return True on success, false on failure.
 */
int8_t sgp30_trx(
    Sgp30Dev *dev,
    uint8_t *data,
    uint32_t command_len,
    uint32_t data_len)
{
    if ((sgp30_null_ptr_check(dev) == SGP30_E_NULL_PTR) || (data == NULL)) {
        return SGP30_E_NULL_PTR;
    }
    int8_t status = sgp30_write(dev, data, command_len);
    if (status != SGP30_OK) {
        return status;
    }

    memset(data, 0, data_len);

    // 0.5ms delay
    sgp30_delay_us(dev, 1000000);
    status = sgp30_read(dev, data, data_len);
    return status;
}

/**
 * @brief Get ID from the chip.
 * Result is stored in sgp30->chip_id, and can be accessed via sgp30_get_id()
 * 
 * This implementation was take from 
 * https://github.com/adafruit/Adafruit_SGP30/blob/master/Adafruit_SGP30.cpp
 *
 * @param[in]  dev : Pointer to sgp30dev instance.
 *
 * @return 0 on success
 * @return < 0 on failure
 */
int8_t sgp30_read_chip_id(Sgp30Dev *dev)
{
    if (sgp30_null_ptr_check(dev) == SGP30_E_NULL_PTR) {
        return SGP30_E_NULL_PTR;
    }

    // response is [msb, lsb, crc, msb, lsb, crc, msb, lsb, crc]
    uint8_t data[SGP30_GET_SERIAL_ID_RESP_LEN] = { 0 };
    data[0] = (uint8_t)((SPG30_GET_SERIAL_ID_CMD & 0xFF00) >> 8);
    data[1] = (uint8_t)(SPG30_GET_SERIAL_ID_CMD & 0x00FF);
    if(sgp30_trx(
           dev,
           data,
           SPG30_GET_SERIAL_ID_CMD_LEN,
           SGP30_GET_SERIAL_ID_RESP_LEN) != SGP30_OK) {
        return SGP30_E_TRX;
    }
    uint8_t crc = sgp30_generate_crc(&data[0], 2);
    if(crc != data[2]) {
        return SGP30_E_CRC_1;
    }
    crc = sgp30_generate_crc(&data[3], 2);
    if(crc != data[5]) {
        return SGP30_E_CRC_2;
    }
    crc = sgp30_generate_crc(&data[6], 2);
    if(crc != data[8]) {
        return SGP30_E_CRC_3;
    }
    uint64_t new_chip_id = 0;
    for(uint32_t i = 0; i < SGP30_GET_SERIAL_ID_RESP_LEN; ++i) {
        if(i == 2 || i == 5 || i == 8) continue;
        new_chip_id |= data[i];
        new_chip_id <<= 8;
    }
    dev->chip_id = new_chip_id;
    return SGP30_OK;
}

/**
 * @brief Get ID from instance.
 *
 * @param[in]  dev  : Pointer to sgp30dev instance.
 *
 * @return 48-bit chip ID as uint64_t.
 */
uint64_t sgp30_chip_id(Sgp30Dev *dev)
{
    if (sgp30_null_ptr_check(dev) == SGP30_E_NULL_PTR) {
        return SGP30_E_NULL_PTR;
    }
    return dev->chip_id;
}

/**
 * @brief Initiate a self test.
 * Run a built-in test, the result of which should be 0xD400.
 * If the self test does not return this value, this function
 * return false.
 * 
 * This implementation was take from 
 * https://github.com/adafruit/Adafruit_SGP30/blob/master/Adafruit_SGP30.cpp
 *
 * @param[in]  dev  : Pointer to Sgp30dev instance.
 *
 * @return True of test passed, false if failed.
 */
int8_t sgp30_measure_test(Sgp30Dev *dev)
{
    if (sgp30_null_ptr_check(dev) == SGP30_E_NULL_PTR) {
        return SGP30_E_NULL_PTR;
    }
    uint8_t data[2] = {0};
    uint8_t resp[3] = {0};
    data[0] = (uint8_t)((SGP30_MEASURE_TEST_CMD & 0xFF00) >> 8);
    data[1] = (uint8_t)(SGP30_MEASURE_TEST_CMD & 0x00FF);

    sgp30_write(dev, data, SGP30_MEASURE_TEST_CMD_LEN);
    sgp30_delay_us(dev, SGP30_MEASURE_TEST_MAX_MS);
    sgp30_read(dev, resp, SGP30_MEASURE_TEST_RESP_LEN);

    uint8_t test_byte0 = (uint8_t)((SGP30_MEASURE_TEST_PASS_VAL & 0xFF00) >> 8);
    uint8_t test_byte1 = (uint8_t)(SGP30_MEASURE_TEST_PASS_VAL & 0x00FF);

    if(resp[0] != test_byte0 || resp[1] != test_byte1) {
        return SGP30_E_SELF_TEST;
    }
    if(resp[2] != sgp30_generate_crc(resp, SGP30_WORD_LEN)) {
        return SGP30_E_CRC_1;
    }
    return SGP30_OK;
}

/**
 * @brief Get_baseline configuration.
 * 
 * @param[in]       dev   : Pointer to Sgp30Dev instance.
 * @param[in,out]   data  : Pointer to uint8_t array to store data 
 * 
 * @return True on success, false on failure.
*/
int8_t sgp30_get_baseline(Sgp30Dev *dev, uint8_t *data)
{
    if (sgp30_null_ptr_check(dev) == SGP30_E_NULL_PTR) {
        return SGP30_E_NULL_PTR;
    }
    data[0] = SGP30_GET_BASELINE_CMD_BYTE_1;
    data[1] = SGP30_GET_BASELINE_CMD_BYTE_2;
    if(sgp30_write(dev, data, SGP30_GET_BASELINE_CMD_LEN) < 0) {
        return SGP30_E_WRITE;
    }
    sgp30_delay_us(dev, 500);
    if(sgp30_read_double_word(dev, data) < 0) {
        return SGP30_E_READ;
    }
    return SGP30_OK;
}

/**
 * @brief Measure air quality.
 * 
 * @param[in]      dev      : Pointer to Sgp30Dev instance.
 * @param[in,out]  readings : Pointer to Sgp30DevReadings instance.
 * 
 * @return Success of air quality measurement
 * @retval 0 on success
 * @retval < 0 on failure
*/
int8_t sgp30_measure_air_quality(Sgp30Dev *dev, Sgp30DevReadings *readings)
{
    if ((sgp30_null_ptr_check(dev) == SGP30_E_NULL_PTR) || (readings == NULL)) {
        return SGP30_E_NULL_PTR;
    }
    uint8_t data[6] = {0};

    data[0] = (uint8_t)((SGP30_MEASURE_AIR_QUALITY_CMD & 0xFF00) >> 8);
    data[1] = (uint8_t)(SGP30_MEASURE_AIR_QUALITY_CMD & 0x00FF);

    if(sgp30_trx(
           dev,
           data,
           SGP30_MEASURE_AIR_QUALITY_CMD_LEN,
           SGP30_MEASURE_AIR_QUALITY_RESP_LEN) < 0) {
        readings->co2_tvoc = 0xFFFFFFFF;
        return SGP30_E_TRX;
    }
    uint8_t crc = sgp30_generate_crc(data, SGP30_WORD_LEN);
    if(crc != data[2]) {
        readings->co2_tvoc = 0xFFFFFFFF;
        return SGP30_E_CRC_1;
    }

    crc = sgp30_generate_crc(&data[3], SGP30_WORD_LEN);
    if(crc != data[5]) {
        readings->co2_tvoc = 0xFFFFFFFF;
        return SGP30_E_CRC_2;
    }

    readings->co2 = (uint16_t)(((data[0] & 0xFF) << 8) | (data[1] & 0xFF));
    readings->tvoc = (uint16_t)(((data[3] & 0xFF) << 8) | (data[4] & 0xFF));
    readings->co2_tvoc =
        (uint32_t)(((readings->co2 & 0xFFFF) << 16) | (readings->tvoc & 0xFFFF));
    return SGP30_OK;
}

/**
 * @brief Soft reset the chip.
 * 
 * I2C "General Call" is used to soft reset the chip.
 * General Calls are seen by all devices on the I2C bus,
 * so this command may affect other devices.
 * 
 * @param[in]  dev  : Pointer to Sgp30 instance;
 * 
 * @return soft reset command set successfully
 * @retval 0 on success
 * @retval < 0 on failure
*/
uint8_t sgp30_soft_reset(Sgp30Dev *dev)
{
    if(sgp30_null_ptr_check(dev) == SGP30_E_NULL_PTR) {
        return SGP30_E_NULL_PTR;
    }
    uint8_t data[2] = { 0 };
    data[0] = (uint8_t)((SGP30_SOFT_REST_CMD_GEN_CALL_ADDR & 0xFF00) >> 8);
    data[1] = (uint8_t)((SGP30_SOFT_REST_CMD_GEN_CALL_ADDR) & 0x00FF);
    uint8_t status = sgp30_write(dev,
                                 data,
                                 SGP30_SOFT_REST_CMD_GEN_CALL_ADDR_LEN);
    return status;
}

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
 * @return CRC checksum byte.
 */
uint8_t sgp30_generate_crc(uint8_t *data, uint8_t datalen)
{
    if (data == NULL) {
        return (uint8_t)0;
    }
    // calculates 8-Bit checksum with given polynomial
    uint8_t crc = SGP30_CRC8_INIT;
    for(uint8_t i = 0; i < datalen; i++) {
        crc ^= data[i];
        for(uint8_t b = 0; b < 8; b++) {
            if(crc & 0x80)
                crc = (crc << 1) ^ SGP30_CRC8_POLYNOMIAL;
            else
                crc <<= 1;
        }
    }
    return crc;
}

/*********************
 * Static functions  *
 *********************/

/* check for NULLs. Not a thorough check */
static int8_t sgp30_null_ptr_check(const Sgp30Dev *dev)
{
    uint8_t rslt = SGP30_OK;
    if ((dev == NULL) || (dev->read == NULL) || (dev->write == NULL) || (dev->delay_us == NULL)) {
        rslt = SGP30_E_NULL_PTR;
    }
    return rslt;
}

/* Internal API function to delay for period useconds */
static int8_t sgp30_delay_us(Sgp30Dev *dev, uint32_t period)
{
    if (sgp30_null_ptr_check(dev) == SGP30_E_NULL_PTR) {
        return SGP30_E_NULL_PTR;
    }
    dev->delay_us(period, dev);
    return SGP30_OK;
}