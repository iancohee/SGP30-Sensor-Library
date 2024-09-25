#ifndef __TESTS_HEADER__
#define __TESTS_HEADER__

#include "../src/sgp30_defs.h"

#define DEBUG "[DEBUG] "
#define INFO  "[INFO] "

/* read succeeds */
SGP30_INTF_RET_TYPE dummy_read_success(uint8_t reg, uint8_t* data, uint32_t data_len, void* intf_ptr);

/* read fails */
SGP30_INTF_RET_TYPE dummy_read_error(uint8_t reg, uint8_t* data, uint32_t data_len, void* intf_ptr);

/* write succeeds */
SGP30_INTF_RET_TYPE dummy_write_success(uint8_t reg, const uint8_t* data, uint32_t data_len, void* intf_ptr);

/* write fails */
SGP30_INTF_RET_TYPE dummy_write_error(uint8_t reg, const uint8_t* data, uint32_t data_len, void* intf_ptr);

/** measure_test test */
/**
 * @brief This function should always return 0xD400, with correct crc, for testing sg30_measure_test
 */
SGP30_INTF_RET_TYPE read_measure_test_success(uint8_t reg, uint8_t* data, uint32_t data_len, void* intf_ptr);

/**
 * @brief This function returns 0xD400 with an incorrect crc
 */
SGP30_INTF_RET_TYPE dummy_read_measure_test_wrong_crc(uint8_t reg, uint8_t* data, uint32_t data_len, void* intf_ptr);

/* test CRC implementation CRC(0xBEEF) = 0x92 */
void test_generate_crc(void);

/* dummy delay */
void dummy_delay_us(uint32_t period, void* intf_ptr);

/* Check NULLs with an incompletely */
void test_read_chip_id_null(void);
void test_null_ptr_check(void);

void test_set_read_fptr_null_ptr(void);
void test_set_write_fptr_null_ptr(void);

/* init tests */
void test_init_dev_null(void);
void test_init_dev_success(void);

/** double free */
void test_double_free(void);

void test_read(void);
void test_write(void);

/** measure test tests */
void test_measure_test_fail_crc(void);
void test_measure_test_success(void);

#endif
