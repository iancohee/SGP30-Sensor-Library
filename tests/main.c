#include "../src/sgp30_defs.h"
#include "../src/sgp30.h"
#include "tests.h"

#include <assert.h>
#include <stdio.h>


int
main(void)
{
    int status = 0;

    test_read_chip_id_null();
    test_null_ptr_check();

    test_set_read_fptr_null_ptr();
    test_set_write_fptr_null_ptr();

    test_init_dev_null();
    test_init_device_not_found();

    test_double_free();

    test_read();

    if (status == 0) {
        printf("[SUCCESS] all tests complete\n");
    } else {
        printf("[FAILURE] something went wrong");
    }

    return 0;
}

/**
 * Helper functions
 */
SGP30_INTF_RET_TYPE dummy_read_success(uint8_t reg, uint8_t* data, uint32_t data_len, void* intf_ptr)
{
    (void) reg;
    (void) data;
    (void) data_len;
    (void) intf_ptr;

    printf(DEBUG "dummy_read -> 0\n");
    return SGP30_OK;
}

SGP30_INTF_RET_TYPE dummy_read_error(uint8_t reg, uint8_t* data, uint32_t data_len, void* intf_ptr)
{
    (void) reg;
    (void) data;
    (void) data_len;
    (void) intf_ptr;

    fprintf(stderr, DEBUG "dummy_read -> SGP30_E_READ\n");
    return SGP30_E_READ;
}

SGP30_INTF_RET_TYPE dummy_write_success(uint8_t reg, const uint8_t* data, uint32_t data_len, void* intf_ptr)
{
    (void) reg;
    (void) data;
    (void) data_len;
    (void) intf_ptr;

    fprintf(stderr, DEBUG "dummy_write -> SGP30_OK\n");
    return SGP30_OK;
}

SGP30_INTF_RET_TYPE dummy_write_error(uint8_t reg, const uint8_t* data, uint32_t data_len, void* intf_ptr)
{
    (void) reg;
    (void) data;
    (void) data_len;
    (void) intf_ptr;

    return SGP30_E_WRITE;
}

/* dummy delay */
void dummy_delay_us(uint32_t period, void* intf_ptr)
{
    (void) intf_ptr;

    fprintf(stderr, DEBUG "dummy_delay_us: %u us\n", period);
    return;
}

/****************
 *    Tests
 ***************/

/* NULL Checks */
void test_read_chip_id_null(void)
{
    printf(DEBUG "test_read_chip_id_null\n");
    Sgp30Dev* dev = NULL;
    assert(sgp30_read_chip_id(dev) == SGP30_E_NULL_PTR);
}

void test_null_ptr_check(void)
{
    printf(DEBUG "test_null_ptr_check\n");
    Sgp30Dev* dev = sgp30_alloc();
    assert(sgp30_read_chip_id(dev) == SGP30_E_NULL_FPTR);
    sgp30_free(&dev);
}

void test_set_read_fptr_null_ptr(void)
{
    printf(DEBUG "test_set_read_fptr_null_ptr\n");
    Sgp30Dev* dev = sgp30_alloc();
    sgp30_read_fptr_t fptr = NULL;
    assert(sgp30_set_read_fptr(dev, fptr) == SGP30_E_NULL_FPTR);
    sgp30_free(&dev);
}

void test_set_write_fptr_null_ptr(void)
{
    printf(DEBUG "test_set_write_fptr_null_ptr\n");
    Sgp30Dev* dev = sgp30_alloc();
    sgp30_write_fptr_t fptr = NULL;
    assert(sgp30_set_write_fptr(dev, fptr) == SGP30_E_NULL_FPTR);
    sgp30_free(&dev);
}

/** Init Tests */
void test_init_dev_null(void)
{
    printf(DEBUG "test_init_dev_null\n");
    Sgp30Dev* dev = sgp30_alloc();
    int8_t status = sgp30_init(dev);
    printf(DEBUG "sgp30_init() -> %d\n", status);
    assert(status == SGP30_E_NULL_FPTR);
    sgp30_free(&dev);
}

void test_init_device_not_found(void)
{
    printf(DEBUG "test_init_device_not_found\n");
    Sgp30Dev* dev = sgp30_alloc();
    sgp30_set_read_fptr(dev, dummy_read_success);
    sgp30_set_write_fptr(dev, dummy_write_success);
    sgp30_set_delay_us_fptr(dev, dummy_delay_us);

    int8_t status = sgp30_init(dev);
    assert(status == SGP30_E_DEV_NOT_FOUND);
    sgp30_free(&dev);
}

void test_double_free(void)
{
    printf(DEBUG "test_double_free\n");
    Sgp30Dev* dev = sgp30_alloc();
    assert(dev != NULL);
    sgp30_free(&dev);
    printf(DEBUG "first free.. ");
    sgp30_free(&dev); // <- not a typo, shouldn't crash
    printf("second free\n");
}

void test_read(void)
{
    Sgp30Dev* dev = sgp30_alloc();

    sgp30_set_read_fptr(dev, dummy_read_success);
    sgp30_set_write_fptr(dev, dummy_write_success);
    sgp30_set_delay_us_fptr(dev, dummy_delay_us);

    uint8_t data[3] = { 0 };
    int8_t status = sgp30_read(dev, data, 2);
    assert(status == SGP30_OK);

    sgp30_set_read_fptr(dev, dummy_read_error);
    status = sgp30_read(dev, data, 2);
    assert(status == SGP30_E_READ);

    sgp30_free(&dev);
}

void test_write(void)
{
    Sgp30Dev* dev = sgp30_alloc();

    sgp30_set_read_fptr(dev, dummy_read_success);
    sgp30_set_write_fptr(dev, dummy_write_success);
    sgp30_set_delay_us_fptr(dev, dummy_delay_us);

    uint8_t data[3] = { 0xBE, 0xEF, 0x00 };
    int8_t status = sgp30_write(dev, data, 2);
    assert(status == SGP30_OK);

    sgp30_set_write_fptr(dev, dummy_write_error);
    status = sgp30_write(dev, data, 2);
    assert(status == SGP30_E_WRITE);

    sgp30_free(&dev);
}
