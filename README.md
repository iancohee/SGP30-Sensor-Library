SGP30 Sensor API
===
A no-frills device library for SGP30 sensors.

Some features:
* Portable!
    * Written in C to be compatible with C++
    * Microcontroller agnostic
* Maybe Linux kernel ready? (not tested)
* No asserts, errors should be recoverable

What is missing that may be nice to have:
* A logging interface
* Unit tests
* Hardware verification

# Sensor Device
The SGP30 (or GY-SGP30) is an I2C gas sensor for indoor industrial applications, that measures and reports CO2eq and Total Volatile Organic Compounds (TVOC) in the air.

> The SGP30 is a digital multi-pixel gas sensor designed for
> easy integration into air purifier, demand-controlled
> ventilation, and IoT applications.

The Mouser Electronics [datasheet](https://www.mouser.com/pdfdocs/Sensirion_Gas_Sensors_SGP30_Datasheet_EN-1148053.pdf?srsltid=AfmBOop0X25kHZhmbr_4qUNt4UUsECiIjjF5Ytr4ZUdnKDzQuUe9mgIy) is pretty short and describes the internals and operative basics of the device. 

This is a very simple C library meant to be hardware agnostic and compatible with C++ and Arduino. It implements interfaces for the SGP30 but does not presume to automate configuration or handle background/asynchronous communication.

**Note:** This code does not take humidity compensation into account. Measurements will be made assuming the default humidity compensation value documented in the datasheet.

> Restarting the sensor (power-on or soft reset) or sending a value of 0x0000
> (= 0 g/m3) sets the humidity value used for compensation to its default value (0x0B92 = 11.> 57 g/m3) until a new humidity value is sent.

Users _must_ provide read, write, and delay functions for the hardware they want to integrate this sensor code with.

# Usage
Typical usage will follow the following flow:

1. Write functions for hardware to handle I2C communication
2. Create `Sgp30Dev`instance using `sgp30_alloc()`
3. Set read/write/delay_us functions using instance setters
4. I2C master sends a measure command
5. Wait either the expected execution time and poll the data, or wait until the execution times out
6. I2C master reads measurement
7. `Sgp30Dev` is freed using `sgp30_free()`

In the future, Arduino and Furi (Flipper Zero) library interfaces may be provided.

# Examples

## Arduino

## Flipper Zero