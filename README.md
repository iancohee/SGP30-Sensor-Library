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
Examples coming soon.

## Arduino

### Step One

```c
#include <sgp30.h>
#include <sgp30_defs.h>

#include <assert.h>
#include <Wire.h>

Sgp30Dev* dev;

SGP30_INTF_RET_TYPE device_read(uint8_t reg, uint8_t* data, uint32_t data_len, void* intf_ptr);
SGP30_INTF_RET_TYPE device_write(uint8_t reg, uint8_t* command, uint32_t data_len, void* intf_ptr);
void device_delay_us(uint32_t period, void* intf_ptr);
```

Arduino uses a library called `Wire` that provides an interface for I2C communication. 

We will set the read and write addresses to `SGP30_I2C_ADDR` and the `Wire` library will do the rest.

Import headers, declare a global device pointer `dev` and prototype read/write/delay functions.

### Step Two

In `setup()`

```c
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(2000);
  Wire.begin(dev->i2c_addr);

  dev = sgp30_alloc();

  sgp30_set_read_fptr(dev, device_read);
  sgp30_set_write_fptr(dev, device_write);
  sgp30_set_delay_us_fptr(dev, device_delay_us);
  sgp30_set_i2c_addr_read(dev, SGP30_I2C_ADDR);
  sgp30_set_i2c_addr_write(dev, SGP30_I2C_ADDR);
```

Initialize Wire communication and allocate a device.

Next we set the read/write/delay function pointers, and the I2C read and write addresses that work with the `Wire` library. 

### Step Three

```c
uint8_t status = sgp30_read_chip_id(dev);
```

Initiate a read test

### Step Four

```c
status = sgp30_measure_test(dev);
```

Initiate a measure test, and *check the return code*.

#### Step Five

```c
status = sgp30_init(dev);
```

Initialize the sensor. Setup is complete.

### Step Five

The main function, `loop()``

```c
void loop() {
  // put your main code here, to run repeatedly:
  Sgp30DevReadings readings = { 0 };

  int8_t status = sgp30_measure_air_quality(dev, &readings);
  ... HANDLE YOUR ERRORS ...
  
  char out_buf[32];
  snprintf(out_buf,
           sizeof(out_buf),
           "co_eq: %u, tvoc: %u",
           readings.co2,
           readings.tvoc);
  Serial.println(out_buf);
  
  delay(1000);
}
```

1. Pass the `dev` pointer and a `Sgp30DevReadings` pointer to `sgp30_measure_air_quality`. The results are stored in the readings object.
2. Access the measurements in
    * `readings.co2` - CO2_eq
    * `readings.tvoc` - Total volatile organic compounds
3. wait 1 second!

The readings must be done in a frequency of 1Hz to maintain calibration (calibration settings support coming soon).

## Flipper Zero
Coming soon
