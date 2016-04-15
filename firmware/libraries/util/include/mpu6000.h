#ifndef MPU6000_H_INCLUDED
#define MPU6000_H_INCLUDED

// Includes
#include "stm32f4xx.h"
#include "spi.h"
#include "system_timer.h"

// Provide setup and access to MPU6000 gyroscope and accelerometer.
class MPU6000
{
public: // methods

    // Constructor
    MPU6000(void);

    // Initialize and verify the sensor. Return a non-zero error code on failure.
    int8_t initialize(void);

    // Read and return the X, Y, Z data from the gyroscope in rad/sec
    void readGyro(float * data);

    // Read and return the X, Y, Z data from the accelerometer in m/s/s
    void readAccel(float * data);

private: // methods

    // Reset the sensor to default configuration values.
    void reset(void);

    // Probe the sensor for correct WHOAMI and product revision.
    // Return true if sensor responds correctly.
    bool probe(void);

    // Configure the rate [Hz] at which the gyroscope is sampled & output. 0 for maximum.
    void setSampleRate(uint16_t desired_rate);

    // Configure the Digital Low Pass (DLP) Filter in [Hz] for gyroscope and the accelerometer.
    void setDLPFilter(uint16_t frequency);

    // Return the value stored in the specified register address.
    uint8_t readRegister(uint8_t address);

    // Set the register found at address to the specified value.
    void setRegister(uint8_t address, uint8_t value);

    // Update the register at specified address by clearing the clearbits and setting the setbits.
    void modifyRegister(uint8_t address,  uint8_t clearbits, uint8_t setbits);

    // Convert raw gyro reading into rad/sec and return result.
    float convertRawGyro(int16_t raw) { return (float)raw * gyro_range_scale_; }

    // Convert raw accel reading into m/s/s and return result.
    float convertRawAccel(int16_t raw) { return (float)raw * accel_range_scale_; }

    // Busy waits for specified number of microseconds.
    void usleep(uint32_t microseconds);

private: // fields

    // Product ID read from device
    uint8_t product_id_;

    // SPI bus information
    SPI        * spi_;     // SPI bus object
    spi_bus_id_t spi_bus_; // SPI bus ID

    // Scale applied to raw gyro/accel readings to convert to useful units.
    float  gyro_range_scale_;
    float  accel_range_scale_;

};

#endif
