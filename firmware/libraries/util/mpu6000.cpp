/*---------------------------------------------------------------------------------------
*                                       INCLUDES
*--------------------------------------------------------------------------------------*/

#include <cstdio>
#include <cstdint>
#include "mpu6000.h"
#include "util_assert.h"

/*---------------------------------------------------------------------------------------
*                                      CONSTANTS
*--------------------------------------------------------------------------------------*/

#define CMD_READ     (0x80)
#define CMD_WRITE    (0x00)
#define ADDR_MASK    (0x7F)
#define WHO_I_AM     (0x68)

// MPU 6000 registers
// NOTE: All registers are reset to 0x00 other than the following:
//       MPUREG_PWR_MGMT_1: 0x40 (Sleep mode/bit)
//       MPUREG_WHOAMI    : 0x68 (WHOAMI Answer)
#define MPUREG_PRODUCT_ID       (0x0C)
#define MPUREG_SMPLRT_DIV       (0x19)
#define MPUREG_CONFIG           (0x1A)
#define MPUREG_GYRO_CONFIG      (0x1B)
#define MPUREG_ACCEL_CONFIG     (0x1C)
#define MPUREG_FIFO_EN          (0x23)
#define MPUREG_INT_PIN_CFG      (0x37)
#define MPUREG_INT_ENABLE       (0x38)
#define MPUREG_INT_STATUS       (0x3A)
#define MPUREG_ACCEL_XOUT_H     (0x3B)
#define MPUREG_ACCEL_XOUT_L     (0x3C)
#define MPUREG_ACCEL_YOUT_H     (0x3D)
#define MPUREG_ACCEL_YOUT_L     (0x3E)
#define MPUREG_ACCEL_ZOUT_H     (0x3F)
#define MPUREG_ACCEL_ZOUT_L     (0x40)
#define MPUREG_TEMP_OUT_H       (0x41)
#define MPUREG_TEMP_OUT_L       (0x42)
#define MPUREG_GYRO_XOUT_H      (0x43)
#define MPUREG_GYRO_XOUT_L      (0x44)
#define MPUREG_GYRO_YOUT_H      (0x45)
#define MPUREG_GYRO_YOUT_L      (0x46)
#define MPUREG_GYRO_ZOUT_H      (0x47)
#define MPUREG_GYRO_ZOUT_L      (0x48)
#define MPUREG_USER_CTRL        (0x6A)
#define MPUREG_PWR_MGMT_1       (0x6B)
#define MPUREG_PWR_MGMT_2       (0x6C)
#define MPUREG_SIGPATH_RESET    (0x68)
#define MPUREG_FIFO_COUNTH      (0x72)
#define MPUREG_FIFO_COUNTL      (0x73)
#define MPUREG_FIFO_R_W         (0x74)
#define MPUREG_WHOAMI           (0x75)

// Configuration register bits

// MPUREG_PWR_MGMT_1
#define BIT_SLEEP               (0x40)
#define BIT_H_RESET             (0x80)
#define MPU_CLK_SEL_INT8MHZ     (0x00)
#define MPU_CLK_SEL_PLLGYROX    (0x01)
#define MPU_CLK_SEL_PLLGYROY    (0x02)
#define MPU_CLK_SEL_PLLGYROZ    (0x03)
#define MPU_CLK_SEL_PLLEXT32KHZ (0x04)
#define MPU_CLK_SEL_PLLEXT19MHZ (0x05)
#define MPU_CLK_SEL_STOPRESET   (0x07)

#define MASK_CLKSEL             (0x07)

// MPUREG_GYRO_CONFIG
// FS = Full Scale (range), DPS = degrees/sample
#define BITS_FS_250DPS          (0x00)
#define BITS_FS_500DPS          (0x08)
#define BITS_FS_1000DPS         (0x10)
#define BITS_FS_2000DPS         (0x18)
#define MASK_FS                 (0x18)

// DLPF = Digital Low Pass Filter
// note: BW values listed in constant correspond to gyro
/*******************************************************************
 *           |   Accelerometer     |           Gyroscope           *
 * DLPF_CFG  | BW (Hz) | Delay(ms) | BW (Hz) | Delay(ms) | Fs(khz) *
 *   0x00    |   260   |    0.0    |   256   |   0.98    |    8    *
 *   0x01    |   184   |    2.0    |   188   |   1.90    |    1    *
 *   0x02    |    94   |    3.0    |    98   |   2.80    |    1    *
 *   0x03    |    44   |    4.9    |    42   |   4.80    |    1    *
 *   0x04    |    21   |    8.5    |    20   |   8.30    |    1    *
 *   0x05    |    10   |   13.8    |    10   |  13.40    |    1    *
 *   0x06    |     5   |   19.0    |     5   |  18.60    |    1    *
 *   0x07    |  RESVD  |    RESVD  |   RESVD |   RESVD   |    8    *
 *******************************************************************/
#define BITS_DLPF_CFG_256HZ_NOLPF2  (0x00)
#define BITS_DLPF_CFG_188HZ         (0x01)
#define BITS_DLPF_CFG_98HZ          (0x02)
#define BITS_DLPF_CFG_42HZ          (0x03)
#define BITS_DLPF_CFG_20HZ          (0x04)
#define BITS_DLPF_CFG_10HZ          (0x05)
#define BITS_DLPF_CFG_5HZ           (0x06)
#define BITS_DLPF_CFG_2100HZ_NOLPF  (0x07)
#define MASK_DLPF_CFG               (0x07)

// MPUREG_INT_PIN_CFG
// Any read clears interrupt
#define BIT_INT_ANYRD_2CLEAR        (0x10)

// MPUREG_INT_ENABLE
#define BIT_RAW_RDY_EN              (0x01)

// MPUREG_USER_CTRL
#define BIT_I2C_IF_DIS              (0x10)

// MPUREG_SIGPATH_RESET
#define BIT_SIGPATH_RESET_ALL       (0x07)

// Product ID Description for MPU6000
// high 4 bits     low 4 bits
// Product Name    Product Revision
#define MPU6000ES_REV_C4            (0x14)
#define MPU6000ES_REV_C5            (0x15)
#define MPU6000ES_REV_D6            (0x16)
#define MPU6000ES_REV_D7            (0x17)
#define MPU6000ES_REV_D8            (0x18)
#define MPU6000_REV_C4              (0x54)
#define MPU6000_REV_C5              (0x55)
#define MPU6000_REV_D6              (0x56)
#define MPU6000_REV_D7              (0x57)
#define MPU6000_REV_D8              (0x58)
#define MPU6000_REV_D9              (0x59)
#define MPU6000_REV_D10             (0x5A)

#define MPU6000_ONE_G   (9.80665f)

#define MPU6000_DEFAULT_ONCHIP_FILTER_FREQ  (42)

// SPI bus frequency when reading sensor data
#define SENSOR_READ_SPI_FREQUENCY (20 * DEFAULT_SPI_FREQUENCY)

/*---------------------------------------------------------------------------------------
*                                        MACROS
*--------------------------------------------------------------------------------------*/

// Chip Selection (CS) Functionality for MPU6000
#define CS_SELECT() GPIOA->BSRRH = GPIO_Pin_15
#define CS_DESELECT() GPIOA->BSRRL = GPIO_Pin_15

/*---------------------------------------------------------------------------------------
*                                     CLASS METHODS
*--------------------------------------------------------------------------------------*/

//*****************************************************************************
MPU6000::MPU6000(void) :
    product_id_(0),
    spi_(NULL),
    spi_bus_(SPI_BUS_3),
    gyro_range_scale_(0),
    accel_range_scale_(0)
{
}

//*****************************************************************************
int8_t MPU6000::initialize(void)
{
    spi_ = SPI::instance(spi_bus_);

    if (spi_ == NULL) { return -2; } // Unsuccessful bus setup.

    if (!probe())
    {
        return -1; // Unsuccessful driver initialization
    }

    reset();

    return 0; // Successful driver initialization
}

//*****************************************************************************
void MPU6000::readGyro(float * data)
{
    // Setup a union to assist in reading/interpreting register data.
    union {
        int16_t raw;
        uint8_t bytes[2];
    } reading;

    CS_SELECT();

    // High-speed for sensor data. Sensor registers can be read at 20MHz.
    spi_->setFrequency(SENSOR_READ_SPI_FREQUENCY);

    spi_->sendByte(CMD_READ | MPUREG_GYRO_XOUT_H); // Start here for gyroscope

    reading.bytes[1] = spi_->sendByte(0);
    reading.bytes[0] = spi_->sendByte(0);
    data[0] = convertRawGyro(reading.raw);

    reading.bytes[1] = spi_->sendByte(0);
    reading.bytes[0] = spi_->sendByte(0);
    data[1] = convertRawGyro(reading.raw);

    reading.bytes[1] = spi_->sendByte(0);
    reading.bytes[0] = spi_->sendByte(0);
    data[2] = convertRawGyro(reading.raw);

    CS_DESELECT();

    // Restore regular SPI bus speeds.  Most registers must be read/written at 1Mhz
    spi_->setFrequency(DEFAULT_SPI_FREQUENCY);
}

//*****************************************************************************
void MPU6000::readAccel(float * data)
{
    // Setup a union to assist in reading/interpreting register data.
    union {
        int16_t raw;
        uint8_t bytes[2];
    } reading;

    CS_SELECT();

    // High-speed for sensor data. Sensor registers can be read at 20MHz.
    spi_->setFrequency(SENSOR_READ_SPI_FREQUENCY);

    spi_->sendByte(CMD_READ | MPUREG_INT_STATUS); // start here for interrupt status, accelerometer data & temperature

    spi_->sendByte(0); // read interrupt status

    reading.bytes[1] = spi_->sendByte(0);  // ACCEL_XOUT_H
    reading.bytes[0] = spi_->sendByte(0);  // ACCEL_XOUT_L
    data[0] = convertRawAccel(reading.raw);

    reading.bytes[1] = spi_->sendByte(0);  // ACCEL_YOUT_H
    reading.bytes[0] = spi_->sendByte(0);  // ACCEL_YOUT_L
    data[1] = convertRawAccel(reading.raw);

    reading.bytes[1] = spi_->sendByte(0);  // ACCEL_ZOUT_H
    reading.bytes[0] = spi_->sendByte(0);  // ACCEL_ZOUT_L
    data[2] = convertRawAccel(reading.raw);

    // NOTE Convert temperature count to temperature via equation:
    //      temp in degrees C = (TEMP_OUT (signed value) / 340) + 36.53
    //reading.bytes[1] = spi_->sendByte(0);
    //reading.bytes[0] = spi_->sendByte(0);
    // temperature = reading.raw;

    CS_DESELECT();

    // Restore regular SPI bus speeds.  Most registers must be read/written at 1Mhz
    spi_->setFrequency(DEFAULT_SPI_FREQUENCY);
}

//*****************************************************************************
void MPU6000::reset(void)
{
    // Reset sensor
    setRegister(MPUREG_PWR_MGMT_1, BIT_H_RESET);
    usleep(10000);

    // Datasheet says SPI should reset the signal path
    setRegister(MPUREG_SIGPATH_RESET, BIT_SIGPATH_RESET_ALL);
    usleep(10000);

    // Wake up device and select GyroZ clock. Note that the
    // MPU6000 starts up in sleep mode, and it can take some time
    // for it to come out of sleep
    setRegister(MPUREG_PWR_MGMT_1, MPU_CLK_SEL_PLLGYROZ);
    usleep(1000);

    // Disable I2C bus since we're using SPI
    setRegister(MPUREG_USER_CTRL, BIT_I2C_IF_DIS);
    usleep(1000);

    setSampleRate(500); // Hz
    usleep(1000);

    // Set DLPF (low pass filter) to default of 42Hz
    // **was 90 Hz, but this ruins quality and does not improve the system response
    setDLPFilter(MPU6000_DEFAULT_ONCHIP_FILTER_FREQ);
    usleep(1000);

    // Set gyroscope scale to 2000 deg/s
    gyro_range_scale_ = (0.0174532f / 16.4f); // 1/(2^15)*(2000/180)*PI
    setRegister(MPUREG_GYRO_CONFIG, BITS_FS_2000DPS);
    usleep(1000);

    // Product-specific scaling for accelerometer.
    switch (product_id_)
    {
        case MPU6000ES_REV_C4:
        case MPU6000ES_REV_C5:
        case MPU6000_REV_C4:
        case MPU6000_REV_C5:
            // Accel scale 8g (4096 LSB/g)
            // Rev C has different scaling than rev D
            setRegister(MPUREG_ACCEL_CONFIG, 1 << 3);
            break;

        case MPU6000ES_REV_D6:
        case MPU6000ES_REV_D7:
        case MPU6000ES_REV_D8:
        case MPU6000_REV_D6:
        case MPU6000_REV_D7:
        case MPU6000_REV_D8:
        case MPU6000_REV_D9:
        case MPU6000_REV_D10:
        // default case to cope with new chip revisions, which
        // presumably won't have the accel scaling bug
        default:
            // Accel scale 8g (4096 LSB/g)
            setRegister(MPUREG_ACCEL_CONFIG, 2 << 3);
            break;
    }

    // Correct accel scale factors of 4096 LSB/g
    // scale to m/s^2 (1g = 9.81 m/s^2)
    accel_range_scale_ = (MPU6000_ONE_G / 4096.0f);

    // Enable read interrupts.
    setRegister(MPUREG_INT_ENABLE, BIT_RAW_RDY_EN);
    usleep(1000);

    // Configure interrupt pin to clear on any read.
    setRegister(MPUREG_INT_PIN_CFG, BIT_INT_ANYRD_2CLEAR);
    usleep(1000);
}

//*****************************************************************************
bool MPU6000::probe(void)
{
    bool probe_successful = true;
    product_id_ = readRegister(MPUREG_PRODUCT_ID);

    switch (product_id_)
    {
        case MPU6000ES_REV_C4:
        case MPU6000ES_REV_C5:
        case MPU6000_REV_C4:
        case MPU6000_REV_C5:
        case MPU6000ES_REV_D6:
        case MPU6000ES_REV_D7:
        case MPU6000ES_REV_D8:
        case MPU6000_REV_D6:
        case MPU6000_REV_D7:
        case MPU6000_REV_D8:
        case MPU6000_REV_D9:
        case MPU6000_REV_D10:
            break;
        default:
            probe_successful = false;
            //assert_always_msg(ASSERT_CONTINUE, "Invalid MPU6000 product ID");
            break;
    }

    return probe_successful;
}

//*****************************************************************************
void MPU6000::setSampleRate(uint16_t desired_rate)
{
    if (desired_rate == 0)
    {
        desired_rate = 1000;
    }

    uint8_t div = 1000 / desired_rate;

    if (div > 200)
    {
        div = 200;
    }
    else if (div < 1)
    {
        div = 1;
    }

    setRegister(MPUREG_SMPLRT_DIV, div-1);
}

//*****************************************************************************
void MPU6000::setDLPFilter(uint16_t frequency)
{
    uint8_t filter = 0;

    // Choose next highest filter frequency available
    if (frequency <= 5)
    {
        filter = BITS_DLPF_CFG_5HZ;
    }
    else if (frequency <= 10)
    {
        filter = BITS_DLPF_CFG_10HZ;
    }
    else if (frequency <= 20)
    {
        filter = BITS_DLPF_CFG_20HZ;
    }
    else if (frequency <= 42)
    {
        filter = BITS_DLPF_CFG_42HZ;
    }
    else if (frequency <= 98)
    {
        filter = BITS_DLPF_CFG_98HZ;
    }
    else if (frequency <= 188)
    {
        filter = BITS_DLPF_CFG_188HZ;
    }
    else if (frequency <= 256)
    {
        filter = BITS_DLPF_CFG_256HZ_NOLPF2;
    }
    else
    {
        filter = BITS_DLPF_CFG_2100HZ_NOLPF;
    }

    setRegister(MPUREG_CONFIG, filter);
}

//*****************************************************************************
uint8_t MPU6000::readRegister(uint8_t address)
{
    address &= ADDR_MASK;

    CS_SELECT();

    spi_->sendByte(CMD_READ | address);
    uint8_t value = spi_->sendByte(0);

    CS_DESELECT();

    return value;
}

//*****************************************************************************
void MPU6000::setRegister(uint8_t address, uint8_t value)
{
    address &= ADDR_MASK;

    CS_SELECT();
    usleep(1);
    spi_->sendByte(CMD_WRITE | address);
    spi_->sendByte(value);
    usleep(1);
    CS_DESELECT();
}

//*****************************************************************************
void MPU6000::modifyRegister(uint8_t address,  uint8_t clearbits, uint8_t setbits)
{
    uint8_t value = readRegister(address);
    value &= ~clearbits;
    value |=  setbits;
    setRegister(address, value);
}

//*****************************************************************************
void MPU6000::usleep(uint32_t microseconds)
{
    sys_timer.busyWait(microseconds * 1.0e-6);
}
