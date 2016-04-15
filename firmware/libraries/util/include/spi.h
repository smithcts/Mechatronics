#ifndef SPI_H_INCLUDED
#define SPI_H_INCLUDED

#include <cstdint>
#include "stm32f4xx_spi.h"

// Constants
#define DEFAULT_SPI_FREQUENCY (1000000) // Default frequency for SPI bus

// SPI bus ID associated with each port.
typedef uint32_t spi_bus_id_t;
enum
{
    SPI_BUS_1,
    SPI_BUS_2,
    SPI_BUS_3,
    SPI_BUS_4,
    SPI_BUS_COUNT
};

// Provide setup and use of SPI bus.
class SPI
{
public: // methods

    // Return the SPI object associated with the bus.  Initialize the bus if it
    // has not been initialized already. Return NULL on failure.
    static SPI * instance(spi_bus_id_t bus);

    // Sets the SPI bus frequency [Hz]
    void setFrequency(uint32_t frequency);

    // Sends AND receives a byte.  Returns the received byte.
    uint8_t sendByte(uint8_t byte_to_send);

private: // fields

    static bool init[SPI_BUS_COUNT]; // Indicates whether each bus is initialized already
    static SPI  objs[SPI_BUS_COUNT]; // Actual SPI bus objects

    // SPI bus information
    spi_bus_id_t bus_;   // SPI bus ID
    SPI_TypeDef * base_; // Base register for the SPI bus

};

#endif
