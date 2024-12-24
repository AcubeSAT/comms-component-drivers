#pragma once

#include "ac7z020definitions.hpp"
#include "stm32h7xx_hal.h"
#include <utility>

const uint16_t AC7Z020_TIMEOUT = 1000;

namespace AC7Z020 {

    enum Error {
        NO_ERRORS,
        FAILED_WRITING_TO_FPGA,
        INVALID_TM_LENGTH,
        FAILED_READING_FROM_REGISTER
    };

    class AC7Z020 {
    public:
        /*
         * Initializer for AC7Z020
         *
         * @param hspi: pointer to the SPI_HandleTypeDef responsible for configuring the SPI.
         *
         */
        AC7Z020(SPI_HandleTypeDef *hspim) :
                hspi(hspim) {
        };

        void set_packet_length(PacketLength length, Error &err);

        void spi_block_write_8(uint16_t n, uint8_t *value,
                               Error &err);

        void spi_block_read_8(uint16_t n, uint8_t *value,
                               Error &err);

    private:
        SPI_HandleTypeDef *hspi;
        uint16_t packet_length;
    };

}
