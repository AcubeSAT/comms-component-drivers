#pragma once

#include <ac7z020definitions.hpp>
#include "stm32h7xx_hal.h"
#include <utility>

const uint16_t TIMEOUT = 1000;

namespace AC7Z020 {

    enum Error {
        NO_ERRORS,
        FAILED_WRITING_TO_FPGA,
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

        void spi_block_write_8(uint16_t n, uint8_t *value,
                               Error &err);

    private:
        SPI_HandleTypeDef *hspi;
    };

}
