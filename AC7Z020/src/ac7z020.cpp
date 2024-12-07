#include "ac7z020.hpp"
#include "main.h"

namespace AC7Z020 {

    void AC7Z020::spi_block_write_8(uint16_t address, uint16_t n, uint8_t *value,
                                      Error &err) {

        HAL_GPIO_WritePin(FPGA_NSS_GPIO_Port, FPGA_NSS_Pin, GPIO_PIN_RESET);
        hal_error = HAL_SPI_Transmit(hspi, value, n, TIMEOUT);

        if (hal_error != HAL_OK) {
            err = Error::FAILED_WRITING_TO_FPGA;
            return;
        }
        HAL_GPIO_WritePin(FPGA_NSS_GPIO_Port, FPGA_NSS_Pin, GPIO_PIN_RESET);

        err = Error::NO_ERRORS;
    }

}

