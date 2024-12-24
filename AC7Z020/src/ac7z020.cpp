#include "ac7z020.hpp"
#include "main.h"

namespace AC7Z020 {

    void AC7Z020::spi_block_write_8(uint16_t n, uint8_t *value, Error &err) {

        HAL_GPIO_WritePin(FPGA_NSS_GPIO_Port, FPGA_NSS_Pin, GPIO_PIN_RESET);
        uint8_t hal_error = HAL_SPI_Transmit(hspi, value, n, AC7Z020_TIMEOUT);

        if (hal_error != HAL_OK) {
            err = Error::FAILED_WRITING_TO_FPGA;
            return;
        }
        HAL_GPIO_WritePin(FPGA_NSS_GPIO_Port, FPGA_NSS_Pin, GPIO_PIN_SET);

        err = Error::NO_ERRORS;
    }

    void AC7Z020::spi_block_read_8(uint16_t n, uint8_t *value, Error &err) {

        HAL_GPIO_WritePin(FPGA_NSS_GPIO_Port, FPGA_NSS_Pin, GPIO_PIN_RESET);
        uint8_t hal_error = HAL_SPI_Receive(hspi, value, n, AC7Z020_TIMEOUT);

        if (hal_error != HAL_OK) {
            err = Error::FAILED_READING_FROM_REGISTER;
            return;
        }

        HAL_GPIO_WritePin(FPGA_NSS_GPIO_Port, FPGA_NSS_Pin, GPIO_PIN_SET);
        err = Error::NO_ERRORS;

        return;
    }

    void AC7Z020::set_packet_length(PacketLength length, Error &err) {
        if (length != UHFTMPacketLength
            && length != SBandPacketLength) {
            err = Error::INVALID_TM_LENGTH;
        }
        packet_length = length;
    }


}

