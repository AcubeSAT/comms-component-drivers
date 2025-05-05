#include "ac7z020.hpp"
#include "main.h"

namespace AC7Z020 {
    // definition
    AC7Z020_Utilities ac7z020Utils = AC7Z020_Utilities();

    etl::expected<void, Error> AC7Z020_Utilities::initializeResources(SPI_HandleTypeDef* spiHandle) {
        hspi = spiHandle;

        resourcesMutexHandle = xSemaphoreCreateMutexStatic(&resourcesMutexBuffer);
        spiWriteCompleteSemaphoreHandle = xSemaphoreCreateBinaryStatic(&spiWriteCompleteSemaphoreBuffer);
        spiReadCompleteSemaphoreHandle = xSemaphoreCreateBinaryStatic(&spiReadCompleteSemaphoreBuffer);
        if (resourcesMutexHandle == nullptr ||
            spiWriteCompleteSemaphoreHandle == nullptr ||
            spiReadCompleteSemaphoreHandle == nullptr) {
            return etl::unexpected(Error::FREERTOS_RESOURCE_INITIALIZATION_FAILED);
        }
        return {};
    }

    etl::expected<void, Error> AC7Z020_Utilities::spi_block_write_8(uint16_t regAddress, uint8_t* sourceBuff, uint16_t numBytes) {
        uint8_t msg[2] = {static_cast<uint8_t>(0x80 | ((regAddress >> 8) & 0x7F)), static_cast<uint8_t>(regAddress & 0xFF)};
        HAL_GPIO_WritePin(FPGA_NSS_GPIO_Port, FPGA_NSS_Pin, GPIO_PIN_RESET); // slave select pin

        uint8_t hal_error = HAL_SPI_Transmit_DMA(hspi, msg, 2);
        if (hal_error != HAL_OK ||
            xSemaphoreTake(spiWriteCompleteSemaphoreHandle, pdMS_TO_TICKS(resouceMutexTimeout)) != pdTRUE) {
            HAL_GPIO_WritePin(FPGA_NSS_GPIO_Port, FPGA_NSS_Pin, GPIO_PIN_SET);
            return etl::unexpected(Error::FAILED_WRITING_TO_FPGA);
        }

        hal_error = HAL_SPI_Transmit_DMA(hspi, sourceBuff, numBytes);
        if (hal_error != HAL_OK ||
            xSemaphoreTake(spiWriteCompleteSemaphoreHandle, pdMS_TO_TICKS(resouceMutexTimeout)) != pdTRUE) {
            HAL_GPIO_WritePin(FPGA_NSS_GPIO_Port, FPGA_NSS_Pin, GPIO_PIN_SET);
            return etl::unexpected(Error::FAILED_WRITING_TO_FPGA);
        }

        HAL_GPIO_WritePin(FPGA_NSS_GPIO_Port, FPGA_NSS_Pin, GPIO_PIN_SET);
        return {};
    }

    etl::expected<void, Error> AC7Z020_Utilities::spi_block_read_8(uint16_t regAddress, uint8_t* destBuff, uint16_t numBytes) {
        uint8_t msg[2] = {static_cast<uint8_t>((regAddress >> 8) & 0x7F), static_cast<uint8_t>(regAddress & 0xFF)};

        HAL_GPIO_WritePin(FPGA_NSS_GPIO_Port, FPGA_NSS_Pin, GPIO_PIN_RESET); // slave select pin
        uint8_t hal_error = HAL_SPI_TransmitReceive_DMA(hspi, msg, destBuff, numBytes + 2);

        if (hal_error != HAL_OK ||
            xSemaphoreTake(spiReadCompleteSemaphoreHandle, pdMS_TO_TICKS(resouceMutexTimeout)) != pdTRUE) {
            HAL_GPIO_WritePin(FPGA_NSS_GPIO_Port, FPGA_NSS_Pin, GPIO_PIN_SET);
            return etl::unexpected(Error::FAILED_READING_FROM_FPGA);
        }

        HAL_GPIO_WritePin(FPGA_NSS_GPIO_Port, FPGA_NSS_Pin, GPIO_PIN_SET);
        return {};
    }

    etl::expected<IrqCode, Error> AC7Z020_Utilities::readIrqCode() {
        if (xSemaphoreTake(resourcesMutexHandle, pdMS_TO_TICKS(resouceMutexTimeout)) != pdTRUE) {
            return etl::unexpected(Error::RESOURCE_MUTEX_TIMEOUT);
        }

        uint8_t irq_reg_val;
        auto status = spi_block_read_8(RegisterAddress::IRQ, &irq_reg_val, 1);
        xSemaphoreGive(resourcesMutexHandle);

        auto irq = static_cast<IrqCode>(irq_reg_val);
        if (irq != IrqCode::UHF_PREAMBLE_RECEIVED &&
            irq != IrqCode::UHF_FULL_FRAME_RECEIVED &&
            irq != IrqCode::UHF_FULL_FRAME_TRANSMISSION_COMPLETE &&
            irq != IrqCode::SBAND_FULL_FRAME_TRANSMISSION_COMPLETE) {
            return etl::unexpected(Error::INVALID_IRQ_CODE);
        }

        return irq;
    }

    etl::expected<void, Error> AC7Z020_Utilities::transmitTMFrameSBAND(uint8_t* sourceBuff) {
        if (xSemaphoreTake(resourcesMutexHandle, pdMS_TO_TICKS(resouceMutexTimeout)) != pdTRUE) {
            return etl::unexpected(Error::RESOURCE_MUTEX_TIMEOUT);
        }

        auto status =
            spi_block_write_8(RegisterAddress::SBAND_FRAME, sourceBuff, FrameRegisterLength::LEN_SBAND_FRAME);
        xSemaphoreGive(resourcesMutexHandle);

        if (!status.has_value()) {
            return etl::unexpected(status.error());
        }

        return {};
    }

    etl::expected<void, Error> AC7Z020_Utilities::transmitTMFrameUHF(uint8_t* sourceBuff) {
        if (xSemaphoreTake(resourcesMutexHandle, pdMS_TO_TICKS(resouceMutexTimeout)) != pdTRUE) {
            return etl::unexpected(Error::RESOURCE_MUTEX_TIMEOUT);
        }

        auto status =
            spi_block_write_8(RegisterAddress::UHF_TM_FRAME, sourceBuff,
                FrameRegisterLength::LEN_UHF_TM_FRAME);
        xSemaphoreGive(resourcesMutexHandle);

        if (!status.has_value()) {
            return etl::unexpected(status.error());
        }

        return {};
    }

    etl::expected<uint16_t, Error> AC7Z020_Utilities::receiveTCFrameUHF(uint8_t* destBuff) {
        if (xSemaphoreTake(resourcesMutexHandle, pdMS_TO_TICKS(resouceMutexTimeout)) != pdTRUE) {
            return etl::unexpected(Error::RESOURCE_MUTEX_TIMEOUT);
        }

        // read the frame's length
        uint8_t length_reg_val[2];
        auto status = spi_block_read_8(static_cast<uint16_t>(RegisterAddress::UHF_TC_FRAME_LEN),
            length_reg_val, 2);

        uint16_t length = (static_cast<uint16_t>(length_reg_val[0]) << 8) | length_reg_val[1];
        if (length == 0 || length > FrameRegisterLength::LEN_UHF_TC_FRAME) {
            xSemaphoreGive(resourcesMutexHandle);
            return etl::unexpected(Error::INVALID_LENGTH);
        }

        // read the frame
        status = spi_block_read_8(RegisterAddress::UHF_TC_FRAME_LEN, destBuff, length);
        xSemaphoreGive(resourcesMutexHandle);

        if (!status.has_value()) {
            return etl::unexpected(status.error());
        }

        return length;
    }
}

