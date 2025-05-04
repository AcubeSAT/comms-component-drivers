#pragma once

#include <utility>
#include "ac7z020definitions.hpp"
#include "stm32h7xx_hal.h"
#include "etl/expected.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "Logger.hpp"

namespace AC7Z020 {

    enum class Error : uint8_t {
        FAILED_WRITING_TO_FPGA,
        FAILED_READING_FROM_FPGA,
        RESOURCE_MUTEX_TIMEOUT,
        INVALID_IRQ_CODE,
        INVALID_LENGTH
    };

    class AC7Z020_Utilities {
    public:

        /// Binary semaphores for signaling certain events (add them inside the proper ISR or freertos task)
        SemaphoreHandle_t spiWriteCompleteSemaphoreHandle;          // completion of spi write from dma callback
        SemaphoreHandle_t spiReadCompleteSemaphoreHandle;           // completion of spi read from dma callback

        /**
         * Initializer for AC7Z020 driver
         */
        explicit AC7Z020_Utilities(SPI_HandleTypeDef* handle) {
            hspi = handle;

            resourcesMutexHandle = xSemaphoreCreateMutexStatic(&resourcesMutexBuffer);
            spiWriteCompleteSemaphoreHandle = xSemaphoreCreateBinaryStatic(&spiWriteCompleteSemaphoreBuffer);
            spiReadCompleteSemaphoreHandle = xSemaphoreCreateBinaryStatic(&spiReadCompleteSemaphoreBuffer);
            if (resourcesMutexHandle == nullptr ||
                spiWriteCompleteSemaphoreHandle == nullptr ||
                spiReadCompleteSemaphoreHandle == nullptr) {
                LOG_ERROR << "[AC7Z020 Driver] Failed to create semaphores";
            }
        }

        /**
         * Read irq register
         *
         * @returns The irq code.
         */
        etl::expected<IrqCode, Error> readIrqCode();

        /**
         * Transmit SBAND frame to FPGA
         * @param sourceBuff The buffer containing the frame to transmit. It's length must be LEN_SBAND_FRAME bytes long
         */
        etl::expected<void, Error> transmitTMFrameSBAND(uint8_t* sourceBuff);

        /**
         * Transmit UHF frame to FPGA
         * @param sourceBuff The buffer containing the frame to transmit. It's length must be LEN_UHF_TM_FRAME bytes long
         */
        etl::expected<void, Error> transmitTMFrameUHF(uint8_t* sourceBuff);

        /**
         * Receive a UHF frame from the FPGA
         * @param destBuff A buffer to place the received frame. It's length must be LEN_UHF_TC_FRAME_LEN
         *                 (the maximum expected UHF TC frame length)
         *
         *  @returns The length of the received frame
         */
        etl::expected<uint16_t, Error> receiveTCFrameUHF(uint8_t* destBuff);

    private:
        SPI_HandleTypeDef *hspi;

        /// Mutex for concurrent access protection
        StaticSemaphore_t resourcesMutexBuffer = {};
        SemaphoreHandle_t resourcesMutexHandle;
        uint16_t resouceMutexTimeout = 100; // in ms

        /// Binary semaphores for signaling external events
        StaticSemaphore_t spiWriteCompleteSemaphoreBuffer = {};
        StaticSemaphore_t spiReadCompleteSemaphoreBuffer = {};

        etl::expected<void, Error> spi_block_write_8(uint16_t regAddress, uint8_t* sourceBuff, uint16_t numBytes);
        etl::expected<void, Error> spi_block_read_8(uint16_t regAddress, uint8_t* destBuff, uint16_t numBytes);
    };

    extern AC7Z020_Utilities ac7z020Utils;
}
