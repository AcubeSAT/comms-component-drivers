#include "eMMC.hpp"

namespace eMMC {
    eMMC_Utilities::eMMC_Utilities() {
        eMMC_semaphoreHandle = xSemaphoreCreateMutexStatic(&eMMC_semaphoreBuffer);

    // Initialize the memoryMap array using the sizes from MemoryItems.def
#define MEMORY_ITEM(name, size) memoryItemMap[name] = MemoryItemHandler(size);
#include "MemoryItems.def"
#undef MEMORY_ITEM

    // Initialize the queue map array using MemoryQueues.def
#define MEMORY_QUEUE(queue_name, item_size, queue_size)                                                                                         \
    queue_name##Queue = xQueueCreateStatic(queue_size, sizeof(MemoryQueueHandler), queue_name##QueueStorageArea, &queue_name##QueueBuffer);     \
    vQueueAddToRegistry(queue_name##Queue, " queue_name queue");                                                                                \
    memoryQueueMap[queue_name] = MemoryQueueHandler(item_size, queue_size, &queue_name##Queue);
#include "MemoryQueues.def"
#undef MEMORY_QUEUE

    // TODO ?
    if (memoryItemMap[0].endAddress != 0) {
        __NOP();
    }

    uint64_t headPointer = 0;
    for (int i = 0; i < memoryItemCount; i++) {
        headPointer += memoryItemMap[i].size;
        if (headPointer > memorySizeInBytes) {
            // TODO memory full error
        } else {
            memoryItemMap[i].endAddress = headPointer;
            memoryItemMap[i].startAddress = headPointer - memoryItemMap[i].size;
        }

        // align for page size (512 bytes)
        headPointer += memoryPageSize - (headPointer % memoryPageSize);
    }

    for (int i = 0; i < memoryQueueCount; i++) {
        uint32_t pagesPerItem = memoryQueueMap[i].sizeOfItem / memoryPageSize;
        if (pagesPerItem * memoryPageSize < memoryQueueMap[i].sizeOfItem) { // if item size is not multiple of page size
            pagesPerItem++;
        }
        uint64_t wholeSizeInBytes = memoryPageSize * pagesPerItem * memoryQueueMap[i].numberOfItems;
        headPointer += wholeSizeInBytes;
        if (headPointer > memorySizeInBytes) {
            // TODO memory full error
        } else {
            memoryQueueMap[i].startAddress = headPointer - wholeSizeInBytes;
            memoryQueueMap[i].firstPage = memoryQueueMap[i].startAddress / memoryPageSize;
            memoryQueueMap[i].lastPage = memoryQueueMap[i].firstPage + (pagesPerItem * memoryQueueMap[i].numberOfItems);
        }
    }

    // float eMMC_usage = (100 * static_cast<float>(headPointer)) / static_cast<float>(memoryCapacity);

    // TODO ?
    if (memoryItemMap[0].endAddress != 0) {
        __NOP();
    }
    __NOP();
}

    etl::expected<void, Error> eMMC_Utilities::getItem(const MemoryItem item, uint8_t* destBuffer, const uint32_t bufferSize) {
        const MemoryItemHandler itemHandler = memoryItemMap[item];

        if (bufferSize < itemHandler.size) {
            return etl::unexpected(Error::EMMC_BUFFER_TOO_SMALL);
        }
        uint32_t continuousPages = itemHandler.size / memoryPageSize;
        if (continuousPages > 0) {
            for (uint32_t i = 0; i < continuousPages; i++) {
                /// TODO: handle errors
                if (auto status = readBlockEMMC(destBuffer + (i * memoryPageSize), itemHandler.startAddress + (i * memoryPageSize), 1); !status.has_value()) {
                    return status;
                }
            }
        }

        // compensate for item sizes not multiple of page size
        uint8_t numberOfLastBytes = itemHandler.size % memoryPageSize;
        if (numberOfLastBytes > 0) {
            uint8_t lastPage[memoryPageSize];
            /// TODO: handle errors
            if (auto status = readBlockEMMC(lastPage, itemHandler.startAddress + continuousPages, 1); !status.has_value()) {
                return status;
            }
            for (int i = 0; i < numberOfLastBytes; i++) {
                destBuffer[(continuousPages * memoryPageSize) + i] = lastPage[i];
            }
        }
        return {}; // success
    }

    etl::expected<void, Error> eMMC_Utilities::getItem(const MemoryItem item, uint8_t* destBuffer, const uint32_t bufferSize, const uint32_t startBlock, const uint32_t numOfBlocks) {
        const MemoryItemHandler itemHandler = memoryItemMap[item];

        uint32_t storeDataSize = numOfBlocks * memoryPageSize;
        if ((startBlock + numOfBlocks) * memoryPageSize > itemHandler.size) {
            storeDataSize -= memoryPageSize - (itemHandler.size % memoryPageSize);
        }
        if (bufferSize < storeDataSize) {
            return etl::unexpected(Error::EMMC_BUFFER_TOO_SMALL);
        }
        if ((startBlock * memoryPageSize) + (numOfBlocks * memoryPageSize) > itemHandler.size) {
            if (numOfBlocks > 1) {
                for (uint32_t i = 0; i < numOfBlocks - 1; i++) {
                    /// TODO: handle errors
                    if (auto status = readBlockEMMC(destBuffer + (i * memoryPageSize), itemHandler.startAddress + ((i + startBlock) * memoryPageSize), 1); !status.has_value()) {
                        return status;
                    }
                }
            }
            // compensate for item sizes not multiple of page size
            uint8_t numberOfLastBytes = itemHandler.size % memoryPageSize;
            if (numberOfLastBytes > 0) {
                uint8_t lastPage[memoryPageSize];
                uint32_t lastPageAdress = itemHandler.startAddress + ((itemHandler.size / memoryPageSize) * memoryPageSize);
                /// TODO: handle errors
                if (auto status = readBlockEMMC(lastPage, lastPageAdress, 1); !status.has_value()) {
                    return status;
                }
                for (int i = 0; i < numberOfLastBytes; i++) {
                    destBuffer[((numOfBlocks - 1) * memoryPageSize) + i] = lastPage[i];
                }
            }
        } else {
            if (numOfBlocks > 0) {
                for (uint32_t i = 0; i < numOfBlocks; i++) {
                    /// TODO: handle errors
                    if (auto status = readBlockEMMC(destBuffer + (i * memoryPageSize), itemHandler.startAddress + ((i + startBlock) * memoryPageSize), 1); !status.has_value()) {
                        return status;
                    }
                }
            }
        }

        return {}; // success
    }

    etl::expected<void, Error> eMMC_Utilities::storeItem(const MemoryItem item, uint8_t* sourceBuffer, const uint32_t bufferSize) {
        const MemoryItemHandler itemHandler = memoryItemMap[item];

        if (bufferSize < itemHandler.size) {
            return etl::unexpected(Error::EMMC_BUFFER_TOO_SMALL);
        }
        uint32_t continuousPages = itemHandler.size / memoryPageSize;

        if (continuousPages > 0) {
            for (uint32_t i = 0; i < continuousPages; i++) {
                // handle errors
                if (auto status = writeBlockEMMC(sourceBuffer + (i * memoryPageSize), itemHandler.startAddress + (i * memoryPageSize), 1); !status.has_value()) {
                    return status;
                }
            }
        }

        //compensate for item sizes not multiple of page size
        uint8_t numberOfLastBytes = itemHandler.size % memoryPageSize;
        if (numberOfLastBytes > 0) {
            uint8_t lastPage[memoryPageSize];
            for (int i = 0; i < numberOfLastBytes; i++) {
                lastPage[i] = sourceBuffer[(continuousPages * memoryPageSize) + i];
            }
            // handle errors
            if (auto status = writeBlockEMMC(lastPage, itemHandler.startAddress + continuousPages, 1); !status.has_value()) {
                return status;
            }
        }
        return {}; // success
    }

    etl::expected<void, Error> eMMC_Utilities::storeItem(const MemoryItem item, uint8_t* sourceBuffer, const uint32_t bufferSize,
                                               const uint32_t startBlock, const uint32_t numOfBlocks) {
        const MemoryItemHandler itemHandler = memoryItemMap[item];

        uint32_t storeDataSize = numOfBlocks * memoryPageSize;
        if ((startBlock + numOfBlocks) * memoryPageSize > itemHandler.size) {
            storeDataSize -= memoryPageSize - (itemHandler.size % memoryPageSize);
        }
        if (bufferSize < storeDataSize) {
            return etl::unexpected(Error::EMMC_BUFFER_TOO_SMALL);
        }
        if ((startBlock * memoryPageSize) + (numOfBlocks * memoryPageSize) > itemHandler.size) {
            if (numOfBlocks > 1) {
                for (uint32_t i = 0; i < numOfBlocks - 1; i++) {
                    // TODO handle errors
                    if (auto status = writeBlockEMMC(sourceBuffer + (i * memoryPageSize),
                                                       itemHandler.startAddress + ((i + startBlock) * memoryPageSize),
                                                       1); !status.has_value()) {
                        return status;
                    }
                }
            }

            //compensate for item sizes not multiple of page size
            uint8_t numberOfLastBytes = itemHandler.size % memoryPageSize;
            if (numberOfLastBytes > 0) {
                uint8_t lastPage[memoryPageSize];
                for (int i = 0; i < numberOfLastBytes; i++) {
                    lastPage[i] = sourceBuffer[((numOfBlocks - 1) * memoryPageSize) + i];
                }
                uint32_t lastPageAdress = itemHandler.startAddress + ((itemHandler.size / memoryPageSize) *
                    memoryPageSize);
                // TODO handle errors
                if (auto status = writeBlockEMMC(lastPage, lastPageAdress, 1); !status.has_value()) {
                    return status;
                }
            }
        } else {
            if (numOfBlocks > 0) {
                // auto status = eMMC::writeBlockEMMC(dataBuffer, itemHandler.startAddress + (startBlock * memoryPageSize), numOfBlocks);
                // // handle errors
                // if (!status.has_value()) {
                //     return status;
                // }
                for (uint32_t i = 0; i < numOfBlocks; i++) {
                    // TODO handle errors
                    if (auto status = writeBlockEMMC(sourceBuffer + (i * memoryPageSize),
                                                     itemHandler.startAddress + ((i + startBlock) *
                                                         memoryPageSize),
                                                     1); !status.has_value()) {
                        return status;
                    }
                }
            }
        }

        return {}; // success
    }

    etl::expected<void, Error> eMMC_Utilities::readBlockEMMC(uint8_t* destBuffer, const uint32_t block_address, const uint32_t numberOfBlocks) const {
        xSemaphoreTake(eMMC_semaphoreHandle, portMAX_DELAY);
        eMMCTransactionFlags.ReadComplete = false;
        eMMCTransactionFlags.ErrorOccured = false;
        eMMCTransactionFlags.transactionAborted = false;

        if (HAL_MMC_ReadBlocks_IT(hmmc, destBuffer, block_address, numberOfBlocks) != HAL_OK) {
            xSemaphoreGive(eMMC_semaphoreHandle);
            return etl::unexpected(Error::EMMC_READ_FAILURE);
        }

        const uint32_t startTime = xTaskGetTickCount();
        while (true) {
            vTaskDelay(1);
            if (eMMCTransactionFlags.ReadComplete) {
                xSemaphoreGive(eMMC_semaphoreHandle);
                return {};
            }

            // Transaction timeout
            if (xTaskGetTickCount() > ((transactionTimeoutPerBlock * numberOfBlocks) + startTime)) {
                xSemaphoreGive(eMMC_semaphoreHandle);
                return etl::unexpected(Error::EMMC_TRANSACTION_TIMED_OUT);
            }

            // Error callback was called
            if (eMMCTransactionFlags.ErrorOccured) {
                /// TODO: handle the error, check hmmc handle for error messages.
                xSemaphoreGive(eMMC_semaphoreHandle);
                return etl::unexpected(Error::EMMC_READ_FAILURE);
            }
        }
    }

    etl::expected<void, Error> eMMC_Utilities::writeBlockEMMC(const uint8_t* sourceBuffer, const uint32_t block_address, const uint32_t numberOfBlocks) {

        xSemaphoreTake(eMMC_semaphoreHandle, portMAX_DELAY);
        eMMCTransactionFlags.WriteComplete = false;
        eMMCTransactionFlags.ErrorOccured = false;
        eMMCTransactionFlags.transactionAborted = false;

        if (HAL_MMC_WriteBlocks_IT(hmmc, sourceBuffer, block_address, numberOfBlocks) != HAL_OK) {

            xSemaphoreGive(eMMC_semaphoreHandle);
            return etl::unexpected(Error::EMMC_WRITE_FAILURE);
        }

        const uint32_t startTime = xTaskGetTickCount();
        while (true) {
            vTaskDelay(1);
            if (eMMCTransactionFlags.WriteComplete) {
                xSemaphoreGive(eMMC_semaphoreHandle);
                return {};
            }

            // Transaction timeout
            if (xTaskGetTickCount() > ((transactionTimeoutPerBlock * numberOfBlocks) + startTime)) {
                xSemaphoreGive(eMMC_semaphoreHandle);
                return etl::unexpected(Error::EMMC_TRANSACTION_TIMED_OUT);
            }

            // Error callback was called
            if (eMMCTransactionFlags.ErrorOccured) {
                /// TODO: handle the error, check eMMCTransactionHandler.hmmcSnapshot for error messages.
                xSemaphoreGive(eMMC_semaphoreHandle);
                return etl::unexpected(Error::EMMC_WRITE_FAILURE);
            }
        }
    }

    etl::expected<void, Error> eMMC_Utilities::eraseBlocksEMMC(const uint32_t block_address_start,const  uint32_t block_address_end) {
        if (block_address_start > block_address_end)
            return etl::unexpected(Error::EMMC_INVALID_START_ADDRESS_ON_ERASE);
        if (HAL_MMC_Erase(hmmc, block_address_start, block_address_end) != HAL_OK)
            return etl::unexpected(Error::EMMC_ERASE_BLOCK_FAILURE);
        return {}; // success
    }

    eMMC_Utilities eMMC_Utils = eMMC_Utilities();
    EMMCTransactionFlags eMMCTransactionFlags;
} // namespace eMMC

// etl::expected<void, Error> eMMC::getItemFromQueue(memoryQueueHandler queueHandler, memoryQueueItemHandler itemHandler, uint8_t* buffer, uint32_t bufferSize) {
//
//     if (bufferSize < itemHandler.size) {
//         return etl::unexpected<Error>(Error::EMMC_BUFFER_TOO_SMALL);
//     }
//     uint8_t localBuffer[memoryPageSize];
//     auto status = eMMC::readBlockEMMC(localBuffer, itemHandler.startPage * memoryPageSize, 1);
//     /// TODO: handle errors
//     if (!status.has_value()) {
//         return status;
//     }
//
//     uint32_t pageCounter = 0;
//     for (int i = 0; i < itemHandler.size; i++) {
//         buffer[i] = localBuffer[i - (pageCounter * memoryPageSize)];
//         if (i >= ((pageCounter + 1) * memoryPageSize) - 1) {
//             pageCounter++;
//             status = eMMC::readBlockEMMC(localBuffer, memoryPageSize * (pageCounter + itemHandler.startPage), 1);
//             /// TODO: handle errors
//             if (!status.has_value()) {
//                 return status;
//             }
//         }
//     }
//
//     return {}; // success
// }


// etl::expected<void, Error> eMMC::storeItemInQueue(memoryQueueHandler queueHandler, memoryQueueItemHandler* itemHandler, uint8_t* buffer, uint32_t bufferSize) {
//     if (bufferSize < itemHandler->size) {
//         return etl::unexpected<Error>(Error::EMMC_BUFFER_TOO_SMALL);
//     }
//     itemHandler->startPage = queueHandler.tailPageOffset + queueHandler.firstPage;
//     queueHandler.tailPageOffset++;
//     if (queueHandler.tailPageOffset > queueHandler.lastPage - queueHandler.firstPage) {
//         queueHandler.tailPageOffset = 0;
//     }
//     uint8_t localBuffer[memoryPageSize];
//     uint32_t pageCounter = 0;
//     for (int i = 0; i < itemHandler->size; i++) {
//         localBuffer[i - (pageCounter * memoryPageSize)] = buffer[i];
//         if (i >= (memoryPageSize * (pageCounter + 1)) - 1 || i == (itemHandler->size - 1)) {
//             auto status = eMMC::writeBlockEMMC(localBuffer, (itemHandler->startPage + pageCounter) * memoryPageSize, 1);
//             pageCounter++;
//             /// TODO: handle errors
//             if (!status.has_value()) {
//                 return status;
//             }
//         }
//     }
//     return {}; // success
// }






