#include <cstring>
#include "eMMC.hpp"
#include "Logger.hpp"

namespace eMMC {
    // definition
    eMMC_Utilities eMMC_Utils = eMMC_Utilities();

    etl::expected<float, Error> eMMC_Utilities::initializeResources(MMC_HandleTypeDef* handle) {
        hmmc = handle;

        eMMC_access_semaphoreHandle = xSemaphoreCreateMutexStatic(&eMMC_access_semaphoreBuffer);

        // Initialize the event group
        eventGroupHandle = xEventGroupCreateStatic(&eventGroupBuffer);

        if (eMMC_access_semaphoreHandle == nullptr || eventGroupHandle == nullptr) {
            return etl::unexpected(Error::EMMC_FREERTOS_RESOURCE_INITIALIZATION_FAILED);
        }

        // Fetch logical block size and count
        HAL_MMC_CardInfoTypeDef pCardInfo;
        HAL_MMC_GetCardInfo(hmmc, &pCardInfo);
        logicalBlockSize = pCardInfo.LogBlockSize;
        logicalBlockCount = pCardInfo.LogBlockNbr;
        memorySizeInBytes = static_cast<uint64_t>(logicalBlockSize) * static_cast<uint64_t>(logicalBlockCount);

        // Initialize the memoryMap array using the sizes from MemoryItems.def
#define MEMORY_ITEM(name, size) memoryItemMap[name] = MemoryItemHandler(size);
#include "MemoryItems.def"
#undef MEMORY_ITEM

        // Calculate MemoryItemHandler parameters
        uint64_t headBlockPointer = 0;
        for (uint16_t i = 0; i < memoryItemCount; i++) {
            if (memoryItemMap[i].size == 0) {
                return etl::unexpected(Error::EMMC_SPECIFIED_ZERO_LENGTH_OBJECT);
            }

            memoryItemMap[i].startBlockAddress = headBlockPointer;
            memoryItemMap[i].hasPartialBlock = static_cast<bool>(memoryItemMap[i].size % logicalBlockSize);
            headBlockPointer += memoryItemMap[i].size / logicalBlockSize + static_cast<uint64_t>(memoryItemMap[i].hasPartialBlock); // points one block past the allocated space

            if (headBlockPointer > static_cast<uint64_t>(logicalBlockCount)) {
                return etl::unexpected(Error::EMMC_SURPASSED_MEMORY_CONSTRAINTS);
            }
            memoryItemMap[i].endBlockAddress = headBlockPointer - 1;
            memoryItemMap[i].semaphoreHandle = xSemaphoreCreateMutexStatic(&memoryItemMap[i].semaphoreBuffer);
        }

        // Initialize the queue map array using MemoryQueues.def
#define MEMORY_QUEUE(queue_name, item_size, queue_size, reboot_persistence) memoryQueueMap[queue_name] = MemoryQueueHandler(item_size, queue_size, reboot_persistence);
#include "MemoryQueues.def"
#undef MEMORY_QUEUE

        // Calculate QueueHandler parameters
        for (uint8_t i = 0; i < memoryQueueCount; i++) {
            if (memoryQueueMap[i].itemSize == 0 || memoryQueueMap[i].maxNumberOfItems == 0) {
                return etl::unexpected(Error::EMMC_SPECIFIED_ZERO_LENGTH_OBJECT);
            }

            memoryQueueMap[i].itemHasPartialBlock = memoryQueueMap[i].itemSize % logicalBlockSize;
            memoryQueueMap[i].slotBlockSize = memoryQueueMap[i].itemSize / logicalBlockSize + memoryQueueMap[i].itemHasPartialBlock;
            memoryQueueMap[i].startBlockAddress = headBlockPointer;
            headBlockPointer += memoryQueueMap[i].slotBlockSize * memoryQueueMap[i].maxNumberOfItems
                                + memoryQueueMap[i].isRebootPersistent; // points one block past the allocated space

            if (headBlockPointer > static_cast<uint64_t>(logicalBlockCount)) {
                return etl::unexpected(Error::EMMC_SURPASSED_MEMORY_CONSTRAINTS);
            }
            memoryQueueMap[i].endBlockAddress = headBlockPointer - 1 - memoryQueueMap[i].isRebootPersistent;
            memoryQueueMap[i].semaphoreHandle = xSemaphoreCreateMutexStatic(&memoryQueueMap[i].semaphoreBuffer);

            if (memoryQueueMap[i].isRebootPersistent) {
                // Extract head, tail pointers.
                // Use the metadata block values only if they satisfy basic sanity checks. Otherwise, reset the queue.
                uint8_t metadataBuff[logicalBlockSize];
                if (readBlockEMMC(metadataBuff, memoryQueueMap[i].endBlockAddress + 1, 1).has_value()) {
                    const uint32_t head =
                        static_cast<uint32_t>(metadataBuff[0]) << 24 |
                        static_cast<uint32_t>(metadataBuff[1]) << 16 |
                        static_cast<uint32_t>(metadataBuff[2]) << 8 |
                        static_cast<uint32_t>(metadataBuff[3]);

                    const uint32_t tail =
                        static_cast<uint32_t>(metadataBuff[4]) << 24 |
                        static_cast<uint32_t>(metadataBuff[5]) << 16 |
                        static_cast<uint32_t>(metadataBuff[6]) << 8 |
                        static_cast<uint32_t>(metadataBuff[7]);

                    const uint32_t numberOfItems =
                        static_cast<uint32_t>(metadataBuff[8]) << 24 |
                        static_cast<uint32_t>(metadataBuff[9]) << 16 |
                        static_cast<uint32_t>(metadataBuff[10]) << 8 |
                        static_cast<uint32_t>(metadataBuff[11]);

                    // The distance should be equal to numberOfItems, except in the edge case where the queue is full,
                    // where head == tail (distance == 0) and numberOfItems == maximum number of items
                    const uint32_t distance = ((tail >= head) ? (tail - head) : (tail + memoryQueueMap[i].maxNumberOfItems - head));
                    const uint32_t maxNumberOfItems = memoryQueueMap[i].maxNumberOfItems;
                    if (head < maxNumberOfItems &&
                        tail < maxNumberOfItems &&
                        numberOfItems <= maxNumberOfItems &&
                         ((distance == numberOfItems) || (distance == 0 && numberOfItems == maxNumberOfItems))) {
                        memoryQueueMap[i].headSlotPointer = head;
                        memoryQueueMap[i].tailSlotPointer = tail;
                        memoryQueueMap[i].currentNumberOfItems = numberOfItems;
                    } else {
                        LOG_DEBUG << "Metadata block check in persistent queue with ID " << i << " failed. Resetting queue...";
                        const etl::expected<void, Error> status = resetQueue(static_cast<MemoryQueue>(i));
                        if (!status.has_value()) {
                            return etl::unexpected(status.error());
                        }
                    }
                }
            }
        }

        emmcUsage = 100 * static_cast<float>(headBlockPointer-1) / logicalBlockCount;
        return emmcUsage;
    }

    etl::expected<void, Error> eMMC_Utilities::getItem(const MemoryItem item, uint8_t* destBuffer, const uint32_t bufferSize, const uint32_t startBlock, const uint32_t numOfBlocks) {
        const MemoryItemHandler& itemHandler = memoryItemMap[item];
        if (xSemaphoreTake(itemHandler.semaphoreHandle, pdMS_TO_TICKS(SemaphoreTimeoutMs)) != pdTRUE) {
            return etl::unexpected(Error::EMMC_MUTEX_LOCK_TIMEOUT);
        }

        // requested block region checks
        if (numOfBlocks == 0 || itemHandler.startBlockAddress + startBlock + (numOfBlocks - 1) > itemHandler.endBlockAddress) {
            xSemaphoreGive(itemHandler.semaphoreHandle);
            return etl::unexpected(Error::EMMC_INVALID_MEMORY_BLOCK_REGION);
        }

        // buffer size checks
        const uint32_t lastRequestedBlock = itemHandler.startBlockAddress + startBlock + (numOfBlocks - 1);
        const bool requestEndsOnLastItemBlock = (lastRequestedBlock == itemHandler.endBlockAddress);
        if (!itemHandler.hasPartialBlock || // item is block aligned (not leftover bits in the end)
            !requestEndsOnLastItemBlock) {     // OR the last block is not requested
            // Standard check
            if (bufferSize < numOfBlocks * logicalBlockSize) {
                xSemaphoreGive(itemHandler.semaphoreHandle);
                return etl::unexpected(Error::EMMC_BUFFER_TOO_SMALL);
            }
        } else { // the last requested block is the last item block AND there are leftover bits in the end
            // The buffer can be smaller than numOfBlocks * logicalBlockSize
            if (bufferSize < (numOfBlocks - 1) * logicalBlockSize + itemHandler.size % logicalBlockSize) {
                xSemaphoreGive(itemHandler.semaphoreHandle);
                return etl::unexpected(Error::EMMC_BUFFER_TOO_SMALL);
            }
        }

        etl::expected<void, Error> status;
        if (!itemHandler.hasPartialBlock ||  // item is block aligned (not leftover bits in the end)
            !requestEndsOnLastItemBlock) {      // OR the last block is not requested
            // so do a full read on the last requested block
            status = readBlockEMMC(destBuffer, itemHandler.startBlockAddress + startBlock, numOfBlocks);
            xSemaphoreGive(itemHandler.semaphoreHandle);
            return status;
        } else { // the last requested block is the last item block AND there are leftover bits in the end
            // so the last block must only be partially copied

            // full read (continuous blocks)
            if (itemHandler.startBlockAddress + startBlock != itemHandler.endBlockAddress) { // check that there is actually more than one block to copy
                status = readBlockEMMC(destBuffer, itemHandler.startBlockAddress + startBlock, numOfBlocks - 1);
                if (!status.has_value()) {
                    xSemaphoreGive(itemHandler.semaphoreHandle);
                    return status;
                }
            }

            // partial read
            uint8_t lastBlock[logicalBlockSize];
            status = readBlockEMMC(lastBlock, lastRequestedBlock, 1);
            if (!status.has_value()) {
                xSemaphoreGive(itemHandler.semaphoreHandle);
                return status;
            }

            // copy tail bytes from last block
            std::memcpy(destBuffer + (numOfBlocks - 1) * logicalBlockSize, lastBlock, itemHandler.size % logicalBlockSize);

            xSemaphoreGive(itemHandler.semaphoreHandle);
            return {}; // success
        }
    }

    etl::expected<void, Error> eMMC_Utilities::getItem(const MemoryItem item, uint8_t* destBuffer, const uint32_t bufferSize) {
        const MemoryItemHandler& itemHandler = memoryItemMap[item];
        return getItem(item, destBuffer, bufferSize, 0, itemHandler.endBlockAddress - itemHandler.startBlockAddress + 1);
    }

    etl::expected<void, Error> eMMC_Utilities::storeItem(const MemoryItem item, uint8_t* sourceBuffer, const uint32_t bufferSize) {
        const MemoryItemHandler& itemHandler = memoryItemMap[item];
        if (xSemaphoreTake(itemHandler.semaphoreHandle, pdMS_TO_TICKS(SemaphoreTimeoutMs)) != pdTRUE) {
            return etl::unexpected(Error::EMMC_MUTEX_LOCK_TIMEOUT);
        }

        if (bufferSize < itemHandler.size) {
            xSemaphoreGive(itemHandler.semaphoreHandle);
            return etl::unexpected(Error::EMMC_BUFFER_TOO_SMALL);
        }

        // full read (continuous pages)
        etl::expected<void, Error> status;
        const uint32_t continuousPages = itemHandler.size / logicalBlockSize;
        if (continuousPages != 0) {
            status = writeBlockEMMC(sourceBuffer, itemHandler.startBlockAddress, continuousPages);
            if (!status.has_value()) {
                xSemaphoreGive(itemHandler.semaphoreHandle);
                return status;
            }
        }

        // if the item is not a multiple of block size, the last block needs to be copied
        // separately (due to sourceBuffer not being large enough)
        if (itemHandler.hasPartialBlock) {
            uint8_t lastBlock[logicalBlockSize] = {0};
            std::memcpy(lastBlock, sourceBuffer + continuousPages * logicalBlockSize, itemHandler.size % logicalBlockSize);

            status = writeBlockEMMC(lastBlock, itemHandler.endBlockAddress, 1);
            if (!status.has_value()) {
                xSemaphoreGive(itemHandler.semaphoreHandle);
                return status;
            }
        }

        xSemaphoreGive(itemHandler.semaphoreHandle);
        return {}; // success
    }

    etl::expected<void, Error> eMMC_Utilities::resetItem(MemoryItem item) {
        const MemoryItemHandler& itemHandler = memoryItemMap[item];
        if (xSemaphoreTake(itemHandler.semaphoreHandle, pdMS_TO_TICKS(SemaphoreTimeoutMs)) != pdTRUE) {
            return etl::unexpected(Error::EMMC_MUTEX_LOCK_TIMEOUT);
        }

        etl::expected<void, Error> status = eraseBlocksEMMC(itemHandler.startBlockAddress, itemHandler.endBlockAddress);
        if (!status.has_value()) {
            xSemaphoreGive(itemHandler.semaphoreHandle);
            return status;
        }

        xSemaphoreGive(itemHandler.semaphoreHandle);
        return {};
    }

    etl::pair<uint32_t, Error> eMMC_Utilities::popItemsFromQueue(const MemoryQueue queue, uint8_t* destBuffer, const uint32_t bufferSize, const uint32_t numItems) {
        MemoryQueueHandler& queueHandler = memoryQueueMap[queue];
        if (xSemaphoreTake(queueHandler.semaphoreHandle, pdMS_TO_TICKS(SemaphoreTimeoutMs)) != pdTRUE) {
            return etl::make_pair(0, Error::EMMC_MUTEX_LOCK_TIMEOUT);
        }

        if (numItems == 0 || numItems > queueHandler.maxNumberOfItems) {
            xSemaphoreGive(queueHandler.semaphoreHandle);
            return etl::make_pair(0, Error::EMMC_INVALID_NUMBER_OF_ITEMS);
        }

        if (queueHandler.currentNumberOfItems == 0) {
            xSemaphoreGive(queueHandler.semaphoreHandle);
            return etl::make_pair(0, Error::EMMC_QUEUE_EMPTY);
        }

        const uint32_t itemsToPop =  numItems > queueHandler.currentNumberOfItems ? queueHandler.currentNumberOfItems : numItems;
        if (bufferSize < queueHandler.itemSize * itemsToPop) {
            xSemaphoreGive(queueHandler.semaphoreHandle);
            return etl::make_pair(0, Error::EMMC_BUFFER_TOO_SMALL);
        }

        const uint32_t initHeadSlotPointer = queueHandler.headSlotPointer;
        const uint32_t initTailSlotPointer = queueHandler.tailSlotPointer;
        const uint32_t initCurrentNumberOfItems = queueHandler.currentNumberOfItems;

        etl::expected<void, Error> status = {};
        uint8_t lastBlock[logicalBlockSize] = {0}; // in case the items are not a multiple of the block size
        for (uint32_t i = 0; i < itemsToPop; i++) {
            if (queueHandler.itemHasPartialBlock) {
                // full read for the continuous blocks
                if (queueHandler.slotBlockSize != 1) {
                    status = readBlockEMMC(destBuffer + i * queueHandler.itemSize, queueHandler.startBlockAddress +
                        queueHandler.headSlotPointer * queueHandler.slotBlockSize, queueHandler.slotBlockSize - 1);
                    if (!status.has_value()) {
                        // operation failed
                        xSemaphoreGive(queueHandler.semaphoreHandle);
                        return etl::make_pair(i, status.error());
                    }
                }

                // partial read for the last block
                status = readBlockEMMC(lastBlock, queueHandler.startBlockAddress +
                    queueHandler.headSlotPointer * queueHandler.slotBlockSize + (queueHandler.slotBlockSize - 1), 1);
                if (!status.has_value()) {
                    // operation failed
                    xSemaphoreGive(queueHandler.semaphoreHandle);
                    return etl::make_pair(i, status.error());
                }
                std::memcpy(destBuffer + i * queueHandler.itemSize + (queueHandler.slotBlockSize - 1) * logicalBlockSize, lastBlock, queueHandler.itemSize % logicalBlockSize);
            } else {
                status = readBlockEMMC(destBuffer + i * queueHandler.itemSize, queueHandler.startBlockAddress +
                    queueHandler.headSlotPointer * queueHandler.slotBlockSize, queueHandler.slotBlockSize);
                if (!status.has_value()) {
                    // operation failed
                    xSemaphoreGive(queueHandler.semaphoreHandle);
                    return etl::make_pair(i, status.error());
                }
            }

            // operation successful -> move pointer
            queueHandler.headSlotPointer = (queueHandler.headSlotPointer + 1) % queueHandler.maxNumberOfItems; // circular increment
            queueHandler.currentNumberOfItems--;
        }

        // update metadata block
        if (queueHandler.isRebootPersistent) {
            const uint8_t metadataBuff[logicalBlockSize] = {
                static_cast<uint8_t>(queueHandler.headSlotPointer >> 24),
                static_cast<uint8_t>(queueHandler.headSlotPointer >> 16),
                static_cast<uint8_t>(queueHandler.headSlotPointer >> 8),
                static_cast<uint8_t>(queueHandler.headSlotPointer),
                static_cast<uint8_t>(queueHandler.tailSlotPointer >> 24),
                static_cast<uint8_t>(queueHandler.tailSlotPointer >> 16),
                static_cast<uint8_t>(queueHandler.tailSlotPointer >> 8),
                static_cast<uint8_t>(queueHandler.tailSlotPointer),
                static_cast<uint8_t>(queueHandler.currentNumberOfItems >> 24),
                static_cast<uint8_t>(queueHandler.currentNumberOfItems >> 16),
                static_cast<uint8_t>(queueHandler.currentNumberOfItems >> 8),
                static_cast<uint8_t>(queueHandler.currentNumberOfItems),
            };
            status = writeBlockEMMC(metadataBuff,queueHandler.endBlockAddress + 1, 1);
            if (!status.has_value()) {
                queueHandler.headSlotPointer = initHeadSlotPointer;
                queueHandler.tailSlotPointer = initTailSlotPointer;
                queueHandler.currentNumberOfItems = initCurrentNumberOfItems;
                xSemaphoreGive(queueHandler.semaphoreHandle);
                return etl::make_pair(0, status.error());
            }
        }
        xSemaphoreGive(queueHandler.semaphoreHandle);
        return etl::make_pair(itemsToPop, Error::EMMC_NO_ERROR); // success
    }

    etl::pair<uint32_t, Error> eMMC_Utilities::pushItemsToQueue(const MemoryQueue queue, uint8_t* sourceBuffer, const uint32_t bufferSize, const uint32_t numItems) {
        MemoryQueueHandler& queueHandler = memoryQueueMap[queue];
        if (xSemaphoreTake(queueHandler.semaphoreHandle, pdMS_TO_TICKS(SemaphoreTimeoutMs)) != pdTRUE) {
            return etl::make_pair(0, Error::EMMC_MUTEX_LOCK_TIMEOUT);
        }

        if (numItems == 0 || numItems > queueHandler.maxNumberOfItems) {
            xSemaphoreGive(queueHandler.semaphoreHandle);
            return etl::make_pair(0, Error::EMMC_INVALID_NUMBER_OF_ITEMS);
        }

        if (queueHandler.currentNumberOfItems == queueHandler.maxNumberOfItems) {
            xSemaphoreGive(queueHandler.semaphoreHandle);
            return etl::make_pair(0, Error::EMMC_QUEUE_FULL);
        }

        const uint32_t itemsToPush = numItems > (queueHandler.maxNumberOfItems - queueHandler.currentNumberOfItems) ?
            (queueHandler.maxNumberOfItems - queueHandler.currentNumberOfItems) : numItems;
        if (bufferSize < queueHandler.itemSize * itemsToPush) {
            xSemaphoreGive(queueHandler.semaphoreHandle);
            return etl::make_pair(0, Error::EMMC_BUFFER_TOO_SMALL);
        }

        const uint32_t initHeadSlotPointer = queueHandler.headSlotPointer;
        const uint32_t initTailSlotPointer = queueHandler.tailSlotPointer;
        const uint32_t initCurrentNumberOfItems = queueHandler.currentNumberOfItems;

        etl::expected<void, Error> status;
        for (uint32_t i = 0; i < itemsToPush; i++) {
            if (queueHandler.itemHasPartialBlock) {
                // full write (for continuous blocks)
                if (queueHandler.slotBlockSize != 1) {
                    status = writeBlockEMMC(sourceBuffer + i * queueHandler.itemSize, queueHandler.startBlockAddress +
                        queueHandler.tailSlotPointer * queueHandler.slotBlockSize, queueHandler.slotBlockSize - 1);
                    if (!status.has_value()) {
                        // operation failed
                        xSemaphoreGive(queueHandler.semaphoreHandle);
                        return etl::make_pair(i, status.error());
                    }
                }

                // partial write for the last block (so there may be no reading beyond the sourceBuffer's edge)
                uint8_t lastBlock[logicalBlockSize] = {0};
                std::memcpy(lastBlock,
                    sourceBuffer + i * queueHandler.itemSize + (queueHandler.slotBlockSize - 1) * logicalBlockSize,
                    queueHandler.itemSize % logicalBlockSize);
                status = writeBlockEMMC(lastBlock,
                    queueHandler.startBlockAddress + queueHandler.tailSlotPointer * queueHandler.slotBlockSize + (queueHandler.slotBlockSize - 1),
                    1);
                if (!status.has_value()) {
                    // operation failed
                    xSemaphoreGive(queueHandler.semaphoreHandle);
                    return etl::make_pair(i, status.error());
                }
            } else {
                status = writeBlockEMMC(sourceBuffer + i * queueHandler.itemSize, queueHandler.startBlockAddress +
                    queueHandler.tailSlotPointer * queueHandler.slotBlockSize, queueHandler.slotBlockSize);
                if (!status.has_value()) {
                    // operation failed
                    xSemaphoreGive(queueHandler.semaphoreHandle);
                    return etl::make_pair(i, status.error());
                }
            }

            // operation successful -> move pointer
            queueHandler.tailSlotPointer = (queueHandler.tailSlotPointer + 1) % queueHandler.maxNumberOfItems; // circular increment
            queueHandler.currentNumberOfItems++;
        }

        // update metadata block
        if (queueHandler.isRebootPersistent) {
            const uint8_t metadataBuff[logicalBlockSize] = {
                static_cast<uint8_t>(queueHandler.headSlotPointer >> 24),
                static_cast<uint8_t>(queueHandler.headSlotPointer >> 16),
                static_cast<uint8_t>(queueHandler.headSlotPointer >> 8),
                static_cast<uint8_t>(queueHandler.headSlotPointer),
                static_cast<uint8_t>(queueHandler.tailSlotPointer >> 24),
                static_cast<uint8_t>(queueHandler.tailSlotPointer >> 16),
                static_cast<uint8_t>(queueHandler.tailSlotPointer >> 8),
                static_cast<uint8_t>(queueHandler.tailSlotPointer),
                static_cast<uint8_t>(queueHandler.currentNumberOfItems >> 24),
                static_cast<uint8_t>(queueHandler.currentNumberOfItems >> 16),
                static_cast<uint8_t>(queueHandler.currentNumberOfItems >> 8),
                static_cast<uint8_t>(queueHandler.currentNumberOfItems),
            };
            status = writeBlockEMMC(metadataBuff,queueHandler.endBlockAddress + 1, 1);
            if (!status.has_value()) {
                queueHandler.headSlotPointer = initHeadSlotPointer;
                queueHandler.tailSlotPointer = initTailSlotPointer;
                queueHandler.currentNumberOfItems = initCurrentNumberOfItems;
                xSemaphoreGive(queueHandler.semaphoreHandle);
                return etl::make_pair(0, status.error());
            }
        }
        xSemaphoreGive(queueHandler.semaphoreHandle);
        return etl::make_pair(itemsToPush, Error::EMMC_NO_ERROR);
    }

    etl::expected<void, Error> eMMC_Utilities::resetQueue(MemoryQueue queue) {
        MemoryQueueHandler& queueHandler = memoryQueueMap[queue];
        if (xSemaphoreTake(queueHandler.semaphoreHandle, pdMS_TO_TICKS(SemaphoreTimeoutMs)) != pdTRUE) {
            return etl::unexpected(Error::EMMC_MUTEX_LOCK_TIMEOUT);
        }

        etl::expected<void, Error> status = eraseBlocksEMMC(queueHandler.startBlockAddress, queueHandler.endBlockAddress);
        if (!status.has_value()) {
            xSemaphoreGive(queueHandler.semaphoreHandle);
            return status;
        }

        queueHandler.currentNumberOfItems = 0;
        queueHandler.headSlotPointer = 0;
        queueHandler.tailSlotPointer = 0;

        // update metadata block
        if (queueHandler.isRebootPersistent) {
            const uint8_t metadataBuff[logicalBlockSize] = {0};
            status = writeBlockEMMC(metadataBuff,queueHandler.endBlockAddress + 1, 1);
            if (!status.has_value()) {
                xSemaphoreGive(queueHandler.semaphoreHandle);
                return status;
            }
        }
        xSemaphoreGive(queueHandler.semaphoreHandle);
        return {};
    }

    etl::expected<void, Error> eMMC_Utilities::readBlockEMMC(uint8_t* destBuffer, const uint32_t block_address, const uint32_t numberOfBlocks) {
        if (xSemaphoreTake(eMMC_access_semaphoreHandle, pdMS_TO_TICKS(SemaphoreTimeoutMs)) != pdTRUE) {
            return etl::unexpected(Error::EMMC_MUTEX_LOCK_TIMEOUT);
        }

        // reset event bits
        xEventGroupClearBits(eventGroupHandle, readCompleteGroupBit | errorOccuredGroupBit | transactionAbortedGroupBit);

        if (HAL_MMC_ReadBlocks_IT(hmmc, destBuffer, block_address, numberOfBlocks) != HAL_OK) {
            xSemaphoreGive(eMMC_access_semaphoreHandle);
            return etl::unexpected(Error::EMMC_READ_FAILURE);
        }

        // wait until an interrupt occurs
        uint32_t eventBits = xEventGroupWaitBits(eventGroupHandle,
            readCompleteGroupBit | errorOccuredGroupBit | transactionAbortedGroupBit,
            pdFALSE, pdFALSE, pdMS_TO_TICKS(TransactionTimeoutPerBlockMs * numberOfBlocks));

        if (!eventBits) {
            // timed out
            xSemaphoreGive(eMMC_access_semaphoreHandle);
            return etl::unexpected(Error::EMMC_TRANSACTION_TIMED_OUT);
        }

        if (eventBits & readCompleteGroupBit) {
            // once a transaction is complete, the card should return to state transfer

            // poll the card state
            uint32_t pollingTimeMs = 0;
            bool success = false;
            while (pollingTimeMs < MaxSuccessfulTransactionDelayMs) {
                vTaskDelay(pdMS_TO_TICKS(SuccessfulTransactionPollingPeriodMs));
                pollingTimeMs += SuccessfulTransactionPollingPeriodMs;
                if (HAL_MMC_GetCardState(hmmc) == HAL_MMC_CARD_TRANSFER) {
                    success = true;
                    break;
                }
            }
            xSemaphoreGive(eMMC_access_semaphoreHandle);

            if (!success) {
                return etl::unexpected(Error::EMMC_READ_FAILURE);
            }
            return {};
        }

        if (eventBits & errorOccuredGroupBit) {
            // error callback was called
            xSemaphoreGive(eMMC_access_semaphoreHandle);
            return etl::unexpected(Error::EMMC_READ_FAILURE);
        }

        if (eventBits & transactionAbortedGroupBit) {
            // abort callback was called
            xSemaphoreGive(eMMC_access_semaphoreHandle);
            return etl::unexpected(Error::EMMC_TRANSACTION_ABORTED);
        }

        // the code should not reach here
        return etl::unexpected(Error::EMMC_READ_FAILURE);
    }

    etl::expected<void, Error> eMMC_Utilities::writeBlockEMMC(const uint8_t* sourceBuffer, const uint32_t block_address, const uint32_t numberOfBlocks) {
        if (xSemaphoreTake(eMMC_access_semaphoreHandle, pdMS_TO_TICKS(SemaphoreTimeoutMs)) != pdTRUE) {
            return etl::unexpected(Error::EMMC_MUTEX_LOCK_TIMEOUT);
        }

        // reset event bits
        xEventGroupClearBits(eventGroupHandle, writeCompleteGroupBit | errorOccuredGroupBit | transactionAbortedGroupBit);

        if (HAL_MMC_WriteBlocks_IT(hmmc, sourceBuffer, block_address, numberOfBlocks) != HAL_OK) {
            xSemaphoreGive(eMMC_access_semaphoreHandle);
            return etl::unexpected(Error::EMMC_WRITE_FAILURE);
        }

        // wait until an interrupt occurs
        uint32_t eventBits = xEventGroupWaitBits(eventGroupHandle,
            writeCompleteGroupBit | errorOccuredGroupBit | transactionAbortedGroupBit,
            pdFALSE, pdFALSE, pdMS_TO_TICKS(TransactionTimeoutPerBlockMs * numberOfBlocks));

        if (!eventBits) {
            // timed out
            xSemaphoreGive(eMMC_access_semaphoreHandle);
            return etl::unexpected(Error::EMMC_TRANSACTION_TIMED_OUT);
        }

        if (eventBits & writeCompleteGroupBit) {
            // once a transaction is complete, the card should return to state transfer

            // poll the card state
            uint32_t pollingTimeMs = 0;
            bool success = false;
            while (pollingTimeMs < MaxSuccessfulTransactionDelayMs) {
                vTaskDelay(pdMS_TO_TICKS(SuccessfulTransactionPollingPeriodMs));
                pollingTimeMs += SuccessfulTransactionPollingPeriodMs;
                if (HAL_MMC_GetCardState(hmmc) == HAL_MMC_CARD_TRANSFER) {
                    success = true;
                    break;
                }
            }
            xSemaphoreGive(eMMC_access_semaphoreHandle);

            if (!success) {
              return etl::unexpected(Error::EMMC_WRITE_FAILURE);
            }
            return {};
        }

        if (eventBits & errorOccuredGroupBit) {
            // error callback was called
            xSemaphoreGive(eMMC_access_semaphoreHandle);
            return etl::unexpected(Error::EMMC_WRITE_FAILURE);
        }

        if (eventBits & transactionAbortedGroupBit) {
            // abort callback was called
            xSemaphoreGive(eMMC_access_semaphoreHandle);
            return etl::unexpected(Error::EMMC_TRANSACTION_ABORTED);
        }

        // the code should not reach here
        return etl::unexpected(Error::EMMC_WRITE_FAILURE);
    }

    etl::expected<void, Error> eMMC_Utilities::eraseBlocksEMMC(const uint32_t block_address_start, const  uint32_t block_address_end) {
        if (xSemaphoreTake(eMMC_access_semaphoreHandle, pdMS_TO_TICKS(SemaphoreTimeoutMs)) != pdTRUE) {
            return etl::unexpected(Error::EMMC_MUTEX_LOCK_TIMEOUT);
        }

        if (block_address_start > block_address_end) {
            xSemaphoreGive(eMMC_access_semaphoreHandle);
            return etl::unexpected(Error::EMMC_INVALID_MEMORY_BLOCK_REGION);
        }

        if (HAL_MMC_Erase(hmmc, block_address_start, block_address_end) != HAL_OK) {
            xSemaphoreGive(eMMC_access_semaphoreHandle);
            return etl::unexpected(Error::EMMC_ERASE_BLOCK_FAILURE);
        }

        HAL_MMC_CardStateTypeDef status = HAL_MMC_GetCardState(hmmc);
        if (status ==  HAL_MMC_CARD_ERROR) {
            xSemaphoreGive(eMMC_access_semaphoreHandle);
            return etl::unexpected(Error::EMMC_WRITE_FAILURE);
        }

        // wait until the erase procedure is complete
        do {
            vTaskDelay(pdMS_TO_TICKS(ErasePollingPeriodMs));
            status = HAL_MMC_GetCardState(hmmc);
        } while (status != HAL_MMC_CARD_STANDBY && status != HAL_MMC_CARD_TRANSFER);

        xSemaphoreGive(eMMC_access_semaphoreHandle);
        return {}; // success
    }

    void eMMC_Utilities::printError(Error error) {
        switch (error) {
        case Error::EMMC_NO_ERROR:
            LOG_DEBUG << "EMMC_NO_ERROR";
            break;
        case Error::EMMC_READ_FAILURE:
            LOG_DEBUG << "EMMC_READ_FAILURE";
            break;
        case Error::EMMC_WRITE_FAILURE:
            LOG_DEBUG << "EMMC_WRITE_FAILURE";
            break;
        case Error::EMMC_ERASE_BLOCK_FAILURE:
            LOG_DEBUG << "EMMC_ERASE_BLOCK_FAILURE";
            break;
        case Error::EMMC_INVALID_MEMORY_BLOCK_REGION:
            LOG_DEBUG << "EMMC_INVALID_MEMORY_BLOCK_REGION";
            break;
        case Error::EMMC_TRANSACTION_TIMED_OUT:
            LOG_DEBUG << "EMMC_TRANSACTION_TIMED_OUT";
            break;
        case Error::EMMC_TRANSACTION_ABORTED:
            LOG_DEBUG << "EMMC_TRANSACTION_ABORTED";
            break;
        case Error::EMMC_MUTEX_LOCK_TIMEOUT:
            LOG_DEBUG << "EMMC_MUTEX_LOCK_TIMEOUT";
            break;
        case Error::EMMC_BUFFER_TOO_SMALL:
            LOG_DEBUG << "EMMC_BUFFER_TOO_SMALL";
            break;
        case Error::EMMC_QUEUE_FULL:
            LOG_DEBUG << "EMMC_QUEUE_FULL";
            break;
        case Error::EMMC_QUEUE_EMPTY:
            LOG_DEBUG << "EMMC_QUEUE_EMPTY";
            break;
        case Error::EMMC_INVALID_NUMBER_OF_ITEMS:
            LOG_DEBUG << "EMMC_INVALID_NUMBER_OF_ITEMS";
            break;
        case Error::EMMC_FREERTOS_RESOURCE_INITIALIZATION_FAILED:
            LOG_DEBUG << "EMMC_FREERTOS_RESOURCE_INITIALIZATION_FAILED";
            break;
        case Error::EMMC_SPECIFIED_ZERO_LENGTH_OBJECT:
            LOG_DEBUG << "EMMC_SPECIFIED_ZERO_LENGTH_OBJECT";
            break;
        case Error::EMMC_SURPASSED_MEMORY_CONSTRAINTS:
            LOG_DEBUG << "EMMC_SURPASSED_MEMORY_CONSTRAINTS";
            break;
        default:
            LOG_DEBUG << "UNKNOWN_ERROR";
            break;
        }
    }
} // namespace eMMC