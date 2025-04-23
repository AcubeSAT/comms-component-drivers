#pragma once
#include <cstdint>
#include "stm32h7xx_hal.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "etl/expected.h"

namespace eMMC {
    /**
     * Error status
     */
    enum class Error : uint8_t {
        EMMC_NO_ERROR,
        EMMC_READ_FAILURE,
        EMMC_WRITE_FAILURE,
        EMMC_ERASE_BLOCK_FAILURE,
        EMMC_INVALID_MEMORY_BLOCK_REGION,
        EMMC_TRANSACTION_TIMED_OUT,
        EMMC_TRANSACTION_ABORTED,
        EMMC_MUTEX_LOCK_TIMEOUT,
        EMMC_BUFFER_TOO_SMALL,
        EMMC_QUEUE_FULL,
        EMMC_QUEUE_EMPTY,
        EMMC_INVALID_NUMBER_OF_ITEMS
    };

    /**
     * Used by ISR to indicate an eMMC transaction is complete.
     */
    struct EMMCTransactionFlags {
        bool WriteComplete = false;
        bool ReadComplete = false;
        bool ErrorOccured = false;
        bool TransactionAborted = false;
    };
    extern EMMCTransactionFlags eMMCTransactionFlags;

    /**
     * @details Memory item: A generic data structure, which can be useful when the user needs to store
     *              only one item.
     * @note Define your memory items in MemoryItems.def
     */
#define MEMORY_ITEM(name, size) name,
    enum MemoryItem {
#include "MemoryItems.def"
        memoryItemCount // This is automatically added after all items
    };
#undef MEMORY_ITEM

    /**
     * @details Memory queue: A data structure with push and pop operations.
     * @note Define your queues in MemoryQueues.def
     */
#define MEMORY_QUEUE(queue_name, item_size, queue_size) queue_name,
    enum MemoryQueue {
#include "MemoryQueues.def"
        memoryQueueCount // This is automatically added after all items
    };
#undef MEMORY_QUEUE

    class eMMC_Utilities {
    public:
        explicit eMMC_Utilities();

        void registerMMC(MMC_HandleTypeDef* handle) {
            hmmc = handle;
        }

        /** Memory item interface**/

        /**
         * @brief Get the item size in bytes
         */
        uint32_t getItemSize(const MemoryItem item) {
            return memoryItemMap[item].size;
        }

        /**
         * @brief Get the entire item
         * @note If the item size is not a multiple of the block size, the function ensures that leftover bytes are not copied
         */
        etl::expected<void, Error> getItem(MemoryItem item, uint8_t* destBuffer, uint32_t bufferSize);

        /**
         * @brief Read a partial item
         * @param startBlock The first block to start reading from. For startBlock = 0, the first portion of the item
         *                   is read.
         * @param numOfBlocks How many blocks to read. The startBlock is also included, therefore it needs to be numOfBlocks >= 1
         * @note If the item size is not a multiple of the block size, the function ensures that leftover bytes are not copied,
         *       in the scenario that the last block is requested.
         */
        etl::expected<void, Error> getItem(MemoryItem item, uint8_t* destBuffer, uint32_t bufferSize, uint32_t startBlock, uint32_t numOfBlocks);

        etl::expected<void, Error> storeItem(MemoryItem item, uint8_t* sourceBuffer, uint32_t bufferSize);

        // TODO this function would make sense if the item is very large and has to be partially copied, but we dont need this right now
        etl::expected<void, Error> storeItem(MemoryItem item, uint8_t* sourceBuffer, uint32_t bufferSize, uint32_t startBlock, uint32_t numOfBlocks);

        /** Queue interface **/
        bool isQueueEmpty(const MemoryQueue queue) {
            return memoryQueueMap[queue].currentNumberOfItems == 0;
        }

        bool isQueueFull(const MemoryQueue queue) {
            return memoryQueueMap[queue].currentNumberOfItems == memoryQueueMap[queue].maxNumberOfItems;
        }

        uint32_t queueMaxSize(const MemoryQueue queue) {
            return memoryQueueMap[queue].maxNumberOfItems;
        }

        uint32_t queueCurrentSize(const MemoryQueue queue) {
            return memoryQueueMap[queue].currentNumberOfItems;
        }

        /**
         * @brief Pop one or more items from the queue. The items are returned in the order they are popped.
         * @note In the scenario the item size is not a multiple of the block size, the function ensures that the leftover
         *       bits in the queue slot are not returned
         * @returns Returns the actual amount of items popped and whether the operation as a whole was successful or not.
         */
        etl::pair<uint32_t, Error> popItemsFromQueue(MemoryQueue queue, uint8_t* destBuffer, uint32_t bufferSize, uint32_t numItems);

        /**
         * @brief Push one or more items to the queue
         * @returns Returns the actual amount of items pushed and whether the operation as a whole was successful or not.
         */
        etl::pair<uint32_t, Error> pushItemsToQueue(MemoryQueue queue, uint8_t* sourceBuffer, uint32_t bufferSize, uint32_t numItems);

    private:
        /**
         * Size parameters for the SDINBDG4-8G
         */
        static constexpr uint32_t blockSize = 512; // in bytes
        static constexpr uint32_t blockCount = 0xE90E80; // TODO confirm this number
        static constexpr uint64_t memorySizeInBytes = static_cast<uint64_t>(blockSize) * static_cast<uint64_t>(blockCount);
        float emmcUsage = 0; // percentage of EMMC memory utilized, calculated upon object construction

        /**
         * Transaction handling parameters
         */
        MMC_HandleTypeDef *hmmc;
        SemaphoreHandle_t eMMC_semaphoreHandle; // for concurrent access protection to the EMMC peripheral itself
        StaticSemaphore_t eMMC_semaphoreBuffer;
        uint32_t transactionTimeoutPerBlock = 100; // ms
        uint32_t semaphoreTimeout = 1000;       // ms

        /**
         * Hold state for memory regions that store a single item
         */
        struct MemoryItemHandler {
            SemaphoreHandle_t semaphoreHandle; // for concurrent access protection to this item
            StaticSemaphore_t semaphoreBuffer;

            uint32_t size;  // in Bytes
            uint32_t startBlockAddress;
            uint32_t endBlockAddress;
            bool hasPartialBlock; // If the item size is not a multiple of the block size, then there is unused space in the last block
            MemoryItemHandler() = default;
            explicit MemoryItemHandler(const uint32_t size)
                    : size(size), startBlockAddress(0), endBlockAddress(0), hasPartialBlock(false) {}
        };

        etl::array<MemoryItemHandler, memoryItemCount> memoryItemMap;

        /**
         * Hold state for memory regions that store a queue of items
         * @note The queue "slots" of the items are always block aligned to make accessing/writing simpler and faster.
         *       For example, if the items have a size of 1.5*blockSize, then the slotSize is 2*blockSize
         */
        struct MemoryQueueHandler {
            SemaphoreHandle_t semaphoreHandle; // for concurrent access protection to this item
            StaticSemaphore_t semaphoreBuffer;

            uint32_t itemSize; // in Bytes
            bool itemHasPartialBlock; // indicates whether the queue items are multiples of 512 (if not, the last block is partial)
            uint32_t maxNumberOfItems;
            uint32_t currentNumberOfItems;

            uint32_t startBlockAddress;
            uint32_t endBlockAddress;
            uint32_t slotBlockSize;
            uint32_t headSlotPointer;  // Note: for the slot pointers, the value 0 indicates the slot that starts in block address
            uint32_t tailSlotPointer;

            MemoryQueueHandler() = default;
            MemoryQueueHandler(const uint32_t itemSize, const uint32_t numberOfItems)
            : itemSize(itemSize), itemHasPartialBlock(false), maxNumberOfItems(numberOfItems), currentNumberOfItems(0),
              headSlotPointer(0), tailSlotPointer(0) {}
        };

        etl::array<MemoryQueueHandler, memoryQueueCount> memoryQueueMap;

        /**
         * @brief Write to eMMC blocks
         */
        etl::expected<void, Error> writeBlockEMMC(const uint8_t* sourceBuffer, uint32_t block_address, uint32_t numberOfBlocks);

        /**
         * @brief Read from eMMC blocks
         */
        etl::expected<void, Error> readBlockEMMC(uint8_t* destBuffer, uint32_t block_address, uint32_t numberOfBlocks) const;

        /**
         * @brief Erases specified memory region from eMMC
         */
        etl::expected<void, Error> eraseBlocksEMMC(uint32_t block_address_start, uint32_t block_address_end);
    };
    extern eMMC_Utilities eMMC_Utils;
} // namespace eMMC