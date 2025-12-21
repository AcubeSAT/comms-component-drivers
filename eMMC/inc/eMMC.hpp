#pragma once
#include <cstdint>
#include "stm32h7xx_hal.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "etl/expected.h"
#include "event_groups.h"

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
        EMMC_INVALID_NUMBER_OF_ITEMS,
        EMMC_FREERTOS_RESOURCE_INITIALIZATION_FAILED,
        EMMC_SPECIFIED_ZERO_LENGTH_OBJECT,
        EMMC_SURPASSED_MEMORY_CONSTRAINTS
    };

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
     *
     * @param reboot_persistence: A value of non-zero will make this queue "reboot persistent". This means that the
     *                            head and tail pointers are stored at an extra "metadata block", and updated with
     *                            every push and pop. This allows data to survive in scenarios where a reboot of the mcu
     *                            occurs. Recommended only for long term data, due to the extra overhead of updating
     *                            the metadata block.
     * @note Define your queues in MemoryQueues.def
     */
#define MEMORY_QUEUE(queue_name, item_size, queue_size, reboot_persistence) queue_name,
    enum MemoryQueue {
#include "MemoryQueues.def"
        memoryQueueCount // This is automatically added after all items
    };
#undef MEMORY_QUEUE

    class eMMC_Utilities {
    public:
        /// "External" event group bits. The user must trigger these events from the proper mmc ISRs
        static constexpr uint32_t writeCompleteGroupBit            = 1U << 0;
        static constexpr uint32_t readCompleteGroupBit             = 1U << 1;
        static constexpr uint32_t errorOccuredGroupBit             = 1U << 2;
        static constexpr uint32_t transactionAbortedGroupBit       = 1U << 3;
        EventGroupHandle_t eventGroupHandle;
        StaticEventGroup_t eventGroupBuffer;

        eMMC_Utilities() = default;

        /**
         * This should be called inside the HAL_MMC_TxCpltCallback
         */
        static void txIrqHandler();

        /**
         * This should be called inside the HAL_MMC_RxCpltCallback
         */
        static void rxIrqHandler();

        /**
         * This should be called inside the HAL_MMC_ErrorCallback
         */
        static void errorIrqHandler();

        /**
         * This should be called inside the HAL_MMC_AbortCallback
         */
        static void abortIrqHandler();

        /**
         * @brief Initializer for the eMMC driver
         *
         * @returns The percentage of allocated memory
         */
        etl::expected<float, Error> initializeResources(MMC_HandleTypeDef* handle);

        /** Memory item interface**/

        /**
         * @brief Get the item size in bytes
         */
        [[nodiscard]] uint32_t getItemSize(const MemoryItem item) {
            return memoryItemMap[item].size;
        }

        /**
         * @brief Get the entire item
         * @note If the item size is not a multiple of the block size, the function ensures that leftover bytes are not copied
         */
        [[nodiscard]] etl::expected<void, Error> getItem(MemoryItem item, uint8_t* destBuffer, uint32_t bufferSize);

        /**
         * @brief Read a partial item
         * @param destBuffer The buffer the data will be copied to. The driver handles cache coherency issues by invalidating
         *                   the cache, forcing a new read from AXI SRAM. In order for irrelevant data to not be
         *                   affected, ensure this buffer is 32 byte aligned, using the alignas(32) specifier
         * @param startBlock The first block to start reading from. For startBlock = 0, the first portion of the item
         *                   is read.
         * @param numOfBlocks How many blocks to read. The startBlock is also included, therefore it needs to be numOfBlocks >= 1
         * @note If the item size is not a multiple of the block size, the function ensures that leftover bytes are not copied,
         *       in the scenario that the last block is requested.
         */
        [[nodiscard]] etl::expected<void, Error> getItem(MemoryItem item, uint8_t* destBuffer, uint32_t bufferSize, uint32_t startBlock, uint32_t numOfBlocks);

        [[nodiscard]] etl::expected<void, Error> storeItem(MemoryItem item, uint8_t* sourceBuffer, uint32_t bufferSize);

        // TODO this function would make sense if the item is very large and has to be partially copied, but we dont need this right now
        [[nodiscard]] etl::expected<void, Error> storeItem(MemoryItem item, uint8_t* sourceBuffer, uint32_t bufferSize, uint32_t startBlock, uint32_t numOfBlocks);

        /**
         * Debugging function. Erases the data of that specific item.
         */
        [[nodiscard]] etl::expected<void, Error> resetItem(MemoryItem item);

        /** Queue interface **/
        [[nodiscard]] bool isQueueEmpty(const MemoryQueue queue) {
            return memoryQueueMap[queue].currentNumberOfItems == 0;
        }

        [[nodiscard]] bool isQueueFull(const MemoryQueue queue) {
            return memoryQueueMap[queue].currentNumberOfItems == memoryQueueMap[queue].maxNumberOfItems;
        }

        [[nodiscard]] uint32_t queueMaxSize(const MemoryQueue queue) {
            return memoryQueueMap[queue].maxNumberOfItems;
        }

        [[nodiscard]] uint32_t queueCurrentSize(const MemoryQueue queue) {
            return memoryQueueMap[queue].currentNumberOfItems;
        }

        [[nodiscard]] uint32_t getQueueElementSize(const MemoryQueue queue) {
            return memoryQueueMap[queue].itemSize;
        }

        /**
         * @brief Pop one or more items from the queue. The items are returned in the order they are popped.
         * @param destBuffer The buffer the data will be copied to. The driver handles cache coherency issues by invalidating
         *                   the cache, forcing a new read from AXI SRAM. In order for irrelevant data to not be
         *                   affected, ensure this buffer is 32 byte aligned, using the alignas(32) specifier
         * @note In the scenario the item size is not a multiple of the block size, the function ensures that the leftover
         *       bits in the queue slot are not returned
         * @returns Returns the actual amount of items popped and whether the operation as a whole was successful or not.
         */
        [[nodiscard]] etl::pair<uint32_t, Error> popItemsFromQueue(MemoryQueue queue, uint8_t* destBuffer, uint32_t bufferSize, uint32_t numItems);

        /**
         * @brief Push one or more items to the queue
         * @param sourceBuffer The buffer the data will be copied from. The driver handles cache coherency issues by cleaning
         *                     the cache, ensuring the data is written to AXI SRAM, before doing an emmc write. In order
         *                     for irrelevant data to not be affected, ensure this buffer is 32 byte aligned,
         *                     using the alignas(32) specifier
         * @returns Returns the actual amount of items pushed and whether the operation as a whole was successful or not.
         */
        [[nodiscard]] etl::pair<uint32_t, Error> pushItemsToQueue(MemoryQueue queue, uint8_t* sourceBuffer, uint32_t bufferSize, uint32_t numItems);

        /**
         * Debugging function. Erases the data of that specific queue.
         */
        [[nodiscard]] etl::expected<void, Error> resetQueue(MemoryQueue queue);

        /**
         * @brief Utility function. Write to eMMC blocks.
         */
        [[nodiscard]] etl::expected<void, Error> writeBlockEMMC(uint8_t* sourceBuffer, uint32_t block_address, uint32_t numberOfBlocks);

        /**
         * @brief Utility function. Read from eMMC blocks.
         */
        [[nodiscard]] etl::expected<void, Error> readBlockEMMC(uint8_t* destBuffer, uint32_t block_address, uint32_t numberOfBlocks);

        void printError(Error error);
    private:
        /**
         * Size parameters obtained from HAL drivers (in bytes)
         */
        uint32_t logicalBlockSize = 512;
        uint32_t logicalBlockCount;
        uint64_t memorySizeInBytes;
        float emmcUsage = 0; // percentage of EMMC memory utilized, calculated upon object construction

        /**
         * Transaction handling parameters
         */
        MMC_HandleTypeDef *hmmc;
        SemaphoreHandle_t eMMC_access_semaphoreHandle; // for concurrent access protection to the EMMC peripheral itself
        StaticSemaphore_t eMMC_access_semaphoreBuffer;
        StaticSemaphore_t isrTriggeredSemaphoreBuffer;
        static constexpr uint32_t TransactionTimeoutPerBlockMs = 100;
        static constexpr uint32_t SemaphoreTimeoutMs = 1000;

        // The card stays in the busy state for a few ms after a transaction is complete. Through testing, a delay
        // of 4 ms always guarantees the card is not in the busy state anymore, so this value is used as the maximum
        // polling time.
        static constexpr uint32_t SuccessfulTransactionPollingPeriodMs = 1;
        static constexpr uint32_t MaxSuccessfulTransactionDelayMs = 4;
        static constexpr uint32_t ErasePollingPeriodMs = 5;

        /**
         * This value is placed in the start of persistent object metadata blocks, helping the driver figure out upon
         * initialization if the emmc is new, hence the objects are invalid.
         */
        static constexpr uint32_t MagicValue = 0x454D4D43;

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
         *
         * @note The extra "metadata block" for storing  the head and tail pointers has the following format:
         *
         *           | MagicValue | headSlotPointer | tailSlotPointer | currentNumberOfItems |   empty   |
         *  Bytes:        0-31         32 - 63           64 - 95            96 - 127           128 - 511
         */
        struct MemoryQueueHandler {
            SemaphoreHandle_t semaphoreHandle; // for concurrent access protection to this item
            StaticSemaphore_t semaphoreBuffer;

            uint32_t itemSize; // in Bytes
            bool itemHasPartialBlock; // indicates whether the queue items are multiples of 512 (if not, the last block is partial)
            uint32_t maxNumberOfItems;
            uint32_t currentNumberOfItems;

            uint32_t startBlockAddress;
            uint32_t endBlockAddress; // in the scenario where a metadata block is present, this points to the last block that contains data
                                      // the metadata block is allocated in endBlockAddress + 1
            uint32_t slotBlockSize;
            uint32_t headSlotPointer;  // Note: for the slot pointers, the value 0 indicates the slot that starts in block address
            uint32_t tailSlotPointer;

            bool isRebootPersistent;

            MemoryQueueHandler() = default;
            MemoryQueueHandler(const uint32_t itemSize, const uint32_t numberOfItems, const bool isRebootPersistent)
            : itemSize(itemSize), itemHasPartialBlock(false), maxNumberOfItems(numberOfItems), currentNumberOfItems(0),
              headSlotPointer(0), tailSlotPointer(0), isRebootPersistent(isRebootPersistent) {}
        };

        etl::array<MemoryQueueHandler, memoryQueueCount> memoryQueueMap;

        /**
         * @brief Erases specified memory region from eMMC
         */
        etl::expected<void, Error> eraseBlocksEMMC(uint32_t block_address_start, uint32_t block_address_end);
    };
    extern eMMC_Utilities eMMC_Utils;
} // namespace eMMC