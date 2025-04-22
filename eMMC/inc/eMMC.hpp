#pragma once
#include "stm32h7xx_hal.h"
#include "etl/expected.h"
#include <cstdint>
#include "FreeRTOS.h"
#include <semphr.h>

namespace eMMC {
    /**
     * Error status
     */
    enum class Error : uint8_t {
        NO_ERRORS = 0,
        EMMC_READ_FAILURE,
        EMMC_WRITE_FAILURE,
        EMMC_ERASE_BLOCK_FAILURE,
        EMMC_INVALID_NUM_OF_BLOCKS,
        EMMC_INVALID_START_ADDRESS_ON_ERASE,
        EMMC_TRANSACTION_TIMED_OUT,
        EMMC_BUFFER_TOO_SMALL,
        EMMC_QUEUE_FULL,
        EMMC_QUEUE_EMPTY
    };

    /**
     * Used by ISR to indicate an eMMC transaction is complete.
     */
    struct EMMCTransactionFlags {
        bool WriteComplete = false;
        bool ReadComplete = false;
        bool ErrorOccured = false;
        bool transactionAborted = false;
    };
    extern EMMCTransactionFlags eMMCTransactionFlags;

    /**
     * @details Memory item: A generic data structure, which can be useful when the user needs to store
     *              only one item or needs to manage the underlying memory blocks manually.
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

        void registerMMCHandle(MMC_HandleTypeDef* handle) {
            hmmc = handle;
        }

        /** Memory item interface**/
        etl::expected<void, Error> getItem(MemoryItem item, uint8_t* destBuffer, uint32_t bufferSize);

        etl::expected<void, Error> getItem(MemoryItem item, uint8_t* destBuffer, uint32_t bufferSize, uint32_t startBlock, uint32_t numOfBlocks);

        etl::expected<void, Error> storeItem(MemoryItem item, uint8_t* sourceBuffer, uint32_t bufferSize);

        etl::expected<void, Error> storeItem(MemoryItem item, uint8_t* sourceBuffer, uint32_t bufferSize, uint32_t startBlock, uint32_t numOfBlocks);

        /** Queue interface **/
        etl::expected<void, Error> popItemFromQueue(MemoryQueue queue, uint8_t* destBuffer, uint32_t bufferSize);

        etl::expected<void, Error> pushItemToQueue(MemoryQueue queue, uint8_t* sourceBuffer, uint32_t bufferSize);

    private:
        /**
         * Size parameters for the SDINBDG4-8G
         */
        static constexpr uint32_t memoryPageSize = 512;
        static constexpr uint32_t memoryPageCount = 0xE90E80 * memoryPageSize;
        static constexpr uint32_t memorySizeInBytes = memoryPageSize * memoryPageCount;

        /**
         * Transaction handling parameters
         */
        MMC_HandleTypeDef *hmmc;
        SemaphoreHandle_t eMMC_semaphoreHandle;
        StaticSemaphore_t eMMC_semaphoreBuffer;
        uint32_t transactionTimeoutPerBlock = 100; // ms
        uint32_t getSemaphoreTimeout = 1000;       //ms

        /**
         * Hold state for memory regions that store a single item
         */
        struct MemoryItemHandler {
            uint32_t size;
            uint32_t startAddress;
            uint32_t endAddress;
            MemoryItemHandler() : size(0), startAddress(0), endAddress(0) {}
            explicit MemoryItemHandler(const uint32_t newSize)
                    : size(newSize), startAddress(0), endAddress(0) {}
        };

        etl::array<MemoryItemHandler, memoryItemCount> memoryItemMap;

        /**
         * Hold state for memory regions that store a queue of items
         */
        struct MemoryQueueHandler {
            QueueHandle_t* queue;
            uint32_t sizeOfItem;
            uint32_t numberOfItems;
            uint32_t tailPageOffset;
            uint32_t firstPage;
            uint32_t lastPage;
            uint64_t startAddress;
            MemoryQueueHandler() : queue(NULL), sizeOfItem(0), numberOfItems(0), tailPageOffset(0), firstPage(0), lastPage(0), startAddress(0) {}
            explicit MemoryQueueHandler(const uint32_t newSize, const uint32_t newNumberOfItems, QueueHandle_t* newQueue)
                    : queue(newQueue), sizeOfItem(newSize), numberOfItems(newNumberOfItems), tailPageOffset(0), firstPage(0), lastPage(0), startAddress(0) {}
        };

#define MEMORY_QUEUE(queue_name, item_size, queue_size)                                                                                                \
        inline static uint8_t queue_name##QueueStorageArea[sizeof(MemoryQueueHandler) * queue_size] __attribute__((section(".dtcmram_data")));         \
        QueueHandle_t queue_name##Queue;                                                                                                               \
        inline static StaticQueue_t queue_name##QueueBuffer;
#include "MemoryQueues.def"
#undef MEMORY_QUEUE

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