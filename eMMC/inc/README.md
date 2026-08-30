This driver's memory items and memory queues are defined at compile time using definition
files provided by the users.

- To define memory items:
    1. Create a file named "MemoryItems.def"
    2. Populate the file with memory items, using the following format: `MEMORY_ITEM(name, sizeInBytes)`

    example:
    ```
    MEMORY_ITEM(memory_item_1, 200)
    MEMORY_ITEM(memory_item_2, 300)
    // ...
    ```

- To define memory queues:
    1. Create a file named "MemoryQueues.def"
    2. Populate the file with memory queues, using the following format: `MEMORY_QUEUE(name, sizeOfItem, numberOfItems, rebootPersistence)`

    example:
    ```
    MEMORY_QUEUE(memory_queue_1, 200, 3, 0)
    MEMORY_QUEUE(memory_queue_1, 200, 3, 1)
    // ...
    ```

