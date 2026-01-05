#include <iostream>
#include <string>

/* FreeRTOS includes must be wrapped in extern "C" */
extern "C" {
    #include "FreeRTOS.h"
    #include "task.h"
    #include "queue.h"

    // Hook for stack overflow
    void vApplicationStackOverflowHook( TaskHandle_t xTask, char *pcTaskName )
    {
        std::cerr << "FATAL: Stack overflow in task: " << pcTaskName << std::endl;
        for( ;; );
    }

    // Hook for malloc failure (Important!)
    void vApplicationMallocFailedHook( void )
    {
        std::cerr << "FATAL: Malloc failed!" << std::endl;
        for( ;; );
    }
}

// --- Global Handles ---
QueueHandle_t xQueue = nullptr;

// --- Task 1: The Producer ---
void vProducerTask(void *pvParameters) {
    int counter = 0;
    const char* taskName = (const char*)pvParameters;

    for (;;) {
        counter++;
        std::cout << "[" << taskName << "] Sending: " << counter << std::endl;
        xQueueSend(xQueue, &counter, portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// --- Task 2: The Consumer ---
void vConsumerTask(void *pvParameters) {
    int receivedValue = 0;
    const char* taskName = (const char*)pvParameters;

    for (;;) {
        if (xQueueReceive(xQueue, &receivedValue, portMAX_DELAY) == pdPASS) {
            std::cout << "    [" << taskName << "] Received: " << receivedValue
            << " (Success)" << std::endl;
        }
    }
}

// --- Main Application ---
int main(void) {
    // Disable stdout buffering to prevent logs from getting stuck
    setbuf(stdout, NULL);

    std::cout << "--- Starting FreeRTOS C++ Demo ---" << std::endl;

    xQueue = xQueueCreate(5, sizeof(int));
    if (xQueue == nullptr) {
        std::cerr << "Failed to create queue." << std::endl;
        return 1;
    }
    std::cout << "--- Created Queue ---" << std::endl;

    // --- CRITICAL FIX: Use a large stack size for Linux ---
    // Linux pthreads need at least ~16KB (PTHREAD_STACK_MIN).
    // We use 64KB (65536) to be safe for C++ iostream usage.
    const uint32_t STACK_SIZE = 65536;

    BaseType_t status;

    status = xTaskCreate(vProducerTask, "Producer", STACK_SIZE, (void*)"PROD", 1, NULL);
    if (status != pdPASS) {
        std::cerr << "Failed to create Producer Task! Error: " << status << std::endl;
        return 1;
    }

    status = xTaskCreate(vConsumerTask, "Consumer", STACK_SIZE, (void*)"CONS", 1, NULL);
    if (status != pdPASS) {
        std::cerr << "Failed to create Consumer Task! Error: " << status << std::endl;
        return 1;
    }

    std::cout << "--- Starting Scheduler ---" << std::endl;

    // Start the scheduler
    vTaskStartScheduler();

    // If we get here, something went wrong with the scheduler itself
    std::cout << "Scheduler failed to start." << std::endl;
    return 0;
}
