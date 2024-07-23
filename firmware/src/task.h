
#pragma once

#include <Arduino.h> // Include Arduino header for compatibility with Arduino functions

// Static polymorphic abstract base class for a FreeRTOS task using the CRTP pattern.
// Concrete implementations should implement a run() method.


/// @brief Basically creates a Task and what you should do in that task wherever its used (Motor, Display, etc)

template<class T>
class Task {
    public:
        /**
         * Constructor for the Task class.
         *
         * @param name The name of the task.
         * @param stackDepth The stack depth for the task.
         * @param priority The priority of the task.
         * @param coreId The core ID for the task (default is no affinity).
         */
        Task(const char* name, uint32_t stackDepth, UBaseType_t priority, const BaseType_t coreId = tskNO_AFFINITY) : 
                name { name },
                stackDepth { stackDepth },
                priority { priority },
                coreId { coreId }
        {}

        /**
         * Virtual destructor for the Task class.
         */
        virtual ~Task() {};

        /**
         * Gets the handle of the task.
         *
         * @return The handle of the task.
         */
        TaskHandle_t getHandle() {
            return taskHandle;
        }

        /**
         * Begins the task by creating it and pinning it to the specified core.
         */
        void begin() {
            // Create the task and pin it to the specified core
            BaseType_t result = xTaskCreatePinnedToCore(taskFunction, name, stackDepth, this, priority, &taskHandle, coreId);
            // Ensure the task was created successfully
            assert("Failed to create task" && result == pdPASS);
        }

    private:
        /**
         * function to run the task.
         * This function is passed to FreeRTOS and calls the run() method of the concrete class.
         *
         * @param params Pointer to the task object.
         */
        static void taskFunction(void* params) {
            T* t = static_cast<T*>(params);
            t->run();
        }

        const char* name; // The name of the task
        uint32_t stackDepth; // How big the allocated stack is in words
        UBaseType_t priority; // The priority of the task
        TaskHandle_t taskHandle; // The handle of the task
        const BaseType_t coreId; // The core ID for the task
};

