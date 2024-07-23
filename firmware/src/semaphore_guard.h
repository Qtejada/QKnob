
#pragma once

///File should be used when creating semaphores to guard and stop copying etc

#include <Arduino.h> 


class SemaphoreGuard {
    public:
        /**
         * Constructor that takes a semaphore.
         * @param handle The handle of the semaphore to be taken.
         * 
         * Takes the semaphore (acquires the lock) when the object is created.
         */
        SemaphoreGuard(SemaphoreHandle_t handle) : handle_{handle} {
            xSemaphoreTake(handle_, portMAX_DELAY); // Take the semaphore, blocking indefinitely until it becomes available
        }

        /**
         * Destructor.
         * 
         * Gives the semaphore (releases the lock) when the object is destroyed.
         */
        ~SemaphoreGuard() {
            xSemaphoreGive(handle_); // Give the semaphore back when the object goes out of scope
        }

        // Delete the copy constructor to prevent copying
        SemaphoreGuard(SemaphoreGuard const&) = delete;

        // Delete the copy assignment operator to prevent copying
        SemaphoreGuard& operator=(SemaphoreGuard const&) = delete;

    private:
        SemaphoreHandle_t handle_; // Handle of the semaphore being managed
};

