#pragma once

#include <FFat.h>
#include <PacketSerial.h>

#include "proto_gen/smartknob.pb.h"
#include "logger.h"

// Define the version of the persistent configuration
const uint32_t PERSISTENT_CONFIGURATION_VERSION = 1;

/**
 * Class to manage persistent configuration for the SmartKnob.
 * Provides methods to load, save, and modify configuration data stored on disk.
 */
class Configuration {
    public:
        Configuration();
        ~Configuration();

        // Set the logger instance to be used by this class
        void setLogger(Logger* logger);

        // Load the configuration from disk
        bool loadFromDisk();

        // Save the configuration to disk
        bool saveToDisk();

        // Get the current persistent configuration
        PB_PersistentConfiguration get();

        // Set motor calibration data and save the configuration to disk
        bool setMotorCalibrationAndSave(PB_MotorCalibration& motor_calibration);

        // Set strain calibration data and save the configuration to disk
        bool setStrainCalibrationAndSave(PB_StrainCalibration& strain_calibration);

    private:
        SemaphoreHandle_t mutex_; // Mutex for thread-safe access to the configuration

        Logger* logger_ = nullptr; // Pointer to the logger instance
        bool loaded_ = false; // Flag to indicate if the configuration has been loaded
        PB_PersistentConfiguration pb_buffer_ = {}; // Buffer to hold the configuration data

        uint8_t buffer_[PB_PersistentConfiguration_size]; // Raw buffer for serialization

        // Log a message using the logger instance
        void log(const char* msg);
};

/**
 * RAII class to manage mounting and unmounting of the FFat filesystem.
 * Ensures that the filesystem is properly mounted and unmounted.
 */
class FatGuard {
    public:
        // Constructor: Mount the FFat filesystem and log the result
        FatGuard(Logger* logger) : logger_(logger) {
            if (!FFat.begin(true)) {
                if (logger_ != nullptr) {
                    logger_->log("Failed to mount FFat");
                }
                return;
            }
            if (logger_ != nullptr) {
                logger_->log("Mounted FFat");
            }
            mounted_ = true;
        }

        // Destructor: Unmount the FFat filesystem if it was mounted
        ~FatGuard() {
            if (mounted_) {
                FFat.end();
                if (logger_ != nullptr) {
                    logger_->log("Unmounted FFat");
                }
            }
        }

        // Disable copy constructor and copy assignment operator
        FatGuard(FatGuard const&) = delete;
        FatGuard& operator=(FatGuard const&) = delete;

        bool mounted_ = false; // Flag to indicate if the filesystem is mounted

    private:
        Logger* logger_; // Pointer to the logger instance
};


