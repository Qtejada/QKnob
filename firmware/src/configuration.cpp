#include <FFat.h> 
#include "pb_decode.h" 
#include "pb_encode.h" 

#include "proto_gen/smartknob.pb.h" 
#include "semaphore_guard.h" 

#include "configuration.h" 

// Path to the configuration file
static const char* CONFIG_PATH = "/config.pb";

//Configuration is from host computer so need to open files

/**
 * Constructor for the Configuration class.
 * Creates a mutex for thread-safe access.
 */
Configuration::Configuration() {
    mutex_ = xSemaphoreCreateMutex(); // Create a mutex
    assert(mutex_ != NULL); // Ensure the mutex was created successfully
}

/**
 * Destructor for the Configuration class.
 * Deletes the mutex.
 */
Configuration::~Configuration() {
    vSemaphoreDelete(mutex_); // Delete the mutex
}

/**
 * Loads the configuration from disk.
 * Uses FFat to open and read the configuration file, and decodes it using nanopb.
 *
 * @return True if the configuration was loaded successfully, otherwise false.
 */
bool Configuration::loadFromDisk() {
    SemaphoreGuard lock(mutex_); // Acquire the mutex for thread-safe access
    FatGuard fatGuard(logger_); // Ensure FFat is mounted using FatGuard
    if (!fatGuard.mounted_) {
        return false; // Return false if FFat could not be mounted
    }

    // Open the configuration file
    File f = FFat.open(CONFIG_PATH);
    if (!f) {
        log("Failed to read config file"); // Log an error if the file could not be opened
        return false;
    }

    // Read the file contents into the buffer
    size_t read = f.readBytes((char*)buffer_, sizeof(buffer_));
    f.close(); 

    // Create a nanopb input stream from the buffer
    pb_istream_t stream = pb_istream_from_buffer(buffer_, read);
    // Decode the configuration from the input stream
    if (!pb_decode(&stream, PB_PersistentConfiguration_fields, &pb_buffer_)) {
        char buf[200];
        snprintf(buf, sizeof(buf), "Decoding failed: %s", PB_GET_ERROR(&stream));
        log(buf); // Log an error if decoding failed
        pb_buffer_ = {}; // Clear the buffer
        return false;
    }

    // Check if the configuration version is correct
    if (pb_buffer_.version != PERSISTENT_CONFIGURATION_VERSION) {
        char buf[200];
        snprintf(buf, sizeof(buf), "Invalid config version. Expected %u, received %u", PERSISTENT_CONFIGURATION_VERSION, pb_buffer_.version);
        log(buf); // Log an error if the version is incorrect
        pb_buffer_ = {}; // Clear the buffer
        return false;
    }
    loaded_ = true; // Set the loaded flag to true

    // Log the motor calibration details
    char buf[200];
    snprintf(
        buf,
        sizeof(buf),
        "Motor calibration: calib=%u, pole_pairs=%u, zero_offset=%.2f, cw=%u",
        pb_buffer_.motor.calibrated,
        pb_buffer_.motor.pole_pairs,
        pb_buffer_.motor.zero_electrical_offset,
        pb_buffer_.motor.direction_cw
    );
    log(buf);
    return true; // Return true if the configuration was loaded successfully
}

bool Configuration::saveToDisk() {
    SemaphoreGuard lock(mutex_); // Acquire the mutex for thread-safe access

    pb_ostream_t stream = pb_ostream_from_buffer(buffer_, sizeof(buffer_)); // Create a nanopb output stream from the buffer
    pb_buffer_.version = PERSISTENT_CONFIGURATION_VERSION; // Set the configuration version
    if (!pb_encode(&stream, PB_PersistentConfiguration_fields, &pb_buffer_)) { // Encode the configuration data into the buffer
        char buf[200];
        snprintf(buf, sizeof(buf), "Encoding failed: %s", PB_GET_ERROR(&stream)); // Log an error if encoding failed
        log(buf);
        return false;
    }

    FatGuard fatGuard(logger_); // Ensure FFat is mounted using FatGuard
    if (!fatGuard.mounted_) {
        return false; // Return false if FFat could not be mounted
    }

    File f = FFat.open(CONFIG_PATH, FILE_WRITE); // Open the configuration file for writing
    if (!f) {
        log("Failed to read config file"); // Log an error if the file could not be opened
        return false;
    }

    size_t written = f.write(buffer_, stream.bytes_written); // Write the encoded data to the file
    f.close(); // Close the file

    char buf[20];
    snprintf(buf, sizeof(buf), "Wrote %d bytes", written); // Log the number of bytes written
    log(buf);

    if (written != stream.bytes_written) { // Check if all bytes were written successfully
        log("Failed to write all bytes to file"); // Log an error if not all bytes were written
        return false;
    }

    return true; // Return true if the configuration was saved successfully
}

PB_PersistentConfiguration Configuration::get() {
    SemaphoreGuard lock(mutex_); // Acquire the mutex for thread-safe access
    if (!loaded_) {
        return PB_PersistentConfiguration(); // Return a default configuration if not loaded
    }
    return pb_buffer_; // Return the loaded configuration
}

bool Configuration::setMotorCalibrationAndSave(PB_MotorCalibration& motor_calibration) {
    {
        SemaphoreGuard lock(mutex_); // Acquire the mutex for thread-safe access
        pb_buffer_.motor = motor_calibration; // Set the motor calibration data
        pb_buffer_.has_motor = true; // Mark the motor calibration data as valid
    }
    return saveToDisk(); // Save the configuration to disk
}

bool Configuration::setStrainCalibrationAndSave(PB_StrainCalibration& strain_calibration) {
    {
        SemaphoreGuard lock(mutex_); // Acquire the mutex for thread-safe access
        pb_buffer_.strain = strain_calibration; // Set the strain calibration data
        pb_buffer_.has_strain = true; // Mark the strain calibration data as valid
    }
    return saveToDisk(); // Save the configuration to disk
}

void Configuration::setLogger(Logger* logger) {
    logger_ = logger; // Set the logger instance
}

void Configuration::log(const char* msg) {
    if (logger_ != nullptr) {
        logger_->log(msg); // Log the message if the logger is set
    }
}
