#include <driver/uart.h> // Include ESP-IDF UART driver
#include "config.h"      // Include configuration header
#include "uart_stream.h" // Include the header for the UartStream class

/**
 * Constructor for UartStream.
 * Calls the constructor of the Stream class.
 */
UartStream::UartStream() : Stream() {
}

/**
 * Initializes the UART communication.
 * Sets up the UART configuration and installs the UART driver.
 */
void UartStream::begin() {
    uart_config_t conf; // UART configuration structure

    // Set the UART parameters
    conf.baud_rate           = MONITOR_SPEED;            // Set the baud rate (defined in config.h)
    conf.data_bits           = UART_DATA_8_BITS;         // 8 data bits
    conf.parity              = UART_PARITY_DISABLE;      // No parity
    conf.stop_bits           = UART_STOP_BITS_1;         // 1 stop bit
    conf.flow_ctrl           = UART_HW_FLOWCTRL_DISABLE; // Disable hardware flow control
    conf.rx_flow_ctrl_thresh = 0;                        // No RX flow control threshold
    conf.use_ref_tick        = false;                    // Don't use reference tick

    // Configure the UART parameters for a given port 
    assert(uart_param_config(uart_port_, &conf) == ESP_OK);

    // Install the UART driver with RX and TX buffer sizes of 32000 bytes
    assert(uart_driver_install(uart_port_, 32000, 32000, 0, NULL, 0) == ESP_OK);
}

/**
 * Returns the next byte in the UART buffer without removing it.
 * Currently not implemented, always returns -1.
 *
 * @return -1 indicating this function is not implemented.
 */
int UartStream::peek() {
    return -1;
}

/**
 * Checks how many bytes are available for reading.
 *
 * @return The number of bytes available to read.
 */
int UartStream::available() {
    size_t size = 0; // Variable to store the size of available data

    // Get the number of bytes in the UART buffer
    assert(uart_get_buffered_data_len(uart_port_, &size) == ESP_OK);

    return size; // Return the number of available bytes
}

/**
 * Reads a single byte from the UART buffer.
 *
 * @return The byte read, or -1 if no data is available.
 */
int UartStream::read() {
    uint8_t b; // Variable to store the read byte
    int res = uart_read_bytes(uart_port_, &b, 1, 0); // Read a single byte from the UART buffer

    return res != 1 ? -1 : b; // Return the byte read, or -1 if no data was read
}

/**
 * Flushes the UART buffer.
 * Currently, this function does nothing.
 */
void UartStream::flush() {
    // No implementation needed for now
}

/**
 * Writes a single byte to the UART.
 *
 * @param b The byte to write.
 * @return The number of bytes written (1).
 */
size_t UartStream::write(uint8_t b) {
    return uart_write_bytes(uart_port_, (char*)&b, 1); // Write the byte to the UART
}

/**
 * Writes multiple bytes to the UART.
 *
 * @param buffer The buffer containing the bytes to write.
 * @param size The number of bytes to write.
 * @return The number of bytes written.
 */
size_t UartStream::write(const uint8_t *buffer, size_t size) {
    return uart_write_bytes(uart_port_, (const char*)buffer, size); // Write the bytes to the UART
}

