
#pragma once

#include <Arduino.h>
#include <driver/uart.h>

/**
 * Implementation of an Arduino Stream for UART serial communications using the ESP UART driver
 * directly, rather than the Arduino HAL which has a small fixed underlying RX FIFO size and
 * potentially other issues that cause dropped bytes at high speeds/bursts.
 * 
 * This is not a full or optimized implementation; just the minimal necessary for this project.
 */
class UartStream : public Stream {
    public:
        UartStream();

        void begin();

        // Stream methods
        /**
         * Checks how many bytes are available for reading.
         * @return The number of bytes available to read.
         */
        int available() override;

        /**
         * Reads a single byte from the UART buffer.
         * @return The byte read, or -1 if no data is available.
         */
        int read() override;

        /**
         * Returns the next byte in the UART buffer without removing it.
         * @return The next byte to be read, or -1 if no data is available.
         */
        int peek() override;

        /**
         * Flushes the UART buffer.
         */
        void flush() override;

        // Print methods
        /**
         * Writes a single byte to the UART.
         * @param b The byte to write.
         * @return The number of bytes written (1).
         */
        size_t write(uint8_t b) override;

        /**
         * Writes multiple bytes to the UART.
         * @param buffer The buffer containing the bytes to write.
         * @param size The number of bytes to write.
         * @return The number of bytes written.
         */
        size_t write(const uint8_t *buffer, size_t size) override;

    private:
        const uart_port_t uart_port_ = UART_NUM_0; // The UART port number
};