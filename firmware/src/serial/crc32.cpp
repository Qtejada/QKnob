/* Simple public domain implementation of the standard CRC32 checksum.
 * Outputs the checksum for each file given as a command line argument.
 * Invalid file names and files that cause errors are silently skipped.
 * The program reads from stdin if it is called with no arguments. */

#include "crc32.h"


/**
 * Calculates the CRC32 value for a single byte.
 *
 * @param r The byte value to process.
 * @return The CRC32 value for the byte.
 */
static uint32_t crc32_for_byte(uint32_t r) {
    // Iterate over each bit in the byte
    for(int j = 0; j < 8; ++j) {
        // If the least significant bit is 1, apply the polynomial 0xEDB88320
        // Otherwise, just right shift the byte
        r = (r & 1 ? 0 : (uint32_t)0xEDB88320L) ^ r >> 1;
    }
    // XOR with 0xFF000000 to finalize the CRC value for the byte
    return r ^ (uint32_t)0xFF000000L;
}

/**
 * Computes the CRC32 checksum for a given data buffer.
 *
 * @param data Pointer to the data buffer.
 * @param n_bytes Number of bytes in the data buffer.
 * @param crc Pointer to the CRC32 value to be updated.
 */
void crc32(const void *data, size_t n_bytes, uint32_t* crc) {
    // Static table to store precomputed CRC32 values for all 256 possible byte values
    static uint32_t table[0x100];

    // If the table is not initialized, initialize it by computing the CRC32 for each byte value
    if (!*table) {
        for(size_t i = 0; i < 0x100; ++i) {
            table[i] = crc32_for_byte(i);
        }
    }

    // Process each byte in the data buffer
    for (size_t i = 0; i < n_bytes; ++i) {
        // Update the CRC value using the precomputed table and the current data byte
        *crc = table[(uint8_t)*crc ^ ((uint8_t*)data)[i]] ^ *crc >> 8;
    }
}

