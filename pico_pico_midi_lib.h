/**
 * MIT License

 * Copyright (c) 2022 rppicomidi

 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:

 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.

 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

#pragma once
#include <stdint.h>
#include "ring_buffer_lib.h"
namespace rppicomidi {
class Pico_pico_midi_lib {
public:
    // Singleton Pattern

    /**
     * @brief Get the Instance object
     *
     * @return the singleton instance
     */
    static Pico_pico_midi_lib& instance()
    {
        static Pico_pico_midi_lib _instance; // Guaranteed to be destroyed.
                                             // Instantiated on first use.
        return _instance;
    }
    Pico_pico_midi_lib(Pico_pico_midi_lib const&) = delete;
    void operator=(Pico_pico_midi_lib const&) = delete;

    /**
     * @brief Constructor: Set up the defined UART for use with this library
     *
     * @param uartnum is either 0 or 1 for UART0 or UART1
     * @param txgpio is the GPIO number for the UART TX pin
     * @param rxgpio is the GPIO number for the UART RX pin
     */
    Pico_pico_midi_lib();

    /**
     * @brief Initialize the the singleton instance of this class
     *
     * @param midi_cb is a pointer to the callback function that gets
     * called if a MIDI message is received
     * @param cmd_cb is a pointer to the callback function that gets
     * called if a command message is received
     * @param err_cb is a pointer to the callback function that gets
     * called if there is a checksum error
     */
    void init(void (*midi_cb_)(uint8_t *buffer, uint8_t buflen, uint8_t cable_num),
        void (*cmd_cb)(uint8_t header, uint8_t* buffer, uint16_t length),
        void (*err_cb)(uint8_t header, uint8_t* buffer, uint16_t length));

    /**
     * @brief poll the UART RX buffer and call the appropriate callback
     * function if a full packet of data has been received
     */
    void poll_rx_buffer();

    /**
     * @brief put the bytes into the UART TX buffer formatted with the a
     * Pico to Pico cmd header, length, payload and checksum
     *
     * @param header is the command header byte. Must be 0x40 or greater
     * @param payload is a pointer to an array of payload bytes
     * @param payload_length is the the number of bytes in the array
     *
     * @return the number of bytes from the buffer loaded; may be less than buflen if the buffer is full
     * @note you must call drain_tx_buffer() to actually send the bytes
     */
    uint8_t write_cmd_to_tx_buffer(uint8_t header, uint8_t *payload, RING_BUFFER_SIZE_TYPE payload_length);

    /**
     * @brief put the bytes into the UART TX buffer formatted with the a
     * Pico to Pico MIDI header and checksum
     *
     * @param buffer is a pointer to an array of bytes that contains the message
     * @param buflen is the the number of bytes in the array
     *
     * @return the number of bytes from the buffer loaded; may be less than buflen if the buffer is full
     * @note you must call drain_tx_buffer() to actually send the bytes
     */
    uint8_t write_midi_to_tx_buffer(uint8_t *buffer, RING_BUFFER_SIZE_TYPE buflen, uint8_t cable_num);

    /**
     * @brief start transmitting bytes from the tx buffer if not already doing so
     *
     * This function is necessary to kickstart data transmission either initially,
     * or after the buffer becomes completely empty.
     * @param instance is a pointer to this UART instance
     */
    void drain_tx_buffer();

private:
    // registered callback functions
    void (*midi_cb)(uint8_t *buffer, uint8_t buflen, uint8_t cable_num);
    void (*cmd_cb)(uint8_t header, uint8_t* buffer, uint16_t length);
    void (*err_cb)(uint8_t header, uint8_t* buffer, uint16_t length);

    // UART IRQ handler
    static void on_uart_irq();

    // Rx Callback Data Management
    void reset_rx_data();


    /**
     * @brief put the bytes in buffer into the UART TX buffer
     *
     * @param buffer is a pointer to an array of bytes to receive the message
     * @param buflen is the the number of bytes in the array
     *
     * @return the number of bytes loaded; may be less than buflen if the buffer is full
     * @note you must call drain_tx_buffer() to actually send the bytes
     */
    uint8_t write_tx_buffer(uint8_t *buffer, RING_BUFFER_SIZE_TYPE buflen);

    // UART selection and pin mapping.
    uart_inst_t *midi_uart;
    uint midi_uart_irq;
    uint midi_uart_tx_gpio;
    uint midi_uart_rx_gpio;

    // UART ring buffers
    static const uint8_t RING_BUFFER_LENGTH=255; // needs to be large enough to hold cmd, checksum, length and max payload
    ring_buffer_t midi_uart_rx, midi_uart_tx;
    uint8_t midi_uart_rx_buf[RING_BUFFER_LENGTH];
    uint8_t midi_uart_tx_buf[RING_BUFFER_LENGTH];
    uint8_t rx_header;
    uint8_t rx_checksum;
    uint8_t pending_rx_buffer[256]; // needs to be large enough to hold a full message
    uint8_t pending_rx_buffer_idx;  // store the next UART bytes starting here
    uint16_t expected_rx_buffer_bytes;
    static const uint PICO_PICO_MIDI_LIB_BAUD_RATE=921600;
    static const uint8_t MIN_MIDI_MESSAGE_HEADER=0x10;
    static const uint8_t MAX_MIDI_MESSAGE_HEADER=0x3F;
};

}
