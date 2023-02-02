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

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
#include "pico/binary_info.h"
#include "pico/sync.h"
#include "ring_buffer_lib.h"
#include "pico_pico_midi_lib_config.h"
#include "pico_pico_midi_lib.h"

#ifndef PICO_PICO_MIDI_LIB_UART_TX_GPIO
#error "projects that use Pico_pico_midi_lib class must define PICO_PICO_MIDI_LIB_UART_TX_GPIO"
#endif
#ifndef PICO_PICO_MIDI_LIB_UART_RX_GPIO
#error "projects that use Pico_pico_midi_lib class must define PICO_PICO_MIDI_LIB_UART_RX_GPIO"
#endif

static volatile uint32_t log_err[128];
static volatile uint8_t log_idx = 0;
void rppicomidi::Pico_pico_midi_lib::on_uart_irq()
{
    //uint32_t status = uart_get_hw(instance().midi_uart)->mis; // the reason this irq happened
    uart_inst_t *midi_uart = instance().midi_uart;
    auto uhw = uart_get_hw(instance().midi_uart);
    while(uart_is_readable(midi_uart)) {
        uint32_t dr = uhw->dr;
        uint8_t val = dr & 0xff;
        if ((dr & 0xf00) == 0) {
            // then data read was OK.
            // ignore the return value. If the buffer is full, nothing can be done
            (void)ring_buffer_push_unsafe(&instance().midi_uart_rx, &val, 1);
        }
        else {
            // log the discarded bytes
            if (log_idx < 128)
                log_err[log_idx++] = dr;
        }
    }
    while ((uhw->fr & UART_UARTFR_TXFF_BITS) == 0 && !ring_buffer_is_empty_unsafe(&instance().midi_uart_tx)) {
        uint8_t val;
        ring_buffer_pop_unsafe(&instance().midi_uart_tx, &val, 1);
        uhw->dr = val;
    }
    // Done sending transmit data; clear the interrupt in case there is not
    // enough data to fill the FIFO past the watermark
    uhw->icr = UART_UARTICR_TXIC_BITS;
    // Disable the transmit interrupt if there is no more data to send
    if (ring_buffer_is_empty_unsafe(&instance().midi_uart_tx)) {
        uhw->imsc &= ~UART_UARTIMSC_TXIM_BITS;
    }
}

rppicomidi::Pico_pico_midi_lib::Pico_pico_midi_lib() :
    midi_cb{nullptr}, cmd_cb{nullptr}
{
    reset_rx_data();
    midi_uart = uart1;
    midi_uart_irq = UART1_IRQ;

    irq_set_exclusive_handler(midi_uart_irq, on_uart_irq);
    midi_uart_tx_gpio = PICO_PICO_MIDI_LIB_UART_TX_GPIO;
    midi_uart_rx_gpio = PICO_PICO_MIDI_LIB_UART_RX_GPIO;
    // Set up the UART hardware
    uart_init(midi_uart, PICO_PICO_MIDI_LIB_BAUD_RATE);
    uart_set_format(midi_uart, 8, 1, UART_PARITY_NONE);
    uart_set_hw_flow(midi_uart, false, false);
    uart_set_fifo_enabled(midi_uart, true);
    uart_set_translate_crlf(midi_uart, false);

    gpio_set_function(midi_uart_tx_gpio, GPIO_FUNC_UART);
    gpio_set_function(midi_uart_rx_gpio, GPIO_FUNC_UART);

    // Prepare the MIDI UART ring buffers and interrupt handler and enable interrupts
    ring_buffer_init(&midi_uart_rx, midi_uart_rx_buf, RING_BUFFER_LENGTH, midi_uart_irq);
    ring_buffer_init(&midi_uart_tx, midi_uart_tx_buf, RING_BUFFER_LENGTH, midi_uart_irq);

    //uart_set_irq_enables(midi_uart, false, false);
    // Disable interrupt masks
    uart_get_hw(midi_uart)->imsc &= ~(UART_UARTIMSC_RXIM_BITS | UART_UARTIMSC_RTIM_BITS | UART_UARTIMSC_TXIM_BITS);

    irq_set_enabled(midi_uart_irq, true);
}

void rppicomidi::Pico_pico_midi_lib::init(void (*midi_cb_)(uint8_t *buffer, uint8_t buflen, uint8_t cable_num),
                            void (*cmd_cb_)(uint8_t header, uint8_t* buffer, uint16_t length))
{
    // Set up callbacks
    midi_cb = midi_cb_;
    cmd_cb = cmd_cb_;
    // Flush the RX fifo in case there is any data in it now
    uint32_t status = uart_get_hw(midi_uart)->ris; // the reason this irq happened
    while (status & UART_UARTRIS_RXRIS_BITS) {
        // Got a byte
        volatile uint32_t dr = uart_get_hw(midi_uart)->dr;
        (void)dr;
        status = uart_get_hw(midi_uart)->ris; // the reason this irq happened
    }    
    uart_set_irq_enables(midi_uart, true, true);
    uart_get_hw(instance().midi_uart)->imsc &= ~UART_UARTIMSC_TXIM_BITS;
}

void rppicomidi::Pico_pico_midi_lib::reset_rx_data()
{
    rx_header = 0;
    rx_checksum = 0;
    pending_rx_buffer_idx = 0;
    expected_rx_buffer_bytes = 0xFFFF; // an illegal value
}

void rppicomidi::Pico_pico_midi_lib::poll_rx_buffer()
{
    uint8_t rx[128];
    uint8_t nread = ring_buffer_pop(&midi_uart_rx, rx, sizeof(rx));
    for (int rx_idx = 0; rx_idx < nread; rx_idx++) {
        //printf("%02x ", rx[rx_idx]);
        if (rx_header == 0) {
            rx_header = rx[rx_idx];
            if (rx_header < MIN_MIDI_MESSAGE_HEADER) {
                // TODO: handle packet synchronization error
                reset_rx_data();
                return;
            }
            if (rx_header <= MAX_MIDI_MESSAGE_HEADER) {
                // MIDI packet
                expected_rx_buffer_bytes = (rx_header >> 4) & 0xf;
                //printf("MIDI: expecting %u bytes\r\n", expected_rx_buffer_bytes);
            }
            else {
                expected_rx_buffer_bytes = 0xFFFF; // will get that with next byte
            }
            rx_checksum = rx_header;
        }
        else if (expected_rx_buffer_bytes == 0xFFFF) {
            expected_rx_buffer_bytes = rx[rx_idx];
            rx_checksum ^= expected_rx_buffer_bytes;
        }
        else if (pending_rx_buffer_idx == expected_rx_buffer_bytes) {
            if (rx_checksum == rx[rx_idx]) {
                if (rx_header <= MAX_MIDI_MESSAGE_HEADER && midi_cb != nullptr) {
                    // MIDI Packet OK; send data and cable number
                    midi_cb(pending_rx_buffer, expected_rx_buffer_bytes, rx_header & 0xf);
                }
                else if (cmd_cb != nullptr) {
                    // Command Packet OK; send the command up to the application
                    cmd_cb(rx_header, pending_rx_buffer, pending_rx_buffer_idx);
                }
            }
            else {
                // TODO handle checksum error
                printf("checksum error %02x != %02x\r\n", rx_checksum, rx[rx_idx]);
                for (uint8_t jdx=0; jdx < log_idx; jdx++) {
                    printf("%08lx ", log_err[jdx]);
                }
                printf("\r\n");
            }
            // make ready for the next packet or drop the packet if the CRC check failed
            reset_rx_data();
        }
        else if (pending_rx_buffer_idx < expected_rx_buffer_bytes) {
            //printf("pending_rx_buffer[%u]=%02x\r\n", pending_rx_buffer_idx, rx[rx_idx]);
            pending_rx_buffer[pending_rx_buffer_idx++] = rx[rx_idx];
            rx_checksum ^= rx[rx_idx];
        }
        else {
            // Should not get here
            printf("illegal pending_rx_buffer_idx\r\n");
        }
        //printf("\r\n");
    }
}

uint8_t rppicomidi::Pico_pico_midi_lib::write_tx_buffer(uint8_t* buffer, RING_BUFFER_SIZE_TYPE buflen)
{
    return ring_buffer_push(&midi_uart_tx, buffer, buflen);
}

uint8_t rppicomidi::Pico_pico_midi_lib::write_cmd_to_tx_buffer(uint8_t header, uint8_t *payload, RING_BUFFER_SIZE_TYPE payload_length)
{
    if ((payload_length !=0 && payload == nullptr) || header < 0x40)
        return 0;

    uint8_t payload_bytes_sent = 0;
    uint8_t checksum = header;
    if (write_tx_buffer(&header, 1) == 1 && write_tx_buffer(&payload_length, 1) == 1) {
        if (payload_length) {
            checksum ^= payload_length;
            payload_bytes_sent = write_tx_buffer(payload, payload_length);
            if (payload_bytes_sent == payload_length) {
                for (int byte = 0; byte < payload_length; byte++) {
                    checksum ^= payload[byte];
                }
                if (write_tx_buffer(&checksum, 1) != 1) {
                    printf("error failed to write cmd checksum\r\n");
                }
            }
            else {
                printf("failed to write %u payload bytes for command %02x\r\n", payload_length - payload_bytes_sent, header);
            }
        }
        else if (write_tx_buffer(&checksum, 1) != 1) {
            printf("error failed to write cmd checksum\r\n");
        }

    }
    else {
        printf("failed to write cmd header and/or length\r\n");
    }
    return payload_bytes_sent;
}

uint8_t rppicomidi::Pico_pico_midi_lib::write_midi_to_tx_buffer(uint8_t *buffer, RING_BUFFER_SIZE_TYPE buflen, uint8_t cable_num)
{
    if (buffer == nullptr || cable_num > 15)
        return 0;

    uint8_t buffer_bytes_sent = 0;
    while (buffer_bytes_sent < buflen) {
        uint8_t bytes_to_send = (buflen-buffer_bytes_sent) >=3 ? 0x3:(buflen-buffer_bytes_sent);
        uint8_t header = (bytes_to_send << 4) | cable_num;
        uint8_t checksum = header;
        if (write_tx_buffer(&header, 1) == 1) {
            if (write_tx_buffer(buffer+buffer_bytes_sent, bytes_to_send) == bytes_to_send) {
                for (int byte = 0; byte < bytes_to_send; byte++) {
                    checksum ^= buffer[buffer_bytes_sent+byte];
                }
                if (write_tx_buffer(&checksum, 1) == 1) {
                    buffer_bytes_sent += bytes_to_send;
                }
                else {
                    break;
                }
            }
            else {
                break;
            }
        }
        else {
            break;
        }
    }
    return buffer_bytes_sent;
}

void rppicomidi::Pico_pico_midi_lib::drain_tx_buffer()
{
    // disable UART interrupts because messing with the UART TX FIFO
    irq_set_enabled(midi_uart_irq, false);
    // If the UART is not transmitting, fill the UART TX FIFO as long
    // as it has space and there is data to fill it with
    auto uhw = uart_get_hw(midi_uart);
    volatile uint32_t flags = uhw->fr;
    if ((flags & UART_UARTFR_BUSY_BITS) == 0) {
        uint8_t val;
        while ((flags & UART_UARTFR_TXFF_BITS) == 0 && !ring_buffer_is_empty_unsafe(&midi_uart_tx)) {
            RING_BUFFER_SIZE_TYPE result = ring_buffer_pop_unsafe(&midi_uart_tx, &val, 1);
            assert(result == 1);
            uhw->dr = val;
            flags = uhw->fr;
        }
    }
    if (!ring_buffer_is_empty_unsafe(&midi_uart_tx)) {
        uart_get_hw(instance().midi_uart)->imsc |= UART_UARTIMSC_TXIM_BITS;
    }
    irq_set_enabled(midi_uart_irq, true);
}
