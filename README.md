# pico_pico_midi_lib
A C++ library for sending commands and MIDI messages between two RP2040 devices via UART

This library was designed to allow two RP2040-based boards to communicate via high speed
(921600 baud) UART. This is much faster than standard 31250 baud MIDI and can keep up with
multiple simulataneous MIDI streams that you might find in USB MIDI systems. It was
developed for [this project](https://github.com/rppicomidi/pico-mc-display-bridge) that
uses one Pico board to provide a USB MIDI device connection and to drive OLED displays,
and uses a second Pico board to provide a USB MIDI Host connection and to manage buttons.
The MIDI data and other commands on the UART link have a checksum byte to help detect
communication errors.

See the [Pico-Pico Message Format](https://github.com/rppicomidi/pico-mc-display-bridge#pico-pico-message-format)
section of the `README.md` file for that project to see an example of one way to use this library
to send MIDI data and commands over the UART connection.
