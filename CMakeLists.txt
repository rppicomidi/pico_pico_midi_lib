cmake_minimum_required(VERSION 3.13)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
add_library(pico_pico_midi_lib INTERFACE)
target_sources(pico_pico_midi_lib INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/pico_pico_midi_lib.cpp
)
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../ring_buffer_lib lib/ring_buffer_lib)
target_include_directories(pico_pico_midi_lib INTERFACE ${CMAKE_CURRENT_LIST_DIR})
