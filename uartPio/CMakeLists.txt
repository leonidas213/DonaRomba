cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)
set(PICO_SDK_PATH "D:/RaspberryPiPico/Pico/pico-sdk")


project(pio_uart C CXX ASM)

add_executable(pio_uart)

pico_sdk_init()
pico_generate_pio_header(pio_uart ${CMAKE_CURRENT_LIST_DIR}/uart.pio OUTPUT_DIR ${CMAKE_CURRENT_LIST_DIR})

target_sources(pio_uart PRIVATE uart.cpp)

pico_set_program_name(pio_uart "pioUart")
pico_set_program_version(pio_uart "0.1")
target_link_libraries(pio_uart PRIVATE
        pico_stdlib
        hardware_pio
        pico_bootsel_via_double_reset
        )

pico_add_extra_outputs(pio_uart)

# add url via pico_set_program_url
#example_auto_set_url(pio_uart)
