cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)
#set(PICO_SDK_PATH "D:/RaspberryPiPico/Pico/pico-sdk")


project(den_hc C CXX ASM)

add_executable(den_hc)

pico_sdk_init()
pico_generate_pio_header(den_hc ${CMAKE_CURRENT_LIST_DIR}/den.pio OUTPUT_DIR ${CMAKE_CURRENT_LIST_DIR})

#target_sources(den_hc PRIVATE uart.cpp)

pico_set_program_name(den_hc "pioUart")
pico_set_program_version(den_hc "0.1")
target_link_libraries(den_hc PRIVATE
        pico_stdlib
        hardware_pio
        pico_bootsel_via_double_reset
        )

pico_add_extra_outputs(den_hc)

# add url via pico_set_program_url
#example_auto_set_url(den_hc)
