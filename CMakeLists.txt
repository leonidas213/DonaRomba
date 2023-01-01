# cmake version
cmake_minimum_required(VERSION 3.13)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
#set(PICO_BOARD pico_w)
# include the sdk.cmake file
include(pico_sdk_import.cmake)

# give the project a name (anything you want)
project(Roomba C CXX ASM)

# initialize the sdk
pico_sdk_init()

#######

add_executable(Roomba main.cpp)
# name anything you want

# must match with executable name and source file names
pico_enable_stdio_uart(Roomba 0)
pico_enable_stdio_usb(Roomba 1)
add_subdirectory(uartPio)
add_subdirectory(motorDriver)
#add_subdirectory(MPU6050)
# must match with executable name
target_link_libraries(Roomba pico_stdlib 
hardware_pwm
hardware_i2c
motorDriver
pico_bootsel_via_double_reset
)

# must match with executable name
pico_add_extra_outputs(Roomba)
