add_library(adxl
        ADXL345.cpp

)

target_link_libraries(adxl
pico_stdlib
hardware_i2c
        )
target_include_directories (adxl PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})