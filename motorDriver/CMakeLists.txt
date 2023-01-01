add_library(motorDriver
        driver.cpp
        
)

target_link_libraries(motorDriver
pico_stdlib
hardware_pwm
        )
target_include_directories (motorDriver PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})