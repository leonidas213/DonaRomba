#add_subdirectory(examples)
add_library(mpu6050
         MPU6050.cpp
         I2Cdev.cpp
)

target_link_libraries(mpu6050
pico_stdlib 
hardware_i2c
pico_double

)
target_include_directories (mpu6050 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
