cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

set(CMAKE_CXX_STANDARD 17)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Os") 
set(CMAKE_C_FLAGS_DEBUG "-Os")
# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME New_Chassis)

# Include toolchain file
include("cmake/gcc-arm-none-eabi.cmake")

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME}
    D:/Project/Project_Robocon/Chassis_WorkSpace/R2/MDK-ARM/Filter/Second_Order.cpp
)
# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    "D:/Project/Project_Robocon/Chassis_WorkSpace/R2/MDK-ARM/Basket/Basket.c"
    "D:/Project/Project_Robocon/Chassis_WorkSpace/R2/MDK-ARM/Can/Can_Bsp.c"
    "D:/Project/Project_Robocon/Chassis_WorkSpace/R2/MDK-ARM/CPU/CPU_Load.c"
    "D:/Project/Project_Robocon/Chassis_WorkSpace/R2/MDK-ARM/Correct/Correct.c"
    "D:/Project/Project_Robocon/Chassis_WorkSpace/R2/MDK-ARM/Communication/Communication.c"
    "D:/Project/Project_Robocon/Chassis_WorkSpace/R2/MDK-ARM/Communication/Television.c"
    "D:/Project/Project_Robocon/Chassis_WorkSpace/R2/MDK-ARM/Debug/Send.c"
    "D:/Project/Project_Robocon/Chassis_WorkSpace/R2/MDK-ARM/Donkey/Chassis.c"
    "D:/Project/Project_Robocon/Chassis_WorkSpace/R2/MDK-ARM/Donkey/HO7213.c"
    "D:/Project/Project_Robocon/Chassis_WorkSpace/R2/MDK-ARM/Donkey/VESC.c"
    "D:/Project/Project_Robocon/Chassis_WorkSpace/R2/MDK-ARM/Filter/Kalman.c"
   

    "D:/Project/Project_Robocon/Chassis_WorkSpace/R2/MDK-ARM/GamePad/crc8.c"
    "D:/Project/Project_Robocon/Chassis_WorkSpace/R2/MDK-ARM/GamePad/mine.c"
    "D:/Project/Project_Robocon/Chassis_WorkSpace/R2/MDK-ARM/GamePad/mngCommu.c"
    "D:/Project/Project_Robocon/Chassis_WorkSpace/R2/MDK-ARM/GamePad/mySerial.c"
    "D:/Project/Project_Robocon/Chassis_WorkSpace/R2/MDK-ARM/GamePad/Nrf.c"
    "D:/Project/Project_Robocon/Chassis_WorkSpace/R2/MDK-ARM/GamePad/recDecode.c"
    "D:/Project/Project_Robocon/Chassis_WorkSpace/R2/MDK-ARM/GamePad/zigbee.c"

    "D:/Project/Project_Robocon/Chassis_WorkSpace/R2/MDK-ARM/Global/Global.c"

    "D:/Project/Project_Robocon/Chassis_WorkSpace/R2/MDK-ARM/Guard/Guard.c"

    "D:/Project/Project_Robocon/Chassis_WorkSpace/R2/MDK-ARM/Master/Fake_Rtos.c"
    "D:/Project/Project_Robocon/Chassis_WorkSpace/R2/MDK-ARM/Master/Flow.c"

    "D:/Project/Project_Robocon/Chassis_WorkSpace/R2/MDK-ARM/Position/Encoder.c"
    "D:/Project/Project_Robocon/Chassis_WorkSpace/R2/MDK-ARM/Position/Gyro.c"
    "D:/Project/Project_Robocon/Chassis_WorkSpace/R2/MDK-ARM/Position/Location.c"

    "D:/Project/Project_Robocon/Chassis_WorkSpace/R2/MDK-ARM/RGB/RGB.c"

    "D:/Project/Project_Robocon/Chassis_WorkSpace/R2/MDK-ARM/SPI_FDCAN/drv_canfdspi_api.c"
    "D:/Project/Project_Robocon/Chassis_WorkSpace/R2/MDK-ARM/SPI_FDCAN/drv_spi.c"
    "D:/Project/Project_Robocon/Chassis_WorkSpace/R2/MDK-ARM/SPI_FDCAN/SPI_FDCAN.c"

    "D:/Project/Project_Robocon/Chassis_WorkSpace/R2/MDK-ARM/Xuan/catchball.c"
    "D:/Project/Project_Robocon/Chassis_WorkSpace/R2/MDK-ARM/Xuan/HighTorque.c"
    "D:/Project/Project_Robocon/Chassis_WorkSpace/R2/MDK-ARM/Xuan/Interact.c"
    # Add user sources here
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    "D:/Project/Project_Robocon/Chassis_WorkSpace/R2/MDK-ARM/Basket"
    "D:/Project/Project_Robocon/Chassis_WorkSpace/R2/MDK-ARM/Can"
    "D:/Project/Project_Robocon/Chassis_WorkSpace/R2/MDK-ARM/Correct"
    "D:/Project/Project_Robocon/Chassis_WorkSpace/R2/MDK-ARM/CPU"
    "D:/Project/Project_Robocon/Chassis_WorkSpace/R2/MDK-ARM/Communication"
    "D:/Project/Project_Robocon/Chassis_WorkSpace/R2/MDK-ARM/Debug"
    "D:/Project/Project_Robocon/Chassis_WorkSpace/R2/MDK-ARM/Donkey"
    "D:/Project/Project_Robocon/Chassis_WorkSpace/R2/MDK-ARM/Filter"
    "D:/Project/Project_Robocon/Chassis_WorkSpace/R2/MDK-ARM/GamePad"
    "D:/Project/Project_Robocon/Chassis_WorkSpace/R2/MDK-ARM/Global"
    "D:/Project/Project_Robocon/Chassis_WorkSpace/R2/MDK-ARM/Guard"
    "D:/Project/Project_Robocon/Chassis_WorkSpace/R2/MDK-ARM/Master"
    "D:/Project/Project_Robocon/Chassis_WorkSpace/R2/MDK-ARM/Position"
    "D:/Project/Project_Robocon/Chassis_WorkSpace/R2/MDK-ARM/RGB"
    "D:/Project/Project_Robocon/Chassis_WorkSpace/R2/MDK-ARM/SPI_FDCAN"
    "D:/Project/Project_Robocon/Chassis_WorkSpace/R2/MDK-ARM/Xuan"

    "D:/Project/Project_Robocon/Chassis_WorkSpace/R2/lib/eigen"
    # Add user defined include paths
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    USE_HAL_DRIVER
    STM32H723xx
    USE_PWR_LDO_SUPPLY
    # Add user defined symbols
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}

    stm32cubemx

    # Add user defined libraries
)
