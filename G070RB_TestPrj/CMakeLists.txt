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


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME G070RB_TestPrj)

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
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    ${CMAKE_SOURCE_DIR}/Drivers/DrvGY_BMP180/Src
    ${CMAKE_SOURCE_DIR}/Drivers/DrvGY_BMP180/Src/DrvGY_BMP180.c
    ${CMAKE_SOURCE_DIR}/Drivers/DrvGY_BMP180/Cfg/DrvGY_BMP180_Cfg.c
    ${CMAKE_SOURCE_DIR}/Drivers/DrvGY_MPU60X0/Src
    ${CMAKE_SOURCE_DIR}/Drivers/DrvGY_MPU60X0/Src/DrvGY_MPU60X0.c
    ${CMAKE_SOURCE_DIR}/Drivers/DrvGY_MPU60X0/Cfg/DrvGY_MPU60X0_Cfg.c
    ${CMAKE_SOURCE_DIR}/Drivers/DrvGY_HMC5883L/Src
    ${CMAKE_SOURCE_DIR}/Drivers/DrvGY_HMC5883L/Src/DrvGY_HMC5883L.c
    ${CMAKE_SOURCE_DIR}/Drivers/DrvGY_HMC5883L/Cfg/DrvGY_HMC5883L_Cfg.c
    ${CMAKE_SOURCE_DIR}/Core/Src/DrvI2c.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    ${CMAKE_SOURCE_DIR}/Core/Inc
    ${CMAKE_SOURCE_DIR}/Drivers/DrvGY_BMP180/Inc
    ${CMAKE_SOURCE_DIR}/Drivers/DrvGY_BMP180/Cfg
    ${CMAKE_SOURCE_DIR}/Drivers/DrvGY_MPU60X0/Inc
    ${CMAKE_SOURCE_DIR}/Drivers/DrvGY_MPU60X0/Cfg
    ${CMAKE_SOURCE_DIR}/Drivers/DrvGY_HMC5883L/Inc
    ${CMAKE_SOURCE_DIR}/Drivers/DrvGY_HMC5883L/Cfg
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    # Add user defined libraries
)
