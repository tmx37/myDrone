set(CMAKE_SYSTEM_NAME               Generic)
set(CMAKE_SYSTEM_PROCESSOR          arm)

set(CMAKE_C_COMPILER_FORCED TRUE)
set(CMAKE_CXX_COMPILER_FORCED TRUE)
set(CMAKE_C_COMPILER_ID GNU)
set(CMAKE_CXX_COMPILER_ID GNU)

# Toolchain lookup, first match wins (same file works on Linux and Windows):
#   1. ARM_TOOLCHAIN_PATH (-D cache variable or environment variable): folder containing arm-none-eabi-gcc
#   2. "gnu-tools-for-stm32" bundle installed by the STM32Cube VS Code extension (newest version)
#   3. arm-none-eabi-* on PATH (e.g. STM32CubeCLT)
if(NOT ARM_TOOLCHAIN_PATH AND DEFINED ENV{ARM_TOOLCHAIN_PATH})
    set(ARM_TOOLCHAIN_PATH "$ENV{ARM_TOOLCHAIN_PATH}")
endif()
if(NOT ARM_TOOLCHAIN_PATH)
    if(CMAKE_HOST_WIN32)
        set(_STM32CUBE_BUNDLES "$ENV{LOCALAPPDATA}/stm32cube/bundles")
    else()
        set(_STM32CUBE_BUNDLES "$ENV{HOME}/.local/share/stm32cube/bundles")
    endif()
    file(GLOB _STM32CUBE_GNU_TOOLS LIST_DIRECTORIES true "${_STM32CUBE_BUNDLES}/gnu-tools-for-stm32/*/bin")
    if(_STM32CUBE_GNU_TOOLS)
        list(SORT _STM32CUBE_GNU_TOOLS COMPARE NATURAL ORDER DESCENDING)
        list(GET _STM32CUBE_GNU_TOOLS 0 ARM_TOOLCHAIN_PATH)
    endif()
endif()

if(ARM_TOOLCHAIN_PATH)
    file(TO_CMAKE_PATH "${ARM_TOOLCHAIN_PATH}" ARM_TOOLCHAIN_PATH)
    set(TOOLCHAIN_PREFIX            "${ARM_TOOLCHAIN_PATH}/arm-none-eabi-")
else()
    set(TOOLCHAIN_PREFIX            arm-none-eabi-)
endif()
if(CMAKE_HOST_WIN32)
    set(TOOLCHAIN_SUFFIX            ".exe")
endif()

set(CMAKE_C_COMPILER                ${TOOLCHAIN_PREFIX}gcc${TOOLCHAIN_SUFFIX})
set(CMAKE_ASM_COMPILER              ${CMAKE_C_COMPILER})
set(CMAKE_CXX_COMPILER              ${TOOLCHAIN_PREFIX}g++${TOOLCHAIN_SUFFIX})
set(CMAKE_LINKER                    ${TOOLCHAIN_PREFIX}g++${TOOLCHAIN_SUFFIX})
set(CMAKE_OBJCOPY                   ${TOOLCHAIN_PREFIX}objcopy${TOOLCHAIN_SUFFIX})
set(CMAKE_SIZE                      ${TOOLCHAIN_PREFIX}size${TOOLCHAIN_SUFFIX})

set(CMAKE_EXECUTABLE_SUFFIX_ASM     ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_C       ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_CXX     ".elf")

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# MCU specific flags
set(TARGET_FLAGS "-mcpu=cortex-m0plus ")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${TARGET_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -Wpedantic -fdata-sections -ffunction-sections")
if(CMAKE_BUILD_TYPE MATCHES Debug)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O0 -g3")
endif()
if(CMAKE_BUILD_TYPE MATCHES Release)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Os -g0")
endif()

set(CMAKE_ASM_FLAGS "${CMAKE_C_FLAGS} -x assembler-with-cpp -MMD -MP")
set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -fno-rtti -fno-exceptions -fno-threadsafe-statics")

set(CMAKE_C_LINK_FLAGS "${TARGET_FLAGS}")
set(CMAKE_C_LINK_FLAGS "${CMAKE_C_LINK_FLAGS} -T \"${CMAKE_SOURCE_DIR}/stm32g070rbtx_flash.ld\"")
set(CMAKE_C_LINK_FLAGS "${CMAKE_C_LINK_FLAGS} --specs=nano.specs")
set(CMAKE_C_LINK_FLAGS "${CMAKE_C_LINK_FLAGS} -Wl,-Map=${CMAKE_PROJECT_NAME}.map -Wl,--gc-sections")
set(CMAKE_C_LINK_FLAGS "${CMAKE_C_LINK_FLAGS} -Wl,--start-group -lc -lm -Wl,--end-group")
set(CMAKE_C_LINK_FLAGS "${CMAKE_C_LINK_FLAGS} -Wl,--print-memory-usage")

set(CMAKE_CXX_LINK_FLAGS "${CMAKE_C_LINK_FLAGS} -Wl,--start-group -lstdc++ -lsupc++ -Wl,--end-group")