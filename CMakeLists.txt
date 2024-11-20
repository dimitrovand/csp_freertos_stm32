#############################################################################################################################
# file:  CMakeLists.txt
#
# To build:
#     cmake -S ./ -B build -G Ninja
#     ninja -C build
#############################################################################################################################
cmake_minimum_required(VERSION 3.20)
set(CMAKE_TOOLCHAIN_FILE ${CMAKE_CURRENT_SOURCE_DIR}/toolchain.cmake)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(PROJECT_NAME "csp_freertos")
project(${PROJECT_NAME} ASM C)

file(GLOB PROJECT_SOURCES
    src/*.c
    src/stm32h7xx-hal-driver/Src/*.c
    # src/Core/Src/*.c
    # src/Core/Startup/*.s
    # src/Drivers/STM32H7xx_HAL_Driver/Src/*.c
)

list(APPEND PROJECT_SOURCES
    src/cmsis-device-h7/Source/Templates/gcc/startup_stm32h753xx.s
    src/cmsis-device-h7/Source/Templates/system_stm32h7xx.c
)

# Exclude files ending with 'template.c'
list(FILTER PROJECT_SOURCES EXCLUDE REGEX ".*template\\.c$")

list(APPEND PROJECT_INCLUDES
    src/
    src/CMSIS_5/CMSIS/Core/Include
    src/cmsis-device-h7/Include
    src/stm32h7xx-hal-driver/Inc
    src/stm32h7xx-hal-driver/Inc/Legacy 
    # src/Core/Inc
    # src/Drivers/STM32H7xx_HAL_Driver/Inc
    # src/Drivers/STM32H7xx_HAL_Driver/Inc/Legacy
    # src/Drivers/CMSIS/Device/ST/STM32H7xx/Include
    # src/Drivers/CMSIS/Include
)

add_compile_definitions(USE_HAL_DRIVER STM32H753xx)

set(CMAKE_EXECUTABLE_SUFFIX ".elf")
set(CMAKE_C_FLAGS "-mcpu=cortex-m7 -g3 -O0 -Wall -ffunction-sections -fdata-sections -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb --specs=nano.specs")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -MMD -MP")
set(CMAKE_EXE_LINKER_FLAGS "-T../STM32H753ZITX_FLASH.ld --specs=nosys.specs -Wl,-Map=freertos.map -Wl,--gc-sections -static -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group")
set(CMAKE_ASM_FLAGS "${CMAKE_C_FLAGS} -x assembler-with-cpp")

add_subdirectory(src/FreeRTOS-Config)

set(FREERTOS_HEAP "4" CACHE STRING "" FORCE)
set(FREERTOS_PORT "GCC_ARM_CM4F" CACHE STRING "" FORCE)
add_subdirectory(src/FreeRTOS-Kernel)

set(CSP_KERNEL_NAME "FreeRTOS" CACHE STRING "Kernel name for libcsp" FORCE)
set(CSP_USE_CUSTOM_ENDIAN ON CACHE BOOL "Use custom endian.h" FORCE)
add_subdirectory(src/libcsp)
target_include_directories(csp PRIVATE src)

add_executable(${PROJECT_NAME} ${PROJECT_SOURCES})
target_compile_definitions(${PROJECT_NAME} PRIVATE ${PROJECT_DEFINES})
include_directories(${PROJECT_INCLUDES})
target_link_libraries(${PROJECT_NAME} freertos_config freertos_kernel csp)

add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${CMAKE_PROJECT_NAME}>)
