add_library(freertos_config INTERFACE)

target_include_directories(freertos_config INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}
)