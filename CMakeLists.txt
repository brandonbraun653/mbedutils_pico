# RPI Pico Based Library Integration for MbedUtils
# Targets are defined in: https://github.com/brandonbraun653/mbedutils
if( PICO_SDK_PATH )
    file(GLOB PICO_INTF_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)
    add_library(mbedutils_lib_pico STATIC ${PICO_INTF_SOURCES})
    target_link_libraries(mbedutils_lib_pico PRIVATE
        mbedutils_headers
        mbedutils_internal_headers

        # These targets are defined in the pico-sdk CMake files after importing.
        hardware_dma_headers
        hardware_timer_headers
        hardware_uart_headers
        pico_sync_headers
    )
endif()
