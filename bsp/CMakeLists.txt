cmake_minimum_required(VERSION 3.15)

#------------------------------------------------------------------------------
# FreeRTOS Setup
#------------------------------------------------------------------------------
# FreeRTOS configuration and hooks
add_library(freertos_config INTERFACE)
add_library(freertos_hooks INTERFACE)

target_include_directories(freertos_config INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/board)

target_sources(freertos_hooks INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/board/freertos.c)
target_link_libraries(freertos_hooks INTERFACE freertos_config)

# Add FreeRTOS kernel
add_subdirectory(FreeRTOS)
target_link_libraries(freertos_kernel PUBLIC freertos_hooks)

#------------------------------------------------------------------------------
# Collect Source Files
#------------------------------------------------------------------------------
# BSP sources
file(GLOB BSP_SOURCES 
    "board/*.c"
    "board/*.cpp"
)

# Firmware sources
file(GLOB FIRMWARE_SOURCES
    "Firmware/CMSIS/GD/GD32F30x/Source/*.c"
    "Firmware/GD32F30x_standard_peripheral/Source/*.c"
    "Firmware/CMSIS/GD/GD32F30x/Source/GCC/startup_gd32f30x_xd.s"
)

# HCL sources
file(GLOB HCL_SOURCES
    "hcl_gd32f3/*.c"
    "hcl_gd32f3/*.cpp"
)

# Remove files that are handled separately
list(REMOVE_ITEM BSP_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/board/freertos.c")

#------------------------------------------------------------------------------
# Add Sources to Main Target
#------------------------------------------------------------------------------
target_sources(${CURRENT_TARGET} PRIVATE
    ${BSP_SOURCES}
    ${FIRMWARE_SOURCES}
    ${HCL_SOURCES}
)

target_include_directories(${CURRENT_TARGET} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/board
    ${CMAKE_CURRENT_SOURCE_DIR}/Firmware/CMSIS
    ${CMAKE_CURRENT_SOURCE_DIR}/Firmware/CMSIS/GD/GD32F30x/Include
    ${CMAKE_CURRENT_SOURCE_DIR}/Firmware/GD32F30x_standard_peripheral/Include
    ${CMAKE_CURRENT_SOURCE_DIR}/hcl_gd32f3
)

target_compile_definitions(${CURRENT_TARGET} PUBLIC
    USE_STDPERIPH_DRIVER
    GD32F30X_XD
)

#------------------------------------------------------------------------------
# Link Dependencies and Set Options
#------------------------------------------------------------------------------
target_link_libraries(${CURRENT_TARGET} PUBLIC
    freertos_kernel
    freertos_config
)

# Linker configuration
target_link_options(${CURRENT_TARGET} PUBLIC
    -T${CMAKE_CURRENT_SOURCE_DIR}/gd32f30x_flash.ld
)
