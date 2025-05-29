cmake_minimum_required(VERSION 3.22)

#------------------------------------------------------------------------------
# Project Options
#------------------------------------------------------------------------------
option(ENABLE_TESTING "Enable testing" ON)
option(ENABLE_COVERAGE "Enable coverage reporting" OFF)

#------------------------------------------------------------------------------
# Toolchain and Project Setup
#------------------------------------------------------------------------------
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/toolchain/gcc-arm-none-eabi.cmake)
project(cleanix_mcu C CXX ASM)
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/config/GlobalConfig.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/config/BSPConfig.cmake)

#------------------------------------------------------------------------------
# Set Target Variable
#------------------------------------------------------------------------------
if(NOT ENABLE_TESTING)
    set(CURRENT_TARGET ${PROJECT_NAME})
else()
    set(CURRENT_TARGET run_tests)
    add_link_options(-u _printf_float)
endif()

#------------------------------------------------------------------------------
# Main Build
#------------------------------------------------------------------------------
# Create the target first
add_executable(${CURRENT_TARGET})

# Add all components
add_subdirectory(bsp/${BSP_NAME})
add_subdirectory(oscl)
add_subdirectory(hcl)
add_subdirectory(modules)
add_subdirectory(third_party/EasyLogger)

if(NOT ENABLE_TESTING)
    message(STATUS "Building firmware with BSP: ${BSP_NAME}")
    add_subdirectory(applications)
    
    # Generate hex and binary files
    set(HEX_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.hex)
    set(BIN_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.bin)
    
    add_custom_command(
        TARGET ${CURRENT_TARGET}
        POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${CURRENT_TARGET}> ${HEX_FILE}
        COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${CURRENT_TARGET}> ${BIN_FILE}
        COMMENT "Building ${HEX_FILE}\nBuilding ${BIN_FILE}"
    )
else()
    message(STATUS "Building with testing enabled")
    enable_testing()

    add_compile_definitions(-DTESTING)
    
    add_subdirectory(tests)

    # Register with CTest
    add_test(NAME all_tests COMMAND ${CURRENT_TARGET})
endif()
