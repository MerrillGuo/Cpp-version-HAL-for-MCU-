# Collect source files
file(GLOB OSCL_SOURCES
    "freertos/*.cpp"
)

# Add sources to main target
target_sources(${CURRENT_TARGET}
    PRIVATE
        ${OSCL_SOURCES}
)

# Add include directories
target_include_directories(${CURRENT_TARGET}
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
)

# Add FreeRTOS dependency
target_link_libraries(${CURRENT_TARGET}
    PUBLIC
        freertos_kernel
) 