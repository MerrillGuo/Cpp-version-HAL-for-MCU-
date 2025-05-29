# Collect source files
file(GLOB_RECURSE HCL_SOURCES
    "*.cpp"
    "*.c"
)

# Add sources to main target
target_sources(${CURRENT_TARGET}
    PRIVATE
        ${HCL_SOURCES}
)

# Add include directories
target_include_directories(${CURRENT_TARGET}
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
) 