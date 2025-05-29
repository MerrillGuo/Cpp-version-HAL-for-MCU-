# Add Unity library
add_library(unity STATIC ${CMAKE_SOURCE_DIR}/third_party/Unity/src/unity.c
                         ${CMAKE_SOURCE_DIR}/third_party/Unity/extras/fixture/src/unity_fixture.c)
target_include_directories(unity
                           PUBLIC ${CMAKE_SOURCE_DIR}/third_party/Unity/src
                           PUBLIC ${CMAKE_SOURCE_DIR}/third_party/Unity/extras/fixture/src
                           PUBLIC ${CMAKE_SOURCE_DIR}/tests)
target_link_libraries(${CURRENT_TARGET} PRIVATE unity)

# Collect test sources
file(GLOB_RECURSE TEST_SOURCES "*.cpp")

# Add sources to test target
target_sources(${CURRENT_TARGET} PRIVATE ${TEST_SOURCES})

# Add include directories
target_include_directories(${CURRENT_TARGET} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

# Add compile definitions
target_compile_definitions(${CURRENT_TARGET} PRIVATE TESTING)

# Configure coverage if enabled
if(ENABLE_COVERAGE)
  target_compile_options(${CURRENT_TARGET} PRIVATE --coverage)
  target_link_options(${CURRENT_TARGET} PRIVATE --coverage)
endif()
