target_sources(${CURRENT_TARGET} PRIVATE
    easylogger/src/elog.c
    easylogger/src/elog_utils.c
    easylogger/port/elog_port.cpp
    easylogger/src/elog_async.c
)

target_include_directories(${CURRENT_TARGET} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/easylogger/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/easylogger/plugins
)
