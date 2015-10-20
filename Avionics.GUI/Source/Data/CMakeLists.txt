
set (SOURCES ${SOURCES}
    ${CMAKE_CURRENT_SOURCE_DIR}/DataSource.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/FileDataSource.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/SerialDataSource.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/SerialPort.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/TelemetrySerialPort.cpp
    PARENT_SCOPE)

set (HEADERS ${HEADERS}
    ${CMAKE_CURRENT_SOURCE_DIR}/DataSource.h
    ${CMAKE_CURRENT_SOURCE_DIR}/FileDataSource.h
    ${CMAKE_CURRENT_SOURCE_DIR}/SerialDataSource.h
    ${CMAKE_CURRENT_SOURCE_DIR}/SerialPort.h
    ${CMAKE_CURRENT_SOURCE_DIR}/TelemetrySerialPort.h
    PARENT_SCOPE)
