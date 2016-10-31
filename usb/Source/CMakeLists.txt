set (SOURCES ${SOURCES}
        ${CMAKE_CURRENT_SOURCE_DIR}/SerialDriver.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/CanSerialDriver.cpp
        PARENT_SCOPE)

set (HEADERS ${HEADERS}
        ${CMAKE_CURRENT_SOURCE_DIR}/SerialDriver.h
        ${CMAKE_CURRENT_SOURCE_DIR}/CanSerialDriver.h
        ${CMAKE_CURRENT_SOURCE_DIR}/Utils.h
        PARENT_SCOPE)