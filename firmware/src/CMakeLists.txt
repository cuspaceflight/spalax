
set(SOURCES ${SOURCES}
        ${CMAKE_CURRENT_SOURCE_DIR}/adis16405.c
        ${CMAKE_CURRENT_SOURCE_DIR}/badthinghandler.c
        ${CMAKE_CURRENT_SOURCE_DIR}/mpu9250.c
        ${CMAKE_CURRENT_SOURCE_DIR}/ms5611.c
        ${CMAKE_CURRENT_SOURCE_DIR}/ublox.c
        ${CMAKE_CURRENT_SOURCE_DIR}/main.c
        PARENT_SCOPE)

set(HEADERS ${HEADERS}
        ${CMAKE_CURRENT_SOURCE_DIR}/adis16405.h
        ${CMAKE_CURRENT_SOURCE_DIR}/badthinghandler.h
        ${CMAKE_CURRENT_SOURCE_DIR}/mpu9250.h
        ${CMAKE_CURRENT_SOURCE_DIR}/mpu9250-reg.h
        ${CMAKE_CURRENT_SOURCE_DIR}/ms5611.h
        ${CMAKE_CURRENT_SOURCE_DIR}/ublox.h
        ${CMAKE_CURRENT_SOURCE_DIR}/ublox_ubx.h
        PARENT_SCOPE)
