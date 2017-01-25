
set (SOURCES ${SOURCES}
        ${CMAKE_CURRENT_SOURCE_DIR}/mpu9250_calibration.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/ms5611_calibration.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/adis16405_calibration.cpp
    PARENT_SCOPE)

set (HEADERS ${HEADERS}
    PARENT_SCOPE)
