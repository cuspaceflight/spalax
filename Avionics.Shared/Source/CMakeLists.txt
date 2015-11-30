

set (SOURCES ${SOURCES}
    ${CMAKE_CURRENT_SOURCE_DIR}/calibration.c
    ${CMAKE_CURRENT_SOURCE_DIR}/logging.c
    ${CMAKE_CURRENT_SOURCE_DIR}/mission.c
    ${CMAKE_CURRENT_SOURCE_DIR}/state_estimate.c
    ${CMAKE_CURRENT_SOURCE_DIR}/telemetry.c
    ${CMAKE_CURRENT_SOURCE_DIR}/kalman.c
    ${CMAKE_CURRENT_SOURCE_DIR}/time_utils.c

    PARENT_SCOPE)

set (HEADERS ${HEADERS}
    ${CMAKE_CURRENT_SOURCE_DIR}/calibration.h
    ${CMAKE_CURRENT_SOURCE_DIR}/logging.h
    ${CMAKE_CURRENT_SOURCE_DIR}/mission.h
    ${CMAKE_CURRENT_SOURCE_DIR}/state_estimate.h
    ${CMAKE_CURRENT_SOURCE_DIR}/telemetry.h
    ${CMAKE_CURRENT_SOURCE_DIR}/time_utils.h
    ${CMAKE_CURRENT_SOURCE_DIR}/math_utils.h
    ${CMAKE_CURRENT_SOURCE_DIR}/kalman.h
    PARENT_SCOPE)
