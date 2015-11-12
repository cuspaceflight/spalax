

set (SOURCES ${SOURCES}
    ${CMAKE_CURRENT_SOURCE_DIR}/calibration.c
    ${CMAKE_CURRENT_SOURCE_DIR}/logging.c
    ${CMAKE_CURRENT_SOURCE_DIR}/mag_rotation_estimator.c
    ${CMAKE_CURRENT_SOURCE_DIR}/mission.c
    ${CMAKE_CURRENT_SOURCE_DIR}/orientation_kalman.c
    ${CMAKE_CURRENT_SOURCE_DIR}/quest_estimator.c
    ${CMAKE_CURRENT_SOURCE_DIR}/state_estimate.c
    ${CMAKE_CURRENT_SOURCE_DIR}/telemetry.c
    ${CMAKE_CURRENT_SOURCE_DIR}/translation_kalman.c
    PARENT_SCOPE)

set (HEADERS ${HEADERS}
    ${CMAKE_CURRENT_SOURCE_DIR}/calibration.h
    ${CMAKE_CURRENT_SOURCE_DIR}/logging.h
    ${CMAKE_CURRENT_SOURCE_DIR}/mag_rotation_estimator.h
    ${CMAKE_CURRENT_SOURCE_DIR}/mission.h
    ${CMAKE_CURRENT_SOURCE_DIR}/orientation_kalman.h
    ${CMAKE_CURRENT_SOURCE_DIR}/quest_estimator.h
    ${CMAKE_CURRENT_SOURCE_DIR}/state_estimate.h
    ${CMAKE_CURRENT_SOURCE_DIR}/telemetry.h
    ${CMAKE_CURRENT_SOURCE_DIR}/translation_kalman.h
    ${CMAKE_CURRENT_SOURCE_DIR}/time_utils.h
    ${CMAKE_CURRENT_SOURCE_DIR}/math_utils.h
    PARENT_SCOPE)
