

set (SOURCES ${SOURCES}
        ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/TestQuest.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/TestQMethod.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/TestEigen.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/TestWMM.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/TestMultipacket.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/TestKalmanStability.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/TestKalmanStatic.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/TestKalmanDynamic.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/TestStateMathUtils.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/math_debug_util.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/KalmanTestUtils.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/TestKalmanAccel.cpp
    PARENT_SCOPE)

set (HEADERS ${HEADERS}
    PARENT_SCOPE)
