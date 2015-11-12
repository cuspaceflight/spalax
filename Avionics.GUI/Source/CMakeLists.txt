add_subdirectory(Data)
add_subdirectory(ThirdParty)

set (SOURCES ${SOURCES}
    ${CMAKE_CURRENT_SOURCE_DIR}/MainScene.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/RocketPathRenderer.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/RocketRenderer.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/State3DRenderer.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/StateDetailView.cpp
    PARENT_SCOPE)

set (HEADERS ${HEADERS}
    ${CMAKE_CURRENT_SOURCE_DIR}/MainScene.h
    ${CMAKE_CURRENT_SOURCE_DIR}/RocketPathRenderer.h
    ${CMAKE_CURRENT_SOURCE_DIR}/RocketRenderer.h
    ${CMAKE_CURRENT_SOURCE_DIR}/State3DRenderer.h
    ${CMAKE_CURRENT_SOURCE_DIR}/StateDetailView.h
    PARENT_SCOPE)
