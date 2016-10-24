cmake_minimum_required (VERSION 3.2)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../external/serial ${CMAKE_CURRENT_BINARY_DIR}/serial)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../external/Frontier.Engine ${CMAKE_CURRENT_BINARY_DIR}/Frontier.Engine)
add_subdirectory(Source)

add_library(spalax.gui ${HEADERS} ${SOURCES} ${CMAKELISTS})

target_include_directories(spalax.gui PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/Source
        ${FRONTIER_ENGINE_INCLUDES}
		../external/serial/include
        )

TARGET_LINK_LIBRARIES(spalax.gui
		spalax.shared
		${FRONTIER_ENGINE_LIBS}
		serial
		)


ADD_EXECUTABLE(spalax.gui.launcher Source/main.cpp)

TARGET_LINK_LIBRARIES(spalax.gui.launcher spalax.gui)

add_custom_command(TARGET spalax.gui.launcher POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_CURRENT_SOURCE_DIR}/Resources ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/Resources)
		
add_subdirectory(Tests)
