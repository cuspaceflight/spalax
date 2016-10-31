cmake_minimum_required (VERSION 3.2)

add_subdirectory(Source)

add_library(spalax.gui ${HEADERS} ${SOURCES} ${CMAKELISTS})

target_include_directories(spalax.gui PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/Source
        ${FRONTIER_ENGINE_INCLUDES}
        )

TARGET_LINK_LIBRARIES(spalax.gui
		spalax.usb
		${FRONTIER_ENGINE_LIBS}
		)


ADD_EXECUTABLE(spalax.gui.launcher Source/main.cpp)

TARGET_LINK_LIBRARIES(spalax.gui.launcher spalax.gui)

add_custom_command(TARGET spalax.gui.launcher POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_CURRENT_SOURCE_DIR}/Resources ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/Resources)
		
add_subdirectory(Tests)
