enable_testing()

SET(SOURCES)
SET(HEADERS)

add_subdirectory(Source)

add_executable(Avionics.GUI.Tests ${SOURCES} ${HEADERS})
target_include_directories(Avionics.GUI.Tests PUBLIC
        Source)

		
FILE(GLOB_RECURSE CMAKELISTS Source/*CMakeLists.txt)

# Rebuild directory structure within Visual Studio
foreach (FILE ${SOURCES} ${HEADERS} ${CMAKELISTS})
    get_filename_component(PARENT_DIR "${FILE}" PATH)

    string(REPLACE ${CMAKE_CURRENT_SOURCE_DIR} "" GROUP "${PARENT_DIR}")
    string(REPLACE "/" "\\" GROUP "${GROUP}")

    source_group("${GROUP}" FILES "${FILE}")
endforeach ()


target_link_libraries(Avionics.GUI.Tests
        Frontier.Engine
        freetype-gl
        freetype
        glfwmock
        glewmock
        glmock
        Avionics
        Avionics.GUI
        )
