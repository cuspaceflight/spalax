enable_testing()

SET(SOURCES)
SET(HEADERS)

add_subdirectory(src)

add_executable(spalax.tests ${SOURCES} ${HEADERS})
target_include_directories(spalax.tests PUBLIC
        Source)


target_link_libraries(spalax.tests
        Frontier.Engine
        freetype-gl
        freetype
        glfwmock
        spalax.shared
        spalax.gui
        )
