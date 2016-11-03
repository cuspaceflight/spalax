enable_testing()

SET(SOURCES)
SET(HEADERS)

add_subdirectory(src)

add_executable(spalax.tests ${SOURCES} ${HEADERS})

add_definitions(-DEIGEN_NO_MALLOC)


target_include_directories(spalax.tests PUBLIC
        Source

        PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../external/eigen)


target_link_libraries(spalax.tests
        Frontier.Engine
        freetype-gl
        freetype
        glfwmock
        spalax.shared
        spalax.gui
        )
