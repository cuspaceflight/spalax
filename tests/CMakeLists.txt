enable_testing()

SET(SOURCES)
SET(HEADERS)

add_subdirectory(src)

add_executable(spalax.tests ${SOURCES} ${HEADERS})

add_definitions(-DEIGEN_NO_MALLOC)


target_include_directories(spalax.tests PUBLIC
        Source

        PRIVATE
        ${FRONTIER_ENGINE_INCLUDES}
        ${CMAKE_CURRENT_SOURCE_DIR}/../external/eigen
        ${CMAKE_CURRENT_SOURCE_DIR}/../external/matplotlibcpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../external/wrappy/include)


target_link_libraries(spalax.tests
        spalax.shared
        gmock
        gtest
        wrappy
        )
