add_definitions(-DEIGEN_NO_MALLOC)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../external/eigen
        ${CMAKE_CURRENT_SOURCE_DIR}/../../external/matplotlibcpp)

link_libraries(spalax.shared python2.7)

add_executable(testmatplotlibcpp testmatplotlibcpp.cpp)

add_executable(altitude altitude.cpp)

add_executable(heading heading.cpp)