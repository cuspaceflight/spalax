add_definitions(-DEIGEN_NO_MALLOC)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../external/eigen
        ${CMAKE_CURRENT_SOURCE_DIR}/../../external/matplotlibcpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../../external/wrappy/include)

link_libraries(spalax.shared wrappy)

add_executable(testmatplotlibcpp testmatplotlibcpp.cpp)

add_executable(altitude altitude.cpp)

add_executable(heading heading.cpp)

add_executable(mapplotter mapplotter.cpp)

add_executable(gyro gyro.cpp)

add_executable(accel accel.cpp)