cmake_minimum_required (VERSION 3.2)
set(CMAKE_DISABLE_SOURCE_CHANGES ON)
set(CMAKE_DISABLE_IN_SOURCE_BUILD ON)

add_subdirectory(src)

add_library(spalax.shared ${SOURCES})

target_include_directories (spalax.shared PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(spalax.shared PUBLIC messaging)
