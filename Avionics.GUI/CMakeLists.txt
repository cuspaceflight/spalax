project(Avionics.GUI)
cmake_minimum_required(VERSION 3.2)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if (COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif (COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
elseif (MSVC)
    message(STATUS "Warning: MSVC's C++11 support is incomplete but should be sufficient")
else ()
    message(FATAL "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif ()

add_definitions(-DGLEW_STATIC)
set(CMAKE_INSTALL_PREFIX ${CMAKE_CURRENT_BINARY_DIR}/install)
if (POLICY CMP0056)
    cmake_policy(SET CMP0056 NEW)
endif ()

# Visual Studio misbehaves unless you set all of these
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/bin)

set(AVIONICS_OS win32)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../Frontier.Engine ${CMAKE_CURRENT_BINARY_DIR}/Frontier.Engine)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../Avionics.Shared ${CMAKE_CURRENT_BINARY_DIR}/Avionics.Shared)
add_subdirectory(Source)

if ("${PROJECT_BINARY_DIR}" STREQUAL "${PROJECT_SOURCE_DIR}")
    message(FATAL_ERROR
            "In-source builds are not permitted!  Make a separate folder for"
            " building, e.g.,"
            "mkdir build; cd build; cmake .."
            "Before that, remove the files created by this failed run with"
            "rm -rf CMakeCache.txt CMakeFiles")
endif ()

add_library(Avionics.GUI ${HEADERS} ${SOURCES} ${CMAKELISTS})

target_include_directories(Avionics.GUI PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/Source
        ${AVIONICS_INCLUDES}
        ${FRONTIER_ENGINE_INCLUDES}
        )

ADD_EXECUTABLE(Avionics.GUI.Launcher Source/main.cpp)

TARGET_LINK_LIBRARIES(Avionics.GUI.Launcher
        Avionics.GUI
        Avionics
        ${FRONTIER_ENGINE_LIBS}
        )

add_custom_command(TARGET Avionics.GUI.Launcher POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_CURRENT_SOURCE_DIR}/Resources ${CMAKE_CURRENT_BINARY_DIR}/Resources)

# MSVC by default runs programs with project's directory as the working directory
# So we need to copy resources there also
if (MSVC)
    add_custom_command(TARGET Avionics.GUI.Launcher POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_directory
            ${CMAKE_CURRENT_SOURCE_DIR}/Resources ${CMAKE_CURRENT_BINARY_DIR}/Resources)
endif ()

add_custom_command(TARGET Avionics.GUI.Launcher POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_CURRENT_SOURCE_DIR}/Resources ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/Resources)