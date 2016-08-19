cmake_minimum_required (VERSION 3.2)
set(CMAKE_DISABLE_SOURCE_CHANGES ON)
set(CMAKE_DISABLE_IN_SOURCE_BUILD ON)

project(spalax)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

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

set(SPALAX_OS std)

# This will create project folders in visual studio for us
get_filename_component(ROOT ${CMAKE_CURRENT_SOURCE_DIR} DIRECTORY)
set(FRONTIER_ENGINE_FOLDER_MACRO_DIR ${ROOT} CACHE STRING "Frontier Engine Folder Macro Directory")

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../external/serial ${CMAKE_CURRENT_BINARY_DIR}/serial)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../external/Frontier.Engine ${CMAKE_CURRENT_BINARY_DIR}/Frontier.Engine)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../shared ${CMAKE_CURRENT_BINARY_DIR}/shared)
add_subdirectory(Source)

# We can use GLOB here as in order for a CMakeList to be used a pre-existing one
# must have been modified and therefore the build system will know to rebuild
FILE(GLOB_RECURSE CMAKELISTS Source/*CMakeLists.txt)

# Rebuild directory structure within Visual Studio
foreach(FILE ${SOURCES} ${HEADERS} ${CMAKELISTS})
  get_filename_component(PARENT_DIR "${FILE}" PATH)

  string(REPLACE ${CMAKE_CURRENT_SOURCE_DIR} "" GROUP "${PARENT_DIR}")
  string(REPLACE "/" "\\" GROUP "${GROUP}")

  source_group("${GROUP}" FILES "${FILE}")
endforeach()

add_library(spalax.gui ${HEADERS} ${SOURCES} ${CMAKELISTS})

target_include_directories(spalax.gui PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/Source
        ${SPALAX_SHARED_INCLUDES}
        ${FRONTIER_ENGINE_INCLUDES}
		../external/serial/include
        )

ADD_EXECUTABLE(spalax.gui.launcher Source/main.cpp)

TARGET_LINK_LIBRARIES(spalax.gui.launcher
        spalax.gui
        spalax.shared
        ${FRONTIER_ENGINE_LIBS}
		serial
        )

add_custom_command(TARGET spalax.gui.launcher POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_CURRENT_SOURCE_DIR}/Resources ${CMAKE_CURRENT_BINARY_DIR}/Resources)

# MSVC by default runs programs with project's directory as the working directory
# So we need to copy resources there also
if (MSVC)
    add_custom_command(TARGET spalax.gui.launcher POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_directory
            ${CMAKE_CURRENT_SOURCE_DIR}/Resources ${CMAKE_CURRENT_BINARY_DIR}/Resources)
endif ()

add_custom_command(TARGET spalax.gui.launcher POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_CURRENT_SOURCE_DIR}/Resources ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/Resources)
		
add_subdirectory(Tests)