project (Avionics.GUI)

add_definitions(-DGLEW_STATIC)
set(CMAKE_INSTALL_PREFIX ${CMAKE_CURRENT_BINARY_DIR}/install)
if(POLICY CMP0056)
  cmake_policy(SET CMP0056 NEW)
endif()

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

cmake_minimum_required (VERSION 3.2)

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

add_library(Avionics.GUI ${HEADERS} ${SOURCES} ${CMAKELISTS})

target_include_directories ( Avionics.GUI PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/Source
    ${AVIONICS_INCLUDES}
    ${FRONTIER_ENGINE_INCLUDES}
)

ADD_EXECUTABLE(Avionics.GUI.Launcher Source/main.cpp)

TARGET_LINK_LIBRARIES(Avionics.GUI.Launcher 
    Avionics
    Avionics.GUI
    ${FRONTIER_ENGINE_LIBS}
)


add_custom_command(TARGET Avionics.GUI.Launcher POST_BUILD
                   COMMAND ${CMAKE_COMMAND} -E copy_directory
                       ${CMAKE_CURRENT_SOURCE_DIR}/Resources ${CMAKE_CURRENT_BINARY_DIR}/Resources)
			
SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)			
set_property(GLOBAL PROPERTY USE_FOLDERS ON)
set(MYPRJOUTDIR ${PROJECT_BINARY_DIR}/bin/${CMAKE_BUILD_TYPE})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${MYPRJOUTDIR})

#add_custom_command(TARGET Avionics.GUI.Launcher POST_BUILD 
#                    COMMAND ${CMAKE_COMMAND} -E  create_symlink
#                        ${CMAKE_CURRENT_SOURCE_DIR}/Resources ${CMAKE_CURRENT_BINARY_DIR}/Resources)