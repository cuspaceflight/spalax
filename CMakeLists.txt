set(CMAKE_INSTALL_PREFIX ${CMAKE_CURRENT_BINARY_DIR}/install)
if(POLICY CMP0056)
  cmake_policy(SET CMP0056 NEW)
endif()

project (Avionics)

if ("${PROJECT_BINARY_DIR}" STREQUAL "${PROJECT_SOURCE_DIR}")
  message(FATAL_ERROR
    "In-source builds are not permitted!  Make a separate folder for"
    " building, e.g.,"
    "mkdir build; cd build; cmake .."
    "Before that, remove the files created by this failed run with"
    "rm -rf CMakeCache.txt CMakeFiles")
endif ()

set_property(GLOBAL PROPERTY USE_FOLDERS ON)
set(MYPRJOUTDIR ${PROJECT_BINARY_DIR}/bin/${CMAKE_BUILD_TYPE})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${MYPRJOUTDIR})

option(BUILD_GUI "Build the Avionics GUI" OFF)
option(BUILD_FIRMWARE "Build the Avionics Firmware" OFF)

cmake_minimum_required (VERSION 3.3)

SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# Rebind these to group up all the ThirdParty Projects so they don't clutter up the IDE
macro (add_library name)
    set (IS_INTERFACE 0)
    FOREACH (e ${ARGN})
        if (e STREQUAL "INTERFACE" OR e STREQUAL "IMPORTED")
            set (IS_INTERFACE 1)
        endif (e STREQUAL "INTERFACE" OR e STREQUAL "IMPORTED")
    ENDFOREACH(e)


    if (${IS_INTERFACE})
        _add_library(${name} ${ARGN})
    else (${IS_INTERFACE})
        string(REPLACE ${CMAKE_SOURCE_DIR}/ "" FOLDER "${CMAKE_CURRENT_SOURCE_DIR}")
        string(REPLACE "/" "//" FOLDER "${FOLDER}")

        _add_library (${name} ${ARGN})
        set_target_properties(${name} PROPERTIES FOLDER ${FOLDER})
    endif (${IS_INTERFACE})
endmacro (add_library)

macro (add_executable name)
    set (IS_INTERFACE 0)
    FOREACH (e ${ARGN})
        if (e STREQUAL "INTERFACE")
            set (IS_INTERFACE 1)
        endif (e STREQUAL "INTERFACE")
    ENDFOREACH(e)



    if (${IS_INTERFACE})
        _add_executable(${name} ${ARGN})
    else (${IS_INTERFACE})
        string(REPLACE ${CMAKE_SOURCE_DIR}/ "" FOLDER "${CMAKE_CURRENT_SOURCE_DIR}")
        string(REPLACE "/" "//" FOLDER "${FOLDER}")

        _add_executable (${name} ${ARGN})
        set_target_properties(${name} PROPERTIES FOLDER ${FOLDER})

    endif (${IS_INTERFACE})
endmacro (add_executable)

if (${BUILD_GUI} AND ${BUILD_FIRMWARE})
    message(FATAL_ERROR "Cannot create a configuration to build both the GUI and the firmware")
endif (${BUILD_GUI} AND ${BUILD_FIRMWARE})

if (${BUILD_GUI})
    add_subdirectory(Avionics.Shared)
    add_subdirectory(Frontier.Engine)
    add_subdirectory(Avionics.GUI)
    
    set_target_properties(glfw PROPERTIES FOLDER Frontier.Engine//ThirdParty//glfw)
elseif (${BUILD_FIRMWARE})
    # For some reason 'set (CMAKE_C_STANDARD 99)' doesn't seem to work
    set (CMAKE_C_FLAGS -std=gnu99)
    add_subdirectory(Avionics.Shared)
    add_subdirectory(Avionics.Firmware)
else (${BUILD_GUI})
    message(FATAL_ERROR "Nothing selected to build (select either BUILD_FIRMWARE or BUILD_GUI)")
endif (${BUILD_GUI})


