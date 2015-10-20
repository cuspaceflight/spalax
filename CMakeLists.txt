SET(CMAKE_INSTALL_PREFIX ${CMAKE_CURRENT_BINARY_DIR}/install)
project (Avionics)

set_property(GLOBAL PROPERTY USE_FOLDERS ON)
set(MYPRJOUTDIR ${PROJECT_BINARY_DIR}/bin/${CMAKE_BUILD_TYPE})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${MYPRJOUTDIR})

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

add_subdirectory(Avionics)
add_subdirectory(Frontier.Engine)
add_subdirectory(Avionics.GUI)

set_target_properties(glfw PROPERTIES FOLDER Frontier.Engine//ThirdParty//glfw)
