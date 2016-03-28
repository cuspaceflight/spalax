cmake_minimum_required (VERSION 3.2)

if ("${PROJECT_BINARY_DIR}" STREQUAL "${PROJECT_SOURCE_DIR}")
  message(FATAL_ERROR
    "In-source builds are not permitted!  Make a separate folder for"
    " building, e.g.,"
    "mkdir build; cd build; cmake .."
    "Before that, remove the files created by this failed run with"
    "rm -rf CMakeCache.txt CMakeFiles")
endif ()

add_subdirectory(include)
add_subdirectory(src)

if (NOT DEFINED AVIONICS_OS)
	message(FATAL_ERROR "AVIONICS_OS not defined")
elseif (${AVIONICS_OS} STREQUAL win32)
    add_subdirectory(os/win32)
elseif (${AVIONICS_OS} STREQUAL chibios)
    add_subdirectory(os/chibios)
else (NOT DEFINED AVIONICS_OS)
    message(FATAL_ERROR "Unrecognised AVIONICS_OS")
endif (NOT DEFINED AVIONICS_OS)

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

add_library(Avionics ${HEADERS} ${SOURCES} ${CMAKELISTS})

set(CMAKE_C_FLAGS ${ADDITIONAL_C_FLAGS})

set_property(TARGET Avionics PROPERTY C_STANDARD 11)

SET (AVIONICS_INCLUDES
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/include/config
)

get_directory_property(hasParent PARENT_DIRECTORY)
if (hasParent)
	SET(AVIONICS_INCLUDES ${AVIONICS_INCLUDES} PARENT_SCOPE)
endif()

target_include_directories ( Avionics PUBLIC
    ${AVIONICS_INCLUDES}
	${OS_INCLUDES}
)
