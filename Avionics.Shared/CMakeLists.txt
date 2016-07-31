cmake_minimum_required (VERSION 3.2)

if ("${PROJECT_BINARY_DIR}" STREQUAL "${PROJECT_SOURCE_DIR}")
  message(FATAL_ERROR
    "In-source builds are not permitted!  Make a separate folder for"
    " building, e.g.,"
    "mkdir build; cd build; cmake .."
    "Before that, remove the files created by this failed run with"
    "rm -rf CMakeCache.txt CMakeFiles")
endif ()

if (NOT DEFINED AVIONICS_OS)
	message(FATAL_ERROR "AVIONICS_OS not defined")
elseif (${AVIONICS_OS} STREQUAL win32)
    set(MESSAGING_OS ${AVIONICS_OS})
elseif (${AVIONICS_OS} STREQUAL chibios)
    set(MESSAGING_OS ${AVIONICS_OS})
else (NOT DEFINED AVIONICS_OS)
    message(FATAL_ERROR "Unrecognised AVIONICS_OS")
endif (NOT DEFINED AVIONICS_OS)

add_subdirectory(messaging)



add_subdirectory(include)
add_subdirectory(src)


# We can use GLOB here as in order for a CMakeList to be used a pre-existing one
# must have been modified and therefore the build system will know to rebuild
FILE(GLOB_RECURSE CMAKELISTS Source/*CMakeLists.txt)

add_library(Avionics ${HEADERS} ${SOURCES} ${CMAKELISTS})
set_property(TARGET Avionics PROPERTY C_STANDARD 11)

SET (AVIONICS_INCLUDES
    ${CMAKE_CURRENT_SOURCE_DIR}/include
	${MESSAGING_INCLUDES}
)

get_directory_property(hasParent PARENT_DIRECTORY)
if (hasParent)
	SET(AVIONICS_INCLUDES ${AVIONICS_INCLUDES} PARENT_SCOPE)
endif()

target_include_directories ( Avionics PUBLIC
    ${AVIONICS_INCLUDES}
	${OS_INCLUDES}
)

target_link_libraries( Avionics PUBLIC messaging)
