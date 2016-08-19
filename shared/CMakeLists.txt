cmake_minimum_required (VERSION 3.2)
set(CMAKE_DISABLE_SOURCE_CHANGES ON)
set(CMAKE_DISABLE_IN_SOURCE_BUILD ON)

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

if (NOT DEFINED SPALAX_OS)
	message(FATAL_ERROR "SPALAX_OS not defined")
elseif (${SPALAX_OS} STREQUAL std)
    set(MESSAGING_OS ${SPALAX_OS})
elseif (${SPALAX_OS} STREQUAL chibios)
    set(MESSAGING_OS ${SPALAX_OS})
else (NOT DEFINED SPALAX_OS)
    message(FATAL_ERROR "Unrecognised SPALAX_OS")
endif (NOT DEFINED SPALAX_OS)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../external/cusf-messaging ${CMAKE_CURRENT_BINARY_DIR}/cusf-messaging)
add_subdirectory(include)
add_subdirectory(src)


# We can use GLOB here as in order for a CMakeList to be used a pre-existing one
# must have been modified and therefore the build system will know to rebuild
FILE(GLOB_RECURSE CMAKELISTS1 src/*CMakeLists.txt)
FILE(GLOB_RECURSE CMAKELISTS2 include/*CMakeLists.txt)
set(CMAKELISTS ${CMAKELISTS1} ${CMAKELISTS2})

# Rebuild directory structure within Visual Studio
foreach(FILE ${SOURCES} ${HEADERS} ${CMAKELISTS})
  get_filename_component(PARENT_DIR "${FILE}" PATH)

  string(REPLACE ${CMAKE_CURRENT_SOURCE_DIR} "" GROUP "${PARENT_DIR}")
  string(REPLACE "/" "\\" GROUP "${GROUP}")

  source_group("${GROUP}" FILES "${FILE}")
endforeach()

add_library(spalax.shared ${HEADERS} ${SOURCES} ${CMAKELISTS})
set(CMAKE_C_FLAGS ${ADDITIONAL_C_FLAGS})
set_property(TARGET spalax.shared PROPERTY C_STANDARD 11)

SET (SPALAX_SHARED_INCLUDES
    ${CMAKE_CURRENT_SOURCE_DIR}/include
	${MESSAGING_INCLUDES}
)

get_directory_property(hasParent PARENT_DIRECTORY)
if (hasParent)
	SET(SPALAX_SHARED_INCLUDES ${SPALAX_SHARED_INCLUDES} PARENT_SCOPE)
endif()

target_include_directories ( spalax.shared PUBLIC
    ${SPALAX_SHARED_INCLUDES}
	${OS_INCLUDES}
)

target_link_libraries( spalax.shared PUBLIC messaging)
