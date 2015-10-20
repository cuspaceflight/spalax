add_definitions(-DGLEW_STATIC)

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

add_library(Avionics ${HEADERS} ${SOURCES} ${CMAKELISTS})

SET (AVIONICS_INCLUDES 
    ${CMAKE_CURRENT_SOURCE_DIR}/Source
)

SET (AVIONICS_INCLUDES ${AVIONICS_INCLUDES} PARENT_SCOPE)

target_include_directories ( Avionics PUBLIC
    ${AVIONICS_INCLUDES}
    ${FRONTIER_ENGINE_INCLUDES}
)
