include (ExternalProject)

set(MAKE_COMMAND "make" CACHE STRING "The name of the make utility to use")

ExternalProject_Add(Avionics.Firmware
  SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}
  PREFIX ${CMAKE_CURRENT_SOURCE_DIR}
  BUILD_IN_SOURCE TRUE
  CONFIGURE_COMMAND ""
  BUILD_COMMAND ${MAKE_COMMAND} AVIONICS_INCLUDES="${AVIONICS_INCLUDES}" AVIONICS_LIB_DIR="$<TARGET_FILE_DIR:Avionics>"
  INSTALL_COMMAND ""
)

# We force CMake to always invoke the Avionics.Firmware makefile
# by inserting a step after the configure step which always runs
# and therefore forces the dependent build step to always be run
ExternalProject_Add_Step(Avionics.Firmware forcebuild
    COMMAND ""
    DEPENDEES configure
    DEPENDERS build
    ALWAYS 1)

add_dependencies(Avionics.Firmware Avionics)
