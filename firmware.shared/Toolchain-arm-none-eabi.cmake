include(CMakeForceCompiler)

# We are not targeting an OS that CMake knows about
set(CMAKE_SYSTEM_NAME Generic)

set(ARM_TOOLCHAIN_BIN_DIR "" CACHE PATH "ARM Toolchain bin directory (optional, will use PATH if not set)")

if (WIN32)
    set(EXECUTABLE_SUFFIX ".exe")
else (WIN32)
    set(EXECUTABLE_SUFFIX "")
endif (WIN32)

# specify the cross compiler
if ("" STREQUAL "${ARM_TOOLCHAIN_BIN_DIR}")
    CMAKE_FORCE_C_COMPILER("arm-none-eabi-gcc${EXECUTABLE_SUFFIX}" GNU)
    CMAKE_FORCE_CXX_COMPILER("arm-none-eabi-g++${EXECUTABLE_SUFFIX}" GNU)
	set(CMAKE_AR "arm-none-eabi-gcc-ar${EXECUTABLE_SUFFIX}" CACHE FILEPATH "GCC Archiver")
	set(CMAKE_RANLIB "arm-none-eabi-gcc-ranlib${EXECUTABLE_SUFFIX}" CACHE FILEPATH "GCC Ranlib")
else ("" STREQUAL "${ARM_TOOLCHAIN_BIN_DIR}")
    CMAKE_FORCE_C_COMPILER("${ARM_TOOLCHAIN_BIN_DIR}/arm-none-eabi-gcc${EXECUTABLE_SUFFIX}" GNU)
    CMAKE_FORCE_CXX_COMPILER("${ARM_TOOLCHAIN_BIN_DIR}/arm-none-eabi-g++${EXECUTABLE_SUFFIX}" GNU)
	set(CMAKE_AR "${ARM_TOOLCHAIN_BIN_DIR}/arm-none-eabi-gcc-ar${EXECUTABLE_SUFFIX}" CACHE FILEPATH "GCC Archiver")
	set(CMAKE_RANLIB "${ARM_TOOLCHAIN_BIN_DIR}/arm-none-eabi-gcc-ranlib${EXECUTABLE_SUFFIX}" CACHE FILEPATH "GCC Ranlib")
endif ("" STREQUAL "${ARM_TOOLCHAIN_BIN_DIR}")


# We are not targeting a system with an environment
set(CMAKE_FIND_ROOT_PATH  / )
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE NEVER)