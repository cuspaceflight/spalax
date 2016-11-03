
set(SPALAX_CONF "spalax")
set(SPALAX_BUILD_FIRMWARE True)
set(SPALAX_BUILD_GUI False)

set(SPALAX_OS chibios)
set(MESSAGING_OS chibios)

get_filename_component(CURRENT_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)
SET(CMAKE_MODULE_PATH ${CURRENT_DIR}/../external/stm32-cmake/cmake) 

set(STM32_CHIP "STM32F407VG")
include(gcc_stm32)

# This is set to arm by gcc_stm32 but libswiftnav expects it to be set to cortex-m4
set(CMAKE_SYSTEM_PROCESSOR cortex-m4)

set(BUILD_SHARED_LIBS OFF)

ADD_DEFINITIONS(-DCORTEX_USE_FPU=TRUE)
