
set(SPALAX_CONF "spalax")
set(SPALAX_BUILD_FIRMWARE True)
set(SPALAX_BUILD_GUI False)

set(SPALAX_OS chibios)
set(MESSAGING_OS chibios)

get_filename_component(CURRENT_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)
SET(CMAKE_MODULE_PATH ${CURRENT_DIR}/../external/stm32-cmake/cmake) 

set(STM32_CHIP "STM32F407VG")
include(gcc_stm32)

ADD_DEFINITIONS(-DCORTEX_USE_FPU=TRUE)
