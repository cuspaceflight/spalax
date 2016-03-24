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

cmake_minimum_required (VERSION 3.2)

SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

if (${BUILD_GUI} AND ${BUILD_FIRMWARE})
    message(FATAL_ERROR "Cannot create a configuration to build both the GUI and the firmware")
endif (${BUILD_GUI} AND ${BUILD_FIRMWARE})

if (${BUILD_GUI})
    # These are needed by Avionics.Shared
	set (AVIONICS_OS win32)
	
    add_subdirectory(Frontier.Engine)
	add_subdirectory(Avionics.Shared)
    add_subdirectory(Avionics.GUI)
    
    set_target_properties(glfw PROPERTIES FOLDER Frontier.Engine//ThirdParty//glfw)
    
elseif (${BUILD_FIRMWARE})
    set (AVIONICS_OS chibios)
	set (CHIBIOS_LOCATION ${CMAKE_SOURCE_DIR}/ChibiOS-RT)
	set (CHIBIOS_CONFIG_LOCATION ${CMAKE_SOURCE_DIR}/Avionics.Firmware/config)
	set (CHIBIOS_PLATFORM_INCLUDES 
		${CHIBIOS_LOCATION}/os/ports/common/ARMCMx/CMSIS/include
		${CHIBIOS_LOCATION}/os/ports/common/ARMCMx
		${CHIBIOS_LOCATION}/os/ports/GCC/ARMCMx
		${CHIBIOS_LOCATION}/os/ports/GCC/ARMCMx/STM32F4xx
		${CHIBIOS_LOCATION}/os/hal/platforms/STM32F4xx
        ${CHIBIOS_LOCATION}/os/hal/platforms/STM32
        ${CHIBIOS_LOCATION}/os/hal/platforms/STM32/GPIOv2
        ${CHIBIOS_LOCATION}/os/hal/platforms/STM32/I2Cv1
        ${CHIBIOS_LOCATION}/os/hal/platforms/STM32/OTGv1
        ${CHIBIOS_LOCATION}/os/hal/platforms/STM32/RTCv2
        ${CHIBIOS_LOCATION}/os/hal/platforms/STM32/SPIv1
        ${CHIBIOS_LOCATION}/os/hal/platforms/STM32/TIMv1
        ${CHIBIOS_LOCATION}/os/hal/platforms/STM32/USARTv1
	)
	
	# This should be the same as the target we are linking against
	set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mcpu=cortex-m4 -fomit-frame-pointer -falign-functions=16 -std=gnu99 -ffunction-sections -fdata-sections -fno-common -mfloat-abi=hard -mfpu=fpv4-sp-d16 -fsingle-precision-constant -mno-thumb-interwork -mthumb -Wall -Wextra -Wstrict-prototypes -DCORTEX_USE_FPU=TRUE -DTHUMB_PRESENT -DTHUMB_NO_INTERWORKING -MD -MP -MF -DTHUMB")
	
	add_subdirectory(Avionics.Shared)
    add_subdirectory(Avionics.Firmware)
else (${BUILD_GUI})
    message(FATAL_ERROR "Nothing selected to build (select either BUILD_FIRMWARE or BUILD_GUI)")
endif (${BUILD_GUI})


