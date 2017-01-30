if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
    # Enable warnings
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-long-long -Werror")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wno-long-long -Werror")
endif()

ENABLE_LANGUAGE(ASM)

add_subdirectory(config_${SPALAX_CONF})
add_subdirectory(src)

set(STM32_LINKER_SCRIPT ${ChibiOS_LINKER_SCRIPT})

add_executable(${SPALAX_CONF}.elf ${SOURCES}  ${ChibiOS_SOURCES})

target_include_directories (${SPALAX_CONF}.elf PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${ChibiOS_INCLUDE_DIRS}
        )

target_link_libraries(${SPALAX_CONF}.elf PUBLIC spalax.shared m)


STM32_SET_TARGET_PROPERTIES(${SPALAX_CONF}.elf)
STM32_ADD_HEX_BIN_TARGETS(${SPALAX_CONF}.elf)


add_custom_command(TARGET ${SPALAX_CONF}.elf POST_BUILD COMMAND ${CMAKE_SIZE} ${CMAKE_BINARY_DIR}/bin/${SPALAX_CONF}.elf)