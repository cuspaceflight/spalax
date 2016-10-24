cmake_minimum_required (VERSION 3.2)

ENABLE_LANGUAGE(ASM)

add_subdirectory(config_${SPALAX_CONF})
add_subdirectory(src)

set(STM32_LINKER_SCRIPT ${ChibiOS_LINKER_SCRIPT})

add_executable(firmware.elf ${SOURCES}  ${ChibiOS_SOURCES})

target_include_directories (firmware.elf PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${ChibiOS_INCLUDE_DIRS}
        )

target_link_libraries(firmware.elf PUBLIC spalax.shared m)


STM32_SET_TARGET_PROPERTIES(firmware.elf)
STM32_ADD_HEX_BIN_TARGETS(firmware.elf)
