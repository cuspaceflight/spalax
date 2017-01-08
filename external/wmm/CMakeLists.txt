project(wmm)
cmake_minimum_required(VERSION 3.2)

option(WMM_BUILD_EXEC "Whether to build the WMM executables" ON)

add_library(wmm STATIC src/GeomagnetismLibrary.c src/EGM9615.c)

# Link math library
target_link_libraries(wmm m)

target_include_directories (wmm PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)



if (${WMM_BUILD_EXEC})
    link_libraries(wmm)

    add_executable(wmm_file src/wmm_file.c)

    add_executable(wmm_grid src/wmm_grid.c)

    add_executable(wmm_point src/wmm_point.c)

    add_custom_command(TARGET wmm POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
            ${CMAKE_CURRENT_SOURCE_DIR}/bin/WMM.COF ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
endif()

