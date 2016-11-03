if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
    # Enable warnings
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-long-long -Werror")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wno-long-long -Werror")
endif()

add_subdirectory(src)

add_definitions(-DEIGEN_NO_MALLOC)

add_library(spalax.shared ${SOURCES})

target_include_directories (spalax.shared
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/src

        PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../external/eigen
)

target_link_libraries(spalax.shared PUBLIC messaging)
