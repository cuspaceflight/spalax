if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
    # Enable warnings
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-long-long")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wno-long-long -Werror")
endif()

add_subdirectory(src)

string(TIMESTAMP Year "%Y")
string(TIMESTAMP Week "%U")
string(TIMESTAMP DayOfWeek "%w")

message(STATUS "Compilation Date: Year - ${Year}, Week - ${Week}, DayOfWeek - ${DayOfWeek}")

add_definitions(-DEIGEN_NO_MALLOC -DTIMESTAMP_YEAR=${Year} -DTIMESTAMP_WEEK=${Week} -DTIMESTAMP_DAY_OF_WEEK=${DayOfWeek})

add_library(spalax.shared ${SOURCES})

target_include_directories (spalax.shared
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/src

        PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../external/eigen
)

target_link_libraries(spalax.shared 
    PUBLIC 
    messaging
    PRIVATE 
    wmm)
