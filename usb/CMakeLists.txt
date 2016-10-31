cmake_minimum_required (VERSION 3.2)


add_subdirectory(Source)

add_library(spalax.usb ${HEADERS} ${SOURCES} ${CMAKELISTS})

target_include_directories(spalax.usb PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/Source
		../external/serial/include
        )

TARGET_LINK_LIBRARIES(spalax.usb
		spalax.shared
		serial
		)


ADD_EXECUTABLE(spalax.extractor extractor.cpp)

TARGET_LINK_LIBRARIES(spalax.extractor spalax.usb)