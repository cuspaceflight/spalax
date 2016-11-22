cmake_minimum_required (VERSION 3.2)

ADD_EXECUTABLE(spalax.extractor extractor.cpp)

TARGET_LINK_LIBRARIES(spalax.extractor spalax.shared)