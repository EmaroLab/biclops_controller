PROJECT(libUtils)

cmake_minimum_required(VERSION 2.8)

#set(CMAKE_BUILD_TYPE Debug)
#set(CMAKE_BUILD_TYPE Release)
set(CMAKE_BUILD_TYPE RelWithDebInfo)

set(LIBRARY_OUTPUT_PATH ${libUtils_SOURCE_DIR}/lib)

include_directories(include ${biclops_controller_SOURCE_DIR}/libPMD/include)

file(GLOB utils_SOURCES RELATIVE ${libUtils_SOURCE_DIR} src/*.cpp)

ADD_DEFINITIONS(-Wall)


ADD_LIBRARY(Utils ${utils_SOURCES} src/Biclops.cpp)
