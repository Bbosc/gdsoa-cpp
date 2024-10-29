cmake_minimum_required(VERSION 3.22)
project(GDSOA)

set(CMAKE_CXX_STANDARD 14) # at least 14 is needed for Eigen
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(EigenPath "/usr/local/include/eigen3/")

include_directories(include)
include_directories(${EigenPath})

add_executable(main src/Link.cpp src/Robot.cpp src/main.cpp)

