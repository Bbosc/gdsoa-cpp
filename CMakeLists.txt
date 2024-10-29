cmake_minimum_required(VERSION 3.22)
project(GDSOA)

set(CMAKE_CXX_STANDARD 14) # at least 14 is needed for Eigen
set(CMAKE_CXX_STANDARD_REQUIRED True)

find_package(pinocchio REQUIRED)

set(EigenPath "/usr/local/include/eigen3/")
set(pinocchioPath "/opt/openrobots/include/")

include_directories(include)
include_directories(${EigenPath})
include_directories(${pinocchioPath})

add_executable(main src/link.cpp src/robot.cpp src/distribution.cpp src/main.cpp)
target_link_libraries(main PUBLIC pinocchio::pinocchio)

