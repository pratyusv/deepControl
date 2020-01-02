cmake_minimum_required(VERSION 2.8.3)
project(mav_deepControl)

set(CMAKE_BUILD_TYPE Release)

find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

add_executable(mav_deepControl_v2
	src/mav_deepControl_v2.cpp)

target_link_libraries(mav_deepControl_v2 )