cmake_minimum_required(VERSION 3.15)
project(relimetrics)

set(CMAKE_CXX_STANDARD 14)
find_package(OpenCV REQUIRED)

add_executable(relimetrics main.cpp cameraPoseEstimation.cpp cameraPoseEstimation.h)

include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(relimetrics ${OpenCV_LIBS})