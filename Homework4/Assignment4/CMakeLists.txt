cmake_minimum_required(VERSION 3.10)
project(BezierCurve)

set(CMAKE_CXX_STANDARD 14)

include_directories(${CMAKE_CURRENT_LIST_DIR}/../../Dependencies/Include)
link_directories(${CMAKE_CURRENT_LIST_DIR}/../../Dependencies/Libs)

add_executable(BezierCurve main.cpp)
target_link_libraries(BezierCurve opencv_world490d.lib)