cmake_minimum_required(VERSION 3.19)
project(reconstruction)

set(CMAKE_CXX_STANDARD 11)

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITION})



add_executable(reconstruction main.cpp)
target_link_libraries(reconstruction ${PCL_LIBRARIES})