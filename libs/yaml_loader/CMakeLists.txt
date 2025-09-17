cmake_minimum_required(VERSION 3.20)
project(yaml_loader CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(yaml_loader_example main.cpp)
target_include_directories(yaml_loader_example PRIVATE ${CMAKE_SOURCE_DIR})
target_link_libraries(yaml_loader_example PUBLIC yaml-cpp)