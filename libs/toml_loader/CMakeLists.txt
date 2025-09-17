cmake_minimum_required(VERSION 3.20)
project(toml_loader CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)
FetchContent_Declare(
        tomlplusplus
        GIT_REPOSITORY https://github.com/marzer/tomlplusplus.git
        GIT_TAG v3.4.0
)
FetchContent_MakeAvailable(tomlplusplus)

add_executable(toml_example example.cpp)
target_include_directories(toml_example PRIVATE ${CMAKE_SOURCE_DIR})
target_link_libraries(toml_example PRIVATE tomlplusplus::tomlplusplus)