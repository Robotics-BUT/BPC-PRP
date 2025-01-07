# CMake

[CMake](https://cmake.org/) is a set of tools that simplifies the compilation of projects and libraries in a way that makes them independent of the operating system and compiler. It works by using a unified configuration file, `CMakeLists.txt`, to generate a Makefile for UNIX-like systems or MSVC workspaces for Windows.

A major advantage of CMake is its dependency management. Applications can define the libraries they depend on, and CMake checks if these libraries are available and, importantly, if they meet the required version. Another significant benefit is the ability to create both executable files and libraries using a single, simple configuration in the `CMakeLists.txt` file.

Here is an example of a `CMakeLists.txt` file for an application:

```cmake
cmake_minimum_required(VERSION 3.7)
project(MyCoolRobot)

set(CMAKE_CXX_STANDARD 17) 

add_executable(MyCoolRobot main.cpp)
```

If we have a project that uses CMake and we want to run it, we can do so using the following commands:

```shell
cd MyCoolProject
mkdir build
cd build
cmake ..
make
./MyCoolProject
```
