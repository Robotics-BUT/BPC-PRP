# CMake

[CMake](https://cmake.org/) is a set of tools that simplifies the compilation of projects and libraries in a way that makes them independent of the operating system and compiler. It works by using a unified configuration file, `CMakeLists.txt`, to generate a Makefile for UNIX-like systems or MSVC workspaces for Windows.

A major advantage of CMake is its dependency management. Applications can define the libraries they depend on, and CMake checks if these libraries are available and, importantly, if they meet the required version. Another significant benefit is the ability to create both executable files and libraries using a single, simple configuration in the `CMakeLists.txt` file.

One way to look at CMake is as another programming language - there are variables, functions, conditions and loops.
The way in which CMake is developed is emphatizing on using targets, because of that most function will begin by `target_` and the first argument will be target.
For some of them there is equivalent without it. These function are global and set the parameters, include directories etc. for the file.


## Basic example
Made up C++ project stucture:
```
MyProject/
    include/
        movement/
            move.hpp
            turn.hpp
        talk.hpp
    src/
        movement/
            move.cpp
            turn.cpp
        talk.cpp
        main.cpp
    CMakeLists.txt
```
CMakeLists.txt:
```cmake
cmake_minimum_required(VERSION 3.15)

project(MyProject VERSION 1.0
                  DESCRIPTION "Very nice project"
                  LANGUAGES CXX)
set(CXX_STANDARD 17) # Can also set custom variables

add_executable(MyProject src/main.cpp src/talk.cpp src/movement/move.cpp src/movement/turn.cpp)
target_include_directories(MyProject include)
```

Lets take a look at the file:
```
cmake_minimum_required(VERSION 3.15)
```
First line tells cmake minimu required version for running the script.

```
project(MyProject VERSION 1.0
                  DESCRIPTION "Very nice project"
                  LANGUAGES CXX)
```
Function `project` have one mandatory argument name of the project. Other arguments are optional. `VERSION` sets multiple cmake variables like PROJECT_VERSION. Default `LANGUAGE` is `C CXX`. The function does not create target `MyProject` which we will use in other functions.

```
set(CXX_STANDARD 17)
```
This function call to set variable `CXX_STANDARD`.  This is the simplest way to set C++ standard, but it will set C++ standard globally to whole project. The others ways have little bit more flexibility.


```
add_executable(MyProject src/main.cpp src/talk.cpp src/movement/move.cpp src/movement/turn.cpp)
```
Function target - executable, first argument is the name of target, others argument are all source files in the project. Header files can by also included, but by cmake they are ignored for compilation.
```
target_include_directories(MyProject include)
```
Functions add include directories to target. Any other argument is assumed to be folder with header file.

### Build basic example project
To run the project:
```shell
cd MyProject
mkdir build     #create folder build
cd build
cmake ..        #run cmake
make            #run make - build project
./MyCoolProject #run executable
```
For debuging (eg. `gdb`) add variable `CMAKE_BUILD_TYPE` with value `Debug` - When variable is not specified value used last time is run, if its first run value used is `Release`
```
cmake -DCMAKE_BUILD_TYPE=Debug ..
```

## Including libraries
This example assumes you have installed the library and it's only used to show bare basics, many external libraries have examples how to use and include them.
```cmake
cmake_minimum_required(VERSION 3.15)
project(MyProject)

find_package( OpenCV REQUIRED ) #find library in the system path if not found cmake will fail

add_executable(MyProject main.cpp)

target_include_directories(MyProject ${OpenCV_INCLUDE_DIRS}) #adds include folders to target

target_link_libraries(MyProject ${OpenCV_LIBS}) #Link library to target
```





## Resources
[An Introduction to Modern CMake](https://cliutils.gitlab.io/modern-cmake/README.html)