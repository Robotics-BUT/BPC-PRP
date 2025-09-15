# CMake

CMake is a cross-platform build system generator. You describe your project in plain text files named `CMakeLists.txt`, and CMake generates native build files for your platform, such as Makefiles (Unix), Ninja files, or Visual Studio solutions (Windows).

Key ideas:
- Target-based: Modern CMake focuses on targets. You create targets (executables or libraries) and attach properties to them (include directories, compile features, linked libraries, compile definitions, etc.). Most modern commands start with `target_...` and take the target name as the first argument.
- Dependency management: `find_package()` locates external dependencies (optionally checking versions/components), and you link them to your targets.


## Basic example
A small C++ project structure:
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
Minimal `CMakeLists.txt`:
```cmake
cmake_minimum_required(VERSION 3.15)

project(MyProject VERSION 1.0
        DESCRIPTION "Very nice project"
        LANGUAGES CXX)

# Prefer target-specific settings in modern CMake. If you want a global default:
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(MyProject
  src/main.cpp
  src/talk.cpp
  src/movement/move.cpp
  src/movement/turn.cpp
)

# Tell the target where to find headers under include/
# Use PUBLIC to propagate include paths to dependents (if this were a library).
target_include_directories(MyProject PUBLIC ${CMAKE_SOURCE_DIR}/include)
```

Explanation:
- `cmake_minimum_required(VERSION 3.15)` sets the minimum CMake version that can process this project.
- `project(...)` defines project metadata. It does not implicitly create a target; you still need `add_executable()` or `add_library()`.
- C++ standard: Using `set(CMAKE_CXX_STANDARD 17)` defines a project-wide default. Alternatively, you can set features per-target with `target_compile_features(MyProject PUBLIC cxx_std_17)`.
- `add_executable(MyProject ...)` declares the build target and lists its source files. Header files need not be listed; they are discovered by the compiler via include paths.
- `target_include_directories(MyProject PUBLIC include)` associates include paths with the target. Visibility keywords: PRIVATE (only this target), PUBLIC (this target and its dependents), INTERFACE (only dependents).

### Build the example
Common out-of-source build workflow:
```bash
# from the project root (MyProject/)
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release   # configure
cmake --build build --config Release             # build
./build/MyProject                                # run the executable (Unix-like)
```
For debugging builds (e.g., with gdb):
```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build --config Debug
```
Note: On multi-config generators (e.g., Visual Studio), specify `--config Debug` when building and running.


## Including libraries
As a simple example with OpenCV. This assumes OpenCV is installed and discoverable by CMake.
```cmake
cmake_minimum_required(VERSION 3.15)
project(MyProject LANGUAGES CXX)

# Optionally request specific components and/or versions
find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui)

add_executable(MyProject main.cpp)

# Link libraries target-based; include paths typically come via the imported target
# but if you rely on variables, you can still use them.
target_link_libraries(MyProject PRIVATE ${OpenCV_LIBS})
# If needed (older packages), add include dirs explicitly
if(OpenCV_INCLUDE_DIRS)
  target_include_directories(MyProject PRIVATE ${OpenCV_INCLUDE_DIRS})
endif()
```
Prefer packages that provide imported CMake targets (e.g., `OpenCV::opencv_core`, etc.) and link to those instead of raw variables when available:
```cmake
# Example when imported targets are available
# target_link_libraries(MyProject PRIVATE opencv_core opencv_imgproc opencv_highgui)
```


## Common target commands (cheat sheet)
- `add_executable(name sources...)` / `add_library(name [STATIC|SHARED|INTERFACE] sources...)`
- `target_link_libraries(tgt PRIVATE|PUBLIC|INTERFACE libs...)`
- `target_include_directories(tgt PRIVATE|PUBLIC|INTERFACE dirs...)`
- `target_compile_definitions(tgt PRIVATE|PUBLIC|INTERFACE MACRO=VALUE ...)`
- `target_compile_features(tgt PRIVATE|PUBLIC cxx_std_17 ...)`


## Resources
- Modern CMake guide: https://cliutils.gitlab.io/modern-cmake/README.html
- CMake official documentation: https://cmake.org/cmake/help/latest/