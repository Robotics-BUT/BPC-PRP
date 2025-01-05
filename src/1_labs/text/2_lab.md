# Lab 2 - C++, CMake & IDE

If you are not familiar with Linux CLI commands, please follow the [Linux](../../3_others/text/1_linux.md) chapter.

## CLI Compilation (45 min)

This exercise will show you how to write and compile the most basic c++ project in Linux OS.

In your home directory create a project folder and enter it.

Write a simple program into the `main.cpp` file.

```c++
#include <iostream>

#define A 5

int sum(int a, int b) {
    return a + b;
}

int main() {
    std::cout << "My Cool CLI Compiled Program" << std::endl;
    int b = 10;
    std::cout << "Sum result: " << sum(A, b) << std::endl;
    return 0;
}
```

Save file and compile it using the `gcc` call ([GCC](https://gcc.gnu.org/) a C/C++ compiler). 
```shell
g++ -o my_cool_program main.cpp
```

And run binary
```shell
./my_cool_program
```

There are other alternatives, like [CLang](https://clang.llvm.org/), [LLVM](https://llvm.org/), and many [others](https://en.wikipedia.org/wiki/List_of_compilers#C++_compilers)

### Challenge 1 

 - In your project folder, create and `include` folder
 - in the `include` folder create and `lib.hpp` file and write some function into it
 - use the function from `lib.hpp` in the `main.cpp`
 - compile and run program (tip: use `-I <folder>` argument for `gcc` to specify folder with header files)

### Challenge 2
 
 - in the projet folder create the `lib.cpp`
 - move the function implementation from `lib.hpp` to `lib.cpp`; keep function declaration in `lib.hpp`
 - compile and run program (tip: you have to compile both `main.cpp` and `lib.cpp`)
 - helper: `gcc -o <output_binary> <source_1.cpp source_2.cpp ...> -I <folder_with_headers>`
 - Discuss the difference between preprocessing code, compiling code and linking objects.

 - Delete project folder

## CMake Project (30 min)

Before reading following text, please get familiar with [CMake](../../3_others/text/7_cmake.md).

Now let's create a similar project, but using `CMake`.

 - Determine your current location in the file system.
 - Switch to your home directory.
 - Create a new project folder.
 - Inside this folder, create several subdirectories so that the structure looks like this (use the tree command to verify):

```
/MyProject
 |--build
 |--include
 | \--MyProject
 \--src
```

 - Using any text editor (like `nano` or `vim`) to create the following files: `main.cpp`, `lib.cpp`, `lib.hpp`, and `CMakeLists.txt` in your `home directory`. 
 - Move (do not copy) the `main.cpp` adn `lib.cpp` files into the `src` subdirectory.
 - Move the `lib.hpp` file into the `include/MyProject` subdirectory.
 - Move the `CMakeLists.txt` file into the root of the project folder.

Now your project should look like this:
```
/MyProject
 |--build
 |--CMakeLists.txt
 |--include
 | \--MyProject
 |   \--lib.hpp
 \--src
   |--lib.cpp
   \--main.cpp
   
```

 - Using text editor fill the `main.cpp`, `lib.cpp` and `lib.hpp` files with required code.
 - Using the text editor fill the `CMakeLists.txt`

```cmake
cmake_minimum_required(VERSION 3.10)
project(MyProject)
set(CMAKE_CXX_STANDARD 17)
include_directories(include/)
add_executable(my_program src/main.cpp src/lib.cpp)
```

Optional: try to compile program manualy.

```shell
g++ <source1 source2 source3 ...> -I <include_directory> -o <output_binary>
```

 - Delete project folder 

## CLion IDE (30 min)

Create a same project using the CLion IDE. 

To learn how to control CLion, please take a look on [](../../3_others/text/4_clion.md) tutorial.
