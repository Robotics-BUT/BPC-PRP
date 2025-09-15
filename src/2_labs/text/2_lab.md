# Lab 2 - C++, CMake & IDE

Responsible: Ing. Adam Ligocki, Ph.D.


If you are not familiar with Linux CLI commands, please follow the [Linux](../../4_others/text/1_linux.md) chapter.

## CLI Compilation (30 min)

This exercise shows how to write and compile a basic C++ program on Linux.

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

Save the file and compile it using g++ (the GCC C++ compiler):
```bash
g++ -o my_cool_program main.cpp
```

Then run the binary:
```bash
./my_cool_program
```

There are other alternatives, like [Clang](https://clang.llvm.org/), [LLVM](https://llvm.org/), and many [others](https://en.wikipedia.org/wiki/List_of_compilers#C++_compilers).

### Challenge 1 

 - In your project folder, create an `include` folder.
 - In the `include` folder, create a `lib.hpp` file and write a simple function in it.
 - Use the function from `lib.hpp` in `main.cpp`.
 - Compile and run the program (tip: use `-I <folder>` with g++ to specify the header search path).

### Challenge 2
 
 - In the project folder, create `lib.cpp`.
 - Move the function implementation from `lib.hpp` to `lib.cpp`; keep the function declaration in `lib.hpp`.
 - Compile and run the program (tip: you have to compile both `main.cpp` and `lib.cpp`).
 - Helper: `g++ -o <output_binary> <source_1.cpp source_2.cpp ...> -I <folder_with_headers>`
 - Discuss the difference between preprocessing, compiling, and linking.

 - Delete project folder

## CMake Project (30 min)

Before continuing, get familiar with [CMake](../../4_others/text/7_cmake.md).

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

 - Using any text editor (like `nano` or `vim`), create the following files in the project root: `main.cpp`, `lib.cpp`, `lib.hpp`, and `CMakeLists.txt`.
 - Move (do not copy) the `main.cpp` and `lib.cpp` files into the `src` subdirectory.
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

 - Using a text editor, fill the `main.cpp`, `lib.cpp`, and `lib.hpp` files with the required code.
 - Using a text editor, fill the `CMakeLists.txt` file.

```cmake
cmake_minimum_required(VERSION 3.10)
project(MyProject)
set(CMAKE_CXX_STANDARD 17)
include_directories(include/)
add_executable(my_program src/main.cpp src/lib.cpp)
```

Now compile the project. From the project folder, run:

```bash
cd my_project_dir  # go to your project directory
mkdir -p build     # create build folder
cd build           # enter the build folder
cmake ..           # configure; looks for CMakeLists.txt one level up
make               # build program
./my_program       # run program
```


Optional: Try to compile the program manually.

```bash
g++ <source1 source2 source3 ...> -I <include_directory> -o <output_binary>
```

 - Delete project folder 

## CLion IDE (30 min)

Create the same project using the CLion IDE.

To learn how to control CLion, please take a look at the [tutorial](../../4_others/text/4_clion.md) or the [official docs](https://www.jetbrains.com/help/clion/clion-quick-start-guide.html#code-assistance).

## Unit Tests, GTest (30 min)

Unit tests are an effective way to develop software. Often called test‑driven development, the idea is: define the required functionality, write tests that cover the requirements, and then implement the code. When tests pass, the requirements are met.

On larger projects with many contributors and frequent changes, unit tests help catch regressions early. This supports Continuous Integration (CI).

There are many testing frameworks. In this course we will use GoogleTest (GTest), a common and well‑supported choice for C++.

### GTest Installation

If there is no GTest installed on the system follow these instructions.

```bash
# install necessary packages 
sudo apt update
sudo apt install cmake build-essential libgtest-dev

# compile gtest
cd /usr/src/gtest
sudo cmake .
sudo make

# install libs into system
sudo cp lib/*.a /usr/lib
```

Verify the libraries are in the system:

```bash
ls /usr/lib | grep gtest

# you should see:
# libgtest.a
# libgtest_main.a
```

### Adding Unit Test to Project

In your project directory add the `test` folder.

```
/MyProject
 |--include
 |--src
 \--test
```

Add the `add_subdirectory(test)` line at the end of `CMakeLists.txt` file.

Create `CMakeLists.txt` file in the `test` folder.

```cmake
cmake_minimum_required(VERSION 3.10)

find_package(GTest REQUIRED)
include(GoogleTest)
enable_testing()

add_executable(my_test my_test.cpp)
target_link_libraries(my_test GTest::GTest GTest::Main)
gtest_discover_tests(my_test)
```

Create `my_test.cpp` file.

```c++
#include <gtest/gtest.h>

// Simple addition function for demonstration.
float add(float a, float b) {
    return a + b;
}

TEST(AdditionTest, AddsPositiveNumbers) {
    EXPECT_FLOAT_EQ(add(5.0f, 10.0f), 15.0f);
    EXPECT_FLOAT_EQ(add(0.0f, 0.0f), 0.0f);
}

TEST(AdditionTest, AddsEqualNumbers) {
    EXPECT_FLOAT_EQ(add(10.0f, 10.0f), 20.0f);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
```

In CLion, open the bottom console and run:

```bash
mkdir build && cd build
cmake ..
make
cd test
ctest
```

You should see the test output.

You can also run tests directly in CLion by reloading CMake; the test target will appear as an executable at the top of the window.

![CLion Ctest](../images/clion_tests.png)

## C++ Training (2h)

Take a look at the [basic C++ tutorial](../../4_others/text/2_cpp.md) and the more advanced [multithreading tutorial](../../4_others/text/8_multithreading.md).
