# Lab 1 - Development Environment & First C++ Program

Responsible: Ing. Petr Šopák

This laboratory introduces the basic development environment used throughout the **BPC-PRP** course.

The goal is to get familiar with Linux, command-line tools, and to compile and run a simple C++ program.

By the end of this lab, you should be able to:
  - navigate the Linux file system using the CLI,
  - create and edit source files,
  - compile and run a basic C++ program,
  - understand the role of an IDE and ROS 2 in this course.

## Linux & Command Line Basics (≈ 60 min)

### Installation (optional)

To install Linux, please follow the [Linux](../../4_others/text/1_linux.md) chapter.

### Exercise

- Explore the system GUI.
- Open a terminal and navigate the file system.
- Practice basic CLI commands (see the Linux chapter):
  - Check the current directory: `pwd`
  - Create a directory: `mkdir <dir>`
  - Enter a directory: `cd <dir>`
  - Create a file: `touch <file>`
  - List directory contents: `ls -la`
  - Rename or move a file: `mv <old> <new>`
  - Copy a file: `cp <src> <dst>`
  - Remove a file: `rm <file>`
  - Create/remove a directory: `mkdir <dir>`, `rm -r <dir>`
- Try a text editor: `nano` or `vim`

<details> <summary>I installed vim and accidentally opened it. What now?</summary>
You can exit Vim with: press <kbd>Esc</kbd>, then hold <kbd>Shift</kbd> and press <kbd>Z</kbd> twice (<kbd>Shift</kbd>+<kbd>Z</kbd>+<kbd>Z</kbd>).
  
More Info: https://www.vim.org/docs.php
</details>

More details about Linux will be introduced during the course.

## First C++ Program – CLI Compilation (≈ 45 min)

Create a new project folder in your home directory and enter it.

Create a file `main.cpp` with the following content:

```c++
#include <iostream>

#define A 5

int sum(int a, int b) {
    return a + b;
}

int main() {
    std::cout << "My first C++ program" << std::endl;
    int b = 10;
    std::cout << "Sum result: " << sum(A, b) << std::endl;
    return 0;
}
```

Compile the program using **g++ (GCC C++ compiler)**:

```bash
g++ -o my_program main.cpp
```

Run the compiled binary:

```bash
./my_program
```

### Optional challenges
- Create an `include` directory and move the function declaration to a header file.
- Split the program into multiple source files.
- Compile the program by specifying multiple source files and include paths.

## IDE Overview – CLion (≈ 15 min)

### Installation

Install CLion using the Snap package manager:

```bash
sudo snap install --classic clion
```

Alternatively, download CLion from the [official website](https://www.jetbrains.com/clion/) and get familiar with it (see [CLion IDE](../../4_others/text/4_clion.md)). By registering with your school email, you can obtain a free student license.

To learn how to control CLion, please take a look at the [tutorial](../../4_others/text/4_clion.md) or the [official docs](https://www.jetbrains.com/help/clion/clion-quick-start-guide.html#code-assistance).

### TASK
1) Learn how to control CLion
2) Open the previously created C++ program (main.cpp) in CLion.
3) Build and run the program using the IDE controls.

## ROS 2 – Course Context (≈ 15 min)

ROS 2 (Robot Operating System 2) is a framework for building robotic systems.
In this course, ROS 2 will be used later for:
  - communication between software components,
  - visualization (RViz, rqt),
  - ~~simulation (Gazebo).~~

For installation and basic commands, see the ROS 2 chapter: [ROS 2](../../4_others/text/6_ros_2.md).
