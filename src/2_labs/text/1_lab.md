# Lab 1 - Laboratory Introduction & Linux

Responsible: Ing. Jakub Minařík

This lab briefly introduces the environment used for development and testing throughout the BPC-PRP course.

The following 3 chapters will take you through installing and setting up the basic environment, and you will practice the Linux CLI.

## Linux (1h)

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
You can exit Vim with: press Esc, then hold Shift and press Z twice (Shift+Z+Z).
For a quick introduction, see Vim basics: https://www.vim.org/docs.php
</details>

More details about Linux will be introduced during the course.

## ROS 2 (30 min)

ROS 2 (Robot Operating System 2) is a modern open‑source framework for building robotic systems. It uses DDS for communication (publish/subscribe, services) and runs on Linux, Windows, and macOS. In this course you will use Python or C++ APIs, RViz for visualization, and Gazebo for simulation.

For installation and basic commands, see the ROS 2 chapter: [ROS 2](../../4_others/text/6_ros_2.md).

## CLion IDE (15 min)

### Installation

Install CLion using the Snap package manager:

```bash
sudo snap install --classic clion
```

Alternatively, download CLion from the [official website](https://www.jetbrains.com/clion/) and get familiar with it (see [CLion IDE](../../4_others/text/4_clion.md)). By registering with your school email, you can obtain a free student license.
