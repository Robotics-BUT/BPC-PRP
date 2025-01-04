# Lab 1 - Laboratory Introduction & Linux 

This lab is going to make a brief introduction onto the environment that will be used for development and testing through the entire BPC-PRP course.

The following 3 chapters will take you through the installation and setup of the basic environment and you will practice the Linux CLI

## Linux (1h)

### Installation (optional)

To Install the Linux, please follow the [Linux](../../3_others/text/1_linux.md) chapter.

### Exercise

- Get familiar with the system's GUI.
- Open the command line and try navigating the file system.
- Practice your CLI skills. Commands overview in [Linux](../../3_others/text/1_linux.md) chapter.
  - check you current working directory `pwd`
  - create directory `mkdir`
  - enter created directory `cd`
  - create file `touch`
  - list directory content `ls`
  - rename file `mv`
  - copy file `cp`
  - remove file `rm`
  - create and remove directory `mkdir` and `rm`
  - ...
- try some text editor `nano`, `vim`

<details> <summary>I installed vim and accidentally opened it. What now?</summary>
You can exit vim with the following sequence: press ESC, release it, then hold LSHIFT and press Z twice.
For those interested, a tutorial on using vim can be found here.
</details>

More details about the Linux is going to be introduced during the course

## ROS2 (30 min)

The ROS 2 (Robot Operating System 2) framework is a modern, open-source platform for developing robotic systems. It builds on the success of the original ROS while addressing the limitations of its predecessor, particularly in scalability, security, and real-time performance. Designed for a wide range of robotic applications, ROS 2 is suitable for research, education, and industrial use.

Key features of ROS 2 include a distributed architecture for modular development, enabling nodes (individual processes) to communicate efficiently. It employs the Data Distribution Service (DDS) middleware standard, providing robust and flexible communication mechanisms, including publish/subscribe and service/request paradigms.

ROS 2 supports multiple platforms, such as Linux, Windows, and macOS, making it versatile for diverse hardware and software ecosystems. It integrates well with modern tools and languages, offering Python and C++ APIs, among others. Real-time capabilities are significantly improved, ensuring precise control and response in time-critical robotic systems.

The framework includes a rich ecosystem of libraries and tools, such as Gazebo for simulation, RViz for visualization, and Nav2 for navigation. Its modular design encourages collaboration and code reuse, fostering an active global community. With ROS 2, developers can create scalable, reliable, and innovative robotics applications tailored to real-world challenges.

For installation and commands overview, please follow the chapter: [Robotic Operating System](../../3_others/text/6_ros_2.md).

## CLion SDK (15 min)

### Installation

Install CLion using the Snap package manager:

```shell
sudo snap install --classic clion
```

Alternatively, download the CLion IDE from the [official website](https://www.jetbrains.com/clion/) and get familiar with it (see [CLion IDE](../chap_1_software/text/clion.md)). By registering with your school email, you can obtain a free software license.
