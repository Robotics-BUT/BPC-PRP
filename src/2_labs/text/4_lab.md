# Lab 4 - Data Capture & Visualization (ROS)

**Responsible:** Ing. Petr Šopák

### Learning Objectives

**1) Fundamentals of ROS 2**
 -  Setting Up a **ROS 2 workspace** (optional)
 -  Creating a **custom ROS 2 node** - implementing a basic **publisher & subscriber**
 -  Exploring essential **ROS 2 CLI commands**
 -  Utilizing **visualization tools** - `rqt_plot` and `rqt_graph`

**2) Implementing Basic Behaviour for BPC-PRP robots**
 -  Establishing **connection to the robots**
 -  Implementing **IO node** - Reading button inputs and controlling LEDs

**BONUS: Advanced Visualizations**
 -  Using `RViz` for graphical representation
 -  Creating basic graphical objects and defining their behavior

> If you are using your own notebook, make sure to configure everything necessary in the Ubuntu environment! Refer to (URL) for details.

## Fundamentals of ROS 2 (Approx. 1 Hour)

### Setting Up a ROS Workspace (5 min) - optional

Last week, you cloned a basic template that we will gradually extend with additional functionalities. The first step is to establish communication between the existing project and **ROS 2**.

There are many ways to set up a ROS project, but typically, a [**ROS workspace**](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html) is created, which is a structured directory for managing multiple packages. However, for this course, we *do not need a full ROS workspace*, as we will be working with only one package. Let’s review the commands from [**Lab 1**](https://github.com/Robotics-BUT/BPC-PRP/blob/master/src/2_labs/text/1_lab.md). Using the CLI, create a workspace folder structured as follows:

```shell
mkdir -p ~/ros_w/src
cd ~/ros_w
```
Next, copy the folders from previous labs into the `src` directory or re-clone the repository from Git ([**Lab 3**](https://github.com/Robotics-BUT/BPC-PRP/blob/master/src/2_labs/text/3_lab.md)). You can rename the template to something like **"ros_package"**, but this is optional.
Finally, you need to **compile the package** and set up the environment:

```shell
colcon build
source install/setup.bash
```

> **Recap Notes: What is a ROS Workspace?**
>
> A **ROS workspace** is a structured environment for developing and managing multiple ROS packages. In this case, we created a workspace named **ros_w** (you can choose any name, but this follows common convention).
>
> A ROS workspace allows for **unified compilation** and **management** of multiple packages. Each **ROS package** is a **CMake project** stored in the `src` folder. Packages contain:
>
> - Implementations of **nodes** (executable programs running in ROS),
> - Definitions of **messages** (custom data structures used for communication),
> - Configuration files,
> - Other resources required for running ROS functionalities.
>
> After (or before) setting up the workspace, **always remember** to source your environment before using ROS:
> 
> ```shell
> source ~/ros_w/install/setup.bash
> ```
> or add sourcing to your shell startup script
>
> ```shell
> echo "source /opt/ros/<distro>/setup.bash" >> ~/.bashrc
> ```

# Creating a custom node (55 min)

In this section, we will create a **ROS 2 node** that will **publish** and **receive data** ([info](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html)). We will then visualize the data accordingly.

**Required Tools:**
 -  [`rqt_graph`](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html#rqt-graph)
 -  [`rqt_plot`](https://wiki.ros.org/rqt_plot)

**Instructions:**
1. Open a terminal and set the ROS_DOMAIN_ID to match the ID on your computer’s case:
   ```shell
   export ROS_DOMAIN_ID=<robot_ID>
   ```
   This change is temporary and applies **only to the current terminal session**. If you want to make it permanent, you need to update your configuration file:
   ```shell
   echo "export ROS_DOMAIN_ID=<robot_ID>" >> ~/.bashrc
   source ~/.bashrc
   ```
   
   Check the `~/.bashrc` content using. The `~/.bashrc` is a script that runs every time the new bash session (cli) is opened and it is used to setup bash environment.
   ```
   cat ~/.bashrc
   ```

   To **verify the domain ID**, use:
    ```shell
   echo $ROS_DOMAIN_ID
   ```
   > If there are any issues with the .bashrc file, please let me know.
2. Open your **CMake project** in **CLion** or another **Editor/IDE**
3. Ensure that the **IDE is launched from a terminal where the ROS environment is sourced**
```shell
source /opt/ros/<distro>/setup.bash  # We are using the Humble distribution
```
3. Write the following code in `main.cpp`:

```c++
#include <rclcpp/rclcpp.hpp>
#include "RosExampleClass.h"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    // Create an executor (for handling multiple nodes)
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

    // Create multiple nodes
    auto node1 = std::make_shared<rclcpp::Node>("node1");
    auto node2 = std::make_shared<rclcpp::Node>("node2");

    // Create instances of RosExampleClass using the existing nodes
    auto example_class1 = std::make_shared<RosExampleClass>(node1, "topic1", 1.0);
    auto example_class2 = std::make_shared<RosExampleClass>(node2, "topic2", 2.0);

    // Add nodes to the executor
    executor->add_node(node1);
    executor->add_node(node2);

    // Run the executor (handles callbacks for both nodes)
    executor->spin();

    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}
```

4. Create a **header file** in the `include` directury to ensure the code runs properly.
5. Adds the following code tot he header file:

```c++

#pragma once

#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

class RosExampleClass {
public:
    // Constructor takes a shared_ptr to an existing node instead of creating one.
    RosExampleClass(const rclcpp::Node::SharedPtr &node, const std::string &topic, double freq)
        : node_(node), start_time_(node_->now()) {

        // Initialize the publisher
        publisher_ = node_->create_publisher<std_msgs::msg::Float32>(topic, 1);

        // Initialize the subscriber
        subscriber_ = node_->create_subscription<std_msgs::msg::Float32>(
            topic, 1, std::bind(&RosExampleClass::subscriber_callback, this, std::placeholders::_1));

        // Create a timer
        timer_ = node_->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(freq * 1000)),
            std::bind(&RosExampleClass::timer_callback, this));

        RCLCPP_INFO(node_->get_logger(), "Node setup complete for topic: %s", topic.c_str());
    }

private:
    void timer_callback() {
        RCLCPP_INFO(node_->get_logger(), "Timer triggered. Publishing uptime...");

        double uptime = (node_->now() - start_time_).seconds();
        publish_message(uptime);
    }

    void subscriber_callback(const std_msgs::msg::Float32::SharedPtr msg) {
        RCLCPP_INFO(node_->get_logger(), "Received: %f", msg->data);
    }

    void publish_message(float value_to_publish) {
        auto msg = std_msgs::msg::Float32();
        msg.data = value_to_publish;
        publisher_->publish(msg);
        RCLCPP_INFO(node_->get_logger(), "Published: %f", msg.data);
    }

    // Shared pointer to the main ROS node
    rclcpp::Node::SharedPtr node_;

    // Publisher, subscriber, and timer
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Start time for uptime calculation
    rclcpp::Time start_time_;
};

```

At this point, you can **compile and run** your project. Alternatively, you can use the CLI:

```shell
colcon build --packages-select <package_name>
source install/setup.bash
ros2 run <package_name> <executable_file>
```
This will **compile the ROS 2 workspace**, **load the compiled packages**, and **execute the program** from the specified package.

--------------------------------------
**TASK 1:** 
 -  **Review the code** – Try to understand what each part does and connect it with concepts from the lecture.
 -  **Observe the program’s output** in the terminal.
--------------------------------------

**How to Check Published data**

There are two main ways to analyze and visualize data in ROS 2 - using **CLI commands** in the terminal or **ROS visualization tools**. 

**1) Inspecting Published Data via CLI**

In a new terminal (Don't forget to **source the ROS environment!**), you can inspect the data published to a specific topic:

```shell
ros2 topic echo <topic_name>
```
If you are unsure about the topic name, you can list all available topics:

```shell
ros2 topic list
```
> Similar commands exist for **nodes, services, and actions** – refer to the documentation for more details (LINK).

**2) Using ROS Visualization Tools**

ROS 2 offers built-in tools for graphical visualization of data and system architecture. **Real-Time Data Visualization** - `rqt_plot` - allows you to **graphically plot topic values** in real-time:

```shell
ros2 run rqt_plot rqt_plot
```
> In the GUI, enter `/<topic_name>/data` into the **input field**, click **+**, and configure the **X and Y axes** accordingly.

**System Architecture Visualization** - `rqt_graph` - displays the **ROS 2 node connections and data flow** within the system:

```shell
rqt_graph
```
> When the system's architecture changes, simply refresh the visualization by clicking the **refresh button**.

---------------------------------------------------------------
**TASK 2**

Modify or extend the code to **publish sine wave (or other mathematical function) values**.

**Don't forget to check the results using ROS 2 tools**

*(Hint: Include the <cmath> library for the mathemtical functions like `std::sin`)*

**(Bonus for fast finishers)**: Modify the code so that each node publishes different data, such as two distinct mathematical functions.

---------------------------------------------------------------


## Implementing Basic Behaviour for BPC-PRP robots (1 hour)

In this section, we will get familiar with the PRP robot, **learn how to connect to it, explore potential issues, and then write a basic input-output node for handling buttons and LEDs**. Finally, we will experiment with these components.

### Connecting to the Robot

The robot operates as an independent unit, meaning it **has its own computing system** (Raspberry Pi) running Ubuntu with an already installed ROS 2 program. *Our goal is to connect to the robot and send instructions to control its behavior.*

1) First, power on the robot and connect to it using SSH. Ensure that you are on the same network as the robot.
```shell
   ssh robot@prp-<color>
```
> The password will be written on the classroom board.
> The robot may take about a minute to boot up. Please **wait before attempting to connect**.

2) Once connected to the Robot:
  - examine the system architecture to understand which ROS 2 nodes are running on the robot and what topics they publish or subscribe to.
  - Additionally, check the important environment variables using:
    ```shell
    env | grep ROS
    ```
   > The `ROS_DOMAIN_ID` is particularly important. It is an identifier used by the `DDS` (Data Distribution Service), which serves as the middleware for communication in ROS 2. **Only ROS 2 nodes with the same ROS_DOMAIN_ID can discover and communicate with each other.**

3) *(if it is necessary)* Open a new terminal **on your local machine** (not on the robot) and change the `ROS_DOMAIN_ID` to match the robot’s domain:
   ```shell
   export ROS_DOMAIN_ID=<robot_ID>
   ```
   This change is temporary and applies **only to the current terminal session**. If you want to make it permanent, you need to update your configuration file:
   ```shell
   echo "export ROS_DOMAIN_ID=<robot_ID>" >> ~/.bashrc
   source ~/.bashrc
   ```
   Alternatively, you can change it by modifying the .bashrc file:
    1) Open the ~/.bashrc file in an editor, for example, using nano:
      ```shell  
      nano ~/.bashrc
      ```
    2) Add/modify the following line at the end of the file:
      ```shell
      export ROS_DOMAIN_ID=<your_ID>
      ```
    3) Save the changes and close the editor (in nano, press CTRL+X, then Y to confirm saving, and Enter to finalize).
    4) To apply the changes, run the following command:
      ```shell
      source ~/.bashrc
      ```
   
   To **verify the domain ID**, use:
    ```shell
   echo $ROS_DOMAIN_ID
   ```
3) After successfully setting the `ROS_DOMAIN_ID`, verify whether you can see the topics published by the robot from your local machine terminal.

### Implementing the IO node

At this point, you should be able to interact with the robot—sending and receiving data. Now, let's set up the basic project structure where you will progressively add files.

#### Setting Up the Project Structure

1) (Open CLion.) Create a `nodes` directory inside both the `include` and `src` folders of your **CMake project**. These directories will hold your node scripts for different components.
 > You can also create additional directories such as `algorithms` if needed.
2) Inside the `nodes` directories, create two files:
 - `src/nodes/io_node.hpp` (for declarations)
 - `include/nodes/io_node.cpp` (for implementation)
3) Open `CMakeLists.txt`, review it, and modify it to ensure that your project can be built successfully.
 > **!Remember to update CMakeLists.txt whenever you create new files!**

#### Writing an IO Node for Buttons
4) First, gather **information about the published topic** for buttons (`/bpc_prp_robot/buttons`). Determine the **message type** and its **structure** using the following [commands](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html#ros2-topic-info):
 
 ```shell
 ros2 topic type <topic_name> # Get the type of the message
 ros2 interface show <type_of_msg> # Show the structure of the message
 ```
 > Ensure that you are using the correct topic name.

5) (Optional) To simplify implementation, create a header file named helper.hpp inside the include folder. Copy and paste the provided code snippet into this file. This helper file will assist you in working with topics efficiently.
```c++
#pragma once

#include <iostream>
 
static const int MAIN_LOOP_PERIOD_MS = 50;
 
namespace Topic {
   const std::string buttons = "/bpc_prp_robot/buttons";
   const std::string set_rgb_leds = "/bpc_prp_robot/rgb_leds";
};

namespace Frame {
    const std::string origin = "origin";
    const std::string robot = "robot";
    const std::string lidar = "lidar";
};
```

------------------------------------------------------------------------------------------

**TASK 3**
1) Using the previous tasks as a reference, complete the code for `io_node.hpp` and `io_node.cpp` to retrieve button press data.
   > Hint: Below is an example `.hpp` file. You can use it for inspiration, but modifications are allowed based on your needs.
   >
   > ```c++
   > #pragma once
   >
   > #include <rclcpp/rclcpp.hpp>
   > #include <std_msgs/msg/u_int8.hpp>
   >
   > namespace nodes {
   >      class IoNode : public rclcpp::Node {
   >      public:
   >          // Constructor
   >          IoNode();
   >          // Destructor (default)
   >          ~IoNode() override = default;
   >
   >          // Function to retireve the last pressed button value
   >          int get_button_pressed() const;
   >  
   >      private:
   >          // Variable to store the last received button press value
   >          int button_pressed_ = -1;
   > 
   >          // Subscriber for button press messages
   >          rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr button_subscriber_; 
   >  
   >          // Callback - preprocess received message
   >          void on_button_callback(const std_msgs::msg::UInt8::SharedPtr msg);
   >      };
   >  }
   > ```
   > 
   > ```c++
   > #include "my_project/nodes/io_node.hpp"
   > namespace {
   >     IoNode::IoNode() {
   >        // ...
   >     }
   > 
   >     IoNode::get_button_pressed() const {
   >        // ...
   >     }
   > 
   >     // ...
   > }   
   >
   > > ```
2) Run your program and check if the button press data is being received and processed as expected.
-------------------------------------------------------------------------------------------------
**TASK 4**
1) Add Code for Controlling LEDs
   > Hints: - The robot subscribes to a topic for controlling LEDs.
   >        - Find out which message type is used for controlling LEDs.
   >        - Use the CLI to [publish test messages](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html#ros2-topic-pub) and analyze their effect:
   >
   >```shell
   > ros2 topic pub <led_topic> <message_type> <message_data>
   >```
2) Test LED Functionality with Simple Publishing
3) Now, **integrate button input with LED output**:
   - **Pressing the first button** → All LEDs turn on.
   - **Pressing the second button** → LEDs cycle through colors in a **your defined sequence**.
   - **Pressing the third button** → The intensity of *each LED color component* will change according to a mathematical function, with **each color phase-shifted by one-third of the cycle**.
-------------------------------------------------------------------------------------------------

# BONUS: Advanced Visualizations (30 min)

**Required Tools:** `rviz2`

Official documentation: [RViz2](https://docs.ros.org/en/humble/Tutorials/Intermediate/RViz/RViz-User-Guide/RViz-User-Guide.html).

In this section, we will learn how to create **visualizations in ROS 2** using `RViz`. You should refer to the [**official RViz documentation**](https://wiki.ros.org/rviz) and the [**marker tutorial**](https://wiki.ros.org/rviz/DisplayTypes/Marker) to get a deeper understanding.

ROS 2 provides **visualization messages** via the [`visualization_msgs`](https://wiki.ros.org/visualization_msgs) package. These messages allow **rendering of various geometric shapes, arrows, lines, polylines, point clouds, text, and mesh grids.**

Our objective will be to implement a **class that visualizes a floating cube in 3D space** while displaying its real-time position next to it.

1) Create the Header File `rviz_example_class.hpp`:

   ```c++
    #pragma once

    #include <iostream>
    #include <memory>
    #include <rclcpp/rclcpp.hpp>
    #include <visualization_msgs/msg/marker_array.hpp>prp_project
    #include <cmath>
    #include <iomanip>
    #include <sstream>
    
    #define FORMAT std::fixed << std::setw(5) << std::showpos << std::setprecision(2)
    
    class RvizExampleClass : public rclcpp::Node {
    public:
        RvizExampleClass(const std::string& topic, double freq)
            : Node("rviz_example_node") // Node name in ROS 2
        {
            // Create a timer with the specified frequency (Hz)
            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(static_cast<int>(1000.0 / freq)),
                std::bind(&RvizExampleClass::timer_callback, this)
            );
    
            // Create a publisher for MarkerArray messages
            markers_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(topic, 10);
        }
    
    private:
        class Pose {
        public:
            Pose(float x, float y, float z) : x_{x}, y_{y}, z_{z} {}
            float x() const { return x_; }
            float y() const { return y_; }
            float z() const { return z_; }
        private:
            const float x_, y_, z_;
        };
    
        void timer_callback() {
            auto time = this->now().seconds();
            auto pose = Pose(sin(time), cos(time), 0.5 * sin(time * 3));
    
            // Create a MarkerArray message
            visualization_msgs::msg::MarkerArray msg;
            msg.markers.push_back(make_cube_marker(pose));
            msg.markers.push_back(make_text_marker(pose));
    
            // Publish the marker array
            markers_publisher_->publish(msg);
        }
    
        visualization_msgs::msg::Marker make_cube_marker(const Pose& pose) {
            visualization_msgs::msg::Marker cube;
    
            // Coordinate system
            cube.header.frame_id = "map"; // In ROS 2, "map" or "odom" is recommended
            cube.header.stamp = this->now();
    
            // Marker Type
            cube.type = visualization_msgs::msg::Marker::CUBE;
            cube.action = visualization_msgs::msg::Marker::ADD;
            cube.id = 0;
    
            // Position
            cube.pose.position.x = pose.x();
            cube.pose.position.y = pose.y();
            cube.pose.position.z = pose.z();
    
            // Orientation (Quaternion)
            cube.pose.orientation.x = 0.0;
            cube.pose.orientation.y = 0.0;
            cube.pose.orientation.z = 0.0;
            cube.pose.orientation.w = 1.0;
    
            // Size
            cube.scale.x = cube.scale.y = cube.scale.z = 0.1;
    
            // Color
            cube.color.a = 1.0; // Alpha (visibility)
            cube.color.r = 0.0;
            cube.color.g = 1.0;
            cube.color.b = 0.0;
    
            return cube;
        }
    
        visualization_msgs::msg::Marker make_text_marker(const Pose& pose) {
            visualization_msgs::msg::Marker text;
    
            // Coordinate system
            text.header.frame_id = "map";
            text.header.stamp = this->now();
    
            // Marker Type
            text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            text.action = visualization_msgs::msg::Marker::ADD;
            text.id = 1;
    
            // Position (slightly above the cube)
            text.pose.position.x = pose.x();
            text.pose.position.y = pose.y();
            text.pose.position.z = pose.z() + 0.2;
    
            // Size
            text.scale.z = 0.1;
    
            // Text content
            std::stringstream stream;
            stream << "* Cool Cube *" << std::endl
                   << "  x: " << FORMAT << pose.x() << std::endl
                   << "  y: " << FORMAT << pose.y() << std::endl
                   << "  z: " << FORMAT << pose.z();
            text.text = stream.str();
    
            // Color
            text.color.a = 1.0;
            text.color.r = 1.0;
            text.color.g = 1.0;
            text.color.b = 0.0;
    
            return text;
        }
    
        // ROS 2 timer
        rclcpp::TimerBase::SharedPtr timer_;
    
        // ROS 2 publisher
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_publisher_;
    };
   ```
   
3) Add the Following Code to `main.cpp`:

   ```c++
     #include "rviz_example_class.hpp"
     #include <rclcpp/rclcpp.hpp>
  
     int main(int argc, char** argv) {
         // Initialize ROS 2
         rclcpp::init(argc, argv);
     
         // Create a node and run it
         auto node = std::make_shared<RvizExampleClass>("rviz_topic", 30.0);
         rclcpp::spin(node);
     
         // Shutdown ROS 2
         rclcpp::shutdown();
         return 0;
     }
   ```
5) Build and run your project. Then open `Rviz`
   ```shell
   rviz2
   ```
6) Add the Visualization Topic
   1) In `RViz`, go to `Add` → `By Topic`
   2) Locate the created topic `rviz_topic`
   3) Select `MarkerArray` to display the cube and text
--------------------------------------------------------------
**TASK BONUS**: 
1) Check the Code and `RViz` Features
2) Experiment with modifying the code to explore different visualization features. 
> The goal of this task is to familiarize yourself with RViz. RViz will be used in future exercises, e.g., visualizing LiDAR data.  
