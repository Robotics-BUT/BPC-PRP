# Lab 5 - Motor, Kinematics & Gamepad

Responsible: Ing. Jakub Minařík


## **Tasks**  

### **1. Motor Node Implementation**  
- Develop a **motor node** that publishes wheel velocity commands to a ROS topic (`/bpc_prp_robot/set_motor_speeds`).  
- Ensure the node can send appropriate velocity commands to drive the robot’s wheels.


### **2. Encoder Node Implementation**  
- Extend the **motor node**  or create separate **encoder node**to **subscribe** to an encoder topic for both wheels(`/bpc_prp_robot/encoders`).



### **3. Robot Parameter Estimation**  
- Measure, estimate, or derive key robot parameters, such as:  
  - The relationship between commanded wheel velocity and actual wheel rotation speed.  
  - The relationship between wheel velocity, wheel radius, and chassis dimensions.  
  - The kinematic constraints affecting the robot’s movement.  
- Motor control values are represented as unsigned 8-bit integers (0–255):
  - A value of 127 corresponds to a neutral state (motors do not move).
  - Values greater than 127 cause the wheels to rotate forward.
  - Values less than 127 cause the wheels to rotate backward.
- The robot should execute the commanded speed for 1 second before stopping.
- Gearbox ration should be 1:48 and number of poles is propably 3 pairs of poles. Best is to test if number of tics makes full rotation of wheel
- Test whether the number of encoder ticks corresponds to a full wheel rotation by counting the ticks per revolution.
- For additional information, refer to the motor datasheets and check the [robot's repository](https://github.com/Robotics-BUT/fenrir-project).


### **4. Kinematics and Odometry Computation**  
- Implement a **class for kinematics and odometry calculations** for a differential drive robot.  
- Compute the robot's pose (position and orientation) based on wheel velocities and time.
- Implement dead reckoning using wheel encoders.  

### **5. Encoder Data Processing**  
- Develop a **class for processing encoder data**:  
  - Estimate the robot’s displacement and position.  
  - Apply correction mechanisms using encoder feedback to improve localization accuracy.  

### **6. (Optional) Gamepad Control**  
- Implement a **gamepad** node to manually control the robot’s movement.  
- Handle relevant gamepad events and publish speed for them.  

## Instruction for Gamepad - SDL2
- Include SLD2 library `#include <SDL2/SDL.h>`
- Inicialize SDL2 - `SDL_Init(SDL_INIT_VIDEO | SDL_INIT_GAMECONTROLLER)`
- Find if joystick/gamepad is connected - `SDL_NumJoysticks()`
- Create gamepad object - `SDL_GameControllerOpen(0)`
- Poll events
  - Create event object - `SDL_Event`
  - Poll events - `SDL_PollEvent()`
  - check event types - e.g. `SDL_CONTROLLERBUTTONDOWN,SDL_CONTROLLERBUTTONUP,SDL_CONTROLLERAXISMOTION`
  - handle the events and set speed and rotation
  - publish ROS2 message
