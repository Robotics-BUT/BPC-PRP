cmake_minimum_required(VERSION 3.20)
project(prp_project)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(SDL2 REQUIRED)



set(SRC
    src/example.cpp
)

set(STD_LIBS ${OpenCV_LIBS} ${SDL2_LIBRARIES})
set(AMENT_LIBS rclcpp std_msgs sensor_msgs cv_bridge image_transport)

add_library(prp_lib STATIC ${SRC})
target_link_libraries(prp_lib ${STD_LIBS})
target_include_directories(prp_lib PUBLIC include ${OpenCV_INCLUDE})
target_include_directories(prp_lib PUBLIC SDL2Test ${SDL2_INCLUDE_DIRS})
target_include_directories(prp_lib PUBLIC "/opt/ros/humble/include/cv_bridge/")
target_include_directories(prp_lib PUBLIC "/opt/ros/humble/include/image_transport/")
target_include_directories(prp_lib PUBLIC "/opt/ros/humble/include/sensor_msgs/")

message("* * * *")
message(${OpenCV_LIBS})

add_executable(${PROJECT_NAME} src/main.cpp)


target_link_libraries(${PROJECT_NAME} ${STD_LIBS} prp_lib  ${OpenCV_LIBS} ${SDL2_LIBS})
ament_target_dependencies(${PROJECT_NAME} ${AMENT_LIBS} rclcpp std_msgs sensor_msgs cv_bridge image_transport)

install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})
ament_package()

add_subdirectory(tests)