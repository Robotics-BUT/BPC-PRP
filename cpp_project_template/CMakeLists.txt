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
find_package(visualization_msgs REQUIRED)

set(SRC
        src/example.cpp
)

set(AMENT_LIBS rclcpp std_msgs sensor_msgs cv_bridge image_transport)

add_executable(${PROJECT_NAME} src/main.cpp ${SRC})
target_include_directories(${PROJECT_NAME} PUBLIC
        include
        ${OpenCV_INCLUDE_DIRS}
        ${SDL2_INCLUDE_DIRS}
        /opt/ros/humble/include
)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${SDL2_LIBS})
ament_target_dependencies(${PROJECT_NAME} rclcpp std_msgs sensor_msgs cv_bridge image_transport visualization_msgs)

install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})
ament_package()

add_subdirectory(tests)
