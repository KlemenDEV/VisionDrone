cmake_minimum_required(VERSION 3.0.2)
project(visiondrone)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        image_transport
        std_msgs
        cv_bridge
        sensor_msgs
        )

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

# Node for monocular-inertial2 camera
add_executable(color2gray
        src/color2gray.cc
        )

target_link_libraries(color2gray
        ${LIBS}
        ${catkin_LIBRARIES}
        )