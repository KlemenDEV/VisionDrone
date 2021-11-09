cmake_minimum_required(VERSION 3.0.2)
project(tf02_lidar_driver)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        sensor_msgs
        )

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(tf02_node src/tf02_node.cc)
add_dependencies(tf02_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(tf02_node ${catkin_LIBRARIES})