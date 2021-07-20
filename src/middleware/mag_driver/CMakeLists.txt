cmake_minimum_required(VERSION 3.0.2)
project(mag_driver)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        ublox_msgs
        sensor_msgs
        geometry_msgs
        )

catkin_package()

include_directories(${catkin_INCLUDE_DIRS} include)

# Node for GPS generator
add_executable(mag_driver_node
        src/mag_driver_node.cc
        src/i2c.cc
        )

add_dependencies(mag_driver_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(mag_driver_node ${catkin_LIBRARIES})