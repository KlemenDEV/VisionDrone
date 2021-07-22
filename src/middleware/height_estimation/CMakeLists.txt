cmake_minimum_required(VERSION 3.0.2)
project(height_estimation)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        ublox_msgs
        sensor_msgs
        geometry_msgs
        mavros_msgs
        geographic_msgs
        )

catkin_package()

include_directories(${catkin_INCLUDE_DIRS} include)

# Node for GPS generator
add_executable(height_estimation_node src/height_estimation_node.cc src/algebra.cc src/altitude.cc src/filters.cc)

add_dependencies(height_estimation_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(height_estimation_node ${catkin_LIBRARIES})