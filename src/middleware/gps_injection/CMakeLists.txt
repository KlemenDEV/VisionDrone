cmake_minimum_required(VERSION 3.0.2)
project(gps_injection)

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
add_executable(gps_injection_node src/gps_injection_node.cc
        src/UBXSender.cc
        src/GPSData.cc
        )

add_dependencies(gps_injection_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(gps_injection_node ${catkin_LIBRARIES})