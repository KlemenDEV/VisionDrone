cmake_minimum_required(VERSION 3.0.2)
project(gps_injection)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        ublox_msgs
        )

catkin_package()

include_directories(${catkin_INCLUDE_DIRS} include)

# Node for GPS generator
add_executable(GPSGenerator
        src/gps_generator.cc
        )
target_link_libraries(GPSGenerator ${catkin_LIBRARIES})