cmake_minimum_required(VERSION 3.0.2)
project(gps_injection)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        ublox
        )

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})