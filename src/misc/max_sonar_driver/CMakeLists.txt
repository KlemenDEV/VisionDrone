cmake_minimum_required(VERSION 3.0.2)
project(max_sonar_driver)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        sensor_msgs
        )

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(sonar_node src/sonar_node.cc)
add_dependencies(sonar_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(sonar_node ${catkin_LIBRARIES})