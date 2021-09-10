cmake_minimum_required(VERSION 3.0.2)
project(dead_reckoning)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        tf
        sensor_msgs
        nav_msgs
        geometry_msgs
        geographic_msgs
        eigen_conversions
        )

find_package(Eigen3 3.1.0 REQUIRED)

set(LIBS
        ${EIGEN3_LIBS}
        -lboost_system
        )

catkin_package()

include_directories(
        ${catkin_INCLUDE_DIRS}
)

add_executable(dead_reckoning src/dead_reckoning.cc)
add_dependencies(dead_reckoning ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(dead_reckoning ${LIBS} ${catkin_LIBRARIES})
