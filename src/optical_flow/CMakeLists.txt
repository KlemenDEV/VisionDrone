cmake_minimum_required(VERSION 3.0.2)
project(optical_flow)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        orb_slam3
        ublox_msgs
        sensor_msgs
        geometry_msgs
        mavros_msgs
        geographic_msgs
        mrs_optic_flow
        )

catkin_package()

include_directories(${catkin_INCLUDE_DIRS} include)

# Node for GPS generator
add_executable(velocity_integrator_node src/VelocityIntegrator.cc)

add_dependencies(velocity_integrator_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(velocity_integrator_node ${catkin_LIBRARIES})