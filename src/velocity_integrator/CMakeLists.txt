cmake_minimum_required(VERSION 3.0.2)
project(velocity_integrator)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        sensor_msgs
        geometry_msgs
        geographic_msgs
        message_generation
        )

add_service_files(
        FILES
        SetDatum.srv
)

# Actually generate the language-specific message and service files
generate_messages(DEPENDENCIES geographic_msgs)

catkin_package(CATKIN_DEPENDS roscpp message_runtime message_generation geographic_msgs)

include_directories(${catkin_INCLUDE_DIRS} include)

# Nodes

add_executable(velocity_integrator_node src/VelocityIntegrator.cc src/PoseManager.cc)
add_dependencies(velocity_integrator_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(velocity_integrator_node ${catkin_LIBRARIES})

add_executable(pose_manager_node src/PoseManagerNode.cc src/PoseManager.cc)
add_dependencies(pose_manager_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pose_manager_node ${catkin_LIBRARIES})

catkin_install_python(PROGRAMS script/velocity_fusion.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})