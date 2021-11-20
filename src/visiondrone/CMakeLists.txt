cmake_minimum_required(VERSION 3.0.2)
project(visiondrone)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        image_transport
        std_msgs
        cv_bridge
        sensor_msgs
        velocity_integrator
        )

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

# Node for monocular-inertial2 camera
add_executable(color2gray
        src/color2gray.cc
        )

target_link_libraries(color2gray
        ${LIBS}
        ${catkin_LIBRARIES}
        )

catkin_install_python(PROGRAMS script/pose2path.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

catkin_install_python(PROGRAMS script/pose2matlab.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

catkin_install_python(PROGRAMS script/imubias.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

catkin_install_python(PROGRAMS script/pfchecklists.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )