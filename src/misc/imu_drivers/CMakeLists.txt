cmake_minimum_required(VERSION 3.0.2)
project(imu_drivers)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        ublox_msgs
        sensor_msgs
        geometry_msgs
        )

catkin_package()

include_directories(${catkin_INCLUDE_DIRS} include)

add_executable(mag_driver_node src/mag_driver_node.cc src/i2c.cc)
add_dependencies(mag_driver_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(mag_driver_node ${catkin_LIBRARIES})

add_executable(imu9dof_driver_node src/imu9dof_driver_node.cc src/i2c.cc)
add_dependencies(imu9dof_driver_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(imu9dof_driver_node ${catkin_LIBRARIES})

add_executable(imu9dof_calibrate src/imu9dof_calibrate.cc src/i2c.cc)
add_dependencies(imu9dof_calibrate ${catkin_EXPORTED_TARGETS})
target_link_libraries(imu9dof_calibrate ${catkin_LIBRARIES})

add_executable(baro_driver_node src/baro_driver_node.cc src/i2c.cc src/bmp280.cc)
add_dependencies(baro_driver_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(baro_driver_node ${catkin_LIBRARIES})