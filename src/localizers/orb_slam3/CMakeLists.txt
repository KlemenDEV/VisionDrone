cmake_minimum_required(VERSION 3.0.2)
project(orb_slam3)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        tf
        sensor_msgs
        geometry_msgs
        sensor_msgs
        image_transport
        cv_bridge
        nav_msgs
        geometry_msgs
        geographic_msgs
        eigen_conversions
        )

find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if (COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
    add_definitions(-DCOMPILEDWITHC11)
    message(STATUS "Using flag -std=c++11.")
elseif (COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
    add_definitions(-DCOMPILEDWITHC0X)
    message(STATUS "Using flag -std=c++0x.")
else ()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif ()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations")

find_package(OpenCV 3.0 QUIET)
if (NOT OpenCV_FOUND)
    find_package(OpenCV 2.4.3 QUIET)
    if (NOT OpenCV_FOUND)
        message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
    endif ()
endif ()
# end c++11 check

set(LIBS
        ${OpenCV_LIBS}
        ${EIGEN3_LIBS}
        ${Pangolin_LIBRARIES}
        ${PROJECT_SOURCE_DIR}/../../../non_ros/orb_slam3/Thirdparty/DBoW2/lib/libDBoW2.so
        ${PROJECT_SOURCE_DIR}/../../../non_ros/orb_slam3/Thirdparty/g2o/lib/libg2o.so
        ${PROJECT_SOURCE_DIR}/../../../non_ros/orb_slam3/lib/libORB_SLAM3.so
        -lboost_system
        )

catkin_package(
        LIBRARIES ${PROJECT_NAME}
        CATKIN_DEPENDS roscpp message_runtime geographic_msgs
)

include_directories(
        ${catkin_INCLUDE_DIRS}
        ${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/../../../non_ros/orb_slam3/
        ${PROJECT_SOURCE_DIR}/../../../non_ros/orb_slam3/include
        ${PROJECT_SOURCE_DIR}/../../../non_ros/orb_slam3/include/CameraModels
        ${Pangolin_INCLUDE_DIRS}
)

add_executable(ORBSLAM3 src/orb_slam3.cc)

add_dependencies(ORBSLAM3 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(ORBSLAM3
        ${LIBS}
        ${catkin_LIBRARIES}
        )

catkin_install_python(PROGRAMS script/slam_absolute.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})