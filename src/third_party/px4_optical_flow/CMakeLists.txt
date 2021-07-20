cmake_minimum_required(VERSION 3.0.2)

if (POLICY CMP0048)
    # Silence CMP0048 warning about missing project VERSION.
    cmake_policy(SET CMP0048 NEW)
endif ()
if (POLICY CMP0042)
    cmake_policy(SET CMP0042 NEW)
endif ()
project(px4_optical_flow)
set(version_major 0)
set(version_minor 1)
set(version_patch 0)
set(version "${version_major}.${version_minor}.${version_patch}")
set(package-contact "christoph@px4.io")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fpic")

catkin_package()

set(${PROJECT_NAME}_INCLUDE_DIRS
        ${PROJECT_SOURCE_DIR}/include
        CACHE INTERNAL "${PROJECT_NAME}: Include Directories" FORCE)

find_package(catkin REQUIRED COMPONENTS
        OpenCV REQUIRED
        roscpp
        rospy
        orb_slam3
        ublox_msgs
        sensor_msgs
        cv_bridge
        image_transport
        geometry_msgs
        mavros_msgs
        geographic_msgs)

OPTION(OPENCV "ON for OpenCV" ON) # -DOPENCV=OFF

add_subdirectory(external/klt_feature_tracker klt_feature_tracker)
set(klt_feature_tracker_LIBS "klt_feature_tracker")

message(STATUS "Building ${PROJECT_NAME} with OpenCV")

set(px4_optical_flow_INCLUDE_DIRS
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${klt_feature_tracker_INCLUDE_DIRS}
        external/klt_feature_tracker/include
        )

include_directories(px4_optical_flow_INCLUDE_DIRS ${catkin_INCLUDE_DIRS} include external/klt_feature_tracker/include)

add_library(px4_optical_flow SHARED
        src/optical_flow.cpp
        src/px4flow.cpp
        src/flow_px4.cpp
        src/flow_opencv.cpp
        )

target_link_libraries(px4_optical_flow
        ${OpenCV_LIBS}
        ${klt_feature_tracker_LIBS}
        )

target_include_directories(px4_optical_flow PUBLIC include)

# Node for optical flow
add_executable(px4flow_node
        src/px4flow_node.cc
        src/flow_opencv.cpp
        src/flow_px4.cpp
        src/optical_flow.cpp
        src/px4flow.cpp
        )

add_dependencies(px4flow_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(px4flow_node ${catkin_LIBRARIES} ${OpenCV_LIBS}
        ${klt_feature_tracker_LIBS})