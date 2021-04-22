# Building instructions

## 1. Build Pangolin

```
cd non_ros/Pangolin
sudo apt install libgl1-mesa-dev libglew-dev cmake
mkdir build
cd build
cmake ..
cmake --build .
```

## 2. Install OpenCV 4

```
cd ~
sudo apt install build-essential cmake git pkg-config libgtk-3-dev libeigen3-dev
sudo apt install libavcodec-dev libavformat-dev libswscale-dev libv4l-dev v4l-utils libxvidcore-dev libx264-dev
sudo apt install libgtk2.0-dev libcanberra-gtk* libgtk-3-dev
sudo apt install libjpeg-dev libpng-dev libtiff-dev gfortran openexr libatlas-base-dev opencl-headers libopenjp2-7-dev
sudo apt install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libavresample-dev
sudo apt install python3-dev python3-numpy libtbb2 libtbb-dev libdc1394-22-dev
git clone --depth 1 --branch 4.5.2 https://github.com/opencv/opencv.git
git clone --depth 1 --branch 4.5.2 https://github.com/opencv/opencv_contrib.git
mkdir -p ~/opencv/build && cd ~/opencv/build
```

```
cmake -D CMAKE_BUILD_TYPE=RELEASE \
      -D CMAKE_INSTALL_PREFIX=/usr/local \
      -D OPENCV_EXTRA_MODULES_PATH=~/opencv_contrib/modules \
      -D OPENCV_ENABLE_NONFREE=ON \
      -D BUILD_PERF_TESTS=OFF \
      -D BUILD_TESTS=OFF \
      -D BUILD_DOCS=OFF \
      -D BUILD_EXAMPLES=OFF \
      -D ENABLE_PRECOMPILED_HEADERS=OFF \
      -D WITH_TBB=ON \
      -D WITH_OPENMP=ON \
      -D OPENCV_EXTRA_EXE_LINKER_FLAGS=-latomic \
      -D PYTHON3_EXECUTABLE=$(which python3) \
      -D PYTHON_EXECUTABLE=$(which python2) \
      ..

make -j1
sudo make install
sudo ldconfig
python3 -c "import cv2; print(cv2.__version__)"
rm -rf ~/opencv
rm -rf ~/opencv_contrib
```

## 3. Build ORB SLAM 3

```
cd non_ros/orb_slam3
sudo apt install libeigen3-dev libblas-dev liblapack-dev libglew-dev libboost-serialization-dev libpcap-dev libssl-dev
./build.sh
```

## 4. Build librealsense

```
cd non_ros/librealsense
sudo apt-get install -y libdrm-amdgpu1 libdrm-amdgpu1-dbgsym libdrm-dev libdrm-exynos1 libdrm-exynos1-dbgsym libdrm-freedreno1 libdrm-freedreno1-dbgsym libdrm-nouveau2 libdrm-nouveau2-dbgsym libdrm-omap1 libdrm-omap1-dbgsym libdrm-radeon1 libdrm-radeon1-dbgsym libdrm-tegra0 libdrm-tegra0-dbgsym libdrm2 libdrm2-dbgsym
sudo apt-get install -y libglu1-mesa libglu1-mesa-dev glusterfs-common libglu1-mesa libglu1-mesa-dev libglui-dev libglui2c2
sudo apt-get install -y libglu1-mesa libglu1-mesa-dev mesa-utils mesa-utils-extra xorg-dev libgtk-3-dev libusb-1.0-0-dev

sudo cp config/99-realsense-libusb.rules /etc/udev/rules.d/ 
sudo udevadm control --reload-rules && udevadm trigger

mkdir  build  && cd build
cmake .. -DBUILD_EXAMPLES=false -DCMAKE_BUILD_TYPE=Release -DFORCE_LIBUVC=true
make -j1
sudo make install
```

## 5. Install ROS and required packages

Full instructions on: http://wiki.ros.org/ROSberryPi/Installing%20ROS%20Melodic%20on%20the%20Raspberry%20Pi

```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt-get update
sudo apt-get upgrade
sudo apt install -y python-rosdep python-rosinstall-generator python-wstool python-rosinstall build-essential cmake
sudo rosdep init
rosdep update

mkdir -p ~/ros_catkin_ws
cd ~/ros_catkin_ws
rosinstall_generator ros_comm movie_publisher dynamic_reconfigure rosbag rostest ublox_msgs ublox sensor_msgs geometry_msgs mavros_msgs mavros tf sensor_msgs image_transport cv_bridge --rosdistro melodic --deps --wet-only --tar > melodic-custom_ros.rosinstall
```

## 6. Build the custom workspace

```
sudo ln -s /usr/include/eigen3/Eigen /usr/include/Eigen
```

Run catkin_make in the workspace
