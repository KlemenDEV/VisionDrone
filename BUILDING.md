# Building instructions

## 1. Install ROS and required packages

Full instructions on: http://wiki.ros.org/ROSberryPi/Installing%20ROS%20Melodic%20on%20the%20Raspberry%20Pi

```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt-get update
sudo apt-get upgrade
sudo apt install -y python-rosdep python-rosinstall-generator python-wstool python-rosinstall build-essential cmake
pip install moviepy
sudo rosdep init
rosdep update

mkdir -p ~/ros_catkin_ws
cd ~/ros_catkin_ws
rosinstall_generator ros_comm movie_publisher dynamic_reconfigure usb_cam rosbag rostest ublox_msgs ublox sensor_msgs geometry_msgs mavros_msgs mavros tf sensor_msgs image_transport cv_bridge --rosdistro melodic --deps --wet-only --tar > melodic-custom_ros.rosinstall
```

## 2. Build ORB SLAM 3

```
cd non_ros/Pangolin
sudo apt install libgl1-mesa-dev libglew-dev cmake
mkdir build
cd build
cmake ..
cmake --build .
```

```
cd non_ros/orb_slam3
sudo apt install libeigen3-dev libblas-dev liblapack-dev libglew-dev libboost-serialization-dev libpcap-dev libssl-dev
sudo ln -s /usr/include/eigen3/Eigen /usr/include/Eigen
./build.sh
```

## 3. Build librealsense

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

## 5. Build the custom workspace

Run catkin_make in the workspace

# Pi config.txt

```
# Pi camera
dtparam=i2c_arm=on
start_x=1
gpu_mem=128

# Multiple UARTs
enable_uart=1
dtoverlay=uart2
dtoverlay=uart3
dtoverlay=uart4
dtoverlay=uart5

# turn off bluetooth
dtoverlay=disable-bt

# shutdown button
dtoverlay=gpio-shutdown,gpio_pin=20,active_low=1,gpio_pull=up,debounce=3000
```

# WiFi configuration

Based on https://thepi.io/how-to-use-your-raspberry-pi-as-a-wireless-access-point/


```
/etc/dhcpcd.conf

NOTHING

/etc/network/interfaces

auto wlan0
iface wlan0 inet static
address 192.168.42.1
netmask 255.255.255.0

/etc/hostapd/hostapd.conf

interface=wlan0
#bridge=br0
hw_mode=g
channel=7
wmm_enabled=0
macaddr_acl=0
auth_algs=1
ignore_broadcast_ssid=0
wpa=2
wpa_key_mgmt=WPA-PSK
wpa_pairwise=TKIP
rsn_pairwise=CCMP
ssid=VisionDrone
wpa_passphrase=visiondrone

/etc/default/hostapd

DAEMON_CONF="/etc/hostapd/hostapd.conf"

/etc/dnsmasq.conf

no-resolv
interface=wlan0
dhcp-range=192.168.42.10,192.168.42.30,255.255.255.0,24h
```
