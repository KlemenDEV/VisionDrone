# Pi config.txt

Make sure to disable console=serial0,115200 in boot command so boot messages are not printed to serial.

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

# Initial Pi setup

```
cd ~
rm -rf Bookshelf  Desktop  Documents  Downloads  Music  Pictures  Public  Templates  Videos
sudo apt purge xserver* lightdm* raspberrypi-ui-mods vlc* lxde* chromium* desktop* gnome* gstreamer* gtk* hicolor-icon-theme* lx* mesa* 'x11-*'
sudo apt --purge autoremove
sudo apt update
sudo apt upgrade
```

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
rosinstall_generator ros_comm --rosdistro melodic --deps --wet-only --tar > melodic-ros_comm-wet.rosinstall
wstool init src melodic-ros_comm-wet.rosinstall

rosinstall_generator ros_comm robot_localization movie_publisher dynamic_reconfigure message_runtime genmsg std_msgs usb_cam rosbag rostest ublox ublox_serialization message_generation roscpp_serialization diagnostic_updater tf ublox_msgs sensor_msgs geometry_msgs mavros_msgs diagnostic_updater nodelet mavros tf image_transport cv_bridge --rosdistro melodic --deps --wet-only --tar > melodic-custom_ros.rosinstall

wstool merge -t src melodic-custom_ros.rosinstall
wstool update -t src

rosdep install --from-paths src --ignore-src --rosdistro melodic -y -r --os=debian:buster

sudo ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/melodic -j2

source /opt/ros/melodic/setup.bash
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
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

# WiFi configuration

Based on https://thepi.io/how-to-use-your-raspberry-pi-as-a-wireless-access-point/

```
sudo systemctl unmask hostapd
sudo systemctl enable hostapd
sudo systemctl start hostapd
```

```
/etc/dhcpcd.conf

interface wlan0
static ip_address=192.168.42.1/24

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
