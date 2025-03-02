# PX4 Drone Reconnaissance  

## Abstract
This repository contains my implementation of drone reconnaissance where a drone will scan a 100m x 100m area to locate a target, and perform precision landing on the target. I will also mention that this repo uses ROS2 combined with PX4 SITL to test out PX4 hardware without having the hardware. Note that in order to set up my entire system, I followed the PX4 Documentation linked in the Credits section of this README file. After setting it up, I went through the the code examples, and figured out how to create my own PX4 gazebo world and use ros topics to publish commands and subscribe to incoming messages from the drone. 

## ⭐ Base Repositories and Datasets (Credits) ⭐ <br />
① [PX4 Documentation](https://docs.px4.io/main/en/ros2/user_guide.html) <br />
② [PX4 ROS2 Message Repo](https://github.com/PX4/px4_msgs) <br />
③ [PX4 ROS2 Communication Repo](https://github.com/PX4/px4_ros_com) <br />
④ [Yolov5 Roboflow Train/Val/Test Dataset](https://universe.roboflow.com/swee-xiao-qi/parking-lot-availability) <br />
  
## Installation
For more information on installation, go to the PX4 Documentation link in credits.
### WSL Installation
Open windows powershell and run:
```
wsl --install -d Ubuntu-22.04
```
### PX4 Development Environment Installation
To retrieve the PX4 ROS2 World and Code, run the following in a new ubuntu terminal
```
sudo apt install cmake
sudo apt install make
sudo apt install python3-pip

cd
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
cd PX4-Autopilot/
make px4_sitl
```
### ROS2 Humble Installation
To install ROS2 Humble run:
```
cd
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update && sudo apt upgrade -y
sudo apt install ros-humble-desktop
sudo apt install ros-dev-tools
source /opt/ros/humble/setup.bash && echo "source /opt/ros/humble/setup.bash" >> .bashrc
pip install --user -U empy==3.3.4 pyros-genmsg setuptools
```
### Micro XRCE-DDS Agent Installation
In a new Ubuntu 22.04.5 LTS terminal, run:
```
cd
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build
cd build
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib/
```
### Q Ground Control (QGC) Installation
Run the following in a new Ubuntu 22.04.5 LTS terminal:
```
cd
sudo usermod -a -G dialout $USER
sudo apt-get remove modemmanager -y
sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y
sudo apt install libfuse2 -y
sudo apt install libxcb-xinerama0 libxkbcommon-x11-0 libxcb-cursor-dev -y
```
Download the following [link](https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl.AppImage) to retrieve teh Q Ground Control App Image. Then run the following to turn the app image into a linux executable:
```
cd
chmod +x ./QGroundControl.AppImage
```
### PX4 ROS2 Workspace Installation
Ensure you installed the PX4 Development Environment before performing this step. In a new terminal, run the following:
```
cd
git clone https://github.com/LohitoBurrito/FireForce.git
mv FireForce/* . && rm -rf PX4_Drone_Recon
cd uav_app
chmod +x setup.sh
cd
sudo apt install ros-humble-cv-bridge
mv ./GazeboPackage/red_car ./PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/red_car
mv ./GazeboPackage/red_car ./PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/building
mv ./GazeboPackage/red_car ./PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/omni_light
mv ./GazeboPackage/red_car ./PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/hydrant
mv ./GazeboPackage/search_and_rescue.world ./PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/search_and_rescue.world
mv ./GazeboPackage/search_and_rescue.world ./PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/fire.world
mv ./GazeboPackage/sitl_targets_gazebo-classic.cmake ./PX4-Autopilot/src/modules/simulation/simulator_mavlink/sitl_targets_gazebo-classic.cmake
sudo rm -rf GazeboPackage
```
## Run Instructions
If you just performed installation, you can close all 4 terminals, and run 4 new Ubuntu 22.04.5 LTS terminals.
### Terminal 1
To start the Micro XRCE-DDS Agent, run:
```
cd Micro-XRCE-DDS-Agent/
MicroXRCEAgent udp4 -p 8888
```
### Terminal 2
To start QGC App Image, run the following:
```
./QGroundControl.AppImage
```
### Terminal 3
Start the Gazebo world by running the following. Note that if it says "gzserver not ready yet, trying again!," keep waiting. (NOTE: If this is your first time running the world, it may take a bit due to creating the build folder)
```
cd PX4-Autopilot/
make px4_sitl gazebo-classic_iris_depth_camera__fire
```
### Terminal 4
Run the simulation by executing the following commands (NOTE: If this is your first time running the simulation, it may take a bit due to creating the build, install, and log folder)
```
cd uav_app/
./setup.sh firforce
```
