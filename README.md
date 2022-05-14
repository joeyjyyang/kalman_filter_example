# kalman_filter_example

## Installation
### Install from Git Repository
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone --single-branch --branch kinetic-devel https://github.com/joeyjyyang/kalman_filter_example.git # Localization package
git clone --single-branch --branch kinetic-devel https://github.com/joeyjyyang/ros_dwm1001.git # UWB package
git clone --single-branch --branch kinetic-devel https://github.com/joeyjyyang/ros_bno055.git # IMU package
cd .. 
sudo apt-get install libi2c-dev # for I2C SMBUS functions
rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y
catkin_make
source devel/setup.bash
```

## Usage
### Example
#### Running the UWB, IMU, and Kalman Filter Together
```
roslaunch kalman_filter_example kalman_filter.launch
```

#### Running the Kalman Filter Alone
```
rosrun kalman_filter_example kalman_filter_node.py
```