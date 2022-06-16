# lidarsim_gazebo

This code is a combination of several parts. 
1) lidar model is based on this tutorial: https://classic.gazebosim.org/tutorials?cat=guided_i&tut=guided_i2 
2) lidar motion controller (velodyne_plugin.cc)is designed to mimic that of the MEMS Lidar (Rastor Z scan pattern). 
3) lidar sensor ray data piping is an adaptation from https://bitbucket.org/DataspeedInc/velodyne_simulator/src/master/

For a youtube video usage demonstration click the image below:

<a href="https://youtu.be/hbRFwD9XrIw
" target="_blank"><img src="https://img.youtube.com/vi/hbRFwD9XrIw/0.jpg" 
alt="A video demonstration of mems lidar plugin" width="1028" height="720" border="10" /></a>


## Installation


This is tested with Ubuntu 20.04, ROS noetic, Gazebo 11



Install ROS noetic "Desktop Full Install"
http://wiki.ros.org/noetic/Installation/Ubuntu

Install Gazebo 
https://classic.gazebosim.org/tutorials?tut=install_ubuntu


```
cd ~/
git clone https://github.com/yuyangch/lidarsim_gazebo.git
```


add the sourcing lines to .bashrc  :
```bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
```
```bash
echo "export GAZEBO_MODEL_PATH=${HOME}/lidarsim_gazebo:${GAZEBO_MODEL_PATH}" >> ~/.bashrc
echo "export GAZEBO_PLUGIN_PATH=${HOME}/lidarsim_gazebo/build:${GAZEBO_PLUGIN_PATH}" >> ~/.bashrc
echo "export GAZEBO_RESOURCE_PATH=${HOME}/lidarsim_gazebo:${GAZEBO_RESOURCE_PATH}" >> ~/.bashrc
```
configure and run (standard cmake commands)
```
cd ~/lidarsim_gazebo
./config.sh
./build.sh

```


## Usage
in ~/lidarsim_gazebo
```
roslaunch l_gz.launch
```

The pointcloud output is in Pointcloud2 format at topic 
```
/libgazebo_ros_velodyne_laser

```
the dataformat is 
```
  msg.fields[0].name = "x";
  msg.fields[0].offset = 0;
  msg.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
  msg.fields[0].count = 1;
  msg.fields[1].name = "y";
  msg.fields[1].offset = 4;
  msg.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
  msg.fields[1].count = 1;
  msg.fields[2].name = "z";
  msg.fields[2].offset = 8;
  msg.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
  msg.fields[2].count = 1;
  msg.fields[3].name = "intensity";
  msg.fields[3].offset = 12;
  msg.fields[3].datatype = sensor_msgs::PointField::FLOAT32;
  msg.fields[3].count = 1;
  msg.fields[4].name = "ring";
  msg.fields[4].offset = 16;
  msg.fields[4].datatype = sensor_msgs::PointField::UINT16;
  msg.fields[4].count = 1;
  msg.fields[5].name = "time";
  msg.fields[5].offset = 18;
  msg.fields[5].datatype = sensor_msgs::PointField::FLOAT32;
  msg.fields[5].count = 1;
```

To dynamically change sensor azimuth/elevation center, 
```
cd ./build
./vel <azimuth> <elevation>
```
The example code is in vel.cc, you can modify it 


## Develop Code

The current update rate is 100Hz both in dot switching(velodyne_plugin.cc:line 140) and lidar update 



The main files to modify is:
The Lidar Controller Plugin  
```
~/lidarsim_gazebo/velodyne_plugin.cc
```
The Lidar Ray Sensor Plugin
```
~/lidarsim_gazebo/src/GazeboRosVelodyneLaser.cpp
~/lidarsim_gazebo/include/velodyne_gazebo_plugins/GazeboRosVelodyneLaser.h
```

The world file
```
velodune.world
```

The lidar model file (change min/max scanning angle, number of samples horizontal/vertical)

```
~/lidarsim_gazebo/velodyne_hdl15/model.sdf
```

The cmake file is worth looking at as well, in case if you need to add your own libraries

```
~/lidarsim_gazebo/CMakeLists.txt
```