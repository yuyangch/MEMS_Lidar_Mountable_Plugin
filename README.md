# MEMS_Lidar_Gazebo_Plugin 

This code is a combination of several parts. 
1) lidar model is based on this tutorial: https://classic.gazebosim.org/tutorials?cat=guided_i&tut=guided_i2 
2) lidar motion controller (velodyne_plugin.cc)software is based on this tutorial https://www.cplusgears.com/lesson-5-adding-a-lidar.html

3) lidar motion controller movement pattern (Raster Z) is based on Dingkang's Arduino code

4) lidar sensor ray data piping is an adaptation from https://bitbucket.org/DataspeedInc/velodyne_simulator/src/master/

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
git clone https://github.com/droneslab/MEMS_Lidar_Gazebo_Plugin.git
```


add the sourcing lines to .bashrc  :
```bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
```
```bash
echo "export GAZEBO_MODEL_PATH=${HOME}/MEMS_Lidar_Gazebo_Plugin:${GAZEBO_MODEL_PATH}" >> ~/.bashrc
echo "export GAZEBO_PLUGIN_PATH=${HOME}/MEMS_Lidar_Gazebo_Plugin/build:${GAZEBO_PLUGIN_PATH}" >> ~/.bashrc
echo "export GAZEBO_RESOURCE_PATH=${HOME}/MEMS_Lidar_Gazebo_Plugin:${GAZEBO_RESOURCE_PATH}" >> ~/.bashrc
```
configure and run (standard cmake commands)
```
cd ~/MEMS_Lidar_Gazebo_Plugin
./config.sh
./build.sh

```


## Usage
in ~/MEMS_Lidar_Gazebo_Plugin
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
~/MEMS_Lidar_Gazebo_Plugin/velodyne_plugin.cc
```
The Lidar Ray Sensor Plugin
```
~/MEMS_Lidar_Gazebo_Plugin/src/GazeboRosVelodyneLaser.cpp
~/MEMS_Lidar_Gazebo_Plugin/include/velodyne_gazebo_plugins/GazeboRosVelodyneLaser.h
```

The world file
```
velodune.world
```

The lidar model file (change min/max scanning angle, number of samples horizontal/vertical)

```
~/MEMS_Lidar_Gazebo_Plugin/velodyne_hdl15/model.sdf
```

The cmake file is worth looking at as well, in case if you need to add your own libraries

```
~/MEMS_Lidar_Gazebo_Plugin/CMakeLists.txt
```

## Parameters
### Real Lidar
```
FOV:+-4.5 degree horizontal/Vertical

Azimuth/Elevation angle input limit: 4-6 degree (this input changes principle axis direction)

Frequency of Rastor Z scanning:100 Hz (need to verify)

Max Frequency of  Azimuth/Elevation angle input change:50 Hz

Resolution 20x20, can be adjusted up,  but points/second won't change, which slows down update rate. Need to verify points/second

Range (Distance): 2-2.6 Meters tested most
```
### Simulated Gazebo/ROS Lidar (All parameters adjustable)
```
FOV:+-4.5 degree horizontal/Vertical

Azimuth/Elevation angle input limit: 4-6 degree (this input changes principle axis direction)

Frequency of Rastor Z scanning:100 Hz 

Max Frequency of  Azimuth/Elevation angle input change:1000Hz

Resolution: 20x20, can be adjusted up,  but points/min won't change, althought this is at 1000 points/second in realtime factor 1. can be higher if lower simulation realtime factor, for example, 2000 points/second at realtime factor .5 

Range: unlimited currently
```

