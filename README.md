

# Step 1 MEMS_Lidar_Gazebo_Plugin 

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
git clone https://github.com/yuyangch/MEMS_Lidar_Mountable_Plugin.git
```


add the sourcing lines to .bashrc  :
```bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
```
```bash
echo "export GAZEBO_MODEL_PATH=${HOME}/MEMS_Lidar_Mountable_Plugin:${GAZEBO_MODEL_PATH}" >> ~/.bashrc
echo "export GAZEBO_PLUGIN_PATH=${HOME}/MEMS_Lidar_Mountable_Plugin/build:${GAZEBO_PLUGIN_PATH}" >> ~/.bashrc
echo "export GAZEBO_RESOURCE_PATH=${HOME}/MEMS_Lidar_Mountable_Plugin:${GAZEBO_RESOURCE_PATH}" >> ~/.bashrc
```
configure and run (standard cmake commands)
```
cd ~/MEMS_Lidar_Mountable_Plugin
./config.sh
./build.sh

```




## Usage
in ~/MEMS_Lidar_Mountable_Plugin
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




To change 2-axis input angular noise standard deviation(refer to world file line 26)
```
      <plugin name='velodyne_control' filename='libvelodyne_plugin.so'>
        <GaussianNoise>0.0</GaussianNoise>
      </plugin>
```
To change 2-axis input delay in ms (refer to world file line 26)
```
      <plugin name='velodyne_control' filename='libvelodyne_plugin.so'>
        <Delay_ms>1000</Delay_ms>
      </plugin>
```
To change lidar ray range measurement noise change this term `<stddev>0.008</stddev>`
```
          <ray>
            <noise>
              <type>gaussian</type>
              <mean>0</mean>
              <stddev>0.008</stddev>
            </noise>
            <scan>
```

## Develop Code

The current update rate is 100Hz both in dot switching(velodyne_plugin.cc:line 196) and lidar update 



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

Azimuth/Elevation angle input limit: no limite (this input changes principle axis direction)

Frequency of Rastor Z scanning:100 Hz (can be 1000Hz at simulation real time factor 1)

Max Frequency of  Azimuth/Elevation angle input change:1000Hz

Resolution: 20x20, can be adjusted up,  but points/min won't change, althought this is at 1000 points/second in realtime factor 1. can be higher if lower simulation realtime factor, for example, 2000 points/second at realtime factor .5 

Range: unlimited currently
```




# Step 2 Install PX4-Gazebo Simulation Environment
This is tested with Ubuntu 20.04, ROS noetic

1.
```
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
```
2.
```
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
```
3. 
```
cd /path/to/PX4-Autopilot
make px4_sitl gazebo-classic
```
4. change world file

![alt text](https://github.com/yuyangch/MEMS_Lidar_Mountable_Plugin/blob/main/lidar_mounted_drone.png?raw=true)
copy world file 

empty_with_houses_v4.world
into 

~/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/

5. 
copy rviz config file 
dual_gt_estmated_odom_rviz.rviz
into 
~/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic


add rviz node 
line 6 of ~/PX4-Autopilot/launch/mavros_posix_sitl.launch
<node type="rviz" name="rviz" pkg="rviz" args="-d $(find mavlink_sitl_gazebo)/dual_gt_estmated_odom_rviz.rviz" />


![alt text](https://github.com/yuyangch/MEMS_Lidar_Mountable_Plugin/blob/main/rviz.png?raw=true)

6. 


install QGroundControl daily builds
https://docs.qgroundcontrol.com/master/en/releases/daily_builds.html

chmod +x QGroundControl.AppImage 
./QGroundControl.AppImage 

open flightplan diamond_flight_plan.plan in the "plan" interface
![alt text](https://github.com/yuyangch/MEMS_Lidar_Mountable_Plugin/blob/main/image1.png?raw=true)
7.
```
cd <PX4-Autopilot_clone>
DONT_RUN=1 make px4_sitl_default gazebo-classic
source Tools/simulation/gazebo-classic/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/simulation/gazebo-classic/sitl_gazebo-classic
roslaunch px4 mavros_posix_sitl.launch
```

8.
record flight data for modified LIO-SAM to process later
```
rosbag record -O flight63_diamondpattern_houses_angular_input_3_degree_360FOV_range_noise_0point008_inputnoise_0point3_delay_120ms.bag /body_pose_ground_truth /clock /gazebo_ros_imu /libgazebo_ros_velodyne_laser /MEMS_rotation
```
or record LIOSAM output
```
rosbag record -O FOV360_range_noise_0point08.bag /lio_sam/mapping/cloud_registered /lio_sam/feature/cloud_corner /lio_sam/feature/cloud_surface /lio_sam/deskew/cloud_deskewed /tf /lio_sam/mapping/odometry /odometry/imu /body_pose_ground_truth /clock 
```

9. to add noise when running LIO-SAM


```
cd ~/
git clone git@github.com:yuyangch/ros_node_add_imu_noise.git
cd ./ros_node_add_imu_noise
mkdir build
cd ./build
cmake ..
make
cd ~/ros_node_add_imu_noise/build/devel/lib/add_noise_to_imu_msgs
./add_noise_to_imu_msgs_node
```




# STEP3: install modified LIO-SAM
go to  https://github.com/yuyangch/Motion_Compensated_LIO-SAM

you can modified weather to motion compensate or not in the last line of 
```
./config/params.yaml
```

you can publish random joint input angular commands with scripts in 
```
./scripts/rotation_publisher.py
```



# Cite this work 
if you use this code base please cite
```
@article{chen2023design,
  title={Design of an Adaptive Lightweight LiDAR to Decouple Robot-Camera Geometry},
  author={Chen, Yuyang and Wang, Dingkang and Thomas, Lenworth and Dantu, Karthik and Koppal, Sanjeev J},
  journal={arXiv preprint arXiv:2302.14334},
  year={2023}
}

```