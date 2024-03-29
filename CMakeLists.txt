cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

#find_package(roscpp REQUIRED)
#find_package(std_msgs REQUIRED)
#include_directories(${roscpp_INCLUDE_DIRS})
#include_directories(${std_msgs_INCLUDE_DIRS})

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  tf
  gazebo_ros
)
# Find Gazebo
find_package(gazebo REQUIRED)
include_directories(include ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

# Build our plugin
add_library(velodyne_plugin SHARED velodyne_plugin.cc)
target_link_libraries(velodyne_plugin ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES})

#add_library(velodyne_plugin SHARED velodyne_plugin.cc )
#target_link_libraries(velodyne_plugin ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES})

add_library(memslidar_sensor_plugin SHARED ray_sensor_plugin.cc)
target_link_libraries(memslidar_sensor_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES} RayPlugin)

add_library(gazebo_ros_velodyne_laser src/GazeboRosVelodyneLaser.cpp)
target_link_libraries(gazebo_ros_velodyne_laser
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  RayPlugin
)

add_library(gazebo_ros_velodyne_gpu_laser src/GazeboRosVelodyneLaser.cpp)
target_link_libraries(gazebo_ros_velodyne_gpu_laser
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  GpuRayPlugin
)



# Build the stand-alone test program
add_executable(vel vel.cc)

if (${gazebo_VERSION_MAJOR} LESS 6)
  # These two
  include(FindBoost)
  find_package(Boost ${MIN_BOOST_VERSION} REQUIRED system filesystem regex)
  target_link_libraries(vel ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
else()
  target_link_libraries(vel ${GAZEBO_LIBRARIES})
endif()
