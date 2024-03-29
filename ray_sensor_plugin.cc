#ifndef MEMSLIDAR_SENSOR_PLUGIN
#define MEMSLIDAR_SENSOR_PLUGIN

// Include Gazebo headers.

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/plugins/RayPlugin.hh>
#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/sensors/RaySensor.hh>

// Include ROS headers so we can communicate with our robot
#include <ros/ros.h>

// Include std::string's because they're pretty darn useful.
#include <string>

// So we don't have to do gazebo::everything
namespace gazebo
{
  // Defining our plugin class
  class MemslidarSensorPlugin  : public RayPlugin
  {
    private:
    sensors::RaySensorPtr _sensor;
    private:
    physics::LinkPtr _l;
    public:
    MemslidarSensorPlugin() {}

    // Runs when the model is loaded
    virtual void Load(sensors::SensorPtr _s, sdf::ElementPtr _sdf)
    {
       if(!ros::isInitialized)
       {
         ROS_FATAL_STREAM("ROS node for Gazebo not established. Plugin failed.");
         return;
       }

       RayPlugin::Load(_s, _sdf);

       _sensor = std::dynamic_pointer_cast<sensors::RaySensor>(_s);
       std::string parentName = _sensor->ParentName();
       std::string modelName = "";
       std::string linkName = "";

       modelName = parentName.substr(0, parentName.find(':'));
       linkName = parentName.substr(parentName.find(':') + 2, parentName.size());

       ROS_INFO("parentName:%s",parentName.c_str());
       ROS_INFO("modelName:%s",modelName.c_str());
       ROS_INFO("linkName:%s",linkName.c_str());

       physics::WorldPtr _w = physics::get_world(_sensor->WorldName());
       physics::ModelPtr _m = _w->ModelByName(modelName.c_str());
       _l = _m->GetLink(linkName);

       ROS_INFO("Laser Sensor Plugin Loaded");
       //std::cout<<"Laser Sensor Plugin Loaded"<<std::endl;
   }

    virtual void OnNewLaserScans()
    {
      std::string out = "";
      std::vector<double> ranges;
      _sensor->Ranges(ranges);

      double yaw = _sensor->Pose().Rot().Yaw() + _l->RelativePose().Rot().Yaw();


      ROS_INFO("[sensor yaw %f]", _sensor->Pose().Rot().Yaw());
      ROS_INFO("[link relative yaw %f]", _l->RelativePose().Rot().Yaw()*180.0/3.14159265);
      ROS_INFO("[link relative pitch %f]", _l->RelativePose().Rot().Roll()*180.0/3.14159265);
      ROS_INFO("[link world yaw %f]", _l->WorldPose().Rot().Yaw()*180.0/3.14159265);
      ROS_INFO("[link world pitch %f]", _l->WorldPose().Rot().Roll()*180.0/3.14159265);
      for(int i = 0; i < ranges.size(); i++)
      {
        out = out + "[";
        out = out + std::to_string(i);
        out = out + "] ";
        out = out + std::to_string(ranges[i]);
        out = out + ", ";
      }
      


      ROS_INFO("[%f] %s", yaw,out.c_str()); 
    };





   };
  

  // Gazebo macro to set up the rest of the plugin functionality
  GZ_REGISTER_SENSOR_PLUGIN(MemslidarSensorPlugin)
}

#endif