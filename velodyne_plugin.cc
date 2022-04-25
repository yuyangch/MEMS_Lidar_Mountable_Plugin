#ifndef _VELODYNE_PLUGIN_HH_
#define _VELODYNE_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

namespace gazebo
{
  /// \brief A plugin to control a Velodyne sensor.
  class VelodynePlugin : public ModelPlugin
  {
    /// \brief Constructor
    public: VelodynePlugin() {}

    /// \brief The load function is called by Gazebo when the plugin is
    /// inserted into simulation
    /// \param[in] _model A pointer to the model that this plugin is
    /// attached to.
    /// \param[in] _sdf A pointer to the plugin's SDF element.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
    
	  // Safety check
	  if (_model->GetJointCount() == 0)
	  {
		std::cerr << "Invalid joint count, Velodyne plugin not loaded\n";
		return;
	  }

	  // Store the model pointer for convenience.
	  this->model = _model;

	  // Get the first joint. We are making an assumption about the model
	  // having one joint that is the rotational joint.
	  this->joint1 = _model->GetJoints()[0];
	  this->joint2 = _model->GetJoints()[1];
	  // Setup a P-controller, with a gain of 0.1.
	  //this->pid = common::PID(.0001, 0, 0.5);

	  // Apply the P-controller to the joint.

	  //this->model->GetJointController()->SetPositionPID(
	//	  this->joint->GetScopedName(), this->pid);
	  this->model->GetJointController()->SetJointPosition(
		  this->joint1->GetScopedName(), 0.0);
	  this->model->GetJointController()->SetJointPosition(
		  this->joint2->GetScopedName(), 0.0);
	  // Set the joint's target velocity. This target velocity is just
	  // for demonstration purposes.
	  //this->model->GetJointController()->SetPositionTarget(
		//  this->joint->GetScopedName(), 1);
    
      // Just output a message for now
      std::cerr << "\nThe velodyne plugin is attach to model[" <<
        _model->GetName() << "]\n";
        
        
       
    }
	/// \brief Pointer to the model.
	private: physics::ModelPtr model;

	/// \brief Pointer to the joint.
	private: physics::JointPtr joint1;
	private: physics::JointPtr joint2;
	/// \brief A PID controller for the joint.
	private: common::PID pid;


  };

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(VelodynePlugin)
}
#endif
