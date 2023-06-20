//This file relates to lidar universal joint/mems angular control
//generate_lidar_scanning_pattern() generates scanning patern for update_current_rastor_z_scan(), which updates the next grid target
//but the above behavior is only suitable for single beam MEMS configuration
//the raztor z grid is currently set to a single point, for 360 scanning lidar configuration
//    private: float angular_noise_std; controls the input noise
// needs variable to decide control delay
// to change range measurement noise, look at GazeboRosVelodyneLaser.cpp instead. check gaussian_noise_ variable
//

// the 


#ifndef _VELODYNE_PLUGIN_HH_
#define _VELODYNE_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "std_msgs/Float32MultiArray.h"
#include <cmath>
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

      //Generate full lidar scanning pattern
      this->generate_lidar_scanning_pattern();
      
      // max. pixels
      /*
        for (i = 0; i < px; i++)
        {
          thxa[i] = thx_bot + i * (thx_top - thx_bot) / (px - 1);
          thxa[2 * px - i - 1] = thxa[i] ;
        }
        for (i = 0; i < py; i++)
        {
          thya[i] = thy_bot + i * (thy_top - thy_bot) / (py - 1);
          thya[2 * py - i - 1] = thya[i] ;
        }
      */
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

      //std::cout<<"0,size of vecotr "<< (_model->GetJoints()).size()<<"get joint count"<<_model->GetJointCount()<<"\n";
      //std::cout<<"model get name is "<<this->model->GetName()<<std::endl;
      //std::cout<<"sdf get name is "<<_sdf->GetName()<<std::endl;


      std::string jointName =  this->model->GetName() +"::velodyne_hdl_15::"+ "elevation_joint";
      
      this->joint = _model->GetJoint(jointName);
      
	     //std::cout<<"1\n";
	     //std::cout<<this->joint->GetName()<<std::endl;
	     
      // Setup a P-controller, with a gain of 0.1.
      this->pid = common::PID( 288, 5, .01);

      // Apply the P-controller to the joint.
      this->model->GetJointController()->SetPositionPID(
          this->joint->GetScopedName(), this->pid);

      // Default to zero velocity
      double position = 0.0;

      // Check that the velocity element exists, then read the value
      if (_sdf->HasElement("position"))
        position = _sdf->Get<double>("position");

      if (_sdf->HasElement("GaussianNoise"))
        angular_noise_std = _sdf->Get<double>("GaussianNoise");
      else
        std::cout<<"lidar controller plugin lacks GaussianNoise parameter, defaults to 0.0"<<std::endl;
      distribution= new std::normal_distribution<double>(0,angular_noise_std);
      std::cout<<"distribution mean and stddev "<<distribution->mean()<<" "<<distribution->stddev()<<std::endl;
 
      if (_sdf->HasElement("Delay_ms"))
        delay = _sdf->Get<int>("Delay_ms");

      this->SetPosition(0.0,0.0);

      // Create the node
      this->node = transport::NodePtr(new transport::Node());
      #if GAZEBO_MAJOR_VERSION < 8
      this->node->Init(this->model->GetWorld()->GetName());
      #else
      this->node->Init(this->model->GetWorld()->Name());
      #endif

      // Create a topic name
      std::string topicName = "~/" + this->model->GetName() + "/vel_cmd";

      // Subscribe to the topic, and register a callback
      this->sub = this->node->Subscribe(topicName,
         &VelodynePlugin::OnMsg, this);
    // Initialize ros, if it has not already bee initialized.
		if (!ros::isInitialized())
		{
		  int argc = 0;
		  char **argv = NULL;
		  ros::init(argc, argv, "gazebo_client",
			  ros::init_options::NoSigintHandler);
		}

		// Create our ROS node. This acts in a similar manner to
		// the Gazebo node
		this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

		// Create a named topic, and subscribe to it.
		ros::SubscribeOptions so =
		  ros::SubscribeOptions::create<geometry_msgs::Vector3>(
			  "/" + this->model->GetName() + "/vel_cmd",
			  1,
			  boost::bind(&VelodynePlugin::OnRosMsg, this, _1),
			  ros::VoidPtr(), &this->rosQueue);
		this->rosSub = this->rosNode->subscribe(so);

    this->rosPub = this->rosNode->advertise<geometry_msgs::Vector3Stamped>("/MEMS_rotation", 1); 



		// Spin up the queue helper thread.
		this->rosQueueThread =
		  std::thread(std::bind(&VelodynePlugin::QueueThread, this));

    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&VelodynePlugin::OnUpdate, this));



    }
	/// \brief Handle an incoming message from ROS
	/// \param[in] _msg A float value that is used to set the velocity
	/// of the Velodyne.
	public: void OnRosMsg(const geometry_msgs::Vector3ConstPtr &_msg)
	{
           azimuth_control_input_setpoint=_msg->z;
           elevation_control_input_setpoint=_msg->y;
		       azimuth_control_input=_msg->z+(*distribution)(generator);
		       elevation_control_input=_msg->y+(*distribution)(generator);
           std::tuple<float,float,int> delay_queue_tuple;
           float time =this->model->GetWorld()->SimTime ().Float();

           int time_ms=static_cast<int>(time*1000);

           delay_queue_tuple=std::make_tuple(azimuth_control_input,elevation_control_input,time_ms);
           this->delay_queue.push(delay_queue_tuple);
            // std::tuple<int,std::string> Three_d_value_tuple;
            // two_d_key_tuple=std::make_tuple(cam_index,frame,gait_name);
            // Three_d_value_tuple=std::make_tuple(frame,gait_name);


	}

  // Called by the world update start event
  public: void OnUpdate()
  {

    // update position every 100 milliseconds
    // 100 Hz MEMS MIRROR/Joint poistioin update
     float time =this->model->GetWorld()->SimTime ().Float();
     int time_ms=static_cast<int>(time*1000);
     // if (!delay_mode){
      if (time_ms%10==0){
        this->update_current_rastor_z_scan();
        std::cout<<"azimuth_control_input elevation_control_input"<< azimuth_control_input<<" "<<elevation_control_input<<std::endl;
        std::cout<<"azimuth_control_input elevation_control_input setpoints"<< azimuth_control_input_setpoint<<" "<<elevation_control_input_setpoint<<std::endl;
        std::cout<<"noise level "<<angular_noise_std<<std::endl;
        std::cout<<"distrubtion mean and std"<<distribution->mean()<<" "<<distribution->stddev()<<std::endl;
        std::cout<<"Delay_ms"<<delay<<std::endl;
        //std::get<0>(Three_d_value_tuple);
        while(!this->delay_queue.empty()){
          int delay_queue_front_time=std::get<2>(this->delay_queue.front());
          if (time_ms>(delay_queue_front_time+this->delay)){
            float azimuth_control_delay_input=std::get<0>(this->delay_queue.front());
            float elevation_control_delay_input=std::get<1>(this->delay_queue.front());
            this->SetPosition((thx+azimuth_control_delay_input)*M_PI/180.0,(thy+elevation_control_delay_input)*M_PI/180.0);
            this->delay_queue.pop();
          }else{
            break;
          }
        }
        geometry_msgs::Vector3Stamped v3s;
        
        v3s.header.stamp=ros::Time::now();
        v3s.vector.y=elevation_control_input_setpoint;
        v3s.vector.z=azimuth_control_input_setpoint;
        this->rosPub.publish(v3s);
     }
     // }else{

     // }

    //std::cout<<"time_ms is" <<time_ms<<std::endl;

    
  }
  //generate lidar full scanning pattern
  private: void generate_lidar_scanning_pattern()
  {
      //Generate full lidar scanning pattern
      // max. pixels
        for (i = 0; i < px; i++)
        {
          thxa[i] = thx_bot + i * (thx_top - thx_bot) / (px - 1);
          thxa[2 * px - i - 1] = thxa[i] ;
        }
        for (i = 0; i < py; i++)
        {
          thya[i] = thy_bot + i * (thy_top - thy_bot) / (py - 1);
          thya[2 * py - i - 1] = thya[i] ;
        }
  }
  private: void update_current_rastor_z_scan()
  {
  
      if (I >= px * 2 - 1) //update vx
        I = 0;
      else I++;

      if (I == 0 || I == px) //update vy
      {
        if (J >= py * 2 - 1) J = 0;
        else J++;
      }
      // Horizontal
      thx = thxa[I];
      thy = thya[J];
  }
	/// \brief ROS helper function that processes messages
	private: void QueueThread()
	{
	  static const double timeout = 0.01;
	  while (this->rosNode->ok())
	  {
		this->rosQueue.callAvailable(ros::WallDuration(timeout));
	  }
	}
    /// \brief Set the velocity of the Velodyne
    /// \param[in] _vel New target velocity
    public: void SetPosition(const double &_x,const double &_y)
    {
      // Set the joint's target velocity.

      this->joint->SetPosition(1,_x); //joint 1 is azimuth
      this->joint->SetPosition(0,_y); //joint 0 is elevation
      this->model->GetJointController()->SetPositionTarget(
          this->joint->GetScopedName(), _y );
      //this->joint->SetPosition(0,_x);
    }

    /// \brief Handle incoming message
    /// \param[in] _msg Repurpose a vector3 message. This function will
    /// only use the x ,ycomponent.
    private: void OnMsg(ConstVector3dPtr &_msg)
    {
       azimuth_control_input=_msg->x();
       elevation_control_input=_msg->y();
    }
    //update event pointer to interact with lidar 
    private: event::ConnectionPtr updateConnection;

    /// Lidar scanning pattern storage
    private: float thxa[1000];
    private: float thya[1000];
    //Lidar related variables, 
    private: float thx_top = 0.0;  //4.5  //horizontal max degree
    private: float thx_bot = -thx_top;
    private: float thy_top = thx_top; //vertical max degree
    private: float thy_bot = -thx_top;
    private: int px = 20; // {6by6, 6px6px, dly7}  //horizontal resolution
    private: int py = 20; // x2  //vertical resolution
    private: int i;     //rastor z scan generation index
    private: int j;
    private: int I = 0, J = 0; //rastor z scan update indexes
    private: float thx,thy;    //rastor z scan output
    private: float azimuth_control_input=0.0,elevation_control_input=0.0,azimuth_control_input_setpoint=0.0,elevation_control_input_setpoint=0.0;
    private: float angular_noise_std=0;
    private: std::default_random_engine generator;
    private: std::normal_distribution<double> *distribution;
    private: int delay=0; //in ms
    private: std::queue<std::tuple<float,float,int>> delay_queue;

    /// \brief A node used for transport
    private: transport::NodePtr node;

    /// \brief A subscriber to a named topic.
    private: transport::SubscriberPtr sub;




    /// \brief Pointer to the model.
    private: physics::ModelPtr model;

    /// \brief Pointer to the joint.
    private: physics::JointPtr joint;

    /// \brief A PID controller for the joint.
    private: common::PID pid;
  /// \brief A node use for ROS transport
	private: std::unique_ptr<ros::NodeHandle> rosNode;

	/// \brief A ROS subscriber
	private: ros::Subscriber rosSub;


	/// \brief A ROS callbackqueue that helps process messages
	private: ros::CallbackQueue rosQueue;

	/// \brief A thread the keeps running the rosQueue
	private: std::thread rosQueueThread;


  private: ros::Publisher rosPub;



  };

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(VelodynePlugin)
}
#endif
