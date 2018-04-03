#ifndef GENERIC_CONTROLLER_PLUGIN_H
#define GENERIC_CONTROLLER_PLUGIN_H

#include <vector>
#include <string>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/opensim/OpensimModel.hh>
#include <gazebo/physics/opensim/OpensimMuscle.hh>
#include <gazebo/physics/opensim/OpensimJoint.hh>
#include <gazebo/physics/opensim/OpensimLink.hh>
#include <gazebo/gazebo.hh>

#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/transport/Node.hh>
#include <gazebo/transport/Publisher.hh>

// Message headers are found in the install directory.
#include <std_msgs/Float64.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/JointState.h>
#include <gazebo_ros_muscle_interface/MuscleStates.h>
#include <gazebo_ros_muscle_interface/GetMuscleActivations.h>
#include <gazebo_ros_muscle_interface/SetMuscleActivations.h>
#include <gazebo_ros_muscle_interface/GetMuscleStates.h>

#include <ros/ros.h>

#include <mutex>

namespace gazebo
{

using namespace gazebo_ros_muscle_interface;

using physics::OpensimMusclePtr;
using physics::OpensimModelPtr;


class MuscleInterfacePlugin : public ModelPlugin
{
public:
    MuscleInterfacePlugin();
    ~MuscleInterfacePlugin();

  // Load the plugin and initilize all controllers
  void Load(physics::ModelPtr parent, sdf::ElementPtr sdf) override;

  void Init() override;

  void Reset() override;

  // Simulation update callback function
  void OnUpdateEnd(/*const common::UpdateInfo &/*_info*/);

private:
  // Generic position command callback function (ROS topic)
  void activationCB(const std_msgs::Float64::ConstPtr &msg, int index);

  bool getActivationsCB(GetMuscleActivations::Request &req,
                        GetMuscleActivations::Response &ret);

  bool setActivationsCB(SetMuscleActivations::Request &req,
                        SetMuscleActivations::Response &ret);

  bool getMuscleStatesCB(GetMuscleStates::Request &req,
                         GetMuscleStates::Response &ret);

  // ROS node handle
  std::unique_ptr<ros::NodeHandle> rosNode;

  // Pointer to the model
  physics::OpensimModelPtr m_model;

  /// \brief Pointer to the physics instance
  physics::OpensimPhysicsPtr m_engine;

  // Pointer to the update event connection
  event::ConnectionPtr m_updateConnection;

  // ROS subscriber for individual muscle control topics
  std::vector<ros::Subscriber> m_activation_sub_vec;

  // ROS muscle state publisher
  private: ros::Publisher m_muscle_states_pub, m_joint_pub;

  // ROS muscle state message cache.
  // Used to answer service callbacks and in transmission of topic messages.
  private: MuscleStates m_muscle_states_msg;

  // ROS Accessor services
  private: ros::ServiceServer getMuscleActivationsService;

  // ROS Accessor services
  private: ros::ServiceServer setMuscleActivationsService;

  // ROS Accessor services
  private: ros::ServiceServer getMuscleStatesService;

  // Get data from gazebo and fill m_msucle_states_msg with it.
  private: void FillStateMessage();
};

} // namespace gazebo

#endif
