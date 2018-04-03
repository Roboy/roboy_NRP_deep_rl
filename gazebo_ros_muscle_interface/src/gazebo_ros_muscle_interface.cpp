#include "gazebo_ros_muscle_interface.h"

#include <gazebo/physics/World.hh>
#include <gazebo/physics/opensim/OpensimPhysics.hh>

#include <boost/bind.hpp>
#include <ros/time.h>
#include <thread>

// Useful links:
// http://wiki.ros.org/roscpp/Overview
// http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29

namespace gazebo
{

//////////////////////////////////////////////////
MuscleInterfacePlugin::MuscleInterfacePlugin()
{
  //gzdbg << __FUNCTION__ << std::endl;
  rosNode = std::make_unique<ros::NodeHandle>("gazebo_muscle_interface");
}

//////////////////////////////////////////////////
MuscleInterfacePlugin::~MuscleInterfacePlugin()
{
  //gzdbg << __FUNCTION__ << std::endl;
  rosNode->shutdown();
}

//////////////////////////////////////////////////
void MuscleInterfacePlugin::Init()
{
  gazebo::ModelPlugin::Init();
  if (!m_model || !m_engine) return;

  const auto &muscles = m_model->GetMuscles();
  for (int k = 0; k < muscles.size(); ++k)
  {
    auto muscle = muscles[k];
    const std::string name = muscle->GetName();

    // generate topic name using the model name as prefix
    std::string topic_name = m_model->GetName() + "/" + name + "/cmd_activation";

    // Add ROS topic for activation control
    m_activation_sub_vec.push_back(
      rosNode->subscribe<std_msgs::Float64>(
        topic_name,
        1,
        boost::bind(&MuscleInterfacePlugin::activationCB, this, _1, k)));

    ROS_INFO("Added new topic (%s) for muscle %s (%p)", topic_name.c_str(), name.c_str(), muscle.get());
  }

  FillStateMessage();

  const std::string topic_name = m_model->GetName() + "/muscle_states";
  m_muscle_states_pub = rosNode->advertise<MuscleStates>( topic_name, 10 );

    m_joint_pub = rosNode->advertise<sensor_msgs::JointState>("/jointState", 10);

  getMuscleActivationsService = rosNode->advertiseService<
    GetMuscleActivations::Request,
    GetMuscleActivations::Response>(
      m_model->GetName() + "/get_activations",
      boost::bind(&MuscleInterfacePlugin::getActivationsCB, this, _1, _2)
  );
  ROS_INFO("ROS muscle interface: get_activations service installed!");

  setMuscleActivationsService = rosNode->advertiseService<
    SetMuscleActivations::Request,
    SetMuscleActivations::Response>(
      m_model->GetName() + "/set_activations",
      boost::bind(&MuscleInterfacePlugin::setActivationsCB, this, _1, _2)
  );
  ROS_INFO("ROS muscle interface: set_activations service installed!");

  getMuscleStatesService = rosNode->advertiseService<
    GetMuscleStates::Request,
    GetMuscleStates::Response>(
      m_model->GetName() + "/get_states",
      boost::bind(&MuscleInterfacePlugin::getMuscleStatesCB, this, _1, _2)
  );
  ROS_INFO("ROS muscle interface: get_states service installed!");

  // Register callback that sends muscle messages with gazebo server.
  m_updateConnection = event::Events::ConnectWorldUpdateEnd(boost::bind(&MuscleInterfacePlugin::OnUpdateEnd, this));

  //gzdbg << __FUNCTION__ << " in thread " << std::this_thread::get_id() << std::endl;
}
//////////////////////////////////////////////////
void MuscleInterfacePlugin::Reset()
{
}

//////////////////////////////////////////////////
void MuscleInterfacePlugin::Load(physics::ModelPtr parent, sdf::ElementPtr sdf)
{
  // Store the pointer to the model
  m_model = boost::dynamic_pointer_cast<physics::OpensimModel>(parent);
  if (m_model)
  {
    physics::WorldPtr world = m_model->GetWorld();
    if (world)
      m_engine = boost::dynamic_pointer_cast<physics::OpensimPhysics>(world->GetPhysicsEngine());
  }
  if (m_model == nullptr || m_engine == nullptr)
    gzwarn << "Muscle control plugin loaded without OpenSim physics plugin." << std::endl;
}

//////////////////////////////////////////////////
/* Called by the world update start event. Runs in same thread as
 * physics::UpdatePhysics after UpdatePhysics. Thus it returns most
 * up to date information.
*/
void MuscleInterfacePlugin::OnUpdateEnd(/*const common::UpdateInfo & _info*/)
{
  FillStateMessage();
  m_muscle_states_pub.publish ( m_muscle_states_msg );
  //physics::Joint_V joints = m_model->GetJoints();
  //for (auto joint:m_model->GetJoints()){
  //}

  sensor_msgs::JointState msg;
  msg.position.push_back(m_model->GetJoint("HumerusBoneJoint")->GetAngle(0).Radian());
  msg.position.push_back(m_model->GetJoint("RadiusBoneJoint")->GetAngle(0).Radian());
  msg.velocity.push_back(m_model->GetJoint("HumerusBoneJoint")->GetVelocity(0));
  msg.velocity.push_back(m_model->GetJoint("RadiusBoneJoint")->GetVelocity(0));

    
  m_joint_pub.publish(msg);
}

//////////////////////////////////////////////////
void MuscleInterfacePlugin::FillStateMessage()
{
  // Sync via global physics mutex. Needed because
  // we use the msg member to answer the "get states"
  // service.
  boost::recursive_mutex::scoped_lock lock(
    *m_engine->GetPhysicsUpdateMutex());

  MuscleStates &msg = m_muscle_states_msg;

  gazebo::common::Time curTime = this->m_model->GetWorld()->GetSimTime();
  msg.header.stamp.sec = curTime.sec;
  msg.header.stamp.nsec = curTime.nsec;

  std::vector<gazebo::math::Vector3> muscle_path;

  const physics::Muscle_V muscles = m_model->GetMuscles();
  msg.muscles.resize(muscles.size());
  for (int muscle_idx = 0; muscle_idx < muscles.size(); ++muscle_idx)
  {
    MuscleState &state_msg = msg.muscles[muscle_idx];
    const physics::OpensimMuscle& muscle = *muscles[muscle_idx];

    state_msg.name = muscle.GetName();
    state_msg.force = muscle.GetForce();
    state_msg.length = muscle.GetLength();
    state_msg.lengthening_speed = muscle.GetLengtheningSpeed();
    // TODO: Constant parameters should perhaps become available upon
    // request by a service rather than being transmitted every update.
    //state_msg.optimal_fiber_length = muscle.GetOptimalFiberLength();
    //state_msg.tendon_slack_length = muscle.GetTendonSlackLength();

    muscle.GetCurrentWorldPath(muscle_path);
    state_msg.path_points.resize(muscle_path.size());
    for (std::size_t i=0; i<muscle_path.size(); ++i)
    {
      gazebo::math::Vector3 p = muscle_path[i];
      state_msg.path_points[i].x = p.x;
      state_msg.path_points[i].y = p.y;
      state_msg.path_points[i].z = p.z;
    }
  }
}

//////////////////////////////////////// ROS topic callback functions //////////////////////////////////////////
void MuscleInterfacePlugin::activationCB(const std_msgs::Float64::ConstPtr &msg, int index)
{
  const physics::Muscle_V &muscles = m_model->GetMuscles();
  if (index < 0 || index >= muscles.size())
  {
    gzdbg << "Ignored attempt to set muscle activation by invalid index!" << std::endl;
    return;
  }

  // Protect setter function which is not thread safe at the moment.
  // We don't want to set the activation in a thread while the physics
  // update loop reads it for the next physics iteration.
  boost::recursive_mutex::scoped_lock lock(
    *m_engine->GetPhysicsUpdateMutex());

  muscles[index]->SetActivationValue(msg->data);
}

//////////////////////////////////////// ROS service callback functions //////////////////////////////////////////
bool MuscleInterfacePlugin::getActivationsCB(
  GetMuscleActivations::Request &req,
  GetMuscleActivations::Response &ret)
{
  const physics::Muscle_V &muscles = m_model->GetMuscles();
  ret.activations.resize(muscles.size());

  // Muscle members are not thread safe. This lock is
  // not super critical but still we want to avoid
  // setting and reading activations concurrently.
  boost::recursive_mutex::scoped_lock lock(
    *m_engine->GetPhysicsUpdateMutex());

  for (std::size_t i = 0; i < muscles.size(); ++i)
    ret.activations[i] = muscles[i]->GetActivationValue();

  ret.success = true;
  return true;
}

//////////////////////////////////////// ROS service callback functions //////////////////////////////////////////
bool MuscleInterfacePlugin::setActivationsCB(
  SetMuscleActivations::Request &req,
  SetMuscleActivations::Response &ret)
{
  const physics::Muscle_V &muscles = m_model->GetMuscles();

  if (req.activations.size() != muscles.size())
  {
    gzdbg << "Ignored attempt to set activations of " << muscles.size() << " muscles with an array of " << req.activations.size() << " values!" << std::endl;
    ret.success = false;
    return false;
  }

  // See above.
  boost::recursive_mutex::scoped_lock lock(
    *m_engine->GetPhysicsUpdateMutex());

  for (std::size_t i = 0; i < muscles.size(); ++i)
    muscles[i]->SetActivationValue(req.activations[i]);

  ret.success = true;
  return true;
}

//////////////////////////////////////// ROS service callback functions //////////////////////////////////////////
bool MuscleInterfacePlugin::getMuscleStatesCB(
  GetMuscleStates::Request &req,
  GetMuscleStates::Response &ret)
{
  boost::recursive_mutex::scoped_lock lock(
    *m_engine->GetPhysicsUpdateMutex());

  // Copy muscle state from the "cached" data to the response.
  MuscleStates &m = m_muscle_states_msg;
  ret.muscles.reserve(m.muscles.size());

  for (std::size_t i = 0; i < m.muscles.size(); ++i)
    ret.muscles.emplace_back(m.muscles[i]);

  return true;
}

////////////////////////////////////////

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(MuscleInterfacePlugin)

} // namespace gazebo
