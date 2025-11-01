#pragma once

// Ros
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

// Ignition
#include <ignition/math.hh>
#include <ignition/math/Quaternion.hh>
// Gazebo
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include "gazebo/util/system.hh"

#include <gazebo_plugins/gazebo_ros_utils.h>

#include <gazebo/msgs/pose_trajectory.pb.h>

#include <gazebo/gazebo.hh>
#include <gazebo/common/Animation.hh>

// STD
#include <string>
#include <vector>
#include <functional>
#include <unordered_map>

// coordinates string operation
#include <sstream>

// service request, response
#include <std_srvs/SetBool.h>
#include <std_msgs/String.h>

///#include "rcsim_task_carry_my_luggage/commands.hpp"

class GAZEBO_VISIBLE Operator : public gazebo::ModelPlugin
{
public:
  /// \brief constructor
  Operator();

  /// \brief load the controller
  void Load(gazebo::physics::ModelPtr parent, sdf::ElementPtr sdf) override;
  /// \brief Documentation Inherited.
  void Reset() override;

  /// \brief Reset operator animation.
 // virtual void ResetAnimation();

 // void CbCommands(const boost::shared_ptr<const gazebo::msgs::Any>& msg);

private:
  void CbGui(const boost::shared_ptr<const gazebo::msgs::Any>& msg);
  void startanim();
  void resetmodel();
  
  // vv in rad/s
  double rotation_velocity_{ IGN_DTOR(180) };
  // v in m/s
  double velocity_;

  gazebo::transport::NodePtr node_;
  gazebo::transport::SubscriberPtr sub_;
  ros::Publisher Pub_Eval;
  std::vector<ignition::math::Vector3d> waypoints_;

  /// \brief Function that is called every update cycle.
  /// \param[in] _info Timing information
  void OnUpdate(const gazebo::common::UpdateInfo& info);

  /// \brief Service to toggle the waving animation
  // private: bool ToggleWaveAnimation(rcsim_task_carry_my_luggage_actor_msgs::ToggleActorWaving::Request &req,
  // rcsim_task_carry_my_luggage_actor_msgs::ToggleActorWaving::Response &res);
   /// \brief Pointer to the parent actor.
  gazebo::physics::ActorPtr actor_;

  /// \brief Pointer to the world, for convenience.
  gazebo::physics::WorldPtr world_;

  /// \brief Pointer to the sdf element.
  sdf::ElementPtr sdf_;

  ignition::math::Pose3d modelpose;

  /// \brief List of connections
  std::vector<gazebo::event::ConnectionPtr> connections_;

  /// \brief Current target location
  ignition::math::Vector3d target_;

  /// \brief Last received linear vel command.
  ignition::math::Vector3d last_linear_;

  /// \brief Last received angle command.
  ignition::math::Angle last_angle_;

  /// \brief Time scaling factor. Used to coordinate translational motion
  /// with the actor's walking animation.
  double animationFactor_ = 4.5;

  /// \brief Z-axes offset for the operator model.
  double z_offset_;

  /// \brief Robot ID
  std::string id_robot_;

  /// \brief Time of the last update.
  gazebo::common::Time lastUpdate_;

  /// \brief Custom trajectory info.
  gazebo::physics::TrajectoryInfoPtr trajectoryInfo_;

  /// \brief Change waypoint following velocity
 // void IncreaseVelocity();
  /// \brief Change waypoint following velocity
 // void DecreaseVelocity();

  /// \brief Reset start position to first waypoint
  void ResetStartPosition();

  /// \brief Start waving animation
 // void StartWavingAnimation();

  /// \brief Mark current waypoint
  int waypoint_marker_{ -1 };

  /// \brief Stop waypoint following
  bool stop_Waypoint_Following_;

  /// \brief ros::ServiceServer wave_toggle_service_;
  bool wave_toggled_;

  void ReadNextWayPoint();
  bool started_{ false };
};
