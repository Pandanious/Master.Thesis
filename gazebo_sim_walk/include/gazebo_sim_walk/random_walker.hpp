#pragma once

// Ros
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

// Ignition
#include <ignition/math.hh>

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

#include "rcsim_task_carry_my_luggage/commands.hpp"

class GAZEBO_VISIBLE RandomWalker : public gazebo::ModelPlugin
{
public:
  /// \brief constructor
  RandomWalker();

  /// \brief load the controller
  void Load(gazebo::physics::ModelPtr parent, sdf::ElementPtr sdf) override;

  /// \brief Documentation Inherited.
  void Reset() override;

private:
  std::vector<ignition::math::Vector3d> waypoints_;

  /// \brief Function that is called every update cycle.
  /// \param[in] _info Timing information
  void OnUpdate(const gazebo::common::UpdateInfo& info);

  void ReadNextWayPoint();
  void ResetStartPosition();

  /// \brief Pointer to the parent actor.
  gazebo::physics::ActorPtr actor_;

  /// \brief Pointer to the world, for convenience.
  gazebo::physics::WorldPtr world_;

  // v in m/s
  double velocity_;

  // vv in rad/s
  double rotation_velocity_{ IGN_DTOR(180) };

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

  /// \brief Time of the last update.
  gazebo::common::Time lastUpdate_;

  /// \brief Custom trajectory info.
  gazebo::physics::TrajectoryInfoPtr trajectoryInfo_;

  /// \brief Mark current waypoint
  int waypoint_marker_{ -1 };
};
