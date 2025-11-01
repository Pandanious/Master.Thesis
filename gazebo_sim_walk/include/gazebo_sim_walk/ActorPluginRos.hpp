/*
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
/**
 *  \author David Leins, Patric Steckstor
 *  \date 28th of November 2018
 *  \desc Gazebo ros plugin to steer an actor with twist messages over ros. Based on actor_plugin.
 */

#ifndef GAZEBO_ROS_ACTOR_VEL_PLUGIN_HH
#define GAZEBO_ROS_ACTOR_VEL_PLUGIN_HH

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

// STD
#include <string>
#include <vector>
#include <functional>

// coordinates string operation
#include <sstream>

// service request, response
#include <std_srvs/SetBool.h>
#include <std_msgs/String.h>

enum Commands
{
  INVALID_CMD,
  START_CMD,
  STOP_CMD,
  TOGGLE_CMD,
  RESET_CMD,
  SLOWER_CMD,
  FASTER_CMD
};

namespace gazebo
{
class GAZEBO_VISIBLE ActorPluginRos : public ModelPlugin
{
public:
  /// \brief constructor
  ActorPluginRos();

  /// \brief load the controller
  void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

  /// \brief Documentation Inherited.
  virtual void Reset();

private:
  /// \brief Function that is called every update cycle.
  /// \param[in] _info Timing information
  void OnUpdate(const gazebo::common::UpdateInfo& _info);

  /// \brief Callback for newly received vel command
  void NewVelCmdCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg);

  /// \brief Callback for newly received operator command
  void OperatorCmdCallback(const std_msgs::String::ConstPtr& op_msg);

  /// \brief Service to toggle the waving animation
  // private: bool ToggleWaveAnimation(rcsim_task_carry_my_luggage_actor_msgs::ToggleActorWaving::Request &req,
  // rcsim_task_carry_my_luggage_actor_msgs::ToggleActorWaving::Response &res);

  /// \brief Pointer to the parent actor.
  physics::ActorPtr actor;

  /// \brief Pointer to the world, for convenience.
  physics::WorldPtr world;

  /// \brief Pointer to the sdf element.
  sdf::ElementPtr sdf;

  /// \brief Velocity of the actor
  ignition::math::Vector3d velocity;

  /// \brief List of connections
  std::vector<event::ConnectionPtr> connections;

  /// \brief Current target location
  ignition::math::Vector3d target;

  /// \brief Last received linear vel command.
  ignition::math::Vector3d last_linear;

  /// \brief Last received angle command.
  ignition::math::Angle last_angle;

  /// \brief Time scaling factor. Used to coordinate translational motion
  /// with the actor's walking animation.
  double animationFactor = 1.0;

  /// \brief Time of the last update.
  common::Time lastUpdate;

  /// \brief Custom trajectory info.
  physics::TrajectoryInfoPtr trajectoryInfo;

  GazeboRosPtr gazebo_ros_;
  ros::Subscriber cmd_vel_subscriber_;

  /// \brief Operator subscriber node for operator control [listener]
  ros::Subscriber operator_subscriber_;

  /// \brief Robot publisher node for messages [takler]
  ros::Publisher robot_publisher_;

  /// \brief Turn waypoint following on / off the actor
  void ToggleFollowing();
  /// \brief Change waypoint following velocity
  void IncreaseVelocity();
  /// \brief Change waypoint following velocity
  void DecreaseVelocity();

  /// \brief Turn waypoint following on / off or reset the actor
  bool ToggleFollowing(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response);

  /// \brief Change waypoint following velocity
  bool MoveFaster(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response);

  /// \brief Reset start position to first waypoint
  void ResetStartPosition();

  /// \brief Send string message to robot / topic
  void SendMessageToRobot(std::string message);

  /// \brief Identify command from string input
  Commands resolveCommand(std::string input);

  /// \brief Service: toggle waypoint following
  ros::ServiceServer follow_toggle_service;

  /// \brief Service: change velocity (true-faster, false-slower)
  ros::ServiceServer change_velocity_service;

  /// \brief Waypoint file - in our case: std::vector<std::vector<double>>
  XmlRpc::XmlRpcValue waypoints_list;

  /// \brief Mark current waypoint
  int waypoint_marker;

  /// \brief Check for valid waypoint structure
  void ValidateWaypoints();

  /// \brief Stop waypoint following
  bool stop_Waypoint_Following;

  /// \brief Read next waypoint from list
  void ReadNextWayPoint();

  /// \brief ros::ServiceServer wave_toggle_service_;
  bool wave_toggled;
};

}  // namespace gazebo

#endif
