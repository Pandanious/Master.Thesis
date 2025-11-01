#pragma once

#include <ros/ros.h>

#include <gazebo/physics/physics.hh>

struct Spectator
{
  static Spectator FromElement(const sdf::ElementPtr& spectator);
  std::string name;

  std::list<ignition::math::Vector3d> waypoints;
};

struct TaskConfig
{
  static TaskConfig* FromElement(const sdf::ElementPtr& config);

  std::string topic_in{ "/rcsim/operator/input" };
  std::string topic_out{ "/ps_adapter/custom_rec" };

  std::string id_robot;
  std::string id_operator{ "operator" };
  ignition::math::Pose3d start;
  ignition::math::Vector3d goal;
  ignition::math::Pose3d tiny_object_pose;

  bool demo_mode{ false };
  double max_goal_dist{ 3 };
  double max_follow_dist{ 8 };
  ros::Duration max_stop_time{ 10 };

  int seed{ 0 };
  std::list<ignition::math::Vector3d> waypoints;
  std::list<Spectator> spectators;

  // currently not parsed from sdf
  ros::Duration talk_repeat_delay{ ros::Duration(20) };

  friend std::ostream& operator<<(std::ostream& out, const TaskConfig& tc);
};
