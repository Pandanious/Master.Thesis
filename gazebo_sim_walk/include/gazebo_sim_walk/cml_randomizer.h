#pragma once

#include <ignition/math/Pose3.hh>
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"

#include "sdf/sdf.hh"

#include <vector>
#include <unordered_map>
#include <list>

#include "rcsim_task_carry_my_luggage/task_config.h"

namespace m = ignition::math;

/**
 * Used to randomize positions and waypoints for Carry My Luggage Task.
 */
class CmlRandomizer
{
private:
  std::mt19937* rng_{ nullptr };

  struct Path* path_{ nullptr };

  std::string tiny_object_;

  sdf::ElementPtr sdf_config_;

  // Selects a random path to spawn car. Sets waypoints respectively.
  void RandomPath(const gazebo::physics::WorldPtr& world, TaskConfig* config, std::list<Path *> paths);

  // Spawns car at given pose.
  void PlaceCar(const gazebo::physics::WorldPtr& world, const m::Pose3d& car_pose);

  // Selects a pose between two random waypoints and places a tiny object;
  void PlaceTinyObject(TaskConfig* config);

public:
  CmlRandomizer(const sdf::ElementPtr& sdf_config, std::mt19937* rng, std::string tiny_object);

  // Calls randomize methods for endpoint, waypoints and other CML randomization.
  // Uses sdf config to read out waypoints and car positions 
  void Randomize(const gazebo::physics::WorldPtr& world, TaskConfig* config);

  template <typename T>
  void ShuffleList(std::list<T>* list)
  {
    // Sample methods for iterators
    std::vector<T> v(list->begin(), list->end());
    shuffle(v.begin(), v.end(), *rng_);
    list->assign(v.begin(), v.end());
  }
};

/**
 * Used to select between different waypoints with given car position at the end of path.
 */
struct Path
{
  static Path* Init(std::list<m::Vector3d> waypoints, const m::Pose3d& car_pose)
  {
    auto* path = new Path();
    path->waypoints = std::move(waypoints);
    path->car_pose = car_pose;
    return path;
  }
  std::list<m::Vector3d> waypoints;
  m::Pose3d car_pose;
};