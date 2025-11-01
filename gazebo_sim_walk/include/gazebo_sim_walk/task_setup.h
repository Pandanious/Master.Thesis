#pragma once

#include <list>

#include <gazebo/gazebo.hh>
#include <sdf/sdf.hh>

//#include <rcsim_task_core/task_score.h>

#include "gazebo_sim_walk/task_config.h"

enum TaskPhase
{
  SETUP,         // wait for robot to see operator (ro: welcome ...)
  INIT,          // currently: wait for op to reach waypoint 0 -> wait for robot to answer bag pickup
  TAKE_BAG,      // wait for robot to have the bag
  INTRODUCTION,  // say follow me, wait for (ro: please go ahead)
  FOLLOWING,     // following running
  STOPPED,       // following paused
  HANDOVER,      // drop bag near car, wait for (ro: here is your bag)
  RETURN         // wait for robot to reach arena
};

class TaskSetup
{
public:
  explicit TaskSetup(TaskConfig config);

//  TaskScore Update(const gazebo::physics::WorldPtr& world);

//  TaskPhase GetCurrentPhase()
//  {
//    return phase_;
//  }

//  bool StartTask();
//  bool StartTakeBag();
//  bool StartIntroduction();
//  bool StartFollowing();
//  bool StopFollowing();
//  bool StartHandover();
//  bool StartReturn();

//  TaskScore FinishTask();

//  std::shared_ptr<TaskScore> GetScorePtr()
//  {
//    return score_;
//  }

//private:
//  bool carry_bag_{ false };
//  bool had_collision_tiny_{ false };
//  std::shared_ptr<TaskScore> score_;
//
//  TaskPhase phase_{ TaskPhase::SETUP };
  TaskConfig config_;
//  ros::Time last_stop_;
};
