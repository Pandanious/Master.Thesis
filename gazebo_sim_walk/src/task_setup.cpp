#include <utility>

#include "gazebo_sim_walk/task_setup.h"

#include "gazebo/physics/physics.hh"

TaskSetup::TaskSetup(TaskConfig config)
{
  config_ = std::move(config);

  score_ = std::make_shared<TaskScore>("Carry My Luggage", ros::Duration(300));

  // Main goal
  score_->AddItem("obj", "Take the bag to the car", 500);

  ScoreItem pho("Hand-over the bag", -200);
  pho.is_penalty = true;
  score_->AddItem("pho", pho);
  ScoreItem pni("Regain operator’s track by natural interaction", -500);
  pni.is_penalty = true;
  score_->AddItem("pni", pni);
  ScoreItem pnn("Regain operator’s track by non-natural interaction", -1000);
  pnn.is_penalty = true;
  score_->AddItem("pnn", pnn);
  ScoreItem pdc("Regain operator’s track by direct contact", -2000);
  pdc.is_penalty = true;
  score_->AddItem("pdc", pdc);

  // Bonus Goal
  score_->AddItem("benter", "Reentering the arena", 100);
  score_->AddItem("bcrowd", "Avoid the crowd obstructing path", 100);
  score_->AddItem("bsmall", "Avoid the small object on the ground", 100);
  score_->AddItem("bhard", "Avoid the hard-to-see 3D object", 100);
  score_->AddItem("bblock", "Avoid the area blocked with retractable barriers", 100);
}

bool TaskSetup::StartTask()
{
  if (phase_ == TaskPhase::SETUP)
  {
    ROS_INFO_STREAM_NAMED("SCORING", "Start Task");
    phase_ = TaskPhase::INIT;
    return true;
  }
  {
    return false;
  }
}
bool TaskSetup::StartTakeBag()
{
  if (phase_ == TaskPhase::INIT)
  {
    phase_ = TaskPhase::TAKE_BAG;
    carry_bag_ = true;
    return true;
  }
  {
    return false;
  }
}
bool TaskSetup::StartIntroduction()
{
  if (phase_ == TaskPhase::INIT || phase_ == TaskPhase::TAKE_BAG)
  {
    phase_ = TaskPhase::INTRODUCTION;
    if (!carry_bag_)
    {
      score_->SetMaxPoints("pho");
    }
    return true;
  }
  {
    return false;
  }
}
bool TaskSetup::StartFollowing()
{
  if (phase_ == TaskPhase::INTRODUCTION)
  {
    ROS_INFO_STREAM_NAMED("SCORING", "Start Following");
    phase_ = TaskPhase::FOLLOWING;
    return true;
  }
  {
    return false;
  }
}
bool TaskSetup::StopFollowing()
{
  if (phase_ == TaskPhase::FOLLOWING)
  {
    phase_ = TaskPhase::STOPPED;
    last_stop_ = ros::Time::now();
    return true;
  }
  {
    return false;
  }
}
bool TaskSetup::StartHandover()
{
  if (phase_ == TaskPhase::STOPPED)
  {
    phase_ = TaskPhase::HANDOVER;
    return true;
  }
  {
    return false;
  }
}
bool TaskSetup::StartReturn()
{
  if (phase_ == TaskPhase::HANDOVER)
  {
    phase_ = TaskPhase::RETURN;
    return true;
  }
  {
    return false;
  }
}

TaskScore TaskSetup::FinishTask()
{
  if (!had_collision_tiny_)
  {
    score_->SetMaxPoints("bsmall");
  }
  score_->FinishTask();
  return *score_.get();
}

TaskScore TaskSetup::Update(const gazebo::physics::WorldPtr& world)
{
  if (!world->EntityByName(config_.id_robot))
  {
    ROS_ERROR_STREAM_ONCE("cant find robot in world");
    return *score_.get();
  }

  // exit if time is up
  if (score_->GetRemainingTime() <= ros::Duration(0))
  {
    ROS_INFO_STREAM_NAMED("SCORING", "TIMEOUT");
    return FinishTask();
  }

  // TODO(lruegeme): check for collision with bonus goals
  bool in_collision = false;
  if (in_collision && !had_collision_tiny_)
  {
    ROS_INFO_STREAM_NAMED("SCORING", "detect collision with tiny object");
    had_collision_tiny_ = true;
  }

  if (phase_ == TaskPhase::HANDOVER)
  {
    if (carry_bag_)
    {
      // todo check if bag is placed near the car
    }
    else
    {
      // add points if the robot is near the operator
      auto robotpose = world->EntityByName(config_.id_robot)->WorldPose();
      auto distance = robotpose.Pos().Distance(config_.goal);
      if (distance < config_.max_goal_dist)
      {
        ROS_INFO_STREAM_NAMED("SCORING", "clear goal: obj");
        score_->SetMaxPoints("obj");
      }
    }
  }
  else if (phase_ == TaskPhase::RETURN)
  {
    // during return phase add points if the robot is near the start position
    auto robotpose = world->EntityByName(config_.id_robot)->WorldPose();
    auto distance = robotpose.Pos().Distance(config_.start.Pos());
    if (distance < config_.max_goal_dist)
    {
      ROS_INFO_STREAM_NAMED("SCORING", "clear goal: bonus enter");
      score_->SetMaxPoints("benter");
    }
  }
  else if (phase_ == TaskPhase::FOLLOWING)
  {
    auto robotpose = world->EntityByName(config_.id_robot)->WorldPose();
    auto operatorpose = world->EntityByName(config_.id_operator)->WorldPose();
    auto distance = robotpose.Pos().Distance(operatorpose.Pos());
    if (distance < config_.max_follow_dist)
    {
      ROS_INFO_STREAM_NAMED("SCORING", "error: Lost the operator");
      score_->AddItem("ERR", "Lost the operator", -1000);
      score_->SetPoints("ERR", -1000);
      score_->FinishTask();
      ROS_INFO_STREAM("Lost operator");
    }
  }
  else if (phase_ == TaskPhase::STOPPED)
  {
    if ((last_stop_ + config_.max_stop_time - ros::Time::now()) <= ros::Duration(0))
    {
      ROS_INFO_STREAM_NAMED("SCORING", "error: Stop Timeout");
      score_->AddItem("ERR", "Stop Timeout", -1000);
      score_->SetPoints("ERR", -1000);
      score_->FinishTask();
      ROS_INFO_STREAM("Stopped too long");
    }
  }

  return *score_.get();
}
