#include "rcsim_task_carry_my_luggage/random_walker.hpp"

#include <ros/ros.h>

#define WALKING_ANIMATION "walking"
#define WAVING_ANIMATION "waving"

#define RANDOM_WALKER_CONSOLE_LEVEL ros::console::levels::Info

/////////////////////////////////////////////////
RandomWalker::RandomWalker() = default;

/////////////////////////////////////////////////
void RandomWalker::Load(gazebo::physics::ModelPtr parent, sdf::ElementPtr sdf)
{
  this->actor_ = boost::dynamic_pointer_cast<gazebo::physics::Actor>(parent);
  this->world_ = this->actor_->GetWorld();

  ros::console::set_logger_level(std::string(ROSCONSOLE_DEFAULT_NAME) + "." + actor_->GetName(),
                                 RANDOM_WALKER_CONSOLE_LEVEL);
  ros::console::notifyLoggerLevelsChanged();
  ROS_INFO_NAMED(actor_->GetName(), "Random Walker Load");

  this->Reset();

  // Read in the animation factor (applied in the OnUpdate function).
  if (sdf->HasElement("animation_factor"))
  {
    this->animationFactor_ = sdf->Get<double>("animation_factor");
  }
  else
  {
    this->animationFactor_ = 4.5;
  }

  if (sdf->HasElement("rotation_velocity"))
  {
    this->rotation_velocity_ = sdf->Get<double>("rotation_velocity");
  }

  if (sdf->HasElement("waypoints"))
  {
    auto elem = sdf->GetElement("waypoints");
    sdf::ElementPtr waypoint = elem->GetFirstElement();
    while (waypoint)
    {
      ignition::math::Vector3d wp;
      waypoint->GetValue()->Get(wp);
      waypoints_.push_back(wp);
      waypoint = waypoint->GetNextElement();
    }
  }

  ResetStartPosition();

  this->connections_.push_back(
      gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&RandomWalker::OnUpdate, this, std::placeholders::_1)));
}

void RandomWalker::ResetStartPosition()
{
  waypoint_marker_ = 0;
  this->target_ = waypoints_.front();

  double x, y, v;
  x = waypoints_[waypoint_marker_].X();
  y = waypoints_[waypoint_marker_].Y();
  v = waypoints_[waypoint_marker_].Z();

  ignition::math::Pose3d pose = this->actor_->WorldPose();
  ignition::math::Vector3d new_target(this->target_);

  // target_ is first position
  new_target.X(x);
  new_target.Y(y);

  this->target_ = new_target;

  this->velocity_ = v;

  ROS_DEBUG_NAMED(actor_->GetName(), "target_ %d, x: %f, y: %f, v: %f ", waypoint_marker_, x, y, v);
}

/////////////////////////////////////////////////
void RandomWalker::Reset()
{
  gazebo::ModelPlugin::Reset();
  this->velocity_ = 1;
  this->lastUpdate_ = 0;

  this->target_ = ignition::math::Vector3d(0, 0, 1.2138);

  auto skel_anims = this->actor_->SkeletonAnimations();
  if (skel_anims.find(WALKING_ANIMATION) == skel_anims.end())
  {
    gzerr << "Skeleton animation " << WALKING_ANIMATION << " not found.\n";
  }
  else
  {
    // Create custom trajectory
    this->trajectoryInfo_.reset(new gazebo::physics::TrajectoryInfo());
    this->trajectoryInfo_->type = WALKING_ANIMATION;
    this->trajectoryInfo_->duration = 1.0;
    this->last_linear_ = ignition::math::Vector3d(0, 0, 0);
    this->last_angle_ = 0;
    this->actor_->SetCustomTrajectory(this->trajectoryInfo_);
  }
}

/////////////////////////////////////////////////
void RandomWalker::OnUpdate(const gazebo::common::UpdateInfo& info)
{
  if (waypoint_marker_ == -1)
  {
    return;
  }

  // Time delta
  double dt = (info.simTime - this->lastUpdate_).Double();

  ignition::math::Pose3d pose = this->actor_->WorldPose();
  ignition::math::Vector3d dir = this->target_ - pose.Pos();
  double yaw = pose.Rot().Yaw();

  // double distance = dir.Length();

  ignition::math::Vector2d modelxy =
      ignition::math::Vector2d(this->actor_->WorldPose().Pos().X(), this->actor_->WorldPose().Pos().Y());
  ignition::math::Vector2d targetxy = ignition::math::Vector2d(this->target_.X(), this->target_.Y());
  double distance = modelxy.Distance(targetxy);

  // Choose a new target_ position
  // if actor_ reached current target_
  if (distance < 0.01)
  {
    this->ReadNextWayPoint();
    dir = this->target_ - pose.Pos();
  }

  dir = dir.Normalize();

  // Compute the yaw orientation
  ignition::math::Angle yawdiff = atan2(dir.Y(), dir.X()) + IGN_PI * 0.5 - yaw;
  yawdiff.Normalize();

  auto dyaw = (std::abs(yawdiff.Radian()) < std::abs(rotation_velocity_ * dt)) ? yawdiff : rotation_velocity_ * dt;
  dyaw = (yawdiff.Radian() > 0) ? dyaw : dyaw * -1;

  if (std::abs(yawdiff.Radian()) > IGN_DTOR(1))
  {
    pose.Rot() = ignition::math::Quaterniond(IGN_PI * 0.5, 0, yaw + dyaw.Radian());
  }
  else
  {
    pose.Pos() += dir * this->velocity_ * dt;
    pose.Rot() = ignition::math::Quaterniond(IGN_PI * 0.5, 0, yaw + dyaw.Radian());
  }

  // Make sure the actor_ stays within bounds
  pose.Pos().X(std::max(-10.0, std::min(10.0, pose.Pos().X())));
  pose.Pos().Y(std::max(-10.0, std::min(10.0, pose.Pos().Y())));
  pose.Pos().Z(1.0);

  // Distance traveled is used to coordinate motion with the walking
  // animation
  double distance_traveled = (pose.Pos() - this->actor_->WorldPose().Pos()).Length();

  this->actor_->SetWorldPose(pose, false, false);
  this->actor_->SetScriptTime(this->actor_->ScriptTime() + (distance_traveled * this->animationFactor_));

  this->lastUpdate_ = info.simTime;
}

/////////////////////////////////////////////////
void RandomWalker::ReadNextWayPoint()
{
  ignition::math::Vector3d new_target(this->target_);
  double x, y, v;
  if (waypoint_marker_ >= waypoints_.size())
  {
    waypoint_marker_ = 0;
  }

  // process ...
  x = waypoints_[waypoint_marker_].X();
  y = waypoints_[waypoint_marker_].Y();
  v = waypoints_[waypoint_marker_].Z();

  // setup new target_
  ROS_DEBUG_NAMED(actor_->GetName(), "target_ %d, x: %f, y: %f, v: %f ", waypoint_marker_, x, y, v);

  new_target.X(x);
  new_target.Y(y);

  this->target_ = new_target;
  this->velocity_ = v;
  waypoint_marker_++;
}

GZ_REGISTER_MODEL_PLUGIN(RandomWalker)
