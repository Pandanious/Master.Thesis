

#include <ros/ros.h>
#include <ros/node_handle.h>
#include <gazebo/msgs/msgs.hh>
#include "gazebo_sim_walk/operator.hpp"

#define WALKING_ANIMATION "walking"
#define WAVING_ANIMATION "waving"

/////////////////////////////////////////////////
Operator::Operator() = default;

/////////////////////////////////////////////////
void Operator::Load(gazebo::physics::ModelPtr parent, sdf::ElementPtr sdf)
{ 
  std::cout<<"\n Inside Load of operator.cpp\n";
  //ROS_DEBUG_STREAM("operator load");
  // setup operator control
  this->stop_Waypoint_Following_ = false;
  this->waypoint_marker_ = 0;
  this->node_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
  this->node_->Init();
  this->sub_ = this->node_->Subscribe("~/gazebo_sim_gui", &Operator::CbGui, this);
  
  //this->node_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
  //this->node_->Init();
  //this->sub_ = this->node_->Subscribe("~/rcsim/operator/commands", &Operator::CbCommands, this);

  this->wave_toggled_ = false;

  this->sdf_ = sdf;
  std::cout<<"\n Inside Load 1\n";
  ros::NodeHandle PubNode ;

  this->Pub_Eval = PubNode.advertise<std_msgs::String>("/evaluation_msg",100);
  
  this->actor_ = boost::dynamic_pointer_cast<gazebo::physics::Actor>(parent);
  std::cout<<"\n Inside Load 2\n";
  this->world_ = this->actor_->GetWorld();
  std::cout<<"\n Inside Load 3\n";
  this->Reset();
 /**
 if (sdf -> HasElement("start"))
 {
  std::cout<<"\n Pose found from operator.cpp load function\n";
  this -> modelpose = sdf->Get<ignition::math::Pose3d>("start");
  std::cout<< "\n" <<this -> modelpose << "\n";
  this ->actor_ -> SetWorldPose(this->modelpose); 
 }
 **/
 #if 0
 
 if (!parent-> EntityByName(config_->id_operator))
  {
   std::cout<<"\n Not Found "<<config_->id_operator<<"\n";
  }
  else
  {
   std::cout<<"\n Found "<<config_->id_operator<<"\n";
  }
 world_->EntityByName(config_->id_operator)->SetWorldPose(pose);
 #endif
 
 if (sdf -> HasElement("pose"))
  {
  std::cout<<"\n Inside Load 4\n";
  this -> modelpose = sdf->Get<ignition::math::Pose3d>("pose");
  std::cout<<"\n Pose found from operator.cpp load function "<<this->modelpose<<"\n";  
  }
  else 
  {
  std::cout<<"\n Pose not found from operator.cpp load function\n";
  //this -> modelpose = ignition::math::Pose3d("5 2 0 0 0 0");
  }
 
  // Read in the animation factor (applied in the OnUpdate function).
  if (sdf->HasElement("animation_factor"))
  {
    std::cout<<"\n Inside Load 5\n";
    this->animationFactor_ = sdf->Get<double>("animation_factor");
  }
  // Read the robot id for location tracking.
  if (sdf->HasElement("id_robot"))
  {
    std::cout<<"\n Inside Load 6\n";
    
    this->id_robot_ = sdf->Get<std::string>("id_robot");
  }
  else
  {
    this->id_robot_ = "tiago";
  }
  // Read in the z axes offset for the model (applied in the OnUpdate function).
  if (sdf->HasElement("z_offset"))
  {
    std::cout<<"\n Inside Load 7 if\n";
    
    this->z_offset_ = sdf->Get<double>("z_offset");
  }
  else
  { 
    std::cout<<"\n Inside Load 7 else \n";
    
    this->z_offset_ = 1.0;
  }
  std::cout<<"\n Inside Load 8\n";
    
  if (sdf->HasElement("rotation_velocity"))
  {
    std::cout<<"\n Inside Load 9\n";
    
    this->rotation_velocity_ = sdf->Get<double>("rotation_velocity");
  }
  std::cout<<"\n Inside Load 9\n";
  if (sdf->HasElement("waypoints"))
  { 
    std::cout<<"\n Inside Load 9-1\n";
    auto elem = sdf->GetElement("waypoints");
    sdf::ElementPtr waypoint = elem->GetFirstElement();
    while (waypoint)
    { 
      
      ignition::math::Vector3d wp;
      waypoint->GetValue()->Get(wp);
      std::cout<<"\n waypoints\n"<< wp <<"\n";
      waypoints_.push_back(wp);
      waypoint = waypoint->GetNextElement();
    }
  }
  std::cout<<"\n Inside Load 10\n";
  //std::cout<<"\n Waypoints: " << waypoints_<<"\n";
    
  this->target_ = waypoints_.front();
  std::cout<<"\n Inside Load 11\n";
  ResetStartPosition();
  waypoint_marker_ = 0;
  std::cout<<"\n Inside Load 12\n";

  this->connections_.push_back(
    gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&Operator::OnUpdate, this, std::placeholders::_1)));
  std::cout<<"\n Inside Load 13\n";
}


void Operator::CbGui(const boost::shared_ptr<const gazebo::msgs::Any>& msg)
{
  std::cout<<"Inside CbGui - "<<msg->string_value()<<"\n";
  
  if (msg->string_value() == "Start Walk")
  {
    std::cout<<"Starting Animation from operator plugin\n";
    startanim();
    
    //ScoreTask();
  }
  else if (msg->string_value() == "Reset")
  {
    std::cout<<"Removing Actor from operator plugin\n";
    resetmodel();
    std_msgs::StringPtr str(new std_msgs::String);
    str->data = "stop eval";
    this->Pub_Eval.publish(str);
    std::cout<<"Removed Actor from operator\n";
    
    //ScoreTask();
  }
}

void Operator::ResetStartPosition()
{
  double x, y;
  x = waypoints_[0].X();
  y = waypoints_[0].Y();

  ignition::math::Pose3d pose = this->actor_->WorldPose();
  ignition::math::Vector3d new_target(this->target_);

  // target is first position
  new_target.X(x);
  new_target.Y(y);

  this->target_ = new_target;
  waypoint_marker_ = 0;
}

void Operator::resetmodel()
{
  started_ = false;
  this->stop_Waypoint_Following_ = true;
  Reset();
  std::cout<<"Model Pose: "<< this->modelpose<<"\n";
  this ->actor_ -> SetWorldPose(this->modelpose);
  ResetStartPosition();
  
  std::cout<<"\n"<<"resetmodel func "<<" Pose: "<<this->actor_->WorldPose()<<"\n";

} 
void Operator::startanim()
{
  
  /**
  gazebo::common::PoseAnimationPtr panimate(new gazebo::common::PoseAnimation("walk",10.0,0));
 // gazebo::common::PoseKeyFrame *key = panimate->CreateKeyFrame(0.0);
  //key -> Rotation(ignition::math::Quaterniond(0,0,-1.57));
  //key -> Translation(ignition::math::Vector3d(2,3.5,1.06));

 // key = panimate->CreateKeyFrame(5.0);
 // key -> Rotation(ignition::math::Quaterniond(0,0,-1.57));
 // key -> Translation(ignition::math::Vector3d(2,2,1.06));
 // key = panimate->CreateKeyFrame(7.5);
 // key -> Rotation(ignition::math::Quaterniond(0,0,-1.57));
 // key -> Translation(ignition::math::Vector3d(2,1,1.06));  
 
  ignition::math::Pose3d pose = ignition::math::Pose3d(config_ -> start);
    std::cout<<"\n"<<"pose: "<<pose<<"\n";
    walkerptr->SetWorldPose(pose,false,false);
 **/
    this -> started_ = true;
    this -> waypoint_marker_ = 0;
    this->stop_Waypoint_Following_ = false;
  /**
    auto model_sdf = this -> actor_ -> GetSDF();
    int plugin =  this -> actor_ -> GetPluginCount();
    //this -> actor_->SetScriptTime(5);
    double tim = this ->actor_ ->ScriptTime();
   
    std::cout<<"\n"<<"Model Sdf: "<< model_sdf <<"\n plugin Count: "<< plugin <<"\n";
    std::cout<<"\n"<<this->actor_<<" <-Actor_ "<<"script time: "<<tim<<" Pose: "<<this->actor_->WorldPose()<<"\n";
  
    
  //this -> actor_ -> SetAnimation(panimate);
  std::cout<<"\n setting animation true\n";
  **/
  

}

void Operator::Reset()
{
  gazebo::ModelPlugin::Reset();
  this->velocity_ = 0.8;
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
    this->wave_toggled_ = false;
  }
}
void Operator::ReadNextWayPoint()
{
  static bool first_time = true;
  if (!started_)
  {
    return;
  }

  ignition::math::Vector3d new_target(this->target_);
  double x, y, v;
  if (waypoints_.size() > waypoint_marker_)
  {
    // process ...
    x = waypoints_[waypoint_marker_].X();
    y = waypoints_[waypoint_marker_].Y();
    v = waypoints_[waypoint_marker_].Z();

    // setup new target
    ROS_DEBUG_THROTTLE(10, "target %d, x: %f, y: %f, v: %f ", waypoint_marker_, x, y, v);

    new_target.X(x);
    new_target.Y(y);

    this->target_ = new_target;
    this->velocity_ = v;
    waypoint_marker_++;
  }
  else
  {
    ROS_INFO_ONCE("=== reached last waypoint ===");
    if (first_time == true)
    {
      first_time = false;
      ignition::math::Pose3d pose = this->actor_->WorldPose();
      std::cout<<"Model Pose: "<<pose<<"\n";
    }
    this->stop_Waypoint_Following_ = true;
  }
} 


#if 0
void Operator::CbCommands(const boost::shared_ptr<const gazebo::msgs::Any>& msg)
{true
  ROS_INFO_STREAM("Operator heard:" << msg->string_value() << " noFollow " << this->stop_Waypoint_Following_);
  switch (commands::StringToCommand(msg->string_value()))
  {
    case Commands::START:
      ROS_INFO_STREAM("start following");
      ResetAnimation();
      started_ = true;
      this->stop_Waypoint_Following_ = false;
      break;
    case Commands::STOP:
      ResetAnimation();
      this->stop_Waypoint_Following_ = true;
      break;
    case Commands::SLOWER:
      DecreaseVelocity();
      break;
    case Commands::FASTER:
      IncreaseVelocity();
      break;
    case Commands::WAVE:
      StartWavingAnimation();
      this->stop_Waypoint_Following_ = true;
      break;
  }
}

void Operator::IncreaseVelocity()
{
  if (this->velocity_ < 2.0)waypoint_marker
  {
    this->velocity_ = this->velocity_ + 0.2;
  }
  else
  {
    ROS_INFO("operator velocity too high.");
  }
}
void Operator::DecreaseVelocity()
{
  if (this->velocity_ <= 0.06)
  {
    ROS_INFO("velocity too low.");
    return;
  }

  if (this->velocity_ > 0.2)
  {
    this->velocity_ = this->velocity_ - 0.2;
  }
  elsewaypoint_marker
  {
    this->velocity_ = this->velocity_ / 2;
    ROS_INFO("operator velocity very low.");
  }
}

void Operator::ResetStartPosition()
{
  double x, y;
  x = waypoints_[0].X();
  y = waypoints_[0].Y();

  ignition::math::Pose3d pose = this->actor_->WorldPose();
  ignition::math::Vector3d new_target(this->target_);

  // target is first position
  new_target.X(x);
  new_target.Y(y);

  this->target_ = new_target;
  waypoint_marker_ = 0;
}

/////////////////////////////////////////////////
void Operator::StartWavingAnimation()
{
  ROS_INFO("request: toggle waving animation");
  this->wave_toggled_ = true;
  auto skel_anims = this->actor_->SkeletonAnimations();
  if (skel_anims.find(WAVING_ANIMATION) == skel_anims.end())
  {
    gzerr << "Skeleton animation " << WAVING_ANIMATION << " not found.\n";
  }
  else
  {
    // Create custom trajectory
    this->trajectoryInfo_.reset(new gazebo::physics::TrajectoryInfo());
    this->trajectoryInfo_->type = WAVING_ANIMATION;
    this->trajectoryInfo_->duration = 1.0;
    this->last_linear_ = ignition::math::Vector3d(0, 0, 0);
    this->last_angle_ = 0;
    this->actor_->SetCustomTrajectory(this->trajectoryInfo_);
  }
}

/////////////////////////////////////////////////
void Operator::Reset()
{
  gazebo::ModelPlugin::Reset();
  this->velocity_ = 0.8;
  this->lastUpdate_ = 0;

  this->target_ = ignition::math::Vector3d(0, 0, 1.2138);

  auto skel_anims = this->actor_->SkeletonAnimations();
  if (skel_anims.find(WALKING_ANIMATION) == skel_anims.end())
  {
    gzerr << "Skeleton animation " << WALKING_ANIMATION << " not found.\n";
  }
  else 1.4 3.5 1.015 1.5708 -0 0

  {
    // Create custom trajectory
    this->trajectoryInfo_.reset(new gazebo::physics::TrajectoryInfo());
    this->trajectoryInfo_->type = WALKING_ANIMATION;
    this->trajectoryInfo_->duration = 1.0;
    this->last_linear_ = ignition::math::Vector3d(0, 0, 0);
    this->last_angle_ = 0;
    this->actor_->SetCustomTrajectory(this->trajectoryInfo_);
    this->wave_toggled_ = false;
  }
}

void Operator::ResetAnimation()
{
  this->wave_toggled_ = false;
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
#endif

/////////////////////////////////////////////////
void Operator::OnUpdate(const gazebo::common::UpdateInfo& info)
{
  if (waypoint_marker_ == -1)
  {
    return;
  }
 
  if(!started_)
  {
    return;
  }
   //std::cout<<"inside OnUpdate of operator.cpp"<<"\n";
  // Time delta
  double dt = (info.simTime - this->lastUpdate_).Double();
  this->lastUpdate_ = info.simTime;

  // movespeed
  double move_speed = (wave_toggled_ || stop_Waypoint_Following_) ? 0 : this->velocity_;

  ignition::math::Pose3d pose = this->actor_->WorldPose();
  ignition::math::Vector3d dir = this->target_ - pose.Pos();
  dir.Z() = 0;
  double yaw = pose.Rot().Yaw();

  // double distance = dir.Length();

  ignition::math::Vector2d modelxy =
      ignition::math::Vector2d(this->actor_->WorldPose().Pos().X(), this->actor_->WorldPose().Pos().Y());
  ignition::math::Vector2d targetxy = ignition::math::Vector2d(this->target_.X(), this->target_.Y());
  double distance = modelxy.Distance(targetxy);

  if (this->wave_toggled_ && this->world_->EntityByName(this->id_robot_))
  {
    ignition::math::Vector3d robot_vec = this->world_->EntityByName(this->id_robot_)->WorldPose().Pos();
    dir = robot_vec - pose.Pos();
  }
  // Choose a new target position
  // if actor reached current target
  else if (distance < 0.01)
  {
    this->ReadNextWayPoint();
    dir = this->target_ - pose.Pos();
    // stop moving for this update
    move_speed = 0;
  }

  dir = dir.Normalize();

  // Compute the yaw orientation
  ignition::math::Angle yawdiff = atan2(dir.Y(), dir.X()) + IGN_PI * 0.5 - yaw;
  yawdiff.Normalize();

  auto dyaw = (std::abs(yawdiff.Radian()) < std::abs(rotation_velocity_ * dt)) ? yawdiff : rotation_velocity_ * dt;
  dyaw = (yawdiff.Radian() > 0) ? dyaw : dyaw * -1;

  if (std::abs(yawdiff.Radian()) > IGN_DTOR(1))
  {
    // only rotate until we move forward
    pose.Rot() = ignition::math::Quaterniond(IGN_PI * 0.5, 0, yaw + dyaw.Radian());
  }
  else
  {
    auto deltapose = dir * move_speed * dt;
    // dont overshoot target
    if (deltapose.Length() > distance)
    {
      deltapose *= distance / deltapose.Length();
    }

    pose.Pos() += deltapose;
    pose.Rot() = ignition::math::Quaterniond(IGN_PI * 0.5, 0, yaw + dyaw.Radian());
  }

  // Make sure the actor stays within bounds
  pose.Pos().X(std::max(-10.0, std::min(10.0, pose.Pos().X())));
  pose.Pos().Y(std::max(-10.0, std::min(10.0, pose.Pos().Y())));
  pose.Pos().Z(this->z_offset_);

  // Distance traveled is used to coordinate motion with the walking
  // animation
  double distance_traveled = (pose.Pos() - this->actor_->WorldPose().Pos()).Length();

  this->actor_->SetWorldPose(pose, false, false);

  if (this->wave_toggled_)
  {
    this->actor_->SetScriptTime(this->actor_->ScriptTime() + dt);
  }
  else
  {
    this->actor_->SetScriptTime(this->actor_->ScriptTime() + (distance_traveled * this->animationFactor_));
  }
}

/////////////////////////////////////////////////

GZ_REGISTER_MODEL_PLUGIN(Operator)
