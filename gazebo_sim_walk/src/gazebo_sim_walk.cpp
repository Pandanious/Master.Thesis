#include "gazebo_sim_walk/rcsim_carry_my_luggage.h"

#include <utility>

#include <ignition/math.hh>

#include "gazebo_sim_walk/commands.hpp"
#include "gazebo_sim_walk/sdf_util.hpp"

void TaskCarryMyLuggage::Load(gazebo::physics::WorldPtr world, sdf::ElementPtr sdf)
{
  std::cout<<"\n Load Called\n";
  //ROS_INFO_STREAM("Gazebo_sim_walk Load...");
  world_ = world;
  this->actorspawn = false;
  
  config_ = TaskConfig::FromElement(sdf);
  //task_ = new TaskSetup(*config_);

  // Init random if no seed is given
  if (config_->seed == 0)
  {
    config_->seed = std::chrono::system_clock::now().time_since_epoch().count();
  }
  rng_ = new std::mt19937(config_->seed);

  tinyObject_ = "";
  std::cout << world_->Name() << std::endl;
  
  ROS_INFO_STREAM("Using Config: " << *config_);

  this->node_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
  this->node_->Init();
  
  this->sub_ = this->node_->Subscribe("~/gazebo_sim_gui", &TaskCarryMyLuggage::CbGui, this);
  this->puboperator_ = this->node_->Advertise<gazebo::msgs::Any>("~/gazebo_sim_actor");
  this->connections_.push_back(gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&TaskCarryMyLuggage::OnUpdate, this, std::placeholders::_1)));

}  

void TaskCarryMyLuggage::SpawnActors()
{
  std::cout<<"\n Inside function SpawnActor start\n";
 // ROS_DEBUG_STREAM("spawn actor");if (!parent-> EntityByName(config_->id_operator))
 // DisplayGuiMessage("spawn actor");
  // if (false)

    // spawn operator
    std::string filename = gazebo::common::ModelDatabase::Instance()->GetModelFile("model://" + config_->id_operator);
    //std::string filename = gazebo::common::ModelDatabase::Instance()->GetModelFile("model://walker_gazebo.sdf");
    
    //filename = "/home/panda/Img_Laser/src/Try.Plugin/src/gazebo_sim_walk/gazebomodel/walker/walker_gazebo.sdf" ;
    //filename = "model://walker.sdf";
    std::cout<<"\n"<< config_ -> id_operator<<" - ";
    std::cout<<"Model File -";
    std::cout<< filename<<"\n";
    
    auto ss = std::ostringstream{};
    std::ifstream file(filename);
    ss << file.rdbuf();
    //std::cout<<"string from file: "<<ss.str()<<"\n";
//change
    sdf::SDF sdf;
    sdf.SetFromString(ss.str());
    sdf::ElementPtr mp = sdf.Root()->GetElement("actor");
    mp->GetAttribute("name")->SetFromString(config_->id_operator);
    std::cout<<"\n Name: "<<config_ -> id_operator<<"\n";
    std::cout<<"\n Name: "<<mp->GetAttribute("name")<<"\n";

    mp->GetElement("pose")->GetValue()->Set(config_->start);
    
    for (const auto& wp : config_->waypoints)
    {
      std::cout<<"Creating Waypoints "<< wp <<"\n";
      //out << " - " << wp << std::endl;
    }
    mp->GetElement("plugin")->InsertElement(CreateWaypointElement(config_->waypoints, config_->demo_mode));
    mp->GetElement("plugin")->GetElement("pose")->GetValue()->Set(config_->start);
    
    std::cout<<"\n"<< typeid(config_ -> start).name()<<" "<< config_ -> start<<"\n";
    
    //std::cout<<"\n"<< config_ -> waypoints<<"\n";
    std::string fname = "file://"+ filename;
    std::cout<<" Fname : "<<fname<<"\n";
    //world_->InsertModelFile("model://walker");
    
    world_->InsertModelSDF(sdf);
    /***
    if (!world_-> EntityByName(config_->id_operator))
    {
      std::cout<<"\n Not Found "<<config_->id_operator<<"\n";
    }
    else
    {
     std::cout<<"\n Found "<<config_->id_operator<<"\n";
    }
    std::cout<<"\n after Insert Model \n";
    auto op = world_->EntityByName(config_ -> id_operator);
    std::cout<<"\n Entity By Name "<<op<<"\n"; 
    ***/
   // auto operatorpose  = world_->EntityByName(config_->id_operator)->WorldPose();
   // std::cout<<"\n After World Pose\n";
   // ignition::math::Pose3d pose = ignition::math::Pose3d(config_ -> start);
   // std::cout<<"\n Pose from Pose3d \n";
    
   // world_->EntityByName(config_->id_operator)->SetWorldPose(pose);
   // std::cout<<"\n Set world pose from pose \n";
   
   /** 
    const std::string st = "walker";
    gazebo::physics::Model_V mod1 = world_ -> Models(); 
    std::cout<<"\n size of model vector from Models() "<<mod1.size()<<"\n";
  **/
    //sdf -> SetWorldPose(pose);
    //std::cout<<"\n Pose: "<< pose <<"\n";
   
#if 0
  for (auto spectator : config_->spectators)
  {
    // use first waypoint as start pose
    ignition::math::Pose3d rwstart = config_->start;
    rwstart.Pos() = spectator.waypoints.front();

    // spawn walker
    std::string filename = gazebo::common::ModelDatabase::Instance()->GetModelFile("model://random_walker");
    auto ss = std::ostringstream{};
    std::ifstream file(filename);
    ss << file.rdbuf();

    sdf::SDF sdf;
    sdf.SetFromString(ss.str());
    sdf::ElementPtr mp = sdf.Root()->GetElement("actor");
    mp->GetAttribute("name")->SetFromString(spectator.name);
    mp->GetElement("pose")->GetValue()->Set(rwstart);
    mp->GetElement("plugin")->InsertElement(CreateWaypointElement(spectator.waypoints, config_->demo_mode));
    mp->GetElement("plugin")->InsertElement(CreateStringElement("id_robot", config_->id_robot));

    world_->InsertModelSDF(sdf);
    ROS_DEBUG_STREAM("spawned random_walker: " << spectator.name);
  }
  #endif
  std::cout<<"\n SpawnActor end\n";
}

void TaskCarryMyLuggage::resetmodel()
{
  this->puboperator_->Publish(gazebo::msgs::ConvertAny("Reset"));
  
  /**
  if (walkerptr)
  {
    ignition::math::Pose3d pose = ignition::math::Pose3d(config_ -> start);
    walkerptr->SetWorldPose(pose,false,false);
    //walkerptr->StopAnimation();
  }
  **/
}

void TaskCarryMyLuggage::startanim()
{ 
  this->puboperator_->Publish(gazebo::msgs::ConvertAny("Start Walk"));
  /**
  if (walkerptr)
  {
    
    ignition::math::Pose3d pose = ignition::math::Pose3d(config_ -> start);
    std::cout<<"\n"<<"pose: "<<pose<<"\n";
    walkerptr->SetWorldPose(pose,false,false);
    double tim = walkerptr ->ScriptTime();
    auto model_sdf = walkerptr -> GetSDF();
    int plugin =  walkerptr -> GetPluginCount();
    std::cout<<"\n"<<"Model Sdf: "<< model_sdf <<"\n plugin Count: "<< plugin <<"\n";
    std::cout<<"\n"<<walkerptr<<" <-Walkerptr "<<"script time: "<<tim<<"\n";
    
    //walkerptr->SetScriptTime(5);
   // gazebo::common::PoseAnimationPtr panimate(new gazebo::common::PoseAnimation("walk",7.5,1));
   // gazebo::common::PoseKeyFrame *key = panimate->CreateKeyFrame(0.0);
   // key -> Rotation(ignition::math::Quaterniond(0,0,-1.57));
  //  key -> Translation(ignition::math::Vector3d(2,3.5,1.06));

 //   key = panimate->CreateKeyFrame(5.0);
 //   key -> Rotation(ignition::math::Quaterniond(0,0,-1.57));
 //   key -> Translation(ignition::math::Vector3d(2,2,1.06));
 //   key = panimate->CreateKeyFrame(7.5);
 //   key -> Rotation(ignition::math::Quaterniond(0,0,-1.57));
 //   key -> Translation(ignition::math::Vector3d(2,1,1.06));  
 //   std::cout<<"\n assign pointer\n";
 // walkerptr->SetScriptTime(2);
 // walkerptr -> SetAnimation(panimate);
  std::cout<<"\n setting animation true\n";


  }
  **/
}



void TaskCarryMyLuggage::SendMessageToRobot(std::string message, ros::Duration delay)
{
  last_talk_ = ros::Time::now();
  ROS_DEBUG_STREAM("say: " << message << " in " << delay.toSec());
  std::thread(&TaskCarryMyLuggage::TalkThread, this, message, delay).detach();
}

void TaskCarryMyLuggage::DisplayGuiMessage(const std::string& message)
{
  pub_gui_msg_->Publish(gazebo::msgs::ConvertAny(message));
}

void TaskCarryMyLuggage::TalkThread(std::string message, const ros::Duration& delay)
{
  delay.sleep();
  DisplayGuiMessage(config_->id_operator + ": " + message);
  std_msgs::String msg;
  msg.data = std::move(message);
  this->speech_publisher_.publish(msg);
}


void TaskCarryMyLuggage::OnUpdate(const gazebo::common::UpdateInfo& /*info*/)
{
  return;
  if (!walkerptr)
  {
   gazebo::physics::EntityPtr eptr = world_->EntityByName(config_->id_operator);
   if (eptr )
   {
   walkerptr = new gazebo::physics::Actor(eptr);   
   
   }
   return;
  }
  else
  {
   //std::cout<<"Found - "<<config_->id_operator<<"\n";
  }
  
  #if 0
  
  if (!started_)
  {
    return;
  }
  if (task_->GetScorePtr()->GetRemainingTime() <= ros::Duration(0))
  {
   // ScoreTask();
    return;
  }

//  auto score = task_->Update(world_);
//  pub_->Publish(gazebo::msgs::ConvertAny(score.GenerateHTMLScoreString()));

//  auto phase = task_->GetCurrentPhase();
//  ROS_DEBUG_STREAM_THROTTLE(5, "current phase: " << phase);

  auto operatorpose = world_->EntityByName(config_->id_operator)->WorldPose();
  auto opxy = ignition::math::Vector2d(operatorpose.Pos().X(), operatorpose.Pos().Y());

  // todo? move stuff to task setup calls?
  // should do all distance checking there
  
  if (phase == TaskPhase::INIT)
  {
    if (ros::Time::now() > last_talk_ + config_->talk_repeat_delay)
    {
      SendMessageToRobot(commands::MSG_RO_BAG);
    }
  }
  else if (phase == TaskPhase::INTRODUCTION)
  {
    if (ros::Time::now() > last_talk_ + config_->talk_repeat_delay)
    {
      SendMessageToRobot(commands::MSG_RO_START);
    }
  }
  else if (phase == TaskPhase::FOLLOWING)
  {
    auto target = config_->waypoints.back();
    ignition::math::Vector2d targetxy = ignition::math::Vector2d(target.X(), target.Y());
    double dist = opxy.Distance(targetxy);
    ROS_DEBUG_THROTTLE(2, "target following, x: %f, y: %f, dist: %f", target.X(), target.Y(), dist);
    bool finished = dist < 0.01;
    if (finished)
    {
      StartCarReached();
    }
  }
  else if (phase == TaskPhase::HANDOVER)
  {
    if (ros::Time::now() > last_talk_ + config_->talk_repeat_delay)
    {
      SendMessageToRobot(commands::MSG_RO_TARGET_REACHED);
    }
  }
  #endif
  
}


// void TaskStoringGroceries::CbGUI(const std::string& msg) {
void TaskCarryMyLuggage::CbGui(const boost::shared_ptr<const gazebo::msgs::Any>& msg)
{
  
  std::cout<<"Inside CbGui - "<<msg->string_value()<<"\n";
  
  if (msg->string_value() == "Spawn Actor")
  {
    std::cout<<"Spawning Actor\n";
    SpawnActors();
    //InitializeTask();
  }
  else if (msg->string_value() == "Start Walk")
  {
    std::cout<<"Starting Animation\n";
    startanim();
    
    //ScoreTask();
  }
  else if (msg->string_value() == "Reset")
  {
    std::cout<<"Removing Actor\n";
    resetmodel();
    std::cout<<"Removed Actor\n";
    
    //ScoreTask();
  }
}


// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(TaskCarryMyLuggage)
