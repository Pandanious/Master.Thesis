#include <list>
#include <random>

#include <gazebo/physics/physics.hh>
//#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/common/common.hh>
#include <gazebo/util/system.hh>
#include <gazebo/gazebo.hh>

#include <sdf/sdf.hh>

#include <ros/ros.h>

#include <std_msgs/String.h>

//#include "gazebo_sim_walk/task_setup.h"
#include "gazebo_sim_walk/task_config.h"
//#include "gazebo_sim_walk/cml_randomizer.h"

namespace m = ignition::math;

class TaskCarryMyLuggage : public gazebo::WorldPlugin
{
public:
  void Load(gazebo::physics::WorldPtr world, sdf::ElementPtr sdf) override;

private:
  void OnUpdate(const gazebo::common::UpdateInfo& info);

  // void CbGUI(const std::string& msg);
  void CbGui(const boost::shared_ptr<const gazebo::msgs::Any>& msg);

  void CbOperator(const boost::shared_ptr<const gazebo::msgs::Any>& msg);
  void CbSpeech(const std_msgs::String::ConstPtr& msg);
  void SpawnActors();
  void resetmodel();
  void startanim();
 
  void SendMessageToRobot(std::string message, ros::Duration delay = ros::Duration(2));
  void DisplayGuiMessage(const std::string& message);
  void TalkThread(std::string message, const ros::Duration& delay);

  // state changes
  void SpawnBag();
  void StartFollowing();
  void PauseFollowing();
  void StartCarReached();
  
  // Using Mersenne Twister Pseudo-number generator
  std::mt19937* rng_{ nullptr };

 // TaskSetup* task_{ nullptr };
  TaskConfig* config_{ nullptr };
  bool started_{ false };
  bool actorspawn;
  ros::Time last_talk_;

  std::vector<gazebo::event::ConnectionPtr> connections_;
  
  gazebo::physics::WorldPtr world_;
  gazebo::physics::Actor *walkerptr { nullptr };
  std::string tinyObject_;

  gazebo::transport::NodePtr node_;
  gazebo::transport::PublisherPtr pub_;
  gazebo::transport::SubscriberPtr sub_;

  gazebo::transport::PublisherPtr pub_gui_msg_;

  gazebo::transport::SubscriberPtr suboperator_;
  gazebo::transport::PublisherPtr puboperator_;

  ros::NodeHandle nh_;
  ros::Subscriber speech_subscriber_;
  ros::Publisher speech_publisher_;
};
