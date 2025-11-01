#include "gazebo_sim_walk/task_config.h"

Spectator Spectator::FromElement(const sdf::ElementPtr& spectator)
{
  Spectator ret;

  sdf::ElementPtr elem = spectator->GetFirstElement();
  while (elem)
  {
    if (elem->GetName() == "waypoints")
    {
      sdf::ElementPtr waypoint = elem->GetFirstElement();
      while (waypoint)
      {
        ignition::math::Vector3d wp;
        waypoint->GetValue()->Get(wp);
        ret.waypoints.push_back(wp);
        waypoint = waypoint->GetNextElement();
      }
    }
    else if (elem->GetName() == "name")
    {
      elem->GetValue()->Get(ret.name);
    }

    elem = elem->GetNextElement();
  }

  return ret;
}

TaskConfig* TaskConfig::FromElement(const sdf::ElementPtr& config)
{
  auto* task = new TaskConfig();
  
  sdf::ElementPtr elem = config->GetFirstElement();
  std::cout<<"\n Start Task config from element \n";
  while (elem)
  { 
  
    std::cout<<"\n elem name:  "<<elem ->GetName()<<"\n";
    if (elem->GetName() == "waypoints")
    {
      sdf::ElementPtr waypoint = elem->GetFirstElement();
      while (waypoint)
      {
        ignition::math::Vector3d wp;
        waypoint->GetValue()->Get(wp);
        task->waypoints.push_back(wp);
        //task->waypoints.push_back(wp);
        waypoint = waypoint->GetNextElement();
      }
    }
    else if (elem->GetName() == "spectators")
    {
      sdf::ElementPtr spectator = elem->GetFirstElement();
      while (spectator)
      {
        task->spectators.push_back(Spectator::FromElement(spectator));
        spectator = spectator->GetNextElement();
      }
    }
    else if (elem->GetName() == "seed")
    {
      elem->GetValue()->Get(task->seed);
    }
    else if (elem->GetName() == "start")
    {
      elem->GetValue()->Get(task->start);
    }
    else if (elem->GetName() == "debug")
    {
      elem->GetValue()->Get(task->demo_mode);
    }
    else if (elem->GetName() == "id_robot")
    {
      elem->GetValue()->Get(task->id_robot);
    }
    else if (elem->GetName() == "id_operator")
    {
      elem->GetValue()->Get(task->id_operator);
      std::cout<<"\n Operator Name: "<<task->id_operator<<"\n";    
    }
    else if (elem->GetName() == "max_goal_dist")
    {
      elem->GetValue()->Get(task->max_goal_dist);
    }
    else if (elem->GetName() == "max_follow_dist")
    {
      elem->GetValue()->Get(task->max_follow_dist);
    }
    else if (elem->GetName() == "topic_in")
    {
      elem->GetValue()->Get(task->topic_in);
    }
    else if (elem->GetName() == "topic_out")
    {
      elem->GetValue()->Get(task->topic_out);
    }
    else if (elem->GetName() == "max_stop_time")
    {
      double val;
      elem->GetValue()->Get(val);
      task->max_stop_time = ros::Duration(val);
    }
    else
    {
    }
    elem = elem->GetNextElement();
  }
  

  return task;
}

std::ostream& operator<<(std::ostream& out, const TaskConfig& tc)
{
  out << "seed:" << tc.seed << std::endl;
  out << "waypoints:" << std::endl;
  for (const auto& wp : tc.waypoints)
  {
    out << " - " << wp << std::endl;
  }

  return out;
}