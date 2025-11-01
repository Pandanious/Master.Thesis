#pragma once

#include <gazebo/gazebo.hh>

#include <gazebo/physics/physics.hh>

#include <sdf/sdf.hh>

inline sdf::ElementPtr CreateWaypointElement(std::list<ignition::math::Vector3d> waypoints, bool debug = false)
{
  sdf::ElementPtr root(new sdf::Element());
  root->SetName("waypoints");

  for (auto wp : waypoints)
  {
    sdf::ElementPtr element(new sdf::Element());
    element->SetName("wp");

    if (debug)
    {
      wp[2] = 2;
    }

    element->AddValue("ignition::math::Vector3d", "", false);
    element->GetValue()->Set(wp);

    root->InsertElement(element);
  }

  return root;
}

inline sdf::ElementPtr CreateStringElement(const std::string& name, const std::string& value)
{
  sdf::ElementPtr root(new sdf::Element());
  root->SetName(name);

  root->AddValue("std::string", "", false);
  root->GetValue()->Set(value);

  return root;
}