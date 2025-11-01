/*
 * Copyright (C) 2014 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#include <sstream>
#include <gazebo/msgs/msgs.hh>
#include "gazebo_sim_walk/gazebo_sim_gui.hh"
#include <ros/ros.h>

using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_GUI_PLUGIN(Gazebo_SGUI)

/////////////////////////////////////////////////
Gazebo_SGUI::Gazebo_SGUI()
  : GUIPlugin()
{
  std::cout<<"Inside Constructor\n";
  // Set the frame background and foreground colors
  this->setStyleSheet(
      "QFrame { background-color : rgba(100, 100, 100, 255); color : white; }");
  this->setStyleSheet("QFrame { background-color : rgba(100, 100, 100, 255); color : white;}");
  
  auto* main_layout = new QVBoxLayout;

  auto* layout = new QHBoxLayout();
  layout->setContentsMargins(0, 0, 0, 0);
  auto* row = new QWidget();
  row->setLayout(layout);

  // add buttons and timer
  spawn_button = new QPushButton(tr("Spawn Actor"));
  spawn_button->setEnabled(true);
  layout->addWidget(spawn_button);

  anim_button = new QPushButton(tr("Start Animation"));
  anim_button->setEnabled(false);
  layout->addWidget(anim_button);

  reset_button = new QPushButton(tr("Reset Actor"));
  reset_button->setEnabled(false);
  layout->addWidget(reset_button);

  // Add the frame to the main layout
  main_layout->addWidget(row);
  layout->setContentsMargins(0, 0, 0, 0);
  main_layout->setContentsMargins(0, 0, 0, 0);

  this->setLayout(main_layout);
  this->move(10, 10);
  this->resize(400, 50);
  connect(spawn_button, SIGNAL(clicked()), this, SLOT(OnSpawn()));
  connect(anim_button, SIGNAL(clicked()), this, SLOT(OnAnimation()));
  connect(reset_button, SIGNAL(clicked()), this, SLOT(OnReset()));

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init();
  this->Pub = this->node->Advertise<msgs::Any>("~/gazebo_sim_gui");

}
Gazebo_SGUI::~Gazebo_SGUI()
{}

void Gazebo_SGUI::OnSpawn()
{ 
  std::cout<<"Spawn Button\n";
  this->spawn_button->setEnabled(false);
  this->anim_button->setEnabled(true);
  this->Pub->Publish(gazebo::msgs::ConvertAny("Spawn Actor"));
}
void Gazebo_SGUI::OnAnimation()
{ 
  std::cout<<"Animate Button\n";
  this->anim_button->setEnabled(false);
  this->reset_button->setEnabled(true);
  this->Pub->Publish(gazebo::msgs::ConvertAny("Start Walk"));
}
void Gazebo_SGUI::OnReset()
{ 
  std::cout<<"Reset Button\n";
  this->reset_button->setEnabled(false);
  this -> anim_button -> setEnabled(true);
  this->Pub->Publish(gazebo::msgs::ConvertAny("Reset"));
}

