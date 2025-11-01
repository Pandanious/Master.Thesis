#include "rcsim_task_carry_my_luggage/cml_randomizer.h"


CmlRandomizer::CmlRandomizer(const sdf::ElementPtr& sdf_config, std::mt19937* rng, std::string tiny_object)
{
	sdf_config_ = sdf_config;
	rng_ = rng;
	tiny_object_ = std::move(tiny_object);
}

void CmlRandomizer::Randomize(const gazebo::physics::WorldPtr& world, TaskConfig* config)
{
	std::list<Path*> paths;
	sdf::ElementPtr elem = sdf_config_->GetFirstElement();
  while (elem)
  {
  	if (elem->GetName() == "paths")
    {
    	sdf::ElementPtr sdf_paths = elem->GetFirstElement();
    	while (sdf_paths)
    	{
    		if (sdf_paths->GetName() == "path")
		    {
		    	sdf::ElementPtr sdf_path = sdf_paths->GetFirstElement();
		    	Path* path;
		    	std::list<m::Vector3d> waypoints;
		    	m::Pose3d car_pose;
		    	while(sdf_path)
		    	{
		    		if (sdf_path->GetName() == "waypoints")
		    		{
		    			sdf::ElementPtr waypoint = sdf_path->GetFirstElement();
				      while (waypoint)
				      {
				        m::Vector3d wp;
				        waypoint->GetValue()->Get(wp);
				        waypoints.push_back(wp);
					      waypoint = waypoint->GetNextElement();
				      }
		    		}
		    		else if (sdf_path->GetName() == "car")
		    		{
		    			sdf_path->GetValue()->Get(car_pose);
		    		}
		    		sdf_path = sdf_path->GetNextElement();
		    	}
		    	path = Path::Init(waypoints, car_pose);
		    	paths.push_back(path);
		    }
		    sdf_paths = sdf_paths->GetNextElement();
    	}    	
    }
    elem = elem->GetNextElement();
  }
	RandomPath(world, config, paths);
	if (!tiny_object_.empty())
	{
		PlaceTinyObject(config);
	}
}

void CmlRandomizer::RandomPath(const gazebo::physics::WorldPtr& world, TaskConfig* config, std::list<Path *> paths)
{
	ShuffleList(&paths);
	path_ = *(paths.begin());
	// Set config values
	config->waypoints = path_->waypoints;
	config->goal = path_->waypoints.back();
	PlaceCar(world, path_->car_pose);
}

void CmlRandomizer::PlaceCar(const gazebo::physics::WorldPtr& world, const m::Pose3d& car_pose)
{
	std::string car_modelname = "low_poly_car";
	std::string car_objectname = "low_poly_car";
	std::string filename = gazebo::common::ModelDatabase::Instance()->GetModelFile("model://" + car_modelname);
  auto ss = std::ostringstream{};
  std::ifstream file(filename);
  ss << file.rdbuf();

  sdf::SDF sdf;
  sdf.SetFromString(ss.str());
  sdf::ElementPtr mp = sdf.Root()->GetElement("model");
  mp->GetAttribute("name")->SetFromString(car_objectname);
  mp->GetElement("pose")->GetValue()->Set(car_pose);

  world->InsertModelSDF(sdf);
}

void CmlRandomizer::PlaceTinyObject(TaskConfig* config)
{
	std::list<m::Vector3d> path_list = path_->waypoints;
	ShuffleList(&path_list);
	m::Vector3d to_pos1 = *(std::next(path_list.begin(), 0));
	m::Vector3d to_pos2 = *(std::next(path_list.begin(), 1));
	double pos_x = ( to_pos1.X() + to_pos2.X() ) / 2;
	double pos_y = ( to_pos1.Y() + to_pos2.Y() ) / 2;
	double pos_z = to_pos1.Z();
	m::Vector3d to_pos = { pos_x, pos_y, pos_z };
	// std::cout << "pos1: " << to_pos1 << std::endl << ", pos2: " << to_pos2 << std::endl << ", pos: " << to_pos << std::endl;
	m::Quaterniond to_rot(0,0,0);
	m::Pose3d to_pose = { to_pos, to_rot };
	config->tiny_object_pose = to_pose;
	/*
	std::string to_objectname = "tiny_object_" + tiny_object_;
	std::string filename = gazebo::common::ModelDatabase::Instance()->GetModelFile("model://" + tiny_object_);
  auto ss = std::ostringstream{};
  std::ifstream file(filename);
  ss << file.rdbuf();

  sdf::SDF sdf;
  sdf.SetFromString(ss.str());
  sdf::ElementPtr mp = sdf.Root()->GetElement("model");
  mp->GetAttribute("name")->SetFromString(to_objectname);
  mp->GetElement("pose")->GetValue()->Set(to_pose);

  world_->InsertModelSDF(sdf);
  */
}