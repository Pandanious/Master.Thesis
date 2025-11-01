# Person Tracking

panda@uni-bielefeld.de

Laser and Camera based Person Tracker.

This code was created for the completion of masters thesis. In this article, the steps needed to start the program will be described. 
Python Scripts can be found under folder fusion-tracker.

For Gazebo Simulation.

The following steps are needed to start the gazebo simulation.
Launching the world file.
The  Gazebo world file along with many 3D modelled items have been compiled into a catkin package. 
Use: 
roslaunch gazebo_sim_walk sim_walk.launch 
to launch the simulation
The TIAGO robot can be spawned using:
roslaunch tobi_sim tiago_spawn.launch 
roslaunch tobi_sim tiago_bringup.launch 
Way points to make the person walk are mentioned in the world file of the package, in order to add way-points, edit them under the <waypoint> section in the world file inside the gazebo_sim_walk package.
The spawn location of the 3D model can be adjusted as well, this is the location the model will return to when reset is clicked. It can be changed by changing values under <start> in the same world file.
There are two levels of Laser scanners attached to the robot. There are a total of 4 trackers for the LRF and 3 for the camera sensors. All of these can be used in any permutation as wanted. 
Run “python2 trans.py” This publishes transformation information.
Run “python3 Gazebo.pose.py” This publishes data for evaluation purposes.
Run “python3 laser_tracking_fusion_gazebo.py <LRF tracker> <Camera tracker> <Laser height>
<LRF Sensor tracker> 
    1. MIL
    2. MOSSE
    3. CSRT
    4. Self-created Tracker
<The Camera tracker>:
    1. MIL
    2. MOSSE
    3. CSRT
<Laser height> :
1. 15 cm
2. 45 cm
Evaluation is optional. The data from the fusion tracking approach can be used further. 
Use “python3 evaluation_fusion_gazebo.py” for evaluation.
Output can be visualized in RVIZ. Global frame should be set to base_laser_link
LRF tracking output named: /polygon_estimate_laser, 
Camera tracking output named: /polygon_estimate_img. 
Left foot Pose:  “/left_foot”
Right foot Pose: ”/right_foot”
3D Model Pose: ”/pose_from_laser” 

----------------------------------------------------------------------------------------------------------------------

Real world application.

This program needs a ground truth file for proper evaluation. If such a file is not present for the present usage, the LRF and Camera initial bounding boxes have to be manually added. The get ROI function within the code allows that functionality. If you use that method, the selected ROI values are displayed in the terminal. Those can be used to update "default_lrf_box" and "default_camera_box" at the start of the code, so as to not select the ROI everytime it is run.

run “python2 trans.py” This publishes transformation information.
run “./fusionrealworld.sh <LRF tracker> <Camera tracker> <timeout in seconds>”
<LRF tracker> 
    1. MIL
    2. MOSSE
    3. CSRT
    4. Self-created Tracker
<The Camera tracker>:
    1. MIL
    2. MOSSE
    3. CSRT

Evaluation is optional. The data from the fusion tracking approach can be used further. 
Use “python3 evaluation_fusion_gazebo.py” for evaluation.
Output can be visualized in RVIZ. Global frame should be set to base_laser_link
LRF tracking output named: "/polygon_estimate_laser" 
Camera tracking output named: "/polygon_estimate_img"
Bayes people tracker Pose:  "/Bayes_position"
leg Detector left foot Pose:  “/leftfoot_detected”
leg Detector right foot Pose: ”/rightfoot_foot_detected”
Grouth truth area: ”/ground_truth_anno” 



# Latex-Documentation

See [docs/README](docs/README).

Compiled PDFs are downloadable from: [gitlab](https://gitlab.ub.uni-bielefeld.de/CLF/projects/theses/ma-person-tracking/-/jobs/artifacts/master/browse?job=artifacts)
