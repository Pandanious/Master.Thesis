#! /usr/bin/env python3

#created by Soham Panda as part of Master thesis at Uni Bielefeld

import rospy
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import Pose

class GazeboLinkPose:
  link_name = ''
  link_pose = Pose()
  def __init__(self, link_name):
    self.link_name = link_name
    self.link_name_rectified = link_name.replace("::", "_")

    if not self.link_name:
      raise ValueError("'link_name' is an empty string")

    
    
    self.pose_pub = rospy.Publisher("/gazebo/" + self.link_name_rectified, Pose, queue_size = 10)
    self.states_sub = rospy.Subscriber("/gazebo/link_states", LinkStates, self.callback)
    

  def callback(self, data):
    try:
      ind = data.name.index(self.link_name)
      self.link_pose = data.pose[ind]
      self.pose_pub.publish(self.link_pose) 
    except ValueError:
      pass

    ## LEFT FOOT X=2.13 Y = 3.19
    ## RIGHT FOOT X=1.92 Y = 3.82

    #0.21

if __name__ == '__main__':
  try:
    rospy.init_node('gazebo_link_pose', anonymous=True)
    
    rp = GazeboLinkPose('tiago::base_footprint')
    hp = GazeboLinkPose('tiago::head_2_link')
    gp = GazeboLinkPose('walker::walker_pose')
    rf = GazeboLinkPose('walker::LeftFoot')
    lf = GazeboLinkPose('walker::RightFoot')
    cb = GazeboLinkPose('cracker_box::link')
    #publish_rate = rospy('~publish_rate', 10)

    #rate = rospy.Rate(publish_rate)
    while not rospy.is_shutdown():
      #gp.pose_pub.publish(gp.link_pose)

      rospy.spin()
  except rospy.ROSInterruptException:
    pass