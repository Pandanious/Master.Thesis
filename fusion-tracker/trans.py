#!/usr/bin/env python3.10  

#created by Soham Panda as part of Master thesis at Uni Bielefeld

import roslib
import rospy
import math
import tf2_ros
import PyKDL
from geometry_msgs.msg import TransformStamped
from math import pi
from std_msgs.msg import Float32MultiArray,MultiArrayLayout,MultiArrayDimension
import numpy as np

#'xtion_depth_frame'
if __name__ == '__main__':
    rospy.init_node('tf_echo')
    pub = rospy.Publisher('transform',TransformStamped,queue_size=10)
    tfbuffer = tf2_ros.Buffer()

    listener = tf2_ros.TransformListener(tfbuffer)
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            transform = tfbuffer.lookup_transform('base_laser_link','xtion_depth_frame' , rospy.Time())
            #print(transform)
            trans = TransformStamped()
            trans.header = transform.header
            trans.transform = transform.transform
            pub.publish(trans)
            #print('In try')
            #print (trans.transform)
            trans = trans.transform
            
            xori = trans.rotation.x
            yori = trans.rotation.y
            zori = trans.rotation.z
            wori = trans.rotation.w
            xx = xori*xori
            xy = xori*yori
            xz = xori*zori
            xw = xori*wori
            yy = yori*yori
            yz = yori*zori
            yw = yori*wori
            zz = zori*zori
            zw = zori*wori
            ww = wori*wori

            m00  = 1 - (2*( yy + zz ))
            m01  =    2 * ( xy - zw )
            m02 =     2 * ( xz + yw )

            m10  =     2 * ( xy + zw )
            m11  = 1 - (2 * ( xx + zz ))
            m12  =     2 * ( yz - xw )

            m20  =     2 * ( xz - yw )
            m21  =     2 * ( yz + xw )
            m22 = 1 - 2 * ( xx + yy )

            m03 = trans.translation.x
            m13 = trans.translation.y
            m23 = trans.translation.z
            m30 = 0
            m31 = 0
            m32 = 0
            m33 = 1

            M = [[m00,m01,m02,m03],[m10,m11,m12,m13],[m20,m21,m22,m23],[m30,m31,m32,m33]]
            #print(M)
            point = [1.6303264, -0.11083412621427319, -0.45426381307540137,1]
            transformedPOINT = np.matmul(M,point)
            #print('Point : ',point)
            #print('height : ',m23-transformedPOINT[2])
            #print('Transformed: ', transformedPOINT)
            
             





            tan1 = 2*((wori*zori)+(xori*yori))
            tan2 = 1 - (2*((yori*yori)+(zori*zori)))
            theta = math.atan2(tan1,tan2)
            z_angle = math.degrees(theta)
            #print('Theta in RAD : ',theta)
            #print('Theta in Deg : ',z_angle)


            rot = PyKDL.Rotation.Quaternion(* [ eval('trans.rotation.'+c) for c in 'xyzw'] )
            #print (' '.join( [ str(eval('trans.rotation.'+c)) for c in 'xyzw'] ))
            ypr = [ i  / pi * 180 for i in rot.GetEulerZYX() ]

            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            print ("Fail", e)
        


        #rate.sleep()