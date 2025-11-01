#! /usr/bin/env python3

#created by Soham Panda as part of Master thesis at Uni Bielefeld
from array import array
from cmath import sin
import ctypes
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray , PolygonStamped , Point32  
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan , PointCloud2, Image
import laser_geometry.laser_geometry as lg
import sensor_msgs.point_cloud2 as pc2
import math
import struct
import cv2
import numpy as np
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import Pose
from ptrackerlib import person_tracker
import message_filters
from geometry_msgs.msg import TransformStamped
import sys
import time




##discsize = .02
##discfactor = 1/discsize
default_lrf_box = [100,100,100,100] ## StartX,StartY,width,height. THis should be entered for when using no groundtruth file. 
default_camera_box = (85, 5, 320, 465)
pixel_multi = 100
laser_range = 4
min_angle = -1.91986000538
max_angle = 1.91986000538
angle_increment = 0.00577401509508
threshold = 8
#2 cm between two pixel.
metertopixel = 2*100 
max_range = laser_range*metertopixel

increments = int((max_angle-min_angle)/angle_increment)
print('number of increments',increments)
image_size_height = max_range
image_size_width = max_range 

print('img size height' , image_size_height ,'img size width' , image_size_width)


def callback(self, data):
    try:
      ind = data.name.index(self.link_name)
      self.link_pose = data.pose[ind]
      #print(self.link_pose)
    except ValueError:
      pass



class scan_to_image:
  def __init__(self,tracker_lrf,tracker_cam, timeout):
    
    self.timeout = timeout
    self.time_first_laser_callback = 0
    self.lp = lg.LaserProjection()
    self.pc_pub = rospy.Publisher("pc2img", PointCloud2, queue_size = 1)
    self.do_correction = True
    ##self.imgpub = rospy.Publisher("scan_to_image",Image, queue_size = 10)
    self.imgcase1 = False
    
    ##self.orientation.publish()
    ##self.imgsub = rospy.Subscriber("scan_to_image", Image, self.tracker)
    self.imstr = 0
    self.case = False 
    self.case1 = False 
    self.case2 = False
    self.tbox = []
    self.no = 0
    self.owntracker = False
    self.do_reinit = True
    self.laser_debug = False   
    self.camera_debug = False
    self.head = None
    self.transform = None
    self.poly_msg_img = None
    self.firsttime = True
    
    self.gt_path = './groundtruth/straight_follow_corner_box.txt'
    
    
    
    
    self.gt_path = './groundtruth/crossing2.txt'
    self.gt_path = './groundtruth/crossing.txt'
    self.gt_path = './groundtruth/walking.txt'
    self.gt_path = './groundtruth/fix_follow_down.txt'
    
    self.gt_path = './groundtruth/fix_follow_corner_box2.txt'
    self.gt_path = './groundtruth/fix_follow_s.txt'
    self.gt_path = None
    if self.gt_path != None:
      self.gt = open(self.gt_path,'r')  ## add ground truth file, either the ones above or the one created. 
    else:
        self.gt = None
    print('OpenCV Version : {0}'.format(cv2.__version__) )

    self.laser_found_count = 0
    self.laser_found_none_count = 0
    self.camera_found_count = 0
    self.camera_found_none_count = 0
    self.imcallbackcount = 0
    self.gtfilecount = 0
    self.lasercallbackcount = 0
    self.do_shutdown = False

    self.K = [589.8432643775249, 0.0, 335.8137957341262, 0.0, 592.9344846013555, 229.9298265156391, 0.0, 0.0, 1.0]
    self.cx = self.K[2]
    self.fx = self.K[0]
    self.fy = self.K[4]
    self.cy = self.K[5]
    self.last_camera_track_ok = False

    if tracker_lrf == '1':
       self.tracker_lrf = cv2.TrackerMIL_create()
       
    elif tracker_lrf == '2':
       self.tracker_lrf = cv2.legacy.TrackerMOSSE_create()
       #self.tracker_lrf = cv2.TrackerMOSSE_create()
       
    elif tracker_lrf == '3':
       self.tracker_lrf = cv2.legacy.TrackerCSRT_create()
       #self.tracker_lrf = cv2.TrackerCSRT_create()
       
    elif tracker_lrf == '4':
       self.owntracker = True
       self.ptracker = person_tracker()

    else:
       print('Wrong Tracker type chosen, defaulting to MIL Tracker')
       self.tracker_lrf = cv2.TrackerMIL_create()
       
    if tracker_cam == '1':
       self.tracker_cam = cv2.TrackerMIL_create()
    elif tracker_cam == '2':
       self.tracker_cam = cv2.legacy.TrackerMOSSE_create()
       #self.tracker_cam = cv2.TrackerMOSSE_create()
    elif tracker_cam == '3':
       self.tracker_cam = cv2.legacy.TrackerCSRT_create()       
       #self.tracker_cam = cv2.TrackerCSRT_create()       
    else:
       print('Wrong Tracker type chosen, defaulting to MIL Tracker')
       self.tracker_cam = cv2.TrackerMIL_create()


    self.bbox = [435,890,50,70]
    self.counter = 0
    
  def scan_data(self):
    rospy.init_node('Scan_Data', anonymous=True)
    print('after init')

    
    sub_dep_cam = message_filters.Subscriber("/xtion/depth_registered/image_raw",Image)
    sub_cam = message_filters.Subscriber("/xtion/rgb/image_raw",Image)
    sub_laser = message_filters.Subscriber("scan_clf",LaserScan)
    trans_sub = rospy.Subscriber('/transform',TransformStamped,self.trans_callback)
    
    self.real_pub = rospy.Publisher('position_estimate', PoseArray, queue_size=10 )
    self.position_pub =  rospy.Publisher("polygon_estimate_laser",PolygonStamped,queue_size=10)
    self.position_pub_img =  rospy.Publisher("polygon_estimate_img",PolygonStamped,queue_size=10)
    self.gt_pub=  rospy.Publisher("ground_truth_anno",PolygonStamped,queue_size=10)

    self.ts = message_filters.ApproximateTimeSynchronizer([sub_laser,sub_dep_cam,sub_cam],10,0.1)        
    self.ts.registerCallback(self.laser_camera_cb)
    self.shutdown_pub =  rospy.Publisher("evaluation_msg",String,queue_size=10)
    eval_sub = rospy.Subscriber('evaluation_msg',String, self.eva)
    
    rospy.spin()
  
  def real_pose(self,msg):
    self.realpose = msg
    box_start_x = self.realpose.poses[0].position.x
    box_start_y = self.realpose.poses[0].position.y

    box_end_x = self.realpose.poses[1].position.x
    box_end_y = self.realpose.poses[1].position.y

    box_x = (box_start_x - box_end_x)
    box_y = (box_start_y - box_end_y)
    center_x = (box_start_x + box_end_x)/2
    center_y = (box_start_y + box_end_y)/2
    area = box_y*box_x

  def trans_callback(self, msg):
    self.transform = msg
    
  def eva(self,msg):
        self.do_shutdown = True
        stime = self.time_first_laser_callback
        etime = time.time()
        print("Shutdown : groundtruth lines read- ", self.gtfilecount, " time elapse - ", etime-stime, " seconds")
        rospy.signal_shutdown('Complete Evaluation')
    
  def get_intersection_ratio(self,x1,y1,w1,h1,x2,y2,w2,h2):
        w_intersect = min(x1+w1, x2+w2) - max(x1, x2)
        h_intersect = min(y1+h1, y2+h2) - max(y1, y2)
        if  w_intersect <= 0 or h_intersect <= 0:
            ratio = 0
        else:
            I = w_intersect * h_intersect
            ratio = I/(w2*h2)
        return ratio
 
  

  def groundtruth(self,startX,startY,endX,endY):
   
      theta = 0
      x1 = startX+0.202
      y1 = startY
      x2 = endX+0.202
      y2  = endY
      tx = 0
      ty = 0
      startarray = [(x1),(y1),1]
      transform = [(np.cos(theta),-np.sin(theta),tx),
               (np.sin(theta),np.cos(theta),ty),
                (0,0,1)]
      endarray = [(x2),(y2),1]
      transformstart = np.matmul(transform,startarray)
      transformend = np.matmul(transform,endarray)
      #print(transformstart,transformend)
      return transformstart,transformend

  def getpicarray(self,xarray,yarray,image_size_height,image_size_width):

      xmin = min(xarray)
      ymin = min(yarray)
      if (xmin < 0):
        xmin = -xmin
      else:
        xmin = 0
      if (ymin < 0):
        ymin = -ymin
      else:
        ymin = 0 
      picxarray = []
      picyarray = []
      length = len(xarray)
      #print('Xmin=',xmin,'ymin=',ymin)
      #print('Xmax=',max(xarray),'ymax=',max(yarray))
      img = np.zeros((image_size_height,image_size_width),dtype=np.uint8)
      for i in range(0,length):
          xpt = xarray[i]
          ypt = yarray[i]
          #print('xpt,ypt', xpt,ypt)

          picy = int(image_size_height-xpt)
          picx = int(image_size_width/2 - ypt)
          if picx < 0 or picy <0:
            print('negative X and Y', picx, picy)
          if (picy >= image_size_height) or (picx >= image_size_width):
            print('Image Size Error','picx = ',picx,'picy=',picy,'img width',image_size_width,'img height',image_size_height)
          else:
            img[int(picy),int(picx)] = 255 

      self.imstr += 1
      return img
  
  def scan_cb(self, msg):
      
      if self.do_shutdown == True:
           return
      self.lasercallbackcount += 1
      self.laser_message = msg
      self.head = self.laser_message.header
      pc2_msg = self.lp.projectLaser(msg)
      leng = len(pc2_msg.data)
      step = pc2_msg.point_step
      c = 0
      xarray = np.empty([leng])
      yarray = np.empty([leng])
      
      for i in range(0, leng, step ):  
        x = struct.unpack('f',pc2_msg.data[i:i+4])[0]
        y = struct.unpack('f',pc2_msg.data[i+4:i+8])[0]
        if x >= 0 and x <= laser_range and abs(y) <= laser_range/2:
          
          xarray[c] = x*metertopixel
          yarray[c] = y*metertopixel
          c = c+1
      xarray.resize([c])
      yarray.resize([c])    
      
      #print('X MIN and MAX:',min(xarray),max(xarray))
      #print('Y MIN AND MAX', min(yarray),max(yarray))
      
      img = self.getpicarray(xarray,yarray,image_size_height,image_size_width)
    
      if self.case1 == False:
            self.time_first_laser_callback = time.time()
            if self.gt != None:
              startX,startY,endX,endY,poly_im = self.gt_polygon()
                   
            else:
              poly_im = default_lrf_box
              if self.owntracker == True:
                  roi = [poly_im[0]-40, poly_im[1]-40, poly_im[2]+80,poly_im[3]+80]
              else:
                  roi = (poly_im[0], poly_im[1], poly_im[2],poly_im[3])

            
            #roi =  cv2.selectROI('Image TracK Selection',img)      #enables manual selection of Region of Interest. This value is printed on the terminal, can be used to update default_lrf_box, to start without selecting ROI. 
            print("Laser - ROI - ", roi)
            self.tbox =  [roi[0], roi[1], roi[0]+roi[2], roi[1]+roi[3]]
            self.bbox[0] = roi[0]
            self.bbox[1] = roi[1]
            self.bbox[2] = roi[2]
            self.bbox[3] = roi[3]
            
            if self.gt != None:
              gt_msg = PolygonStamped()
              gt_msg.header = self.laser_message.header
              gt_msg.polygon.points =  [Point32(x=startX, y=startY, z=0),
                                          Point32(x=endX, y=startY, z=0),
                                          Point32(x=endX, y=endY, z=0),
                                          Point32(x=startX, y=endY, z=0)]
              self.gt_pub.publish(gt_msg)
          
            
            #for start of bounding box
            opicx = (self.bbox[0])
            opicy = (self.bbox[1])
            startX = (image_size_height - opicy)/metertopixel
            startY = (image_size_width/2 - opicx)/metertopixel
            #print('X,Y',startX,startY)
            #print('X,Y ',opicx,opicy)
            
            #for end of bounding box

            opicx = (self.bbox[0]+self.bbox[2])
            opicy = (self.bbox[1]+self.bbox[3])
            endX = (image_size_height - opicy)/metertopixel
            endY = (image_size_width/2 - opicx)/metertopixel
            #print('End X,Y ',endX,endY)
            #print('End X,Y ',opicx,opicy)
            
            if self.owntracker == False:
              self.tracker_lrf.init(img,roi)
            else:
               im = str(self.imstr)
               file_name = './origimg/RealScan3-'+im+'.png'
               #cv2.imwrite(file_name, img)
               ok, bbox = self.ptracker.init(img, self.tbox, threshold, False, True)
               if ok == False:
                 print("ptracker init error")
               else:
                 print("ptracker init ok")
               im = str(self.imstr)
               if self.laser_debug == True:
                   file_name = './img/RealScan3-'+im+'.png'
                   #cv2.rectangle(img, (self.tbox[0], self.tbox[1]), (self.tbox[2],self.tbox[3]),100,1)
                   #cv2.rectangle(img, (poly_im[0], poly_im[1]), (poly_im[0]+poly_im[2],poly_im[1]+poly_im[3]),100,1)
                   #cv2.imwrite(file_name, img)

            p1 = (int(self.bbox[0]), int(self.bbox[1]))
            p2 = (int(self.bbox[0] + self.bbox[2]), int(self.bbox[1] + self.bbox[3]))
            #print('Selected x y coordinates:', p1, p2)
            self.case1 = True

      else: 
            
            curr_time = time.time()
            if (curr_time - self.time_first_laser_callback) > self.timeout:
               self.shutdown_pub.publish("Shutdown")
               self.do_shutdown = True
               return
               
            if self.gt != None:
              startX,startY,endX,endY,poly_im = self.gt_polygon()

            
            if self.do_shutdown == True:
              return 

            if self.owntracker == True:

              im = str(self.imstr)
              file_name = './origimg/RealScan3-'+im+'.png'
              #cv2.imwrite(file_name, img)
              temp_bbox = self.ptracker.get_bbox()
              ok, boundingbox = self.ptracker.update(img, False, True)
              cx = int(boundingbox[0] + boundingbox[2]/2)
              cy = int(boundingbox[1] + boundingbox[3]/2)
              if self.do_reinit == True and self.last_camera_track_ok == True and self.cam_sy > 0: 
                    x1 = self.cam_sx 
                    y1 = self.cam_sy 
                    w1 = self.cam_ex - self.cam_sx
                    h1 = self.cam_ey - self.cam_sy
                    x2 = boundingbox[0]
                    y2 = boundingbox[1]
                    w2 = boundingbox[2]
                    h2 = boundingbox[3]
                    if self.get_intersection_ratio(x1,y1,w1,h1,x2,y2,w2,h2) < 0.05:
                        sx1 = max(self.cam_sx-40, 0)
                        sy1 = max(self.cam_sy-40, 0)
                        ex1 = min(self.cam_ex+40, 799)
                        ey1 = min(self.cam_ey+40, 799)
                        tbox = [sx1, sy1, ex1, ey1]
                        ok, boundingbox = self.ptracker.init(img, tbox, threshold, False, True)
                        print("reinitialized tracker at ", self.imstr)


              if self.laser_debug == True:
                  im = str(self.imstr)
                  file_name = './img/RealScan3-'+im+'.png'
                  #cv2.rectangle(img, (temp_bbox[0], temp_bbox[1]), (temp_bbox[2],temp_bbox[3]),100,1)
                  #cv2.rectangle(img, (poly_im[0], poly_im[1]), (poly_im[0]+poly_im[2],poly_im[1]+poly_im[3]),100,1)
                  if  self.last_camera_track_ok == True and self.cam_sy >0:
                     cv2.rectangle(img, (self.cam_sx, self.cam_sy), (self.cam_ex, self.cam_ey),200,1)
                  cv2.imwrite(file_name, img)
            else:
              ok,boundingbox = self.tracker_lrf.update(img) 
              if self.do_reinit == True and self.last_camera_track_ok == True and self.cam_sy > 0: 
              
                    x1 = self.cam_sx 
                    y1 = self.cam_sy 
                    w1 = self.cam_ex - self.cam_sx
                    h1 = self.cam_ey - self.cam_sy
                    x2 = boundingbox[0]
                    y2 = boundingbox[1]
                    w2 = boundingbox[2]
                    h2 = boundingbox[3]
                    if self.get_intersection_ratio(x1,y1,w1,h1,x2,y2,w2,h2) < 0.05:
                        sx1 = max(self.cam_sx-40, 0)
                        sy1 = max(self.cam_sy-40, 0)
                        ex1 = min(self.cam_ex+40, 799)
                        ey1 = min(self.cam_ey+40, 799)
                        boundingbox = (sx1, sy1, ex1-sx1, ey1-sy1)
                        self.tracker_lrf.init(img, boundingbox)
                        print("reinitialized tracker at ", self.imstr)

            if self.gt != None:
              gt_msg = PolygonStamped()
              gt_msg.header = self.laser_message.header
              gt_msg.polygon.points =  [Point32(x=startX, y=startY, z=0),
                                          Point32(x=endX, y=startY, z=0),
                                          Point32(x=endX, y=endY, z=0),
                                          Point32(x=startX, y=endY, z=0)]
              self.gt_pub.publish(gt_msg)

            if ok:
                self.laser_found_count += 1
                #print("laser: boundingbox after update : \n", boundingbox)
                p1 = (int(boundingbox[0]), int(boundingbox[1]))
                p2 = (int(boundingbox[0] + boundingbox[2]), int(boundingbox[1] + boundingbox[3]))
                startX = (image_size_height - p1[1])/metertopixel
                startY = (image_size_width/2 - p1[0])/metertopixel
                endX = (image_size_height - p2[1])/metertopixel
                endY = (image_size_width/2 - p2[0])/metertopixel
                sx,sy = self.groundtruth(startX,startY,endX,endY)
                pose_start = Pose()
                pose_start.position.x = sx[0]
                pose_start.position.y = sx[1]
                pose_start.position.z = 0
                pose_start.orientation.x = pose_start.orientation.y = pose_start.orientation.z = pose_start.orientation.w = 0
                
                pose_end = Pose()
                pose_end.position.x = sy[0]
                pose_end.position.y = sy[1]
                pose_end.position.z = 0
                pose_end.orientation.x = pose_end.orientation.y = pose_end.orientation.z = pose_end.orientation.w = 0

                self.person_pose = PoseArray()
                self.person_pose.header = self.laser_message.header
                self.person_pose.poses = [pose_start,pose_end]
                
                self.real_pub.publish(self.person_pose)
                poly_msg = PolygonStamped()
                poly_msg.header = self.laser_message.header
                poly_msg.polygon.points =  [Point32(x=startX, y=startY, z=0),
                                            Point32(x=endX, y=startY, z=0),
                                            Point32(x=endX, y=endY, z=0),
                                            Point32(x=startX, y=endY, z=0)]
                self.position_pub.publish(poly_msg)
                if self.last_camera_track_ok == False :
                    return
                if self.poly_msg_img != None:
                    self.position_pub_img.publish(self.poly_msg_img)
                
              
            else:
                self.laser_found_none_count += 1
                print('Tracking Update error')      
      
  def gt_polygon(self):
    self.gtfilecount += 1 
    if self.gt == None:
          return 0,0,0,0,[0,0,0,0]
    st = self.gt.readline()
    if st == '':
        print("groundtruth file over at ", self.imcallbackcount, self.imstr)
        self.shutdown_pub.publish("Shutdown")
        self.do_shutdown = True
        return 0,0,0,0, 0
    result = st[st.find('(')+1:st.find(')')]
    li = list(map(int, result.split(',')))
    
    #for start of bounding box
    opicx = (li[0])
    opicy = (li[1])
    startX = (image_size_height - opicy)/metertopixel
    startY = (image_size_width/2 - opicx)/metertopixel
    #print('X,Y',startX,startY)
    #print('X,Y ',opicx,opicy)
    
    
    #for end of bounding box
    
    opicx = (li[0]+li[2])
    opicy = (li[1]+li[3])
    endX = (image_size_height - opicy)/metertopixel
    endY = (image_size_width/2 - opicx)/metertopixel
    
    return startX,startY,endX,endY, li

    

  def convert_pixel_to_laser(self,u,v,depth):

    x = ( u - self.cx)*depth*(1/self.fx)
    y = ( v - self.cy)*depth*(1/self.fy)
    trans = self.transform.transform
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
    point = [depth,x,y,1]
    transformedPOINT = np.matmul(M,point)
    #print('Point : ',point)
    #print('height : ',m23-transformedPOINT[2])
    #print('Transformed: ', transformedPOINT)
    return transformedPOINT
                
                
  def laser_camera_cb(self, laser_msg,depth_msg,rgb_msg):
      #print("In laser camera callback")
      self.head = laser_msg.header
      self.imgcamsync(depth_msg,rgb_msg)
      self.scan_cb(laser_msg)


  def imgcamsync(self, depth_image, sub_cam):
        if self.do_shutdown == True:
           return

        self.imcallbackcount += 1
        while self.head == None or self.transform == None:
            time.sleep(0.1)
        header = self.head
        depth_imgdata = depth_image.data
        rgb_image = sub_cam
        mdata =np.frombuffer(rgb_image.data,dtype = np.ubyte)
        height = rgb_image.height
        width = rgb_image.width
        buff = np.frombuffer(depth_imgdata, dtype=np.float32)
        img = np.reshape(mdata,(height,width,3))

    
        
        if self.firsttime == True:
            self.firsttime = False
            #bbox = (203, 5, 325, 460) # straight_follow_corner_box.bag
            #bbox = (65, 7, 391, 467) # crossing2.bag
            #bbox = (93, 44, 237, 427) # crossing.bag
            #bbox = (125, 8, 335, 472) # walking.bag
            #bbox = (204, 10, 236, 407) # fix_follow_down.bag
            #bbox = (86, 2, 332, 478) # fix_follow_corner_box2.bag
            #bbox = (85, 5, 320, 465) # fix_follow_s.bag
            bbox = default_camera_box
            #bbox = cv2.selectROI('Object Selection',img)            #uncomment and use for manual selection of ROI for camera. Once this is selected, default_camera_box can be updated to run code without manual selection.
            print('cam bbox: ', bbox)
            self.gotroi = True
            #print("Bounding Box : ",bbox)
            self.tracker_cam.init(img,bbox)
            #print("tracker Init")
            p1 = (int(bbox[0]), int(bbox[1]))
            p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
        
        else: 
            #print("before update call")
            ok, boundingbox = self.tracker_cam.update(img)
             #print("Tracker Okay")
          
            if ok:
                #print("camera: boundingbox after update : \n", boundingbox)
                p1 = (int(boundingbox[0]), int(boundingbox[1]))
                p2 = (int(boundingbox[0] + boundingbox[2]), int(boundingbox[1] + boundingbox[3]))
                if p1[0] < 0 or p1[1] < 0 or abs(p1[0]-p2[0]) <= 0 or abs(p1[1]- p2[1]) <= 0:
                    print("Camera Tracking error 1: - negative or null bounding box at ", self.imcallbackcount, "-",self.imstr,":",p1, p2)
                    self.camera_found_none_count += 1
                    return

                s_time = time.time()
                depth_buff = np.reshape(buff, (height, width))
                
                
                
                srow = p1[1]
                erow = p2[1]
                scol = p1[0] + 4
                ecol = p2[0] - 4

                #depth = buff[v*640+u]
                #d = depth_buff[v,u]
                #print("depth,d " , depth,d)
                d1 = depth_buff[srow:erow, scol:ecol]
                im1 = img[srow:erow, scol:ecol]
                #print("im1 shape = ", im1.shape)
                ##im2 = cv2.cvtColor(im1, cv2.COLOR_BGR2GRAY)
                #edges = cv2.Canny(im2, threshold1 = 100, threshold2=500)
                ##edges = im2
                e_time1 = time.time()
                if self.camera_debug == True:
                   fname= "imgout/img_"+str(self.imcallbackcount)+".jpg"
                   cv2.imwrite(fname, im1)
                
                #d1 = depth_buff
                d2 = np.where(np.isfinite(d1), d1, [10]) 
                ##r,c = np.where(d2 >= 3.5)
                e_time6 = time.time()
                ##for i in range(len(r)):
                 ##    edges[r[i],c[i]] = 0
                d3 = np.where(d2 > 3, [0], [1])
                edges = d3.astype(np.uint8) * 255
                (nrow,ncol) = edges.shape
                cropped_edges = edges[0:int(nrow*0.6),0:ncol]

                d4 = np.multiply(d2, d3)
                dmax = np.max(d4)
                d3 = np.where(d4 < 0.01, [255], d4)
                dmin = np.min(d3)
                
                #print(" d2 shape, edges shape, cropped_edges shape - ", d2.shape, edges.shape, cropped_edges.shape)
                #print("cropped edges\n", cropped_edges)
                r, c = np.where(cropped_edges == 255)
                if len(c) == 0:
                    print("Camera Tracking error 2: - null cropped edges on img -", self.imcallbackcount,"-",self.imstr)
                    self.camera_found_none_count += 1
                    if self.camera_debug == True:
                       fname= "imgout//out/img_"+str(self.imcallbackcount)+".jpg"
                       cv2.imwrite(fname, edges)
                    return
                

                cind = np.argmin(c)
                minc = c[cind] 
                minr = r[cind]
                #print("camera: imgno, minr, minc-",self.imcallbackcount,minr,minc)
                if (ncol - minc) < 9:
                    print("Camera Tracking error 3:",self.imcallbackcount,"-",self.imstr )
                    self.last_camera_track_ok = False
                    self.camera_found_none_count += 1
                    if self.camera_debug == True:
                        fname= "imgout//out/img_"+str(self.imcallbackcount)+".jpg"
                        cv2.imwrite(fname, edges)
                    return
                while edges[minr, minc+1] != 255 or edges[minr, minc+2] != 255 or edges[minr, minc+3] != 255 or edges[minr,minc+4] != 255:
                    c[cind] = 10000
                    cind = np.argmin(c)
                    minc = c[cind] 
                    if (minc == 10000) or (ncol - minc) < 9:
                        print("Camera Tracking error 3:",self.imcallbackcount,"-",self.imstr )
                        self.last_camera_track_ok = False
                        self.camera_found_none_count += 1
                        if self.camera_debug == True:
                           fname= "imgout//out/img_"+str(self.imcallbackcount)+".jpg"
                           cv2.imwrite(fname, edges)
                        return
                    minr = r[cind]
                 
                      
                #print("camera: imgno, modified minr, minc-",self.imcallbackcount,minr,minc)
                c1 = np.where(c == 10000, [0],c)
                #c1 = np.where(c == minc)
                #minr = r[c1[0][0]]

                cind = np.argmax(c1)
                maxc = c1[cind] 
                maxr = r[cind]
                #print("camera: imgno, maxr, maxc-",self.imcallbackcount,maxr,maxc)
                if maxc < 4:
                    print("Camera Tracking error 4:",self.imcallbackcount,"-",self.imstr)
                    self.last_camera_track_ok = False
                    self.camera_found_none_count += 1
                    if self.camera_debug == True:
                        fname= "imgout//out/img_"+str(self.imcallbackcount)+".jpg"
                        cv2.imwrite(fname, edges)
                    return
                while edges[maxr, maxc-1] != 255 or edges[maxr, maxc-2] != 255 or edges[maxr, maxc-3] != 255 or edges[maxr, maxc-4] != 255:
                    c1[cind] = 0
                    cind = np.argmax(c1)
                    maxc = c1[cind] 
                    if maxc < 4:
                        print("Camera Tracking error 4:",self.imcallbackcount,"-",self.imstr)
                        self.last_camera_track_ok = False
                        self.camera_found_none_count += 1
                        if self.camera_debug == True:
                           fname= "imgout//out/img_"+str(self.imcallbackcount)+".jpg"
                           cv2.imwrite(fname, edges)
                        return
                    maxr = r[cind]

                #print("camera: imgno, modified maxr, maxc-",self.imcallbackcount,maxr,maxc)
                #print(str(self.imcallbackcount), ": minr, minc, maxr, maxc",minr, minc, maxr, maxc)
                if maxc - minc < 20:
                        print("Camera Tracking error 5:",self.imcallbackcount,"-",self.imstr)
                        self.last_camera_track_ok = False
                        self.camera_found_none_count += 1
                        if self.camera_debug == True:
                           fname= "imgout//out/img_"+str(self.imcallbackcount)+".jpg"
                           cv2.imwrite(fname, edges)
                        return


                iscol = minc + scol
                isrow = minr + srow
                iecol = maxc + scol
                ierow = maxr + srow

                e_time2 = time.time()
                cx1 = int((minc+maxc)/2)
                cy1 = int((minr+maxr)/2)
                dep1 = d2[cy1, cx1]
                cx2 = int((iscol+iecol)/2)
                cy2 = int((isrow+ierow)/2)
                dep2 = depth_buff[cy2, cx2]
 
                #print(str(self.imcallbackcount), ": cx1,cy1,dep1, cx2,cy2,dep2", cx1,cy1,dep1,cx2,cy2,dep2)


                
                if self.camera_debug == True:
                   fname= "imgout//out/img_"+str(self.imcallbackcount)+".jpg"
                   cv2.rectangle(edges, (minc, minr),(maxc,maxr),(100),2,1)
                   #cv2.rectangle(edges, (iscol, isrow),(iecol,isrow),(200),2,1)
                   cv2.imwrite(fname, edges)


                e_time3 = time.time()
                center_x = int((iscol+iecol)/2)
                center_y = int((isrow+ierow)/2)
                center_depth = depth_buff[center_y,center_x]

                
                if  math.isnan(center_depth):
                    #print("1: nan found : - ", center_x, center_y, center_depth, " on ", self.camera_found_count)
                    for i in range (1,6):
                         c_y = center_y - i
                         c_x = center_x - i
                         c_depth = depth_buff[c_y, c_x]
                         if math.isnan(c_depth):
                            c_y = center_y + i
                            c_x = center_x + i
                            c_depth = depth_buff[c_y, c_x]
                         if math.isnan(c_depth) == False:
                              center_y = c_y
                              center_x = c_x
                              center_depth = c_depth
                              break
  
                if  math.isnan(center_depth):
                        print("Camera Tracking error 6: nan found",self.imcallbackcount,"-",self.imstr)
                        self.last_camera_track_ok = False
                        self.camera_found_none_count += 1
                        return

                transformedpoint1 = self.convert_pixel_to_laser(isrow,iscol,center_depth)
                transformedpoint2 = self.convert_pixel_to_laser(ierow,iecol,center_depth)

                if abs(transformedpoint1[1] - transformedpoint2[1]) > 0.8:
                        print("Camera Tracking error 7:",self.imcallbackcount,"-",self.imstr)
                        self.last_camera_track_ok = False
                        self.camera_found_none_count += 1
                        return
                 
                transformedpoint = self.convert_pixel_to_laser(center_x,center_y,center_depth)
                startY = -transformedpoint[1] - 0.4
                endY = -transformedpoint[1] + 0.4
                startX = center_depth - 0.4
                endX = center_depth + 0.4

                # convert camera polygon to laser image coordinate
                if (dmax - dmin) <= 0.6:
                     startX = dmin - 0.2
                     endX = dmax + 0.2
                else:
                   dl = depth_buff[isrow,iscol]
                   dr = depth_buff[ierow,iecol]
                   if math.isnan(dl) == False and math.isnan(dr) == False:
                        dc = (dl+dr)/2
                        startX = dc - 0.4
                        endX = dc + 0.4

                #print(self.imstr,"-",self.imcallbackcount,": dmin,dmax,dl,dr,center_depth-",dmin,dmax,dl,dr,center_depth,"-",dmax-dmin)

                #if math.isnan(dl) or math.isnan(dr):
                #     print("Null depth:",dl,dr)
                #else:
                     #dc = (dl+dr)/2
                     #transformedpoint_l = self.convert_pixel_to_laser(iscol, isrow, dl)
                     #transformedpoint_r = self.convert_pixel_to_laser(iecol, ierow, dr)
                     #sY = -transformedpoint_l[1] - 0.4
                     #eY = -transformedpoint_r[1] + 0.4
                     #if dl < dr:
                     #   sX = dl - 0.1
                     #   eX = dr + 0.3
                     #else:
                     #   sX = dr - 0.2
                     #   eX = dl + 0.3

                     
                     #print(self.imcallbackcount,": sY,startY,eY,endY,sX,startX,eX,endX: ",sY,startY,eY,endY,sX,startX,eX,endX)
                     #startX = sX
                     #startY = sY
                     #endX = eX
                     #startY = eY

                

                self.cam_ex = int(image_size_width/2 - startY  * metertopixel)
                self.cam_ey = int(image_size_height - startX  * metertopixel)
                self.cam_sx = int(image_size_width/2 - endY  * metertopixel)
                self.cam_sy = int(image_size_height - endX  * metertopixel)

                #print(self.imcallbackcount, " : startX, endX, self.cam_ey, self.cam_sy", startX, endX,self.cam_ey, self.cam_sy)
                if self.cam_sx < 0 or self.cam_sy  < 0 or self.cam_ex < 0 or self.cam_ey <0:
                     #print("Negative cam box on : ",self.imcallbackcount, ":",self.cam_sx, self.cam_sy, self.cam_ex, self.cam_ey)
                     print("Negative cam poly : startX,startY,endX,endY,center_x,center_y,center_depth, transformedpoints : ",startX, startY, endX, endY,center_x,center_y,center_depth, transformedpoint[0], transformedpoint[1])
                        

                
                

                e_time4 = time.time()
                self.poly_msg_img = PolygonStamped()
                self.poly_msg_img.header = header
                self.poly_msg_img.polygon.points =  [Point32(x=startX, y=startY, z=0),
                                            Point32(x=endX, y=startY, z=0),
                                            Point32(x=endX, y=endY, z=0),
                                            Point32(x=startX, y=endY, z=0)]
                #print('Publish Poly Message, ', poly_msg.polygon.points)
                #if self.do_shutdown == False:
                #    self.position_pub_img.publish(self.poly_msg_img)        
                e_time5 = time.time()
                self.camera_found_count += 1
                self.last_camera_track_ok = True

                #print("elapse1, elapse6, elapse2, elapse3, elapse4,elapse5 ", (e_time1-s_time)*1000, (e_time2-s_time)*1000,(e_time3-s_time)*1000, (e_time4-s_time)*1000, (e_time5-s_time)*1000)
                #print(self.imcallbackcount, ": elapse5 on: ", (e_time5-s_time)*1000)

                

            else:
                self.camera_found_none_count += 1
                self.last_camera_track_ok = False
                print('Camera Tracking Error 8:',self.imcallbackcount,"-",self.imstr)
                
  


if __name__ == '__main__':
    print('start main')
    #print('OpenCV Version : {0}'.format(cv2.__version__) )
    if len(sys.argv) != 4:
      print('python3 laser_tracking_fusion_realworld.py laser tracking ( 1 = MIL, 2 = MOSSE, 3 = CSRT, 4 = selftracker ) camera tracking ( 1 = MIL, 2 = MOSSE, 3 = CSRT, 4 = GOTURN ) timeout(seconds)' )
      sys.exit(0)
    #print('Chosen INPUT = ')
    
    #tracker_types = '1 = MIL, 2 = MOSSE, 3 = CSRT'
    #print('Select Tracker Type:', tracker_types)
    #inpt = input('Input : ')
    
    #print('Chosen INPUT = ')
    inpt_laser = sys.argv[1]
    inpt_camera = sys.argv[2]
    timeout = int(sys.argv[3])
    scan = scan_to_image(inpt_laser, inpt_camera, timeout)
    scan.scan_data()
    
    print("camera callback ount = ", scan.imcallbackcount) 
    print("laser callback ount = ", scan.lasercallbackcount )

    total_laser_count = scan.laser_found_count + scan.laser_found_none_count
    laser_found_perc = (scan.laser_found_count/total_laser_count)*100
    laser_notfound_perc = (scan.laser_found_none_count/total_laser_count)*100
    print("laser found count = ",scan.laser_found_count, "(",laser_found_perc,"%)")
    print("laser not found count = ",scan.laser_found_none_count, "(",laser_notfound_perc,"%)")
    
    total_camera_count = scan.camera_found_count + scan.camera_found_none_count
    camera_found_perc = (scan.camera_found_count/total_camera_count)*100
    camera_notfound_perc = (scan.camera_found_none_count/total_camera_count)*100
    print("camera found count = ",scan.camera_found_count, "(",camera_found_perc,"%)")
    print("camera not found count = ",scan.camera_found_none_count, "(",camera_notfound_perc,"%)")


    

