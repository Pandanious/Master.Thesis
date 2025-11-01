#! /usr/bin/env python3
#created by Soham Panda as part of Master thesis at Uni Bielefeld
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped,PolygonStamped
import numpy as np
import math
import matplotlib.pyplot as plt


class evaluation:
    def __init__(self):
        self.do_eval =  True
        self.count = 0
        self.total_score = 0 
        self.total_score_feet = 0
        self.total_score_cam = 0
        self.yarray = []
        self.farray = []
        self.carray = []
        self.iouarr = []
        self.total_iou = 0
        self.camiouarr = []
        self.total_camiou = 0
        self.total_ratio = 0
        self.wrong_detection = 0
        self.partial_wrong_detection = 0
        self.correct_detection = 0

        
        self.gotposition = False
        #self.f = open("abc_test.txt","a")
        rospy.init_node('Evaluation',anonymous=True)
        pos_sub = rospy.Subscriber('/position_from_laser', PoseStamped, self.pos)
        right_sub = rospy.Subscriber('rightfoot', PoseStamped, self.right_pos)
        left_sub = rospy.Subscriber('leftfoot', PoseStamped, self.left_pos)
        polygon_sub =  rospy.Subscriber("polygon_estimate_laser",PolygonStamped, self.polygon)
        polygon_sub_cam =  rospy.Subscriber("polygon_estimate_img",PolygonStamped, self.polygon_cam)
        eval_sub = rospy.Subscriber('evaluation_msg',String, self.eva)
    
    
    
    def eva(self,msg):
        #print('Average Score of Tracking = ', self.total_score/self.count)
        self.do_eval = False
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

    def get_iou(self,x1,y1,w1,h1,x2,y2,w2,h2):
        w_intersect = min(x1+w1, x2+w2) - max(x1, x2)
        h_intersect = min(y1+h1, y2+h2) - max(y1, y2)
        if  w_intersect <= 0 or h_intersect <= 0:
            iou = 0
        else:
            I = w_intersect * h_intersect
            U = w1 * h1 + w2*h2 - I
            iou = I/U
        return iou



    def evaluate(self,walker_x,walker_y,poly_start_x,poly_start_y,poly_end_x,poly_end_y,lx, ly, rx, ry):
        #print('Walker X,Y: ',walker_x,walker_y)
        #print('Poly StartX,EndX,StartY,EndY: ',poly_start_x,poly_end_x,poly_start_y,poly_end_y)
        centre_x = (poly_end_x+poly_start_x)/2
        centre_y = (poly_end_y+poly_start_y)/2
        
        feet_center_x = (lx+rx)/2
        feet_center_y = (ly+ry)/2
        
        distance_pose = math.sqrt((centre_x - walker_x)**2+(centre_y-walker_y)**2)
        distance_feet = math.sqrt((feet_center_x-centre_x)**2+(feet_center_y-centre_y)**2)

	#calculate IOU - Intersection over union
        x1 = min(rx,lx) - 0.02
        y1 = min(ry,ly) - 0.02
        w1 = abs(rx-lx) + 0.04
        h1 = abs(ry-ly) + 0.04
        x2 = min(poly_start_x, poly_end_x) 
        y2 = min(poly_start_y, poly_end_y) 
        w2 = abs(poly_end_x - poly_start_x) 
        h2 = abs(poly_end_y - poly_start_y) 
        #print("x1,y1,w1,h1", x1,y1,w1,h1)
        #print("x2,y2,w2,h2", x2,y2,w2,h2)

        iou = self.get_iou(x1,y1,w1,h1,x2,y2,w2,h2)
        area_covered = self.get_intersection_ratio(x1,y1,w1,h1,x2,y2,w2,h2)
        
        if area_covered < 0.01:
           self.wrong_detection += 1
        elif area_covered < 0.75:
           self.partial_wrong_detection += 1
        else:
           self.correct_detection += 1

        self.total_ratio += area_covered

        rx = self.polycam_start_x
        lx = self.polycam_end_x
        ry = self.polycam_start_y
        ly = self.polycam_end_y
        x1 = min(rx,lx) 
        y1 = min(ry,ly)
        w1 = abs(rx-lx)
        h1 = abs(ry-ly)

        cam_center_x = (lx+rx)/2
        cam_center_y = (ly+ry)/2
        distance_cam = math.sqrt((cam_center_x-centre_x)**2+(cam_center_y-centre_y)**2)
        camiou = self.get_iou(x1,y1,w1,h1,x2,y2,w2,h2)
           

        #print("x1,y1,w1,h1", x1,y1,w1,h1)
        #print("x2,y2,w2,h2,camiou", x2,y2,w2,h2,camiou)

	
        return distance_pose, distance_feet, distance_cam, iou, camiou
    
    def pos(self,msg):
        #print('in position callback')
        self.position_walker = msg
        self.gotposition = True
    
    def left_pos(self,msg):
        #print('in left callback')
        self.left= msg
        
    def right_pos(self,msg):
        #print('in right callback')
        self.right= msg
        
        
    def polygon_cam(self,msgs):
        #print('in polygon_cam callback')
        polygon_cam = msgs
        self.polycam_start_x = polygon_cam.polygon.points[0].x
        self.polycam_start_y = polygon_cam.polygon.points[0].y
        self.polycam_end_x = polygon_cam.polygon.points[2].x
        self.polycam_end_y = polygon_cam.polygon.points[2].y


    def polygon(self,msgs):
        #print('in polygon callback')
        if self.do_eval == False:
            return
        if self.gotposition == False:
            return

        #print('boo')
        polygon_walker = msgs
        avg_count = 0
        poly_start_x = polygon_walker.polygon.points[0].x
        poly_start_y = polygon_walker.polygon.points[0].y
        poly_end_y = polygon_walker.polygon.points[2].y
        poly_end_x = polygon_walker.polygon.points[2].x
    
        walker_x  = self.position_walker.pose.position.x
        walker_y = self.position_walker.pose.position.y
        
        rx = self.right.pose.position.x
        lx = self.left.pose.position.x
        ry = self.right.pose.position.y
        ly =self.left.pose.position.y

        #if walker_x <= poly_start_x and walker_x >= poly_end_x and walker_y <= poly_start_y and walker_y >= poly_end_y:
        #    print('The walker Pose is within the bounding box created')
        #    score = 0
        #else:
        score_pose,score_feet, score_cam, iou, camiou = self.evaluate(walker_x,walker_y,poly_start_x,poly_start_y,poly_end_x,poly_end_y,lx,ly, rx,ry)
        
        self.yarray.append(score_pose)
        self.total_score += score_pose
        self.farray.append(score_feet)
        self.total_score_feet += score_feet
        self.carray.append(score_cam)
        self.total_score_cam += score_cam
        self.iouarr.append(iou) 
        self.total_iou += iou
        self.camiouarr.append(camiou) 
        self.total_camiou += camiou
        
        self.count += 1
        
     
if __name__=='__main__':
    #print('In Main')    
    eva = evaluation()
    rospy.spin()
    xarray = np.arange(0,len(eva.yarray),1)
    #print("total score, total_score_feet, eva_count", eva.total_score, eva.total_score_feet, eva.count)
    avg_pose = (eva.total_score/eva.count)*100
    avg_feet = (eva.total_score_feet/eva.count)*100
    form = "{:.2f}".format
    plt.plot(xarray,eva.yarray, label = 'Model Pose from Gazebo, Average Distance = '+str(form(avg_pose))+' cm' , color = 'red')
    plt.plot(xarray,eva.farray, label = 'Feet Pose from Gazbeo, Average Distance = '+str(form(avg_feet))+' cm', color = 'green' )
    #plt.plot(xarray,eva.carray, label = 'From camera center'+' Average Score = '+str(eva.total_score_cam/eva.count), color = 'blue' )
    plt.ylabel('Distance from centre of LRF tracker output.')
    plt.xlabel('Frames')
    plt.legend()
    plt.show()

    avgarr = [avg_pose,avg_feet]
    types = ['Model Pose','Feet Pose']
    

    print("Correct detection = ", eva.correct_detection, " Wrong detection = ", eva.wrong_detection, " Partally wrong detection = ",eva.partial_wrong_detection)
    tot_count = eva.correct_detection + eva.wrong_detection + eva.partial_wrong_detection
    print("Average area detection ratio = ", eva.total_ratio / tot_count)
    
    a = np.asarray(eva.yarray)
    pose_20 = (a <= 0.2).sum()
    pose_40 = (a > 0.4).sum()
    pose_20_40 = ((a > 0.2) & ( a <= 0.4)).sum()
    print("distance from pose: less than 20 cm- ", pose_20, " between 20 and 40 cm-  ", pose_20_40, " more than 40 cm- ", pose_40)

    a = np.asarray(eva.farray)
    feet_20 = (a <= 0.2).sum()
    feet_40 = (a > 0.4).sum()
    feet_20_40 = ((a > 0.2) & ( a <= 0.4)).sum()
    print("distance from feet: less than 20 cm- ", feet_20, " between 20 and 40 cm-  ", feet_20_40, " more than 40 cm- ", feet_40)

    print('Average distance between LRF tracker output and Model pose from Gazebo: ',str(eva.total_score/eva.count))

    print('Average distance between LRF tracker output and Feet Pose from Gazebo: ',str(eva.total_score_feet/eva.count))
    
    print("Correct detection = ", eva.correct_detection, " Wrong detection = ", eva.wrong_detection, " Partally wrong detection = ",eva.partial_wrong_detection)
    tot_count = eva.correct_detection + eva.wrong_detection + eva.partial_wrong_detection
    print("Average area detection ratio = ", eva.total_ratio / tot_count)
    
    y = np.array([eva.correct_detection,eva.partial_wrong_detection,eva.wrong_detection])
    det_type = ['Correct detection','Partially Wrong detection','Wrong detection']
    plt.pie(y,labels= det_type)
    plt.title('Detection Types')
    plt.legend()
    plt.show()
    
    a = np.asarray(eva.yarray)
    pose_20 = (a <= 0.2).sum()
    pose_40 = (a > 0.4).sum()
    pose_20_40 = ((a > 0.2) & ( a <= 0.4)).sum()  
    
    a = np.asarray(eva.farray)
    feet_20 = (a <= 0.2).sum()
    feet_40 = (a > 0.4).sum()
    feet_20_40 = ((a > 0.2) & ( a <= 0.4)).sum()
    
    bar_width = 0.25
    total_bayes = pose_20+pose_20_40+pose_40
    total_feet = feet_20+feet_20_40+feet_40
    pose_20 = (pose_20/total_bayes)*100
    pose_20_40 = (pose_20_40/total_bayes)*100
    pose_40 = (pose_40/total_bayes)*100

    feet_20 = (feet_20/total_feet)*100
    feet_20_40 = (feet_20_40/total_feet)*100
    feet_40 = (feet_40/total_feet)*100

    A = [pose_20, feet_20]
    B = [pose_20_40,feet_20_40]
    C = [pose_40,feet_40]
    # Set position of bar on X axis
    br1 = np.arange(len(A))
    br2 = [x + bar_width for x in br1]
    br3 = [x + bar_width for x in br2]   
    bars = plt.bar(br1, A, color ='coral', width = bar_width,
            edgecolor ='grey', label ='Less than 20cm')
        
    for bar in bars:
        yval = bar.get_height()
        plt.text( bar.get_x(),yval+0.25,form(yval))
    
    
    bars = plt.bar(br2, B, color ='yellowgreen', width = bar_width,
            edgecolor ='grey', label ='Between 20 and 40cm')
    
    for bar in bars:
        yval = bar.get_height()

        plt.text(bar.get_x(),yval+0.25,form(yval))
    
    bars = plt.bar(br3, C, color ='skyblue', width = bar_width,
            edgecolor ='grey', label ='More than 40cm')
        
    for bar in bars:
        yval = bar.get_height()
        plt.text(bar.get_x(),yval+0.25,form(yval))
    
    plt.ylabel('Occurence percentage')
    plt.xticks([r + bar_width for r in range(len(A))],
            ['Model Pose from Gazebo','Feet Pose from Gazebo'])
    plt.title('Distance of LRF center')
    plt.legend()
    plt.show()
        
