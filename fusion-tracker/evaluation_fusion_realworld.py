#! /usr/bin/env python3
#created by Soham Panda as part of Master thesis at Uni Bielefeld
import rospy
import time
from std_msgs.msg import String
from people_msgs.msg import People, PositionMeasurementArray
from geometry_msgs.msg import PoseStamped,PolygonStamped,Pose
import numpy as np
import math
import matplotlib.pyplot as plt


class evaluation:
    def __init__(self):
        self.do_eval =  True
        self.count = 0
        self.track_name = 'None'
        self.total_score = 0 
        self.total_score_feet = 0
        self.total_score_cam = 0
        self.yarray = []
        self.farray = []
        self.carray = []
        self.gtarr = []
        self.total_gt = 0
        self.camiouarr = []
        self.total_camiou = 0
        self.total_ratio = 0
        self.wrong_detection = 0
        self.partial_wrong_detection = 0
        self.correct_detection = 0
        self.pose_w = PoseStamped()
        self.left_w = PoseStamped()
        self.right_w = PoseStamped()
        self.rseq = 0
        self.lseq = 0
        self.polygon_gt = None
        self.gotposition = False
        #self.f = open("abc_test.txt","a")
        rospy.init_node('Evaluation',anonymous=True)
        pos_sub = rospy.Subscriber('/people_tracker/people', People, self.position)
        position_sub = rospy.Subscriber('leg_tracker_measurements', PositionMeasurementArray, self.pos)
        self.leftfoot_pub = rospy.Publisher('leftfoot_detected', PoseStamped, queue_size=10 )
        self.rightfoot_pub = rospy.Publisher('rightfoot_detected', PoseStamped, queue_size=10 )
        self.position_pub = rospy.Publisher('Bayes_position', PoseStamped, queue_size=10 )
        polygon_sub =  rospy.Subscriber("polygon_estimate_laser",PolygonStamped, self.polygon)
        polygon_sub_cam =  rospy.Subscriber("polygon_estimate_img",PolygonStamped, self.polygon_cam)
        eval_sub = rospy.Subscriber('evaluation_msg',String, self.eva)
        ground_truth = rospy.Subscriber("ground_truth_anno",PolygonStamped,self.groundtruth)

    def groundtruth(self,msg):
        polygon_gt = msg
        self.polygt_start_x = polygon_gt.polygon.points[0].x
        self.polygt_start_y = polygon_gt.polygon.points[0].y
        self.polygt_end_x = polygon_gt.polygon.points[2].x
        self.polygt_end_y = polygon_gt.polygon.points[2].y
        
    def track_type(self,msg):
        self.track_name = msg.data
    
    
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
        if self.polygon_gt != None:
            gt_center_x = (self.polygt_start_x + self.polygt_end_x)/2
            gt_center_y = (self.polygt_start_y + self.polygt_end_y)/2
            distance_gt = math.sqrt((gt_center_x-centre_x)**2+(gt_center_y-centre_y)**2)
            x1 = min(self.polygt_start_x,self.polygt_end_x)
            y1 = min(self.polygt_end_x,self.polygt_end_y)
            w1 = abs(self.polygt_start_x-self.polygt_end_x)
            h1 = abs(self.polygt_start_y-self.polygt_end_y)

            area_covered = self.get_intersection_ratio(x1,y1,w1,h1,x2,y2,w2,h2)
        
            if area_covered < 0.01:
             self.wrong_detection += 1
            elif area_covered < 0.75:
                self.partial_wrong_detection += 1
            else:
                self.correct_detection += 1
            self.total_ratio += area_covered
        else:
            distance_gt = 0
        return distance_pose, distance_feet, distance_cam, distance_gt
    
    def pos(self,msg):
        leg_measurement_pos = msg
        #print('leg measure')
        #print(leg_measurement_pos.people[0].pos)
        leglength = len(leg_measurement_pos.people)
        legarr = np.zeros((leglength), dtype='float32')
        for i in range(leglength):
            legarr[i] = leg_measurement_pos.people[i].reliability
        max1ind = np.argmax(legarr)
        legarr[max1ind] = 0
        max2ind = np.argmax(legarr)

        

        #for i in range(leglength):
        #    if leg_measurement_pos.people[i].reliability > max1:
        #        max2 = max1
        #        max2ind = max1ind
        #        max1 = leg_measurement_pos.people[i].reliability
        #        max1ind = i
        #    elif leg_measurement_pos.people[i].reliability > max2:
        #        max2 = leg_measurement_pos.people[i].reliability
        #        max2ind = i
                
                
        #print("no of legs = ",leglength)
        self.leftleg_x = leg_measurement_pos.people[max1ind].pos.x
        self.rightleg_x = leg_measurement_pos.people[max2ind].pos.x

        self.leftleg_y = leg_measurement_pos.people[max1ind].pos.y
        self.rightleg_y = leg_measurement_pos.people[max2ind].pos.y

        #print('leg X,Y : ' ,self.legx,self.legy)
        #print(leg_measurement_pos.people[1].pos)

        
        self.lseq += 1
        self.left_w.header.seq = self.lseq
        self.left_w.header.stamp.secs = int(time.time())
        self.left_w.header.stamp.nsecs = 0
        self.left_w.header.frame_id = "base_laser_link"
        self.left_w.pose.position.x = self.leftleg_x
        self.left_w.pose.position.y = self.leftleg_y
        self.left_w.pose.position.z = 0
        self.leftfoot_pub.publish(self.left_w)

        self.rseq += 1
        self.right_w.header.seq = self.rseq
        self.right_w.header.stamp.secs = int(time.time())
        self.right_w.header.stamp.nsecs = 0
        self.right_w.header.frame_id = "base_laser_link"
        self.right_w.pose.position.x = self.rightleg_x
        self.right_w.pose.position.y = self.rightleg_y
        self.right_w.pose.position.z = 0
        self.rightfoot_pub.publish(self.right_w)

    def position(self,msg):
        #print('in position callback')
        walker_position = msg
        sz = len(walker_position.people)
        if sz != 0:
            self.position_walker = walker_position
            self.pose_w.header = self.position_walker.header
            self.pose_w.pose.position.x = self.position_walker.people[0].position.x
            self.pose_w.pose.position.y = self.position_walker.people[0].position.y
            self.pose_w.pose.position.z = 0
            self.position_pub.publish(self.pose_w)
            self.gotposition = True

            
        
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
    
        walker_x  = self.position_walker.people[0].position.x 
        walker_y = self.position_walker.people[0].position.y
        
        lx  = self.leftleg_x
        rx  = self.rightleg_x
        ly  = self.leftleg_y
        ry  = self.rightleg_y

        #if walker_x <= poly_start_x and walker_x >= poly_end_x and walker_y <= poly_start_y and walker_y >= poly_end_y:
        #    print('The walker Pose is within the bounding box created')
        #    score = 0
        #else:
        score_pose,score_feet, score_cam, score_gt = self.evaluate(walker_x,walker_y,poly_start_x,poly_start_y,poly_end_x,poly_end_y,lx,ly,rx,ry)
        
        self.yarray.append(score_pose)
        self.total_score += score_pose
        self.farray.append(score_feet)
        self.total_score_feet += score_feet
        self.carray.append(score_cam)
        self.total_score_cam += score_cam
        self.gtarr.append(score_gt) 
        self.total_gt += score_gt
        
        self.count += 1
        
     
if __name__=='__main__':
    #print('In Main')    
    eva = evaluation()
    rospy.spin()
    xarray = np.arange(0,len(eva.yarray),1)
    avg_pose = (eva.total_score/eva.count)*100
    avg_feet = (eva.total_score_feet/eva.count)*100
    gt_avg = (eva.total_gt/eva.count)*100
    form = "{:.2f}".format
    
    #print("total score, total_score_feet, eva_count", eva.total_score, eva.total_score_feet, eva.count)
    plt.plot(xarray,eva.yarray, label = 'Pose from Bayes Tracker, Average Distance = '+str(form(avg_pose))+' cm', color = 'black')
    plt.plot(xarray,eva.farray, label = 'Pose from Leg Detector, Average Distance = '+str(form(avg_feet))+' cm', color = 'green' )
    #plt.plot(xarray,eva.carray, label = 'From camera center'+' Average Score = '+str(eva.total_score_cam/eva.count), color = 'blue' )
    if eva.polygon_gt != None:
        plt.plot(xarray,eva.gtarr, label = 'Pose from Ground truth annotation, Average Distance = '+str(form(gt_avg))+' cm', color = 'red' )
    plt.ylabel('Distance from centre of LRF tracker output')
    plt.xlabel('Frames')
    plt.legend()
    plt.show()

    
    print('Average distance between LRF tracker output and Bayes tracker pose: ',str(eva.total_score/eva.count))

    print('Average distance between LRF tracker output and Leg Detector output: ',str(eva.total_score_feet/eva.count))
    
    if eva.polygon_gt != None:
        print('Average distance between LRF tracker output and Ground truth annotation: ',str(eva.total_gt/eva.count))
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
    
    print('Evaluation using Pose from Bayes Tracker: ')
    print("distance from pose: less than 20 cm- ", pose_20, " between 20 and 40 cm-  ", pose_20_40, " more than 40 cm- ", pose_40)

    a = np.asarray(eva.farray)
    feet_20 = (a <= 0.2).sum()
    feet_40 = (a > 0.4).sum()
    feet_20_40 = ((a > 0.2) & ( a <= 0.4)).sum()
    print('Evaluation using Pose from Leg Detector Measurements: ')
    print("distance from feet: less than 20 cm- ", feet_20, " between 20 and 40 cm-  ", feet_20_40, " more than 40 cm- ", feet_40)

    if eva.polygon_gt != None:
        
        a = np.asarray(eva.gtarr)
        gt_20 = (a <= 0.2).sum()
        gt_40 = (a > 0.4).sum()
        gt_20_40 = ((a > 0.2) & ( a <= 0.4)).sum()
        print('Evaluation using Pose from Ground truth annotation: ')
        print("distance from Ground truth: less than 20 cm- ", gt_20, " between 20 and 40 cm-  ",gt_20_40, " more than 40 cm- ", gt_40)
        
    bar_width = 0.25
 #   multiplier = 0
 #   
 #   dist_types = ('Evaluation using Pose from Bayes Tracker','Evaluation using Pose from Leg Detector Measurements','Evaluation using Pose from Ground truth annotation')
#    x = np.arange(len(dist_types))
#    fig, ax = plt.subplots()
#    data = {
#        'Less than 20cm' : (pose_20,feet_20,gt_20), 
#        'Between 20 and 40cm' : ( pose_20_40, feet_20_40, gt_20_40),
#        'More than 40cm' : (pose_40,feet_40,gt_40)
#    }
#    for attribute, measurement in data.items():
#        offset = width*multiplier
#        rects = ax.bar( x+ offset,  measurement, width, label=attribute)
#        #ax.bar_label(rects, padding=3)
#        multiplier += 1#

#    ax.set_ylabel = 'Occurences'
#    ax.set_title('Distance of LRF center')
#    ax.get_xticks (x+width, dist_types)
#    plt.show()
    total_bayes = pose_20+pose_20_40+pose_40
    total_feet = feet_20+feet_20_40+feet_40
    if eva.polygon_gt == None:
        gt_20 = gt_20_40 = gt_40 = 0
        total_gt = 1
    else:
        total_gt = gt_20+gt_20_40+gt_40
        
    pose_20 = (pose_20/total_bayes)*100
    pose_20_40 = (pose_20_40/total_bayes)*100
    pose_40 = (pose_40/total_bayes)*100

    feet_20 = (feet_20/total_feet)*100
    feet_20_40 = (feet_20_40/total_feet)*100
    feet_40 = (feet_40/total_feet)*100

    gt_20 = (gt_20/total_gt)*100
    gt_20_40 = (gt_20_40/total_gt)*100
    gt_40 = (gt_40/total_gt)*100

    A = [pose_20, feet_20, gt_20]
    B = [pose_20_40,feet_20_40,gt_20_40]
    C = [pose_40,feet_40,gt_40]
    form = "{:.2f}".format
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
            ['Pose from Bayes Tracker','Pose from Leg Detector','Pose from Ground truth'])
    plt.title('Distance of LRF center')
    plt.legend()
    plt.show()
        


    

