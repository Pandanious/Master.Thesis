#! /usr/bin/env python3


#created by Soham Panda as part of Master thesis at Uni Bielefeld

import cv2
import numpy as np
import math
import time


#gazebo
MIN_NUM_PIXELS = 3
MAX_FEET_ROW_COL = 60
#real
#MIN_NUM_PIXELS = 5
MAX_FEET_DISTANCE_ROW_COL = 140
MIN_FEET_DISTANCE_SQ = 100
MAX_FEET_DISTANCE_SQ = 20000
MIN_BOX_THRESHOLD = 100
BOX_SHIFT_STEP = 50
MAX_BOX_COL = 799
MAX_BOX_ROW = 799


class person_tracker:
    def init(self, img, bbox, threshold, drawdonebox=False, drawrect=False):
        self.bbox = bbox
        self.im_obj = ImageCorrection(threshold)
        #print("self.im_obj assigned - ", self.im_obj)
        stime = time.time()
        self.im_obj.StartCorrection(img, bbox)
        etime = time.time()
        #print ("time taken to join - ", (etime-stime)*1000, " milliseconds")
        if drawdonebox == True:
            self.im_obj.draw_doneboxes()
        ok, box = self.im_obj.get_polygon()
        if ok == True and drawrect == True:
            cv2.rectangle(img, (box[0], box[1]), (box[0] + box[2],box[1] + box[3]), 255,1)
        return ok, box
        
    def update(self, img, drawdonebox=False, drawrect=False):
        
        self.im_obj.StartCorrection(img, self.bbox)
       
        if drawdonebox == True:
            self.im_obj.draw_doneboxes()
        ok, box = self.im_obj.get_polygon()
        if ok == True and drawrect == True:
            cv2.rectangle(img, (box[0], box[1]), (box[0] + box[2],box[1] + box[3]), 255,1)

        
        boxsx = self.bbox[0]
        boxsy = self.bbox[1]
        boxex = self.bbox[2]
        boxey = self.bbox[3]
        sx = box[0]
        sy = box[1]
        ex = box[0]+box[2]
        ey = box[1]+box[3]

        #adjust bbox if the polygon is too close to edges of bbox
        
        width = boxex - boxsx
        height = boxey - boxsy
        bbox_changed = False
        old_box = [boxsx,boxsy,boxex,boxey]
        
        if (sx - boxsx) < MIN_BOX_THRESHOLD:
            bbox_changed = True
            boxsx = boxsx - BOX_SHIFT_STEP
            if boxsx < 0:
                boxsx = 0
            boxex = boxsx + width
        elif (boxex - ex) < MIN_BOX_THRESHOLD:
            bbox_changed = True
            boxex = boxex + BOX_SHIFT_STEP
            if boxex > MAX_BOX_COL:
                boxex = MAX_BOX_COL
            boxsx = boxex - width
                    
                
        if (sy - boxsy) < MIN_BOX_THRESHOLD:
            bbox_changed = True
            boxsy = boxsy - BOX_SHIFT_STEP
            if boxsy < 0:
                boxsy = 0
            boxey = boxsy + height
        elif (boxey - ey) < MIN_BOX_THRESHOLD:
            bbox_changed = True
            boxey = boxey + BOX_SHIFT_STEP
            if boxey > MAX_BOX_ROW:
                boxey = MAX_BOX_ROW
            boxsy = boxey - height
                    
         
        self.bbox = [boxsx, boxsy, boxex, boxey]
        #if bbox_changed == True:
        #    print("rect = [",sx,sy,ex,ey,"]")
        #    print("old bbox = ",old_box)
        #    print("new bbox = ",self.bbox)
            
        return ok, box
    
    def get_bbox(self):
        return self.bbox

        
class ImageCorrection:
    #def __init__(self, img, bbox, threshold):
    def __init__(self, threshold):
        #self.img = img
        #self.original_img = img.copy()
        
        self.threshold = threshold
        self.polygon = [0,0,0,0]
        self.first_foot_cx = 0
        self.first_foot_cy = 0
        self.second_foot_cx = 0
        self.second_foot_cy = 0


    def add_donebox(self, bbox):
        self.donebox.append(bbox)

    def is_in_donebox(self, row, col):
        for i in range (len(self.donebox)):
            if row >= self.donebox[i][1] and row <= self.donebox[i][3]:
                if col >= self.donebox[i][0] and col <= self.donebox[i][2]:
                    #print("is_in_donebox returns True for ",row,col)
                    return True
        #print("is_in_donebox returns False for ", row,col)
        return False

    def print_doneboxes(self):
         for i in range (len(self.donebox)):
            print("Box = ", self.donebox[i])

         #for i in range (len(self.donebox)):
            #print( "image pixel of Boxes: [ ",self.donebox[i][0]+self.start_x, self.donebox[i][1]+self.start_y,self.donebox[i][2]+self.start_x,self.donebox[i][3]+self.start_y,"]")



    def draw_doneboxes(self):
         for i in range (len(self.donebox)):
            p1 = (int(self.start_x + self.donebox[i][0]), int(self.start_y + self.donebox[i][1])) 
            p2 = (int(self.start_x + self.donebox[i][2]), int(self.start_y + self.donebox[i][3]))
            #print("p1, p2 = ",p1,p2)
            cv2.rectangle(self.img, p1, p2, 120,1)

    def get_closest_pair(self, candidates, local_c):
             l = len(local_c)
             dist_array = []
             pairs = []
             for i in range(l-1):
                li = int(local_c[i])
                scx = (candidates[li][0] + candidates[li][2])/2
                scy = (candidates[li][1] + candidates[li][3])/2
                for j in range(i+1,l):
                   lj = int(local_c[j])
                   ccx = (candidates[lj][0] + candidates[lj][2])/2
                   ccy = (candidates[lj][1] + candidates[lj][3])/2
                   dist = (scx-ccx)**2 + (scy - ccy)**2 
                   dist_array.append(dist)
                   pairs.append((i,j))
             p = np.argmin(dist_array)
             first, second = pairs[p]
             #print("get_closest_pair: dist - , pair- ",dist_array[p],pairs[p])
             return first, second

    def get_closest_candidate(self, candidates,  first, local_c):
             l = len(local_c)
             dist = np.zeros(l)
             scx = (candidates[first][0] + candidates[first][2])/2
             scy = (candidates[first][1] + candidates[first][3])/2
             for j in range(l):
                   if j == first:
                       dist[j] = 200000 # some high value
                       continue
                   lj = int(local_c[j])
                   ccx = (candidates[lj][0] + candidates[lj][2])/2
                   ccy = (candidates[lj][1] + candidates[lj][3])/2
                   dist[j] = (scx-ccx)**2 + (scy - ccy)**2 
             second = np.argmin(dist)
             #print("get_closest_candidate: dist - ",dist[second],first,second)
             return second
                  
    def get_person(self):
        candidates = []
        final_candidates = []
        candidate_score = []
        local_donebox = []
        local_array = np.zeros(len(self.donebox))
        for i in range (len(self.donebox)):
            local_array[i] = self.donebox[i][0]
        indarray = np.argsort(local_array)
        for i in range(len(indarray)):
            local_donebox.append(self.donebox[indarray[i]])
        
        self.donebox = local_donebox
        #print("Boxes = ", self.donebox  )

        poly_center_x = int(self.polygon[0] - self.start_x + self.polygon[2]/2)
        poly_center_y = int(self.polygon[1] - self.start_y + self.polygon[3]/2)
        for i in range (len(self.donebox)):
            bbox = self.donebox[i]
            #print("current bbox = ", bbox)
            sx = self.start_x
            sy = self.start_y
            #print("poly-",self.polygon)
            if self.polygon[2] != 0: #only if self.polygon has been assigned by previous get_polygon()
                center_x = int((bbox[0]+bbox[2])/2)
                center_y = int((bbox[1]+bbox[3])/2)
                distfrompoly = (poly_center_x-center_x)**2 + (poly_center_y-center_y)**2
                #print("px,py,cx,cy,distfrompoly", poly_center_x,poly_center_y,center_x, center_y,distfrompoly)
                if distfrompoly > MAX_FEET_DISTANCE_SQ: #box too far away from last polygon
                    continue

            #print("cur bbox=[",bbox[0]+sx,bbox[1]+sy,bbox[2]+sx,bbox[3]+sy,"]")
            #print("cur bbox=[",bbox[0],bbox[1],bbox[2],bbox[3],"]")
            if bbox[2] - bbox[0] >= 2 and bbox[3] - bbox[1] >= 2:
                startx = bbox[0] + self.start_x
                starty = bbox[1] + self.start_y
                endx = bbox[2] + self.start_x
                endy = bbox[3] + self.start_y

                bbox_img = self.original_img[starty:endy+1,startx:endx+1]
                # make pixels 1 or 0

                b = np.where(bbox_img > 200,[1],[0])

                #print("b=\n", b)
                numpixels = np.sum(b)
                numrows = endy - starty
                numcols = endx - startx
                maxrowcol = max(numrows,numcols)
                #print("numpixels in box ", numpixels, bbox)
                #print("numrows,numcols= ", numrows,numcols)
                if numpixels > MIN_NUM_PIXELS and maxrowcol <= MAX_FEET_ROW_COL: # number of pixels in the original image within box
                    candidates.append(bbox)
                    #check for curve

                    (nr,nc) = b.shape 
                    nc -= 1
                    a1 = b[:,[nc]]
                    r,c = np.where(a1>0)
                    p_r = r[0]
                    nc -= 1
                    move_up = move_down = False
                    #print("check for curve on bbox", bbox)
                    while nc >= 0 and (move_up == False or move_down == False):
                       a1 = b[:,[nc]]
                       r,c = np.where(a1>0)
                       if len(r) > 0:
                          c_r = r[0]
                          #print("curve: nc, cr,pr", nc, c_r, p_r)
                          if c_r < p_r:
                              move_up = True
                          elif c_r > p_r:
                              move_down = True
                          p_r = c_r
                       nc -= 1
                       
                    if move_down == True and move_up == True:
                        #give advantage if the box has a curve
                        if numrows >= 5 and numcols >= 5:
                            score = 5
                        else:
                            score = 3
                    else:
                        score = 0
                    candidate_score.append(score)
        #print("candidates = ",candidates)
       	#print("candidate scores after curve detection", candidate_score) 
        
        sx = self.start_x
        sy = self.start_y
        if len (candidates) == 0:
            return [0,0,0,0], [0,0,0,0]
        elif len(candidates) == 1:
            return candidates[0], [0,0,0,0]
        #elif len(candidates) == 2:
            #return candidates[0], candidates[1]

        if self.polygon[2] != 0: # previous poygon has been assigned        
           p_sc = self.polygon[0]  
           p_sr = self.polygon[1] 
           p_ec = p_sc + self.polygon[2]
           p_er = p_sr + self.polygon[3]

           score = np.zeros(len(candidates))
           dist_feet = np.zeros(len(candidates))
           for i in range (len(candidates)):
              erow = candidates[i][3] + self.start_y
              ecol = candidates[i][2] + self.start_x
              srow = candidates[i][1] + self.start_y
              scol = candidates[i][0] + self.start_x
              #check if the box is close to some corner of last polygon
              #print("proximity: candidate, p_sc,p_sr, p_ec, p_er ", candidates[i], p_sc, p_sr, p_ec, p_er)
              if (abs(p_sr - srow) <= 4 or abs(p_er - erow) <= 4) and (abs(p_sc - scol) <=4 or abs(p_ec - ecol) <=4):
                  candidate_score[i] += 3 
              cx = (ecol+scol)/2 
              cy = (erow+srow)/2 
              if self.first_foot_cx != 0:
                  dist_from_first_foot = (cx-self.first_foot_cx)**2 + (cy-self.first_foot_cy)**2
                  dist_from_second_foot = (cx-self.second_foot_cx)**2 + (cy-self.second_foot_cy)**2
                  if dist_from_first_foot <= dist_from_second_foot:
                     dist_from_feet = dist_from_first_foot
                  else:
                     dist_from_feet = dist_from_second_foot
                  dist_feet[i] = dist_from_feet
              
           
              
           #print("candidate scores after polygon proximity detection", candidate_score) 
           if self.first_foot_cx != 0 and len(dist_feet) > 1:
              ind = np.argsort(dist_feet)
              candidate_score[ind[0]] += 3
              candidate_score[ind[1]] += 2
              
        #print("candidate scores after feet proximity detection", candidate_score) 
        sx = self.start_x
        sy = self.start_y
        if len (candidates) == 0:
            return [0,0,0,0], [0,0,0,0]
        elif len(candidates) == 1:
            return candidates[0], [0,0,0,0]
        #elif len(candidates) == 2:
            #return candidates[0], candidates[1]
        else:
            maxval = np.max(candidate_score)
            maxind = np.where(candidate_score == maxval)
            #print("maxind = , maxind[0]",maxind,maxind[0],len(maxind), len(maxind[0]))
            if len(maxind[0]) > 2:
                    local_candidates = np.zeros(len(maxind[0]))
                    #print("local_cand shape ", local_candidates.shape)
                    for i in range (len(maxind[0])):
                        local_candidates[i] = maxind[0][i]  
                    first, second = self.get_closest_pair(candidates, local_candidates)
            elif len(maxind[0]) == 2:
                first = maxind[0][0]
                second = maxind[0][1]
            else:
                first = maxind[0][0]
                c_score = candidate_score
                c_score[first] = 0
                maxval = np.max(c_score)
                maxind = np.where(c_score == maxval)
                if len(maxind[0]) == 1:    
                    second = maxind[0][0]
                else:
                    local_candidates = np.zeros(len(maxind[0]))
                    for i in range (len(maxind[0])):
                        local_candidates[i] = maxind[0][i]  
                    second = self.get_closest_candidate(candidates, first, local_candidates)
               
               
                    
            #for i in range (len(candidates)):
                               
            #    erow1 = candidates[i][3]
            #    ecol1 = candidates[i][2]
            #    srow1 = candidates[i][1]
            #    scol1 = candidates[i][0]
                
            #    for j in range (len(candidates)):
            #        if j == i :
            #            continue
            #        else:
            #            erow2 = candidates[j][3]
            #            ecol2 = candidates[j][2]
            #            srow2 = candidates[j][1]
            #            scol2 = candidates[j][0]
            #            if abs(srow1 - srow2) > MAX_FEET_DISTANCE_ROW_COL or abs(scol1 - scol2) > MAX_FEET_DISTANCE_ROW_COL or abs(erow1-erow2) > MAX_FEET_DISTANCE_ROW_COL or abs(ecol1 - ecol2) > MAX_FEET_DISTANCE_ROW_COL:
            #                   continue

            #            diff_row = abs(erow2 - erow1)
            #            diff_col = abs(ecol2 - ecol1)
            #            distsquare = diff_row**2 + diff_col**2
            #            diff_row = abs(erow2 - srow1)
            #            diff_col = abs(ecol2 - scol1)
            #            distsquare1 = diff_row**2 + diff_col**2
            #            diff_row = abs(srow2 - erow1)
            #            diff_col = abs(scol2 - ecol1)
            #            distsquare2 = diff_row**2 + diff_col**2
            #            #print("get person: srow1, erow2, scol1, ecol2, srow2, erow1, scol2, ecol1", srow1, erow2, scol1, ecol2, srow2, erow1, scol2, ecol1)
            #            #print("distsquare, distsquare1, distsquare2", distsquare, distsquare1, distsquare2)
                        
            #            if distsquare < MAX_FEET_DISTANCE_SQ:
            #                if len(candidates) == 2:
            #                    final_candidates.append([i,j,distsquare])

            #                elif distsquare1 > MIN_FEET_DISTANCE_SQ and distsquare2 > MIN_FEET_DISTANCE_SQ:
            #                    final_candidates.append([i,j,distsquare])
                            

            #print("get_person: final candidates: ", final_candidates)
            #if len (final_candidates) == 0:
            #    print("no final candidate")
            #    return [0,0,0,0], [0,0,0,0]
            #else:
            #    mindist = 40000 # some high value
            ##    
            #    for i in range(len(final_candidates)):
            #        if final_candidates[i][2] < mindist : #compare distsquare
            #            mindist = final_candidates[i][2]
            #            candidate1 = final_candidates[i][0]
            #            candidate2 = final_candidates[i][1]
                
            #    minval = np.min(candidate_score)
            #    minind = np.where( candidate_score == minval)
            #    print("get_person: minind - ",minind[0], "len-minind, len-candidate_score ", len(minind[0]), len(candidate_score))
            #    if len(minind[0]) == len(candidate_score): # all scores equal
            #        candidate_score[candidate1] += 2
            #        candidate_score[candidate2] += 2
            #    else:
            #       no_mindist_score = False
            #       for i in range (len(minind[0])):
            #          if candidate1 == minind[0][i] or candidate2 == minind[0][i]:
            #             no_mindist_score = True
            #             break
            #       if no_mindist_score == False:
            #           candidate_score[candidate1] += 2
            #           candidate_score[candidate2] += 2
            #    print("candidate scores = ", candidate_score)
            #    first = np.argmax(candidate_score) 
            #    f_score = candidate_score[first]
            #    candidate_score[first] = 0
            #    second = np.argmax(candidate_score) 
            #    s_score = candidate_score[second]
            #    print("first, score,second, score: ", first, f_score, second, s_score)
            #print("get_person return: ", first, second, candidates[first], candidates[second])
            return candidates[first], candidates[second]            


    def get_polygon(self):
        boxes = self.get_person()
        if len(boxes) < 2:
            #print("get_polygon: len(boxes)-", len(boxes))
            #print("polygon = ", self.polygon)
            return False, self.polygon
        else:
            sr1 = boxes[0][1]
            sc1 = boxes[0][0]
            er1 = boxes[0][3]
            ec1 = boxes[0][2]

            sr2 = boxes[1][1]
            sc2 = boxes[1][0]
            er2 = boxes[1][3]
            ec2 = boxes[1][2]
            if sr1 == 0 and er1 == 0: #no candidate found
                print("No candidate found")
                print("boxes = ", boxes)
                return False, self.polygon
            if sr2 == 0 and er2 == 0: #only on candidate found
                sr2 = self.polygon[1] - self.start_y
                er2 = self.polygon[3] + sr2
                sc2 = self.polygon[0] - self.start_x
                ec2 = self.polygon[2] + sc2
                #will use last polygon dimensions.
                #need to determine if the current candidate should align with upper left
                # or lower bottom corner of the last ploygon
                dist_start = abs(sr2 - sr1) + abs(sc2 - sc1)
                dist_end = abs(er2 - er1) + abs(ec2 - ec1)
                box_height = self.polygon[3]
                box_width = self.polygon[2]
                if dist_start < dist_end:
                    sr = sr1 + self.start_y
                    sc = sc1 + self.start_x
                    self.first_foot_cx = (sc1+ec1)/2 + self.start_x
                    self.first_foot_cy = (sr1+er1)/2 + self.start_y
                else:
                    sr = er1 + self.start_y - box_height
                    sc = ec1 + self.start_x - box_width
                    self.second_foot_cx = (sc1+ec1)/2 + self.start_x
                    self.second_foot_cy = (sr1+er1)/2 + self.start_y

                self.polygon = [sc, sr, box_width, box_height]
                
            else: # both candidates found
                if sr1 < sr2:
                    sr = sr1
                else:
                    sr = sr2
                if sc1 < sc2:
                    sc = sc1
                else:
                    sc = sc2
                if er1 > er2:
                    er = er1
                else:
                    er = er2
                if ec1 > ec2:
                    ec = ec1
                else:
                    ec = ec2
                #sc -= 20
                #ec += 20
                #sr -= 20
                #er += 20
                self.first_foot_cx = (sc1+ec1)/2 + self.start_x
                self.first_foot_cy = (sr1+er1)/2 + self.start_y
                self.second_foot_cx = (sc2+ec2)/2 + self.start_x
                self.second_foot_cy = (sr2+er2)/2 + self.start_y

                self.polygon = [self.start_x+sc, self.start_y+sr, ec-sc, er - sr]

            #print("get_polygon - boxes - ", boxes)
            #print("self.polygon=", self.polygon)
            return True, self.polygon



    
    def get_upper_object(self, bbox):
            strow = bbox[1] # start row
            rindex = len(self.on_rows) -1
            while rindex >= 0:
                cur_row = self.on_rows[rindex]
                cur_col = self.on_cols[rindex]
                if cur_row < strow and self.is_in_donebox(cur_row,cur_col) == False:
                    return rindex
                else:
                    rindex -= 1
            return -1

    def get_left_object(self, bbox):
            #print("Get left object - start_y, start_x, box=",self.start_y,self.start_x,bbox)
            erow = bbox[3] # end row
            stcol = bbox[0]
            rindex = len(self.on_rows) -1
            while rindex >= 0:
                cur_row = self.on_rows[rindex]
                cur_col = self.on_cols[rindex]
                #print("find left object - cur_row, cur_col, erow, stcol", cur_row, cur_col, erow, stcol)
                if cur_row <= erow and cur_col < stcol and self.is_in_donebox(cur_row, cur_col) == False:
                    #print("found left object - cur_row, cur_col, erow, stcol", cur_row, cur_col, erow, stcol)
                    return rindex
                else:
                    rindex -= 1
            return -1

    def get_right_object(self, bbox):
            #print("Get right object -  start_y, start_x,box=",self.start_y,self.start_x,bbox)
            erow = bbox[3] # end row
            ecol = bbox[2]
            rindex = len(self.on_rows) -1
            while rindex >= 0:
                cur_row = self.on_rows[rindex]
                cur_col = self.on_cols[rindex]
                #print("find right object - cur_row, cur_col, erow, e0col", cur_row, cur_col, erow, ecol)

                if cur_row <= erow and cur_col > ecol and self.is_in_donebox(cur_row, cur_col) == False:
                    #print("found right object - cur_row, cur_col, erow, ecol", cur_row, cur_col, erow, ecol)
                    return rindex
                else:
                    rindex -= 1
            return -1
                    

    def ConnectOneColumn(self, rowindex):
         # take bottom most row
        
        #colindex = rowlen -1
        
        while rowindex > 0:
            cur_row = self.on_rows[rowindex]
            rindex = rowindex
            
            row = self.on_rows[rowindex] 
            col = self.on_cols[rowindex]

            imrow = self.start_y+self.on_rows[rowindex]
            imcol = self.start_x + self.on_cols[rowindex]
            #print("ConnectOneColumn - rowindex, row, col, imrow, imcol= ", rowindex, row, col, imrow, imcol)
            if self.is_in_donebox(row, col) == False:
                self.join_edges(rowindex)
                #cv2.imshow("Join Edges", self.img)
                #cv2.waitKey(0)
                #print("ConnectOneColumn-Donebox - ", self.donebox)
                rowindex = self.get_upper_object(self.donebox[-1])
                left_index = self.get_left_object(self.donebox[-1])
                if left_index != -1:
                    self.left_objects.append(left_index)
                right_index = self.get_right_object(self.donebox[-1])   
                if right_index != -1:
                    self.right_objects.append(right_index) 
                #print("CorrectOneColumn: left, right: - ", self.left_objects, self.right_objects)
            
            else:
                break
            
            #print("join_edges returned - ", rowindex, ' imrow, imcol', imrow, imcol) 
            
            #self.draw_doneboxes()
            self.show_count += 1
            #lab = "Image " + str(self.show_count)
            #cv2.imshow(lab, self.img)
            #cv2.waitKey(0)

            
            
      
               


    def StartCorrection(self, img, bbox):
        
        self.img = img
        self.original_img = img.copy()
        self.donebox = []
        self.right_object_row = -1
        self.left_object_row = -1
        self.right_objects = []
        self.left_objects = []
        self.show_count = 0
        

        self.bbox = bbox
        self.start_x = self.bbox[0]
        self.start_y = self.bbox[1]
        self.end_x = self.bbox[2]
        self.end_y = self.bbox[3]
        
        #print("SX,SY,EX,EY=", self.start_x, self.start_y, self.end_x,self.end_y)
        bbox_img = self.img[self.start_y:self.end_y,self.start_x:self.end_x]
        # make pixels 255 or 0
        b = np.where(bbox_img>200,[255],[0])
        self.on_rows, self.on_cols = np.where(b==255)
        #print ("StartConnection \n",self.on_rows, "\n",self.on_cols)
        #print ("StartConnection \n",self.start_y+self.on_rows,"\n", self.start_x+self.on_cols)
        
        rowlen = len(self.on_rows)
        rowindex = rowlen-1
        join_count = 0
        self.ConnectOneColumn(rowindex)
        self.show_count += 1
        #print("length of right, left objects - ",len(self.right_objects), len(self.left_objects) )
        #print("StartConnection - right object \n", self.right_objects)
        #print("StartConnection - left object \n", self.left_objects)
        #print("Number of done boxes - ", len(self.donebox))

        #self.print_doneboxes()
        #self.draw_doneboxes()
        

        lab = "Image - "+str(self.show_count)
        #cv2.imshow(lab, self.img)
        #cv2.waitKey(0)


        
        #print("StartConnection - length of right, left objects - ",len(self.right_objects), len(self.left_objects) )
        #print("StartConnection - right object \n", self.right_objects)
        #print("StartConnection - left object \n", self.left_objects)

        while  len(self.right_objects) >0 or len(self.left_objects) > 0:
          while len(self.right_objects) >0:
            rowindex = self.right_objects.pop(0)
            row = self.on_rows[rowindex]
            col = self.on_cols[rowindex]
            #print("StartConnection - Check right object - ", self.start_y+row, self.start_x+col)
                
            if self.is_in_donebox(row, col) == False:
                #print("StartConnection - Doing right object - ", self.start_y+row, self.start_x+col)
                self.ConnectOneColumn(rowindex)
                self.show_count += 1
                #lab = "Image - "+str(self.show_count)
                #cv2.imshow(lab, self.img)
                #cv2.waitKey(0)
        
          #self.print_doneboxes()

          #print("StartConnection - length of right, left objects - ",len(self.right_objects), len(self.left_objects) )
          #print("StartConnection - right object \n", self.right_objects)
          #print("StartConnection - left object \n", self.left_objects)
        
          while len(self.left_objects) >0:
            rowindex = self.left_objects.pop(0)
            row = self.on_rows[rowindex]
            col = self.on_cols[rowindex]
            #print("StartConnection - Check left object - ", self.start_y+row, self.start_x+col)
            
            if self.is_in_donebox(row, col) == False:
                #print("StartConnection - Doing left object- ", self.start_y+row, self.start_x+col)
                self.ConnectOneColumn(rowindex)
                self.show_count += 1
                #lab = "Image - "+str(self.show_count)
                #cv2.imshow(lab, self.img)
                #cv2.waitKey(0)
        
        #self.print_doneboxes()
        

        
        
        

    

    def join_one_right_edge(self, cur_rowindex):
        #print ("join_one_right_edge - cur_rowindex= ", cur_rowindex)
        cur_row = self.on_rows[cur_rowindex]
        cur_col = self.on_cols[cur_rowindex]
        
        while cur_rowindex >= 0 :
            cur_row = self.on_rows[cur_rowindex]
            cur_col = self.on_cols[cur_rowindex]
            
            next_rowindex = self.find_nearest_right(cur_rowindex)
            if next_rowindex == -1:
                return (-1)
            next_row = self.on_rows[next_rowindex]
            next_col = self.on_cols[next_rowindex]

            if self.ytop > next_row:
                self.ytop = next_row
            if self.ybottom < next_row:
                self.ybottom = next_row
            if self.xleft > next_col:
                self.xleft = next_col
            if self.xright < next_col:
                self.xright = next_col

            diff_row = cur_row - next_row
            diff_col = next_col - cur_col

            #if diff_row == 0 and diff_col == 1:
            #    cur_rowindex = next_rowindex
            #    continue
            #if diff_col == 0 and diff_row == 1:
            #    cur_rowindex = next_rowindex
            #    continue

            #if cur_row == 517 - self.start_y:
            #print("In while, cur_rowindex, next_rowindex= ", cur_rowindex, next_rowindex)
            #print("In while, cur_row, next_row, cur_col, next_col= ", cur_row+self.start_y, next_row+self.start_y, cur_col+self.start_x, next_col+self.start_x)
            
            if diff_row == 0:
                #print("right going to horizontal join")
                i = 1
                while diff_col >= i:
                    self.img[self.start_y+cur_row, self.start_x+cur_col + i] = 255
                    #print("right horizontal - y,x",self.start_y+cur_row, self.start_x+cur_col + i )
                    i += 1
            elif diff_col == 0:
                #print("right goiung to vertical join")
                while diff_row > 0:
                    self.img[self.start_y+next_row + diff_row, self.start_x+cur_col] = 255
                    #print("right vertical - y,x",self.start_y+next_row + diff_row, self.start_x+cur_col  )
                    diff_row -= 1
            else:
                if diff_row > 0:
                    #print("Right going to Step up join")
                    i = 0
                    while diff_row >0 and diff_col > 0:
                        self.img[self.start_y+cur_row-i, self.start_x+cur_col+i+1] = 255
                        self.img[self.start_y+cur_row-i-1, self.start_x+cur_col+i+1] = 255
                        #print("right step  up join - y, x", self.start_y+cur_row-i,self.start_x+cur_col+i+1 )
                        #print("right step  up join - y, x", self.start_y+cur_row-i-1,self.start_x+cur_col+i+1 )
                        i += 1
                        diff_row -= 1
                        diff_col -= 1
                    if (diff_row != 0 ):
                        while diff_row > 0:
                            self.img[self.start_y+cur_row-i, self.start_x+next_col] = 255
                            #print ("right step residual vertical up : y,x", self.start_y+cur_row-i, self.start_x+next_col)
                            i += 1
                            diff_row -= 1
                    elif diff_col > 0:
                        while diff_col > 0:
                            self.img[self.start_y+next_row, self.start_x+cur_col+i+1] = 255
                            #print ("right step up residual horizontal : y,x", self.start_y+next_row, self.start_x+cur_col+i+1)
                            i += 1
                            diff_col -= 1
                    

                            
                else:
                    #print("Right going to Step down join")
                    i = 0
                    while diff_row < 0 and diff_col > 0:
                        self.img[self.start_y+cur_row+i, self.start_x+cur_col+i+1] = 255
                        self.img[self.start_y+cur_row+i+1, self.start_x+cur_col+i+1] = 255
                        #print("right step down join - y, x", self.start_y+cur_row+i,self.start_x+cur_col+i+1 )
                        #print("right step down join - y, x", self.start_y+cur_row+i+1,self.start_x+cur_col+i+1 )
                        
                        i += 1
                        diff_row += 1
                        diff_col -= 1
                    if (diff_row != 0 ):
                        while diff_row < 0:
                            self.img[self.start_y+cur_row+i, self.start_x+next_col] = 255
                            #print ("right step residual vertical down : y,x", self.start_y+cur_row+i, self.start_x+next_col)
                            i += 1
                            diff_row += 1
                    elif diff_col > 0:
                        while diff_col > 0:
                            self.img[self.start_y+next_row, self.start_x+cur_col+i+1] = 255
                            #print ("right step down residual horizontal : y,x", self.start_y+next_row, self.start_x+cur_col+i+1)
                            i += 1
                            diff_col -= 1

            cur_rowindex = next_rowindex        

        
        return(-1)


    def join_one_left_edge(self, cur_rowindex):
        #print ("join_one_left_edge - cur_rowindex= ", cur_rowindex)
        
        cur_row = self.on_rows[cur_rowindex]
        cur_col = self.on_cols[cur_rowindex]
        
        while cur_rowindex >= 0 :
            cur_row = self.on_rows[cur_rowindex]
            cur_col = self.on_cols[cur_rowindex]
            
            next_rowindex = self.find_nearest_left(cur_rowindex)
            if next_rowindex == -1:
                return (-1)
            next_row = self.on_rows[next_rowindex]
            next_col = self.on_cols[next_rowindex]

            if self.ytop > next_row:
                self.ytop = next_row
            if self.ybottom < next_row:
                self.ybottom = next_row
            if self.xleft > next_col:
                self.xleft = next_col
            if self.xright < next_col:
                self.xright = next_col

            diff_row = cur_row - next_row
            diff_col = cur_col - next_col
            #if diff_row == 0 and diff_col == 1:
            #    cur_rowindex = next_rowindex
            #    continue
            #if diff_col == 0 and diff_row == 1:
            #    cur_rowindex = next_rowindex
            #    continue




            #if cur_row == 517 - self.start_y:
            #print("In while, cur_rowindex, next_rowindex= ", cur_rowindex, next_rowindex)
            #print("In while, cur_row, next_row, cur_col, next_col= ", cur_row+self.start_y, next_row+self.start_y, cur_col+self.start_x, next_col+self.start_x)
            
            if diff_row == 0:
                #print("left going to horizontal join")
                i = 1
                while diff_col >= i:
                    self.img[self.start_y+cur_row, self.start_x+cur_col - i] = 255
                    #print("left horizontal - y,x",self.start_y+cur_row, self.start_x+cur_col - i )
                    i += 1
            elif diff_col == 0:
                #print("left goiung to vertical join")
                while diff_row > 0:
                    self.img[self.start_y+next_row + diff_row, self.start_x+cur_col] = 255
                    #print("left vertical - y,x",self.start_y+next_row + diff_row, self.start_x+cur_col  )
                    diff_row -= 1
            else:
                if diff_row > 0:
                    #print("Right going to Step up join")
                    i = 0
                    while diff_row >0 and diff_col > 0:
                        self.img[self.start_y+cur_row-i, self.start_x+cur_col-i-1] = 255
                        self.img[self.start_y+cur_row-i-1, self.start_x+cur_col-i-1] = 255
                        #print("left step  up join 1- y, x", self.start_y+cur_row-i,self.start_x+cur_col-i-1 )
                        #print("left step  up join 2- y, x", self.start_y+cur_row-i-1,self.start_x+cur_col-i-1 )
                        i += 1
                        diff_row -= 1
                        diff_col -= 1
                    if (diff_row != 0 ):
                        while diff_row > 0:
                            self.img[self.start_y+cur_row-i, self.start_x+next_col] = 255
                            #print ("left step residual vertical up : y,x", self.start_y+cur_row-i, self.start_x+next_col)
                            i += 1
                            diff_row -= 1
                    elif diff_col > 0:
                        while diff_col > 0:
                            self.img[self.start_y+next_row, self.start_x+cur_col-i-1] = 255
                            #print ("left step up residual horizontal : y,x", self.start_y+next_row, self.start_x+cur_col-i-1)
                            i += 1
                            diff_col -= 1
                    

                            
                else:
                    #print("Left going to Step down join")
                    i = 0
                    while diff_row < 0 and diff_col > 0:
                        self.img[self.start_y+cur_row+i, self.start_x+cur_col-i-1] = 255
                        self.img[self.start_y+cur_row+i+1, self.start_x+cur_col-i-1] = 255
                        #print("left step down join - y, x", self.start_y+cur_row+i,self.start_x+cur_col-i-1 )
                        #print("left step down join - y, x", self.start_y+cur_row+i+1,self.start_x+cur_col-i-1 )
                        i += 1
                        diff_row += 1
                        diff_col -= 1
                    if (diff_row != 0 ):
                        while diff_row < 0:
                            self.img[self.start_y+cur_row+i, self.start_x+next_col] = 255
                            #print ("left step residual vertical down : y,x", self.start_y+cur_row+i, self.start_x+next_col)
                            i += 1
                            diff_row += 1
                    elif diff_col > 0:
                        while diff_col > 0:
                            self.img[self.start_y+next_row, self.start_x+cur_col-i-1] = 255
                            #print ("left step down residual horizontal : y,x", self.start_y+next_row, self.start_x+cur_col+i+1)
                            i += 1
                            diff_col -= 1
            cur_rowindex = next_rowindex
        
        return(-1)



    def find_nearest_right(self, rowindex):
        cur_row = self.on_rows[rowindex]
        cur_col = self.on_cols[rowindex]

        min_rowindex = -1
        min_diff_row = 1000
        min_diff_col = self.threshold+1

        next_row = cur_row
        next_col = cur_col
        diff_row = diff_col = 0
        
        #look upward
        
        nextrowindex = rowindex -1
        while nextrowindex >= 0:
            next_row = self.on_rows[nextrowindex]
            next_col = self.on_cols[nextrowindex]
            if self.is_in_donebox(next_row, next_col) == True:
                nextrowindex -= 1
                continue
            diff_row = cur_row - next_row
            diff_col = next_col - cur_col


                
            if diff_row <= self.threshold:
                if diff_col >= 0 and diff_col < min_diff_col:
                    min_diff_col = diff_col
                    min_rowindex = nextrowindex
                nextrowindex -= 1
            else:
                break
        #look downward
        #                   
        maxrowindex = len(self.on_rows)
        nextrowindex = rowindex +1
        while nextrowindex < maxrowindex:
            next_row = self.on_rows[nextrowindex]
            next_col = self.on_cols[nextrowindex]
            if self.is_in_donebox(next_row, next_col) == True:
                nextrowindex += 1
                continue

            diff_row = next_row - cur_row
            diff_col = next_col - cur_col
            
            if diff_row >= 0 and diff_col == 0: #vertical down not allowed
                    nextrowindex += 1
                    continue

            if diff_row <= self.threshold:
                if diff_col >= 0 and diff_col < min_diff_col:
                    min_diff_col = diff_col
                    min_rowindex = nextrowindex
                nextrowindex += 1
            else:
                break

        #print("find_nearest_right returns - ", min_rowindex)
        #if min_rowindex != -1:
         #   print("find_nearest_right - row, col", self.on_rows[min_rowindex] + self.start_y, self.on_cols[min_rowindex]+self.start_x)
        return (min_rowindex)


        
        
    def find_nearest_left(self, rowindex):
        cur_row = self.on_rows[rowindex]
        cur_col = self.on_cols[rowindex]

        min_rowindex = -1
        min_diff_row = 1000
        min_diff_col = self.threshold + 1

        next_row = cur_row
        next_col = cur_col
        diff_row = diff_col = 0
        #look upward
        
        nextrowindex = rowindex -1
        while nextrowindex >= 0:
            next_row = self.on_rows[nextrowindex]
            next_col = self.on_cols[nextrowindex]
            if self.is_in_donebox(next_row, next_col) == True:
                nextrowindex -= 1
                continue
            diff_row = cur_row - next_row
            diff_col = cur_col - next_col
            
            
            if diff_row <= self.threshold:
                if diff_col >= 0 and diff_col < min_diff_col :
                    min_diff_col = diff_col
                    min_rowindex = nextrowindex
                nextrowindex -= 1
            else:
                break
        if min_rowindex != -1:
            next_row = self.on_rows[min_rowindex]
            next_col = self.on_cols[min_rowindex]
        else:
            next_row = -1
            next_col = -1

        #print("FNL-up: cur_row, cur_col, next_row, next_col, diff_col, min_rowindex",self.start_y+cur_row, self.start_x+cur_col, self.start_y+next_row, self.start_x+next_col, diff_col,min_rowindex)
        #look downward
        #                   
        maxrowindex = len(self.on_rows)
        nextrowindex = rowindex +1
        while nextrowindex < maxrowindex:
            next_row = self.on_rows[nextrowindex]
            next_col = self.on_cols[nextrowindex]
            if self.is_in_donebox(next_row, next_col) == True:
                nextrowindex += 1
                continue

            diff_row = next_row - cur_row
            diff_col = cur_col - next_col

            if diff_row >= 0 and diff_col == 0: #vertical down not allowed
                    nextrowindex += 1
                    continue
    
            if diff_row <= self.threshold:
                if diff_col >= 0 and diff_col < min_diff_col :
                    min_diff_col = diff_col
                    min_rowindex = nextrowindex
                nextrowindex += 1
            else:
                break

        if min_rowindex != -1:
            next_row = self.on_rows[min_rowindex]
            next_col = self.on_cols[min_rowindex]
        else:
            next_row = -1
            next_col = -1
        #print("FNL-dn: cur_row, cur_col, next_row, next_col, diff_col, min_rowindex",self.start_y+cur_row, self.start_x+cur_col, self.start_y+next_row, self.start_x+next_col, diff_col,min_rowindex)
            
        #print("find_nearest_left returns - ", min_rowindex)
        #if min_rowindex != -1:
         #   print("find_nearest_left - row, col", self.on_rows[min_rowindex] + self.start_y, self.on_cols[min_rowindex]+self.start_x)
        return (min_rowindex)

        

            


            

                    

        





    def join_edges(self, rowindex):
        
        cur_row = self.on_rows[rowindex]
        right_col = self.on_cols[rowindex]
        
        self.ytop =  self.ybottom = cur_row
        self.xleft = self.xright = right_col
        
        # do right edge join
        rind = self.join_one_right_edge(rowindex)
        #cv2.imshow("After join right edge", img)
        #cv2.waitKey(0)

        #print("Join_one_right_edge returned ", rind)

        #do left edge join
        lind = self.join_one_left_edge(rowindex)
        left_row = self.on_rows[lind]+self.start_y
        right_row = self.on_rows[rind]+self.start_y
        #print ("join_edge()  - lind, rind", lind, rind, "lrow, rrow= ",left_row, right_row)
        bbox = [self.xleft, self.ytop, self.xright, self.ybottom]
        self.add_donebox(bbox)
        #self.print_doneboxes()

        #cv2.imshow("After join left edge", img)
        #cv2.waitKey(0)



        if rind > lind:
            return rind
        else:
            return lind






