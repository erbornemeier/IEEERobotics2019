import cv2
import numpy as np
from math import atan2
from geometryHelpers import *

class FeatureDetector:
    RAD_TO_DEG = 180 / 3.14159265
    def __init__(self, frameChecksPerFeature, debug = False):
        #video capture device
        self.cap = cv2.VideoCapture(1)
        #kernel for erode and dilate
        self.kernel = np.ones((5,5), np.uint8)
        #center of screen for comparisons
        self.SCREEN_CENTER = [int(i/2) for i in self.cap.read()[1].shape[:2]][::-1]
        self.frame_checks = frameChecksPerFeature
        self.debug = debug

    def read(self):
        #base img
        _, self.orig_img = self.cap.read()
        img_gray = cv2.cvtColor(self.orig_img,cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur (img_gray, (5,5), 0)
        ret, thresh = cv2.threshold(blur, 60, 255,
                                    cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        thresh = cv2.erode(thresh, self.kernel, iterations=1)
        self.thresh = cv2.dilate(thresh, self.kernel, iterations=1)
        cv2.imshow("Otsu",self.thresh) 
        _, contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, 
                                          cv2.CHAIN_APPROX_SIMPLE)
        #contours exist, successful read
        if contours:
            #get the maximum contour
            self.max_contour = contours[np.argmax(
                                        np.array([cv2.contourArea(cnt) for cnt in contours]),
                                        axis=0)]
            #get convex hull of max contour
            self.hull = cv2.convexHull(self.max_contour, returnPoints = False)
            #get convexity defects between contour and convex hull
            self.defects = cv2.convexityDefects(self.max_contour,self.hull)
            return True
        #no contours found, failed read
        else:
            return False

    def lineDetect(self, orientation):
        errs = (0, 0, 0)
        
        succ_reads = 0

        if orientation == "horizontal":
            getHorizontal = True
        elif orientation == "vertical":
            getHorizontal = False 
        else:
            raise ValueError("'horizontal' or 'vertical' only")

        for f in range(self.frame_checks):
            self.read()
            pts = [tuple(pt) for pt in cv2.boxPoints(
                                       cv2.minAreaRect(
                                       self.max_contour))]
            mid_pts = []
            while pts:
                fst = pts.pop()
                if not pts:
                    break
                snd = min(pts, key=lambda p: length_squared(p, fst))
                pts.remove(snd)
                mid_pts.append(midpoint(fst, snd, toInt=True))

                if self.debug:
                    cv2.circle(self.orig_img,mid_pts[-1],5,[0,255,0],-1)

            # Distance from the line to the screen
            proj = projection(mid_pts[0], mid_pts[1], self.SCREEN_CENTER, toInt=True)
            err = length(proj, self.SCREEN_CENTER)
            err = [0, err] 

            if getHorizontal:
                angle_ref = max(mid_pts, key=lambda p: p[0])
            else:
                angle_ref = max(mid_pts, key=lambda p: p[1])
            line_center = ((mid_pts[0][0] + mid_pts[1][0]) / 2, (mid_pts[0][1] + mid_pts[1][1]) / 2) 
            r_err = -(atan2(angle_ref[1] - line_center[1],
                            angle_ref[0] - line_center[0])
                            * self.RAD_TO_DEG) + (0 if getHorizontal else 90)

            if self.debug and len(mid_pts) > 1:
                cv2.line(self.orig_img, mid_pts[0], mid_pts[1], [0,255,0], 2)
                cv2.line(self.orig_img, tuple(proj), tuple(self.SCREEN_CENTER), [0,255,0], 2)
                cv2.imshow("lineCheck", self.orig_img)
                if cv2.waitKey(1) & 0xff == ord('q'):
                    cv2.destroyAllWindows()
            if getHorizontal:
                err[0] *= 1 if self.SCREEN_CENTER[1] - proj[1] > 0 else -1
            else:
                err[0] *= 1 if self.SCREEN_CENTER[0] - proj[0] > 0 else -1

            frame_err = (*err, r_err)
            print(frame_err)
            errs = [errs[i] + frame_err[i] for i in range(3)]
            succ_reads += 1
        errs = [errs[i] / succ_reads for i in range(3)]
        print(errs)
        return errs

    def crossDetect(self):
        errors = (0, 0, 0)

        succ_reads = 0

        for frame in range(self.frame_checks):
            self.read()
            if self.defects is not None:
                #get the 4 largest defects
                top_defects = self.defects[self.defects[:,0][:,3].argsort()[-4:]]
                pts = []
                dists = [] #BONUS: helps to determine if a cross exists
                for i in range(top_defects.shape[0]):
                    #get start, end, defect pt, dist
                    s,e,f,d = top_defects[i,0]
                    start = tuple(self.max_contour[s][0])
                    end = tuple(self.max_contour[e][0])
                    #append relevant hull points 
                    pts.append(start)
                    pts.append(end)
                    far = tuple(self.max_contour[f][0])
                    
                    #BONUS: projection to determine if a cross exists
                    #get the projection point
                    proj = projection(start, end, far)
                    #convert to pixel-friendly values
                    proj = proj.astype(np.int32)
                    #save shortline distance
                    dists.append(length_squared(proj, far)) 
                    
                    #draw
                    if self.debug:
                        cv2.line(self.orig_img,far,tuple(proj),[255,0,0],2)
                        cv2.line(self.orig_img,start,end,[0,255,0],2)
                        cv2.circle(self.orig_img,far,5,[0,0,255],-1)

                #BONUS: check if cross exists here **

                #get the midpoints of the cross lines
                mid_pts = []
                while pts:
                    first = pts.pop()
                    second = min(pts, key=lambda p: length_squared(first, p))
                    pts.remove(second)
                    mid_pts.append(midpoint(first, second, toInt = True))
                    if self.debug:
                        cv2.circle(self.orig_img, mid_pts[-1], 5, [255,0,0], -1)
                #sort left to right
                mid_pts.sort(key=lambda p: p[0])
                if len(mid_pts) > 3:
                    cross_center = seg_intersect(mid_pts[0], mid_pts[3],
                                                 mid_pts[1], mid_pts[2], toInt = True)
                    #x difference
                    x_proj = projection(mid_pts[0], mid_pts[3], self.SCREEN_CENTER, toInt = True)
                    x_error = length(cross_center, x_proj)
                    x_error *= -1 if self.SCREEN_CENTER[0] - cross_center[0] < 0 else 1
               
                    #y difference
                    y_proj = projection(mid_pts[1], mid_pts[2], self.SCREEN_CENTER, toInt = True)
                    y_error = length(cross_center, y_proj)
                    y_error *= -1 if self.SCREEN_CENTER[1] - cross_center[1] > 0 else 1
               
                    #r difference
                    angle_ref = max(mid_pts, key=lambda p: p[1] - self.SCREEN_CENTER[1])
                    r_error = (atan2(angle_ref[1] - cross_center[1],
                                    angle_ref[0] - cross_center[0])
                                    * self.RAD_TO_DEG) - 90
                    if self.debug:
                        cv2.line(self.orig_img, tuple(cross_center), tuple(x_proj), [0,255,0], 2)
                        cv2.line(self.orig_img, tuple(cross_center), tuple(y_proj), [0,255,0], 2)
                        cv2.circle(self.orig_img, tuple(cross_center), 5, [0,0,255], -1)
                        cv2.circle(self.orig_img, tuple(self.SCREEN_CENTER), 5, [255,0,0], -1)
                        cv2.imshow("crossCheck", self.orig_img)
                        if cv2.waitKey(1) & 0xff == ord('q'):
                            cv2.destroyAllWindows()
                    frame_error = (x_error, y_error, r_error)
                    print(frame_error)
                    errors = [errors[i] + frame_error[i] for i in range(3)]
                    succ_reads += 1
        if succ_reads == 0: #REMOVE THIS
            return (-1,-1,-1)
        
        errors = [errors[i] / succ_reads for i in range(3)]
        print (errors)
        return errors

    #TODO get midpoint of defect hull line
    def forkDetect(self, orientation):
        
        diffLine = False
        #target angle in reference to line left of the diagonal
        if orientation == 'up':
            targetAngle = 135
        elif orientation == 'down':
            targetAngle = -45
        elif orientation == 'down-right':
            targetAngle = 0
        elif orientation == 'up-right':
            targetAngle = 90
        elif orientation == 'up-left':
            targetAngle = 90
            diffLine = True
        elif orientation == 'down-left':
            targetAngle == -90
        else:
            raise ValueError("Not a valid orientation of fork")

        errors = (0, 0, 0)
        circ_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (120,120))
        succ_reads = 0

        for frame in range(self.frame_checks):
            self.read()
            #erode all data away except the center of the image
            center_find = cv2.erode(self.thresh, circ_kernel, iterations=1)
            _, center_cnts, _ = cv2.findContours(center_find, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            #get the maximum image contour
            if center_cnts:
                max_center_cnt = center_cnts[np.argmax(
                                             np.array([cv2.contourArea(cnt) 
                                             for cnt in center_cnts]), axis=0)]
                #get moments
                M = cv2.moments(max_center_cnt)
                #get center
                if M['m00'] != 0:
                    cx, cy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])
                    center = (cx, cy)
                    if self.debug:
                        cv2.circle(self.orig_img, center, 5, [0,0,255], -1)
                  

                    if self.defects is not None:
                        #get the 4 largest defects
                        top_defects = self.defects[self.defects[:,0][:,3].argsort()[-4:]]
                        pts = []
                        for i in range(top_defects.shape[0]):
                            #get start, end, defect pt, dist
                            s,e,f,d = top_defects[i,0]
                            start = tuple(self.max_contour[s][0])
                            end = tuple(self.max_contour[e][0])
                            #append relevant hull points 
                            far = tuple(self.max_contour[f][0])
                            btwn = angle_between(start, far, end)
                            pts.append((far, btwn))
                            if self.debug:
                                cv2.line(self.orig_img, start, end, [0,255,0],2)
                        
                        #sort by angle between hull line and defect
                        pts.sort(key=lambda x: x[1])
                        #get two smallest values
                        pts = pts[:2]
                        if self.debug:
                            for p in pts:
                                cv2.circle(self.orig_img, p[0], 5, [255,0,0], -1)
                        
                        if len(pts) > 1:
                            #get midpoint of two defects
                            defect_mid = midpoint(tuple(pts[0][0]), tuple(pts[1][0]), 
                                                  toInt=True)
                            diag1 = ((defect_mid[0]-cx)*1000  + cx, (defect_mid[1]-cy)*1000  + cy)
                            diag2 = ((defect_mid[0]-cx)*-1000 + cx, (defect_mid[1]-cy)*-1000 + cy)
                            
                            diag  = (diag1, diag2)
                            left  = rotate_around(center, diag1, -45, toInt = True)
                            left2 = rotate_around(center, diag1, 135, toInt = True)
                            right = rotate_around(center, diag1, 45, toInt = True)
                            right2= rotate_around(center, diag1, -135, toInt = True)

                            x_proj = projection(left, left2, self.SCREEN_CENTER, toInt=True)
                            x_error = length(x_proj, self.SCREEN_CENTER)
                            x_ref_point = (center[0] + self.SCREEN_CENTER[0] - x_proj[0],
                                           center[1] + self.SCREEN_CENTER[1] - x_proj[1])
                            if angle_between(x_ref_point, center, right) > math.pi/2.0:
                                x_error *= -1

                            y_proj = projection(right, right2, self.SCREEN_CENTER, toInt=True)
                            y_error = length(y_proj, self.SCREEN_CENTER)
                            y_ref_point = (center[0] + self.SCREEN_CENTER[0] - y_proj[0],
                                           center[1] + self.SCREEN_CENTER[1] - y_proj[1])
                            if angle_between(y_ref_point, center, left) > math.pi/2.0:
                                y_error *= -1

                            #r difference
                            if not diffLine:
                                r_error = (atan2(left[1] - center[1],
                                                 left[0] - center[0]) 
                                                 * self.RAD_TO_DEG)
                            else:
                                r_error = (atan2(right[1] - center[1],
                                                 right[0] - center[0]) 
                                                 * self.RAD_TO_DEG)
                            
                            r_error += targetAngle 

                            if self.debug:
                                cv2.line(self.orig_img, defect_mid, center, [255,0,0], 2)
                                cv2.line(self.orig_img, left, center, [255,0,0], 2)
                                cv2.line(self.orig_img, right, center,[255,0,0], 2)
                                cv2.line(self.orig_img, tuple(x_proj), 
                                         tuple(self.SCREEN_CENTER), [0,255,0], 2)
                                cv2.line(self.orig_img, tuple(y_proj), 
                                         tuple(self.SCREEN_CENTER), [0,255,0], 2)
                                cv2.imshow("Fork Detect", self.orig_img)
                                if cv2.waitKey(1) & 0xff == ord('q'):
                                    cv2.destroyAllWindows()
                            
                            frame_error = (x_error, y_error, r_error)
                            print(frame_error)
                            errors = [errors[i] + frame_error[i] for i in range(3)]
                            succ_reads += 1

        errors = [errors[i] / succ_reads for i in range(3)]
        print (errors)
        return errors

    def LDetect(self):
        errors = (0, 0, 0)

        succ_reads = 0

        for frame in range(self.frame_checks):
            self.read()
            if self.defects is not None:
                #get the largest defect
                top_defect = self.defects[self.defects[:,0][:,3].argmax()]
                print (top_defect)
                #get start, end, defect pt, dist
                s,e,f,d = top_defect[0]

                pts = []
                pts.append(tuple(self.max_contour[s][0]))
                pts.append(tuple(self.max_contour[e][0]))
                far = tuple(self.max_contour[f][0])
                
                #sort to get highest point first
                if pts[0][1] < pts[1][1]:
                    pts = pts[::-1]

                #get the x projection point
                x_end = (far[0] - 1000 * (pts[0][0] - far[0]),
                        (far[1] - 1000 * (pts[0][1] - far[1])))
                x_proj = projection(pts[0], x_end, self.SCREEN_CENTER, toInt=True)
                x_error = length(x_proj, self.SCREEN_CENTER)
                if self.SCREEN_CENTER[0] < x_proj[0]:
                    x_error *= -1
               
                #get the y projection point
                y_end = (far[0] - 1000 * (pts[1][0] - far[0]),
                        (far[1] - 1000 * (pts[1][1] - far[1])))
                y_proj = projection(pts[1], y_end, self.SCREEN_CENTER, toInt=True)
                y_error = length(x_proj, self.SCREEN_CENTER)
                if self.SCREEN_CENTER[1] < y_proj[1]:
                    y_error *= -1
                
                #r difference
                r_error = (atan2(pts[1][1] - far[1], 
                                 pts[1][0] - far[0])
                                 * self.RAD_TO_DEG)
                    
                #draw
                if self.debug:
                    cv2.line(self.orig_img, far,pts[0], [255,0,0], 2)
                    cv2.line(self.orig_img, far,pts[1], [0,255,0], 2)
                    cv2.circle(self.orig_img,far,5,[0,0,255],-1)
                    cv2.imshow("LCheck", self.orig_img)

                    if cv2.waitKey(1) & 0xff == ord('q'):
                        cv2.destroyAllWindows()

                frame_error = (x_error, y_error, r_error)
                print(frame_error)
                errors = [errors[i] + frame_error[i] for i in range(3)]
                succ_reads += 1

        errors = [errors[i] / succ_reads for i in range(3)]
        print (errors)
        return errors

if __name__=="__main__":
    fd = FeatureDetector(30,debug=True)
    while True:
        fd.crossDetect()
