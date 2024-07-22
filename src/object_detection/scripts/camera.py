#!/usr/bin/env python3

import rospy
import cv2 as cv
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from icecream import ic
import numpy as np
from geometry_msgs.msg import Point
def colorPass(x):
    pass

def setTrackBar():
     cv.namedWindow("Tuning", 0)
     cv.createTrackbar("h_min","Tuning",0,255,colorPass)
     cv.createTrackbar("h_max","Tuning",0,255,colorPass)
     cv.createTrackbar("s_min","Tuning",0,255,colorPass)
     cv.createTrackbar("s_max","Tuning",0,255,colorPass)
     cv.createTrackbar("v_min","Tuning",0,255,colorPass)
     cv.createTrackbar("v_max","Tuning",0,255,colorPass)

def getTrackBar():
    trackbar_names = ["h_min","h_max","s_min","s_max","v_min","v_max"]
    return {key:cv.getTrackbarPos(key, "Tuning") for key in trackbar_names}


def normalise_keypoint(cv_image, kp):
    rows = float(cv_image.shape[0])
    cols = float(cv_image.shape[1])
    # print(rows, cols)
    center_x    = 0.5*cols
    center_y    = 0.5*rows
    # print(center_x)
    x = (kp.pt[0] - center_x)/(center_x)
    y = (kp.pt[1] - center_y)/(center_y)
    return cv.KeyPoint(x, y, kp.size/cv_image.shape[1])


def convert_rect_perc_to_pixels(rect_perc, image):
    rows = image.shape[0]
    cols = image.shape[1]

    scale = [cols, rows, cols, rows]

    
    # x_min_px    = int(cols*window_adim[0])
    # y_min_px    = int(rows*window_adim[1])
    # x_max_px    = int(cols*window_adim[2])
    # y_max_px    = int(rows*window_adim[3]) 
    return [int(a*b/100) for a,b in zip(rect_perc, scale)]

def draw_window2(image,              #- Input image
                rect_px,        #- window in adimensional units
                color=(255,255,0),    #- line's color
                line=5,             #- line's thickness
               ):
    
    #-- Draw a rectangle from top left to bottom right corner

    return cv.rectangle(image,(rect_px[0],rect_px[1]),(rect_px[2],rect_px[3]),color,line)

def apply_search_window(image, window_adim=[0.0, 0.0, 1.0, 1.0]):
    rows = image.shape[0]
    cols = image.shape[1]
    x_min_px    = int(cols*window_adim[0]/100)
    y_min_px    = int(rows*window_adim[1]/100)
    x_max_px    = int(cols*window_adim[2]/100)
    y_max_px    = int(rows*window_adim[3]/100)    
    
    #--- Initialize the mask as a black image
    mask = np.zeros(image.shape,np.uint8)
    
    #--- Copy the pixels from the original image corresponding to the window
    mask[y_min_px:y_max_px,x_min_px:x_max_px] = image[y_min_px:y_max_px,x_min_px:x_max_px]   
    
    #--- return the mask
    return(mask)



class ImageVideoRead:

    def __init__(self, use_trackbar = False):
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.callbackIMG)
        self.point_pub = rospy.Publisher("/blob/point", Point, queue_size=1)
        
        
        self.bridge_img = CvBridge()
        self.use_trackbar = use_trackbar
        if(self.use_trackbar):
            setTrackBar()        
       
    def streamDetection(self):
        cap = cv.VideoCapture(2)
        cap.set(3, 640)
        cap.set(4, 480)

        while True:
            success ,image = cap.read()
            
            color_min = (5, 84, 101)
            color_max = (21, 255, 255)

            
            if(self.use_trackbar):
                value_params = getTrackBar()
                color_min = (value_params["h_min"], value_params["s_min"], value_params["v_min"])
                color_max = (value_params["h_max"], value_params["s_max"], value_params["v_max"])
            ic(color_min, color_max)
            hsv_image = cv.cvtColor(image, cv.COLOR_BGR2HSV)
        
            #Start Destroy noise
            search_window = [0, 0, 100, 100]

            mask_image = cv.inRange(hsv_image, color_min, color_max)

            search_window_px = convert_rect_perc_to_pixels(search_window, image)        

            mask_image = apply_search_window(mask_image, search_window)
            cv.imshow("MASK", mask_image)
    # In    vert the image to suit the blob detector
            mask_image = 255-mask_image
            cv.imshow("AFTER REVERSE", mask_image)

            #try to find point object
            params = cv.SimpleBlobDetector_Params()

            # Change thresholds
            params.minThreshold = 0
            params.maxThreshold = 100

            # Filter by Area.
            params.filterByArea = True
            params.minArea = 30
            params.maxArea = 20000

            # Filter by Circularity
            params.filterByCircularity = True
            params.minCircularity = 0.1

            # Filter by Convexity
            params.filterByConvexity = True
            params.minConvexity = 0.5

            # Filter by Inertia
            params.filterByInertia =True
            params.minInertiaRatio = 0.5

            detector = cv.SimpleBlobDetector_create(params)

            keypoints = detector.detect(mask_image)
            # ic(keypoints)

            size_min_px = 0 * mask_image.shape[1] / 100.0 
            size_max_px = 100 * mask_image.shape[1] / 100.0

            keypoints = [k for k in keypoints if k.size > size_min_px and k.size < size_max_px]
            # ic(keypoints)
            line_color=(255,0,0)

            out_image = cv.drawKeypoints(image, keypoints, np.array([]), line_color, cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

            # ic(search_window_px)

            out_image = draw_window2(out_image, search_window_px)

            keypoints_normalised = [normalise_keypoint(mask_image, k) for k in keypoints]
            # ic(keypoints_normalised)

            point_out = Point()

                # Keep the biggest point
                # They are already converted to normalised coordinates
            for i, kp in enumerate(keypoints_normalised):
                    x = kp.pt[0]
                    y = kp.pt[1]
                    s = kp.size

                    # rospy.loginfo(f"Pt {i}: ({x},{y},{s})")

                    if (s > point_out.z):                    
                        point_out.x = x
                        point_out.y = y
                        point_out.z = s

            if (point_out.z > 0):
                    self.point_pub.publish(point_out)    

            ic(point_out.x, point_out.y, point_out.z)
            cv.imshow("Blob", out_image)
            # cv.imshow("Mask", mask_image)
            result = cv.bitwise_and(image, image, mask = mask_image)
            # mask_image = cv.dilate(mask_image, None, iterations=1)
            # after_boost = cv.erode(mask_image, None, iterations = 2)

            # cv.imshow("after Boost", after_boost)        

            # cv.imshow("RES", image)
            key = cv.waitKey(1)
            if key == 27 or key == 99:
             break
        cap.release()
        cv.destroyAllWindows()

    def callbackIMG(self, data):
        try :
            cv_image = self.bridge_img.imgmsg_to_cv2(data, desired_encoding="bgr8")
        
        except CvBridgeError as e:
            rospy.logerr(e)
        
        color_min = (7 , 206, 59)
        color_max = (151, 255, 255)
        
        if(self.use_trackbar):
            value_params = getTrackBar()
            color_min = (value_params["h_min"], value_params["s_min"], value_params["v_min"])
            color_max = (value_params["h_max"], value_params["s_max"], value_params["v_max"])
            # ic(value_params["h_min"], value_params["h_max"])
            # ic(cv_image)
                        
        ic(color_min, color_max)                
        hsv_image = cv.cvtColor(cv_image, cv.COLOR_BGR2HSV)
        
        #Start Destroy noise
        search_window = [0, 0, 100, 100]
        
        mask_image = cv.inRange(hsv_image, color_min, color_max)
        
        search_window_px = convert_rect_perc_to_pixels(search_window, cv_image)        
        
        mask_image = apply_search_window(mask_image, search_window)

    # Invert the image to suit the blob detector
        mask_image = 255-mask_image
        cv.imshow("AFTER REVERSE", mask_image)
       
        #try to find point object
        params = cv.SimpleBlobDetector_Params()
        
        # Change thresholds
        params.minThreshold = 0
        params.maxThreshold = 100
            
        # Filter by Area.
        params.filterByArea = True
        params.minArea = 30
        params.maxArea = 20000
            
        # Filter by Circularity
        params.filterByCircularity = True
        params.minCircularity = 0.1
            
        # Filter by Convexity
        params.filterByConvexity = True
        params.minConvexity = 0.5
            
        # Filter by Inertia
        params.filterByInertia =True
        params.minInertiaRatio = 0.5
        
        detector = cv.SimpleBlobDetector_create(params)
        
        keypoints = detector.detect(mask_image)
        # ic(keypoints)
        
        size_min_px = 0 * mask_image.shape[1] / 100.0 
        size_max_px = 100 * mask_image.shape[1] / 100.0
        
        keypoints = [k for k in keypoints if k.size > size_min_px and k.size < size_max_px]
        # ic(keypoints)
        line_color=(255,0,0)

        out_image = cv.drawKeypoints(cv_image, keypoints, np.array([]), line_color, cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        
        # ic(search_window_px)
    
        out_image = draw_window2(out_image, search_window_px)

        keypoints_normalised = [normalise_keypoint(mask_image, k) for k in keypoints]
        # ic(keypoints_normalised)
        
        point_out = Point()

            # Keep the biggest point
            # They are already converted to normalised coordinates
        for i, kp in enumerate(keypoints_normalised):
                x = kp.pt[0]
                y = kp.pt[1]
                s = kp.size
                
                # rospy.loginfo(f"Pt {i}: ({x},{y},{s})")

                if (s > point_out.z):                    
                    point_out.x = x
                    point_out.y = y
                    point_out.z = s
                    
        if (point_out.z > 0):
                self.point_pub.publish(point_out)    
                         
        ic(point_out.x, point_out.y, point_out.z)
        cv.imshow("Blob", out_image)
        # cv.imshow("Mask", mask_image)
        result = cv.bitwise_and(cv_image, cv_image, mask = mask_image)
        # mask_image = cv.dilate(mask_image, None, iterations=1)
        # after_boost = cv.erode(mask_image, None, iterations = 2)
        
        # cv.imshow("after Boost", after_boost)        
    
        cv.imshow("Image", result)
     
        # cv.imshow("HSV" , hsv_image)    
        
        cv.waitKey(2)


def main():
    use_track = False
    camera = ImageVideoRead(use_trackbar= use_track)
    # ic()
    camera.streamDetection()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.logerr("Shutting Down")

    cv.destroyAllWindows()


if(__name__ == "__main__"):
    rospy.init_node("camera", anonymous= False)
    main()