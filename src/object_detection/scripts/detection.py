#!/usr/bin/env python3 

import rospy
from geometry_msgs.msg import Point
import cv2 as cv
from icecream import ic
import numpy as np
import yaml


cap = cv.VideoCapture(0)

cap.set(3, 640)
cap.set(4, 480)


def nothing(val):
    pass

def readHSV(file):
    upper_color = []
    lower_color = []

    with open(file, "r") as data:
        try:
            config = yaml.safe_load(data)
            lower_color = np.array([config["hsv"]["lower"]["h"],config["hsv"]["lower"]["s"],config["hsv"]["lower"]["v"]])
            upper_color = np.array([config["hsv"]["upper"]["h"],config["hsv"]["upper"]["s"],config["hsv"]["upper"]["v"]])
            
            ic(lower_color, upper_color)
        except yaml.YAMLError as error:
            ic(error)
    return lower_color , upper_color

def saveHSV(file, lower = list, upper = list):
    config = yaml.safe_dump()


def get_center_obj(contour):
    M = cv.moments(contour)
    if M["m00"] != 0:
        cx = int(M['m10'] / M['m00'])
        cy = int(M["m01"] / M["m00"])
    else:
        cx = 0
        cy = 0
    return cx , cy

def detection(img, thresh_detect = 500):
    lower_color , upper_color = readHSV("../config/hsv_conf.yaml")
    ic(lower_color, upper_color)
    img_gray = cv.cvtColor(img , cv.COLOR_BGR2HSV)

    mask = cv.inRange(img_gray, lower_color , upper_color)

    # find countours
    contours, __ = cv.findContours(mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    for contour in contours:
        if cv.contourArea(contour) > thresh_detect:  # Filter out small contours
            cX, cY = get_center_obj(contour)
            ic(cX, cY)
            cv.drawContours(frame, [contour], -1, (0, 255, 0), 2)
            cv.circle(frame, (cX, cY), 7, (255, 0, 0), -1)
            cv.putText(frame, f"Center: ({cX}, {cY})", (cX - 20, cY - 20),
                        cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
    
    # Display the resulting frame
    cv.imshow('Frame', frame)
    cv.imshow('Mask', mask)

if __name__ == "__main__":

    while(cap.isOpened):
        succ, frame = cap.read()
        detection(frame, 400)

        key = cv.waitKey(1) & 0xFF


        if(key == ord('s')):
            ic("save")


        cv.imshow("frame", frame)

        if(key == 27):
            break



        
    cv.destroyAllWindows()
    cap.release()