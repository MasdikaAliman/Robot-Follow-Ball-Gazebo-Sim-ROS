#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
import time
from icecream import ic
class FollowBall:
    
    def __init__(self):
        self.ball_sub = rospy.Subscriber("/blob/point", Point, self.ball_CB)
        self.vel_pub =  rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        
        self.target_val = 0
        self.target_dist = 0
        self.lasttime = time.time() - 10000
        
    def mainloop(self):
        msg = Twist()
        ic(time.time() - self.lasttime)
        ic(self.target_dist)
        if (time.time() - self.lasttime < 1):
            rospy.loginfo('Target: {}'.format(self.target_val))
            print(self.target_dist)
            if (self.target_dist < 0.2):
                msg.linear.x = 0.1
            else:
                msg.linear.x = 0
            msg.angular.z = -0.7*self.target_val
        else:
            rospy.loginfo('Target lost')
            msg.linear.x = 0
            msg.angular.z = 0.5
            
        # msg.angular.z = 0.5    
        ic(msg.angular.z, msg.linear.x)
        self.vel_pub.publish(msg)   
    
    def ball_CB(self, data):
        f  = 0.9
        self.target_val = self.target_val * f + data.x * (1-f)
        self.target_dist = self.target_dist * f + data.z * (1-f)
        
        self.mainloop()
        
        self.lasttime = time.time()
        

def main():
    followBall = FollowBall()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.logerr("Shutting Down")


if(__name__ == "__main__"):
    rospy.init_node("follow_ball", anonymous= False)
    main()