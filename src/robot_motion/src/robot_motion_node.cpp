#include "iostream"
#include "icecream.hpp"
// #include "ros/ros.h"
#include "motion_ros.hpp"

int main(int argc, char  **argv)
{
    ros::init(argc, argv, "motion_node");
    ros::NodeHandle n;
    
    motion_ros::getInstance()->setNode(n);
    /* code */
    ros::Rate hz(30);
    while (ros::ok())
    {
        // motion_ros::getInstance()->update();
        /* code */
        ros::spinOnce();
        hz.sleep();
    // IC("CHECK");
    }
    
    
    return 0;
}
