#ifndef MOTION_ROS_H
#define MOTION_ROS_H
#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Point.h"
#include "PID.h"

#define base_speed 120

class motion_ros
{
public:
    motion_ros();
    ~motion_ros() {}

    static motion_ros *getInstance();

    void setNode(ros::NodeHandle node);
    void sendMotor(int pwmLeft, int pwmRight , int speedOffsetLeft = 0, int speedOffsetRight = 0);
    void imuCB(const sensor_msgs::ImuConstPtr &imuMsg);
    void pointCB(const geometry_msgs::PointConstPtr &pointMsg);

    bool update();
    double getErrorOrientation();

    void setOrientationVisual();
    void setPositionVisual(double x, double y, double z);
    void sendVisual(double x, double y, double z);

    void mainRobot();

    int changeSpeed(int speed, int increment);
    void rotate();

    int constraint(int val, int minimum , int maximum);

  

    // void printData()
    // {
    //     ROS_INFO_STREAM(dataImu.orientation.z);
    // }

private:
    static motion_ros *instance;
    ros::NodeHandle nh;
    
    ros::Publisher pwmLeftPub;
    ros::Publisher pwmRightPub;

    ros::Subscriber imuSub;
    ros::Subscriber pointSub;
    // Data Subscriber
    sensor_msgs::Imu dataImu;
    geometry_msgs::Point dataPoint;
    // std_msgs::Float64 targetAngle;

    int counterStateMax = 100;
    int max_speed = 150;
    double testTarget = 0;

    double target_val = 0 , target_dist = 0;

    std::chrono::high_resolution_clock::time_point  last_time, current;     

    //Class
    PID *pid;
};



#endif
