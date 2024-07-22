#include "motion_ros.hpp"
#include "icecream.hpp"
#include "generalFunc.hpp"
motion_ros *motion_ros::instance = 0;

motion_ros *motion_ros::getInstance()
{
    if (instance == 0)
    {
        instance = new motion_ros();
    }
    return instance;
}
void motion_ros::setNode(ros::NodeHandle node)
{
    this->nh = node;
}
motion_ros::motion_ros()
{
    // pwmPub = nh.advertise<motion_pkg::pwmMotor>("speedMotor", 1);
    pwmLeftPub = nh.advertise<std_msgs::Int16>("speedLeft", 1);
    pwmRightPub = nh.advertise<std_msgs::Int16>("speedRight", 1);

    imuSub = nh.subscribe("data_imu", 1, &motion_ros::imuCB, this);
    pointSub = nh.subscribe("blob/point", 1, &motion_ros::pointCB, this);
    last_time = std::chrono::high_resolution_clock::now();
    // Class Init
    pid = new PID(10.0, 180);
}

void motion_ros::imuCB(const sensor_msgs::ImuConstPtr &imuMsg)
{
    // ros::Time
    dataImu = (*imuMsg);
    IC(dataImu.orientation.z);
}

void motion_ros::pointCB(const geometry_msgs::PointConstPtr &pointMsg)
{
    IC(pointMsg);
    // double f = 0.9;
    dataPoint = (*pointMsg);
    // target_val = f * target_val + dataPoint.x * (1 - f);
    // target_dist = f * target_dist + dataPoint.z * (1 - f);

    target_dist = sqrt(pow(dataPoint.x, 2) + pow(dataPoint.y, 2)) / 100.0;
    IC(target_dist);
    update();
    last_time = current;
    IC(dataPoint.x, dataPoint.y, dataPoint.z);
}

bool motion_ros::update()
{
    current = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> duration = current - last_time;

    static double v, w;
    
    double dt = duration.count() / 1e3;
    motor outputMotor;
    pwmMotor outPWM;
    static double speedPID;
    if (dt < 1)
    {
        double errorOrientation = dataPoint.z - dataImu.orientation.z;
        if (errorOrientation > 180)
            errorOrientation -= 360;
        if (errorOrientation < -180)
            errorOrientation += 360;
        IC(errorOrientation);
        if (fabs(errorOrientation) < 5)
        {
            outPWM.right = 150;
            outPWM.left = 150;
        }
        else
        {
            pid->setParamRot(0.5, 0, 0);
            IC(dataPoint.z);
            pid->calc_PIDRot(dataPoint.z * M_PI / 180.0, dataImu.orientation.z * M_PI / 180.0, speedPID);
            IC(speedPID);
            inversKinematic(outputMotor, 0, speedPID);
            rpsToPwm(outPWM, outputMotor.w1, outputMotor.w2);
            static double speedLeft , speedRight;
            speedLeft = outPWM.left;
            speedRight = outPWM.right;
            IC(speedLeft, speedRight);
            outPWM.left = base_speed + speedLeft;
            outPWM.right = base_speed + speedRight;

            IC(outPWM.left, outPWM.right);
        }

        if (target_dist < 0.3)
        {
            outPWM.left = 0;
            outPWM.right = 0;
        }
    }
    else
    {
        std::cout << "Target Lost || dist reach" << std::endl;
        outPWM.left = 0;
        outPWM.right = 0;
    }

    sendMotor(outPWM.left, outPWM.right);

    IC(outputMotor.w1, outputMotor.w2);
    IC(outPWM.left, outPWM.right);
    IC(dt);
    // IC();

    return true;
}

void motion_ros::sendMotor(int pwmLeft, int pwmRight, int speedOffsetLeft, int speedOffsetRight)
{
    std_msgs::Int16 left;
    left.data = pwmLeft + speedOffsetLeft;
    pwmLeftPub.publish(left);

    std_msgs::Int16 right;
    right.data = pwmRight + speedOffsetRight;
    pwmRightPub.publish(right);
    ROS_INFO("PUBLISH");
}
