#include <stdio.h>
#include <string.h>

// ZED include
#include <sl/Camera.hpp>
#include <opencv2/opencv.hpp>
#include <barelangFile.h>
#include <barelangTrackbar.h>
#include "def.h"
#include "RtDB2.h"
#include "icecream.hpp"
#include "ros/ros.h"
#include "geometry_msgs/Point.h"
// Using std and sl namespaces
using namespace std;
using namespace sl;

barelang::TrackbarColour *Bola{nullptr};

float getMedian(std::vector<float> &v)
{
    size_t n = v.size() / 2;
    std::nth_element(v.begin(), v.begin() + n, v.end());
    return v[n];
}

void get_3d_coordinates(bbox_t &cur_box, const sl::Mat &pointCloud, sl::float4 &point)
{
    bool valid_measure;
    int i, j;
    const unsigned int R_max_global = 10;

    const unsigned int obj_size = std::min(cur_box.w, cur_box.h);
    const unsigned int R_max = std::min(R_max_global, obj_size / 2);
    int center_i = cur_box.x + cur_box.w * 0.5f;
    int center_j = cur_box.y + cur_box.h * 0.5f;

    std::vector<float> x_vect, y_vect, z_vect;
    for (int R = 0; R < R_max; R++)
    {
        for (int y = -R; y <= R; y++)
        {
            for (int x = -R; x <= R; x++)
            {
                i = center_i + x;
                j = center_j + y;
                sl::float4 out(NAN, NAN, NAN, NAN);
                if (i >= 0 && i < pointCloud.getWidth() && j >= 0 &&
                    j < pointCloud.getHeight())
                {
                    pointCloud.getValue(i, j, &out);
                }
                valid_measure = std::isfinite(out.z);
                if (valid_measure)
                {
                    x_vect.push_back(out.x);
                    y_vect.push_back(out.y);
                    z_vect.push_back(out.z);
                }
            }
        }
    }

    if (x_vect.size() > 0)
    {
        point.x = getMedian(x_vect);
        point.y = getMedian(y_vect);
        point.z = getMedian(z_vect);
    }
    else
    {
        point.x = NAN;
        point.y = NAN;
        point.z = NAN;
    }
}

int main(int argc, char **argv)
{
    sl::Camera zed;

    barelangFile<int> Conf("/home/dika/Desktop/UAS_ROS/src/object_detection/config/zed_cam.txt");
    InitParameters init_parameters;
    init_parameters.depth_mode = DEPTH_MODE::QUALITY;    // Use ULTRA depth mode
    init_parameters.coordinate_units = UNIT::CENTIMETER; // Use millimeter units (for depth measurements)
    init_parameters.camera_resolution = RESOLUTION::VGA;
    init_parameters.camera_fps = 30;
    init_parameters.depth_minimum_distance = 10;
    ros::init(argc, argv, "camera_node");
    
    ros::NodeHandle nh_;

    ros::Publisher pubBlob;
    pubBlob = nh_.advertise<geometry_msgs::Point>("/blob/point", 1);
    Bola = new barelang::TrackbarColour(
        "BOLA", "OBJEK", barelang::ColorChannel::HSV, Conf["Bola_H_MIN"],
        Conf["Bola_H_MAX"], Conf["Bola_S_MIN"], Conf["Bola_S_MAX"],
        Conf["Bola_V_MIN"], Conf["Bola_V_MAX"], 255, true);

    auto returned_state = zed.open(init_parameters);
    IC(returned_state);
    if (returned_state != ERROR_CODE::SUCCESS)
    {
        cout << "Error " << returned_state << ", exit program." << endl;
        return EXIT_FAILURE;
    }

    geometry_msgs::Point pointBlob;

    while (ros::ok())
    {

        if (zed.grab() == ERROR_CODE::SUCCESS)
        {
            sl::Mat image, point_cloud;

        

            zed.retrieveImage(image, VIEW::LEFT);

            zed.retrieveMeasure(point_cloud, MEASURE::XYZ, sl::MEM::CPU);

            cv::Mat pointCloud = cv::Mat((int)point_cloud.getHeight(), (int)point_cloud.getWidth(), CV_8UC4, point_cloud.getPtr<sl::uchar1>(sl::MEM::CPU));

            cv::Mat cvImage = cv::Mat((int)image.getHeight(), (int)image.getWidth(), CV_8UC4, image.getPtr<sl::uchar1>(sl::MEM::CPU));

            cv::Mat HSV;

            cv::cvtColor(cvImage, HSV, cv::COLOR_BGR2HSV);

            cv::Mat ball_detected;
            // IC(Bola->getLower(), Bola->getUpper());
            
            cv::inRange(HSV, Bola->getLower(), Bola->getUpper(), ball_detected);
            // cv::erode(HSV, ball_detected, cv::getStructuringElement(cv::MORPH_RECT , cv::Size(5,5)));
            cv::imshow("Bola", ball_detected);
            cv::Mat ball_field;
            // cv::bitwise_and(ball_detected, ball_detected, ball_field, field_detect);

            // cv::imshow("bola and field", ball_field);

            std::vector<std::vector<cv::Point>> ball_contours;

            cv::findContours(ball_detected, ball_contours, cv::RETR_EXTERNAL,
                             cv::CHAIN_APPROX_SIMPLE);

            double largest = 0;
            std::vector<cv::Point> ballContour;
            for (auto &contour : ball_contours)
            {
                double area = cv::contourArea(contour);

                if (area > largest)
                {
                    largest = area;
                    ballContour = contour;
                }
            }
            // IC(ball_contours.size());
            if (ball_contours.size() && largest > 4)
            {
                cv::Moments moments = cv::moments(ballContour);

                cv::Point centroid(moments.m10 / moments.m00,
                                   moments.m01 / moments.m00);

                int pos = centroid.y * cvImage.cols + centroid.x;
                double thetaBall;
                sl::float4 point;
                // int tmpp;
                point_cloud.getValue(centroid.x, centroid.y, &point);
                // IC(point.x);
                if (std::isfinite(point.z))
                {
                    // ballPos.x = sqrt((point.z * point.z) + (point.y * point.y)) + 13;
                    // ballPos.x = point.z;
                    // ballPos.y = point.x - 6;

                    // ballPos.x = sqrt((point.z * point.z) + (point.y * point.y)) + 20;
                    // ballPos.y = point.x - 6;
                    // ballPos.x = point.z + 13;
                    // ballPos.y = point.x - 2;
                    pointBlob.x = point.z;
                    pointBlob.y = point.x;

                    thetaBall = atan2(-pointBlob.y, pointBlob.x) * 180.0 / M_PI;
                    // thetaBall += 1;
                    if (thetaBall > 360)
                        thetaBall -= 360;
                    if (thetaBall < 0)
                        thetaBall += 360;

                        pointBlob.z = thetaBall;

                    // ballPos.z = thetaBall;
                    // ballPos.isDetected = true;
                }
                else
                {
                    cv::Rect boundingRect = cv::boundingRect(ballContour);

                    bbox_t bb;

                    bb.x = boundingRect.x;
                    bb.y = boundingRect.y;
                    bb.w = boundingRect.width;
                    bb.h = boundingRect.height;

                    get_3d_coordinates(bb, point_cloud, point);

                    if (std::isfinite(point.z))
                    {
                        // ballPos.x = point.z;
                        // ballPos.x = sqrt((point.z * point.z) + (point.y * point.y)) + 13;
                        // ballPos.y = point.x - 6;

                        // ballPos.x = sqrt((point.z * point.z) + (point.y * point.y)) + 20;
                        // ballPos.y = point.x - 6;
                        // ballPos.x = point.z + 13;
                        // ballPos.y = point.x - 2;

                        // ballPos.z = -point.y + 47;
                        // thetaBall = atan2(-ballPos.y, ballPos.x) * 180.0 / M_PI;
                        // // thetaBall += 1;

                        pointBlob.x = point.z;
                        pointBlob.y = point.x;

                        thetaBall = atan2(-pointBlob.y, pointBlob.x) * 180.0 / M_PI;
                        // thetaBall += 1;
                        if (thetaBall > 360)
                            thetaBall -= 360;
                        if (thetaBall < 0)
                            thetaBall += 360;

                        pointBlob.z = thetaBall;

                        // if (thetaBall > 360)
                        //     thetaBall -= 360;
                        // if (thetaBall < 0)
                        //     thetaBall += 360;
                        // ballPos.z = thetaBall;
                        // ballPos.isDetected = true;
                    }
                }

                // if (ballPos.isDetected)
                // {
                //     cout << ballPos.x << " | " << ballPos.y << " | " << ballPos.z << endl;
                // }

                cv::circle(cvImage, centroid, 3, cv::Scalar(0, 255, 0), -1);
                cv::circle(cvImage, centroid, 5, cv::Scalar(0, 0, 255), 2);
                // IC("sini");
                cv::line(cvImage, cv::Point(cvImage.rows, cvImage.cols), cv::Point(centroid.x, centroid.y), cv::Scalar(0, 0, 255), 2);
            }
            else{
                pointBlob.x= 0;
                pointBlob.y= 0;
                pointBlob.z= 0;

            }
            IC(pointBlob.x , pointBlob.y , pointBlob.z);
            pubBlob.publish(pointBlob);

            cv::imshow("Hasil", cvImage);

            int key = cv::waitKey(1);
            // IC(key);
            if(key == 32)
                break;

            if (key == 's')
            {
                Conf.saveDataToFile();
            }
            // usleep(1000);
        }

        ros::spinOnce();
    }
    zed.close();
    return 0;
}
