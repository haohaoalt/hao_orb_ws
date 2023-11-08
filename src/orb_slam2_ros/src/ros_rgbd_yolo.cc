/*
 * @Author: https://github.com/haohaoalt
 * @Date: 2023-11-08 16:07:29
 * @LastEditors: haohaoalt haohaoalt@163.com
 * @LastEditTime: 2023-11-08 16:21:20
 * @FilePath: /hao_orb_ws/src/orb_slam2_ros/src/ros_rgbd_yolo.cc
 * @Description: 
 * Copyright (c) 2023 by haohaoalt@163.com, All Rights Reserved. 
 */
/**
 * This file is part of ORB-SLAM2.
 *
 * Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
 * For more information see <https://github.com/raulmur/ORB_SLAM2>
 *
 * ORB-SLAM2 is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ORB-SLAM2 is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
 */

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <opencv2/core/core.hpp>

#include "System.h"
#include "ROSPublisher.h"
#include "utils.h"

// for increasing stack size
// in order to handle large maps
#include <sys/resource.h>

// used to measure running time of SLAM system
#include <time.h>

// hayden publish orbslam2 pose for rgbd
#include <tf/transform_broadcaster.h>
#include "Converter.h"
#include <nav_msgs/Path.h>

// hayden darknet ros darknet_ros_msgs/BoundingBoxes sub
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/BoundingBoxes.h>

using namespace std;
vector<std::pair<vector<double>, int>> detect_result;
ros::Publisher pose_pub;
nav_msgs::Path rgbd_path;
ros::Publisher rgbd_path_pub;
void personTrack(const darknet_ros_msgs::BoundingBoxes::ConstPtr &msg)
{
    double xmin = msg->bounding_boxes[0].xmin;
    double ymin = msg->bounding_boxes[0].ymin;
    double xmax = msg->bounding_boxes[0].xmax;
    double ymax = msg->bounding_boxes[0].ymax;
    int classid = msg->bounding_boxes[0].id;
    string classtype = msg->bounding_boxes[0].Class;
    float probability = msg->bounding_boxes[0].probability;
    detect_result.clear();
    vector<double> result_parameter;
    result_parameter.push_back(xmin);
    result_parameter.push_back(ymin);
    result_parameter.push_back(xmax);
    result_parameter.push_back(ymax);

    int dynamic_id = -1; // 存入识别物体的种类
    string class_label = classtype;

    if (class_label == "tv" || // 低动态物体(在程序中可以假设为一直静态的物体):tv,refrigerator
        class_label == "refrigerator" ||
        class_label == "teddy bear" ||
        class_label == "laptop")
    {
        dynamic_id = 1;
    }

    if (class_label == "chair" || // 中动态物体,在程序中不做先验动态静态判断
        class_label == "car")
    {
        dynamic_id = 2;
    }

    if (class_label == "person")
    { // 高动态物体:人,动物等
        dynamic_id = 3;
    }

    detect_result.emplace_back(result_parameter, dynamic_id);

    // std::cout << "I'm in PersonTracking callback" << std::endl;
    // std::cout << "xmin: " << xmin << std::endl;
    // std::cout << "ymin: " << xmax << std::endl;
    // std::cout << "ymax: " << ymax << std::endl;
    // std::cout << "classid:" << classid << std::endl;
    // std::cout << "classtype: " << classtype << std::endl;
    // std::cout << "probability: " << probability << std::endl;
}
class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System *pSLAM) : mpSLAM(pSLAM) {}

    void GrabRGBD(const sensor_msgs::ImageConstPtr &msgRGB, const sensor_msgs::ImageConstPtr &msgD);

    ORB_SLAM2::System *mpSLAM;
};

int main(int argc, char **argv)
{
    // ================== Set stack size =============//
    // Fix map saving/loading problem when large Maps are processed
    const rlim_t kStackSize = 16L * 2014L * 1024L;
    struct rlimit rl;
    int result;

    result = getrlimit(RLIMIT_STACK, &rl);
    if (result == 0)
    {
        if (rl.rlim_cur < kStackSize)
        {
            rl.rlim_cur = kStackSize;
            result = setrlimit(RLIMIT_STACK, &rl);
            if (result != 0)
            {
                fprintf(stderr, "setrlimit returned result = %d\n", result);
            }
        }
    }
    // hayden: init rgbd
    ros::init(argc, argv, "RGBD");
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
    {
        ros::console::notifyLoggerLevelsChanged();
    }

    ros::start();
    if (argc != 3)
    {
        cerr << endl
             << "Usage: rosrun ORB_SLAM2 RGBD path_to_vocabulary path_to_settings" << endl;
        ros::shutdown();
        return 1;
    }
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    const double freq = 100.0;
    // Set up a namespace for topics
    ros::NodeHandle nh("/orb_slam2");
    ros::Subscriber sub = nh.subscribe("/darknet_ros/bounding_boxes", 10, personTrack);
    ORB_SLAM2::System SLAM(make_unique<ROSSystemBuilder>(argv[1], argv[2], ORB_SLAM2::System::RGBD, freq, nh));
    ImageGrabber igb(&SLAM);
 
    // hayden orbslam2 rgbd subscribe topic
    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/color/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/camera/aligned_depth_to_color/image_raw", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub, depth_sub);

    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD, &igb, _1, _2));

    // haoz add pose pub and stereo_path_pub
    pose_pub = nh.advertise<geometry_msgs::PoseStamped>("ORB_SLAM/pose", 5);
    rgbd_path_pub = nh.advertise<nav_msgs::Path>("/orbslam2/camera_path", 10);

    clock_t tStart = clock();
    SLAM.Start();

    // processes messages from the callback queue
    ros::spin();

    // Stop all threads
    SLAM.Shutdown();
    printf("Time taken between SLAM.Start() and SLAM.Shutdown(): %.2fs\n", (double)(clock() - tStart) / CLOCKS_PER_SEC);

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    SLAM.SaveTrajectoryTUM("FrameTrajectory_TUM_Format.txt");
    SLAM.SaveTrajectoryKITTI("FrameTrajectory_KITTI_Format.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr &msgRGB, const sensor_msgs::ImageConstPtr &msgD)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv::Mat Tcw;
    // Step rgbd 1 : TrackRGBD
    Tcw = mpSLAM->TrackRGBD(cv_ptrRGB->image, cv_ptrD->image, cv_ptrRGB->header.stamp.toSec(), detect_result);

    detect_result.clear();

    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "odom2";

    cv::Mat Rwc = Tcw.rowRange(0, 3).colRange(0, 3).t(); // Rotation information
    cv::Mat twc = -Rwc * Tcw.rowRange(0, 3).col(3);      // translation information
    vector<float> q = ORB_SLAM2::Converter::toQuaternion(Rwc);

    tf::Transform new_transform;
    new_transform.setOrigin(tf::Vector3(twc.at<float>(0, 0), twc.at<float>(0, 1), twc.at<float>(0, 2)));
    tf::Quaternion quaternion(q[0], q[1], q[2], q[3]);
    new_transform.setRotation(quaternion);
    tf::poseTFToMsg(new_transform, pose.pose);
    pose_pub.publish(pose);

    rgbd_path.header.frame_id = "odom2";
    rgbd_path.header.stamp = ros::Time::now();
    rgbd_path.poses.push_back(pose);
    rgbd_path_pub.publish(rgbd_path);
}
