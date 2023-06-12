/*
 * @Author: zhanghao
 * @Date: 2023-06-10 23:29:28
 * @LastEditTime: 2023-06-10 23:29:29
 * @FilePath: /hao_orb_ws/src/gnss_with_imu/src/gnss_with_imu.cpp
 * @Description: 
 */
#include "ros/ros.h"
#include "bits/stdc++.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"
#include "nav_msgs/Odometry.h"
#include "GeographicLib/LocalCartesian.hpp"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

using namespace std;
// 调库将经纬度转成东北天系下三维坐标
GeographicLib::LocalCartesian geo;
// 设置初始位置经纬度,可通过代码获得
double latitude = 48.982545235866;
double longitude = 8.390366100045;
double altitude = 116.382141113280;
// 声明三维坐标变量
double x, y, z;
// 声明发布者
ros::Publisher pubOdom;

void callback(const sensor_msgs::NavSatFix::ConstPtr &gpsMsg, const sensor_msgs::Imu::ConstPtr &imuMsg)
{
    static bool gpsInited = true;
    // 只设置一次初始经纬高
    if (gpsInited)
    {
        geo.Reset(latitude, longitude, altitude);
        cout.precision(12);
        cout << fixed << gpsMsg->latitude << endl
             << gpsMsg->longitude << endl
             << gpsMsg->altitude << endl;
        gpsInited = false;
    }
    // 经纬高转化为东北天坐标系下坐标
    geo.Forward(gpsMsg->latitude, gpsMsg->longitude, gpsMsg->altitude, x, y, z);
    nav_msgs::Odometry msgOdom;
    msgOdom.header.frame_id = "map";
    msgOdom.header.stamp = gpsMsg->header.stamp;
    msgOdom.pose.pose.position.x = x;
    msgOdom.pose.pose.position.y = y;
    msgOdom.pose.pose.position.z = z;
    // 四元数由imu提供，认为该imu获得的四元数没有累积误差
    msgOdom.pose.pose.orientation = imuMsg->orientation;
    pubOdom.publish(msgOdom);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "odom_fusion");
    ros::NodeHandle n;
    pubOdom = n.advertise<nav_msgs::Odometry>("gnss_with_imu_odom", 2000);
    // 使用message_filters对gps和imu话题时间同步
    message_filters::Subscriber<sensor_msgs::NavSatFix> subGps(n, "/gps/fix", 1000);
    message_filters::Subscriber<sensor_msgs::Imu> subImu(n, "/imu", 1000);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::NavSatFix, sensor_msgs::Imu> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), subGps, subImu);
    sync.registerCallback(boost::bind(&callback, _1, _2));
    ros::spin();
}
