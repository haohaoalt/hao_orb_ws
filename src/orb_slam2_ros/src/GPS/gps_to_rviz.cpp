#include <ros/ros.h>
#include "turtlesim/Pose.h"
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <math.h>
#include <iostream>
#include <vector>
#include <dirent.h>
#include <boost/filesystem.hpp>
#include "colorutils.h"
// 经纬度转换
#include <string>
#include <cmath>
#include <orb_slam2_ros/mutimap.h>

// 全局变量
static double EARTH_RADIUS = 6378.137; // 地球半径
ros::Publisher state_pub_;
nav_msgs::Path ros_path_;
bool init;
struct my_pose
{
    double latitude;
    double longitude;
    double altitude;
};
my_pose init_pose;
// 声明一个全局的 mutimap_publisher
ros::Publisher mapstatepub;

// 将度转换为度分秒
std::string degreesToDMS(double degrees)
{
    int deg = static_cast<int>(degrees);
    double minutes = (degrees - deg) * 60;
    int min = static_cast<int>(minutes);
    double seconds = (minutes - min) * 60;
    int sec = static_cast<int>(seconds);

    // 使用整数部分表示度，分钟和秒的整数部分，以及秒的小数部分
    return std::to_string(deg) + std::to_string(min) + std::to_string(sec);
}

// 生成地图名
std::string generateMapName(double latitude, double longitude)
{
    std::string lat_dms = degreesToDMS(latitude);
    std::string lon_dms = degreesToDMS(longitude);

    // 根据经纬度的度分秒值生成地图名
    return "N" + lat_dms + "E" + lon_dms;
}

// Function to generate map names from filenames
std::string LoadMapName(const std::string &filename)
{
    // Extract necessary parts from the filename and construct the map name
    // This part depends on your filename format
    return filename;
}

// 角度制转弧度制
double rad(double d)
{
    return d * 3.1415926 / 180.0;
}

// Function to get all filenames in a directory
std::vector<std::string> getFilenames(const std::string &directory)
{
    std::vector<std::string> filenames;
    DIR *dir;
    struct dirent *ent;
    if ((dir = opendir(directory.c_str())) != NULL)
    {
        while ((ent = readdir(dir)) != NULL)
        {
            // Exclude directories and hidden files
            if (ent->d_type == DT_REG && ent->d_name[0] != '.')
            {
                filenames.push_back(ent->d_name);
            }
        }
        closedir(dir);
    }
    else
    {
        std::cerr << "Error opening directory: " << directory << std::endl;
    }
    return filenames;
}

void gpsCallback(const sensor_msgs::NavSatFixConstPtr &gps_msg_ptr)
{
    // STEP1 发布 mutimap 消息
    //  创建并填充 mutimap 消息对象
    orb_slam2_ros::mutimap mutimap_msg;
    // 填充 mutimap_msg 的字段
    // 假设您有一组要添加到消息中的字符串
    // Specify the directory path
    std::string directory = "/home/ylsdb20/dev/hao_orb_ws/maps";
    // Get all filenames in the directory
    std::vector<std::string> filenames = getFilenames(directory);

    for (const auto &filename : filenames)
    {
        mutimap_msg.data.push_back(LoadMapName(filename));
    }
    // 发布 mutimap 消息
    mapstatepub.publish(mutimap_msg);

    // STEP2 处理 gps 数据
    //  初始化
    if (!init)
    {
        init_pose.latitude = gps_msg_ptr->latitude;
        init_pose.longitude = gps_msg_ptr->longitude;
        init_pose.altitude = gps_msg_ptr->altitude;
        init = true;
    }
    else
    {
        ROS_INFO("\033[1;32m( ORIGN gps x:%0.6f ,y:%0.6f ,z:%0.6f)\033[0m", gps_msg_ptr->latitude, gps_msg_ptr->longitude, gps_msg_ptr->altitude);
        double mutilatitude = gps_msg_ptr->latitude;
        double mutilongitude = gps_msg_ptr->longitude;
        std::string map_name = generateMapName(mutilatitude, mutilongitude);
        ROS_INFO("Generated map name: %s", map_name.c_str());
        // 计算相对位置
        double radLat1, radLat2, radLong1, radLong2, delta_lat, delta_long, x, y;
        radLat1 = rad(init_pose.latitude);
        radLong1 = rad(init_pose.longitude);
        radLat2 = rad(gps_msg_ptr->latitude);
        radLong2 = rad(gps_msg_ptr->longitude);

        // 计算x
        delta_long = 0;
        delta_lat = radLat2 - radLat1; //(radLat1,radLong1)-(radLat2,radLong1)
        if (delta_lat > 0)
            x = 2 * asin(sqrt(pow(sin(delta_lat / 2), 2) + cos(radLat1) * cos(radLat2) * pow(sin(delta_long / 2), 2)));
        else
            x = -2 * asin(sqrt(pow(sin(delta_lat / 2), 2) + cos(radLat1) * cos(radLat2) * pow(sin(delta_long / 2), 2)));

        x = x * EARTH_RADIUS * 1000;

        // 计算y
        delta_lat = 0;
        delta_long = radLong2 - radLong1; //(radLat1,radLong1)-(radLat1,radLong2)
        if (delta_long > 0)
            y = 2 * asin(sqrt(pow(sin(delta_lat / 2), 2) + cos(radLat2) * cos(radLat2) * pow(sin(delta_long / 2), 2)));
        else
            y = -2 * asin(sqrt(pow(sin(delta_lat / 2), 2) + cos(radLat2) * cos(radLat2) * pow(sin(delta_long / 2), 2)));
        // double y = 2*asin( sin( delta_lat/2 ) + cos( radLat2 )*cos( radLat2)* sin( delta_long/2 )   );
        y = y * EARTH_RADIUS * 1000;

        // 计算z
        double z = gps_msg_ptr->altitude - init_pose.altitude;

        // 发布轨迹
        ros_path_.header.frame_id = "path";
        ros_path_.header.stamp = ros::Time::now();

        geometry_msgs::PoseStamped pose;
        pose.header = ros_path_.header;

        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = z;

        ros_path_.poses.push_back(pose);

        // ROS_INFO("\033[1;32m( gps path x:%0.6f ,y:%0.6f ,z:%0.6f)\033[0m", x, y, z);
        // STEP3 发布轨迹
        state_pub_.publish(ros_path_);
    }

    //STEP4 清空 mutimap_msg
    // 清空 mutimap_msg
    mutimap_msg.data.clear();
}

// TODO save TUM TRAJECTORY
void saveTrajectoryTUM(const nav_msgs::Path::ConstPtr &path, const std::string &filename)
{
    // Construc the full path to the map file
    boost::filesystem::path maps_directory = "/home/ylsdb20/dev/hao_orb_ws/results";
    // // 构造完整的文件路径 绝对路径
    boost::filesystem::path full_path = boost::filesystem::absolute(maps_directory) / filename;
    // std::cout << "full_path: " << full_path << std::endl;

    // 检查文件是否存在
    if (!boost::filesystem::exists(full_path))
    {
        // 文件不存在，创建文件
        std::ofstream create_file(full_path.string());
        create_file.close(); // 创建文件后立即关闭，确保文件被创建

        if (!create_file.is_open())
        {
            std::cerr << "Failed to create file: " << filename << std::endl;
            return;
        }
    }
    // 文件写入的准备工作
    boost::filesystem::ofstream f;
    // NOTE 可以理解为，在输出浮点数的时候使用0.3141592654这样的方式而不是使用科学计数法
    f.open(full_path.c_str());
    // 写入数据到文件
    for (const auto &pose : path->poses)
    {
        const auto &position = pose.pose.position;
        const auto &orientation = pose.pose.orientation;

        // Write timestamp, translation, and quaternion
        // 使用 ROS 时间
        ros::Time timestamp = ros::Time::now();
        f << timestamp.toNSec() << " "
          << position.x << " " << position.y << " " << position.z << " "
          << orientation.x << " " << orientation.y << " " << orientation.z << " " << orientation.w << std::endl;
    }
    f.close();
}

int main(int argc, char **argv)
{
    init = false;
    ros::init(argc, argv, "gps_to_rviz");
    ros::NodeHandle n;
    // 周期发布 msg 消息
    mapstatepub = n.advertise<orb_slam2_ros::mutimap>("mapstate", 10);
    ros::Subscriber pose_sub = n.subscribe("/gps/fix", 20, gpsCallback);

    state_pub_ = n.advertise<nav_msgs::Path>("gps_path", 20);
    ros::Subscriber sub = n.subscribe<nav_msgs::Path>("gps_path", 1, [&](const nav_msgs::Path::ConstPtr &msg)
                                                      { saveTrajectoryTUM(msg, "gpstrajectory.txt"); });
    std::cout << colouredString("GPS tum trajectory saved to gpstrajectory.txt", GREEN, BOLD) << std::endl;
    ros::spin();
    return 0;
}
