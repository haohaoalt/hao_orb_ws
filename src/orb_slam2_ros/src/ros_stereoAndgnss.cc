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
#include <ros/console.h>
// for increasing stack size
// in order to handle large maps
#include <sys/resource.h>

#include "colorutils.h"
#include <termios.h>
#include <unistd.h>
// used to measure running time of SLAM system
#include <time.h>

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

using namespace std;

// 全局变量
//保存或者加载地图名
string mutimapfile="map.bin";
//订阅gnss数据得到地图名
string mapstateCallbackmapfile="callbackmap";
//订阅gnss数据标志位
bool received_data = false;
//订阅gnss数据得到地图序列
vector<string> mapstateCallbackdata;
// 地球半径
double EARTH_RADIUS = 6378.137; 
// ros的状态发布者
ros::Publisher state_pub_;
// ros的轨迹发布者
nav_msgs::Path ros_path_;
// 初始化标志位
bool init;
// 初始化位置
struct my_pose
{
    double latitude;
    double longitude;
    double altitude;
};
my_pose init_pose;
// 声明一个全局的 mutimap_publisher
ros::Publisher mapstatepub;
//  创建并填充 mutimap 消息对象
orb_slam2_ros::mutimap mutimap_msg;
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
    // 填充 mutimap_msg 的字段
    // 假设您有一组要添加到消息中的字符串
    // Specify the directory path
    std::string directory = "/home/ylsdb20/dev/hao_orb_ws/maps";

    std::string map_name = generateMapName(gps_msg_ptr->latitude, gps_msg_ptr->longitude);
    // ROSINFO generateMapName
    //  ROS_INFO("Generated map name: %s", map_name.c_str());
    mutimap_msg.mapfile = map_name;

    // Get all filenames in the directory
    std::vector<std::string>
        filenames = getFilenames(directory);

    for (const auto &filename : filenames)
    {
        mutimap_msg.data.push_back(LoadMapName(filename));
    }
    // 发布 mutimap 消息
    mapstatepub.publish(mutimap_msg);

    // STEP2 处理 gps 数据
    //  初始化
    // ROSINFO 原始 gps 数据
    // ROS_INFO("\033[1;32m( ORIGN gps x:%0.6f ,y:%0.6f ,z:%0.6f)\033[0m", gps_msg_ptr->latitude, gps_msg_ptr->longitude, gps_msg_ptr->altitude);
    if (!init)
    {
        init_pose.latitude = gps_msg_ptr->latitude;
        init_pose.longitude = gps_msg_ptr->longitude;
        init_pose.altitude = gps_msg_ptr->altitude;
        init = true;
    }
    else
    {
        double mutilatitude = gps_msg_ptr->latitude;
        double mutilongitude = gps_msg_ptr->longitude;
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
        // TODO global frame id
        ros_path_.header.frame_id = "orbslamex/frame";
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

    // STEP4 清空 mutimap_msg
    //  清空 mutimap_msg
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

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System *pSLAM) : mpSLAM(pSLAM) {}

    void GrabStereo(const sensor_msgs::ImageConstPtr &msgLeft, const sensor_msgs::ImageConstPtr &msgRight);

    ORB_SLAM2::System *mpSLAM;
    bool do_rectify;
    cv::Mat M1l, M2l, M1r, M2r;
};

void ImageGrabber::GrabStereo(const sensor_msgs::ImageConstPtr &msgLeft, const sensor_msgs::ImageConstPtr &msgRight)
{
    // Copy the ros image message to cv::Mat. - CVBridge provides the required interface between ROS and OpenCV
    // Const pointers to avoid modifying the original data -> toCvShare()
    cv_bridge::CvImageConstPtr cv_ptrLeft;
    try
    {
        cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrRight;
    try
    {
        cv_ptrRight = cv_bridge::toCvShare(msgRight);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // rectify images, not needed with images from zed
    if (do_rectify)
    {
        cv::Mat imLeft, imRight;
        cv::remap(cv_ptrLeft->image, imLeft, M1l, M2l, cv::INTER_LINEAR);
        cv::remap(cv_ptrRight->image, imRight, M1r, M2r, cv::INTER_LINEAR);
        // starts the tracking thread
        mpSLAM->TrackStereo(imLeft, imRight, cv_ptrLeft->header.stamp.toSec());
    }
    else
    {
        mpSLAM->TrackStereo(cv_ptrLeft->image, cv_ptrRight->image, cv_ptrLeft->header.stamp.toSec());
    }
}

// 订阅mapstate 获得此时的地图名字 以及  已有的地图序列
void mapstateCallback(const orb_slam2_ros::mutimap::ConstPtr &msg)
{
    mapstateCallbackmapfile = msg->mapfile;
    mapstateCallbackdata = msg->data;
    received_data = true;
}


int main(int argc, char **argv)
{

    //  ================== Set stack size =============//
    // Fix map saving/loading problem when large Maps are processed
    // 该值的含义是设置堆栈大小为 16 * 2014 * 1024 字节，即约为 32 MB。
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
    // STEP STEREO 1 初始化 ROS
    ros::init(argc, argv, "StereoAndGnss");
    ros::start();
    if (argc <= 4)
    {
        cerr << endl
             << "Usage: rosrun ORB_SLAM2 Stereo path_to_vocabulary path_to_settings do_rectify sequence" << endl;
        ros::shutdown();
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    const double freq = 100.0;
    // Set up a namespace for topics
    // STEP STEREO 2 创建 ORB_SLAM2::System
    //"/orbslamex" 是节点的命名空间。通过指定命名空间，你可以将节点与其他节点进行区分，并防止话题、服务等名称的冲突。
    ros::NodeHandle nh("/orbslamex");
    ros::Subscriber mapstatesub = nh.subscribe("/mapstate", 10, mapstateCallback);
    cout << colouredString("Waiting for mapstate data...", BLUE, BOLD) << endl;

    while (!received_data)
    {
        ros::spinOnce(); // 处理一次 ROS 消息队列
    }
    cout << colouredString("mapstateCallbackmapfile: ", BLUE, REGULAR) << mapstateCallbackmapfile << endl;
    for (const auto &str : mapstateCallbackdata)
    {
        cout << colouredString("mapstateCallbackdata: ", BLUE, REGULAR) << str << endl;
    }
    //NOTE 初始化要保存的地图名
    mutimapfile = mapstateCallbackmapfile + ".bin";
    // 创建了一个 ORB_SLAM2 系统对象 SLAM，使用了自定义的 ROS 系统构建器 ROSSystemBuilder。
    // 构建器接受了一些参数，包括 VOC 文件、设置文件、传感器类型等，并且传递了 ROS 节点句柄 nh。这意味着 ORB_SLAM2 系统将在名为 /orbslamex 的 ROS 节点下运行
    ORB_SLAM2::System SLAM(make_unique<ROSSystemBuilder>(argv[1], argv[2], ORB_SLAM2::System::STEREO, freq, nh, mutimapfile));
    // 创建了一个 ImageGrabber 对象 igb，并将之前创建的 ORB_SLAM2 系统对象的指针传递给它，以便它可以使用该系统来抓取图像。
    ImageGrabber igb(&SLAM);

    // Stream class to operate on strings.
    stringstream ss(argv[3]);
    // When the boolalpha format flag is set, bool values are inserted/extracted by their textual representation: either true or false, instead of integral values.
    ss >> boolalpha >> igb.do_rectify;
    if (igb.do_rectify)
    {
        cout << colouredString("igb.do_rectify: ", BLUE, REGULAR) << igb.do_rectify << endl;
        // Load settings related to stereo calibration
        cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);
        if (!fsSettings.isOpened())
        {
            cerr << "ERROR: Wrong path to settings" << endl;
            return -1;
        }

        cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
        fsSettings["LEFT.K"] >> K_l;
        fsSettings["RIGHT.K"] >> K_r;

        fsSettings["LEFT.P"] >> P_l;
        fsSettings["RIGHT.P"] >> P_r;

        fsSettings["LEFT.R"] >> R_l;
        fsSettings["RIGHT.R"] >> R_r;

        fsSettings["LEFT.D"] >> D_l;
        fsSettings["RIGHT.D"] >> D_r;

        int rows_l = fsSettings["LEFT.height"];
        int cols_l = fsSettings["LEFT.width"];
        int rows_r = fsSettings["RIGHT.height"];
        int cols_r = fsSettings["RIGHT.width"];

        if (K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
            rows_l == 0 || rows_r == 0 || cols_l == 0 || cols_r == 0)
        {
            cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
            return -1;
        }

        // Computes the undistortion and rectification transformation map
        // contains cameraMatrix, distCoeffs , R , newCameraMatrix ...
        cv::initUndistortRectifyMap(K_l, D_l, R_l, P_l.rowRange(0, 3).colRange(0, 3), cv::Size(cols_l, rows_l), CV_32F, igb.M1l, igb.M2l);
        cv::initUndistortRectifyMap(K_r, D_r, R_r, P_r.rowRange(0, 3).colRange(0, 3), cv::Size(cols_r, rows_r), CV_32F, igb.M1r, igb.M2r);
    }

    // nh is startup and shutdown of the internal node inside a roscpp program

    message_filters::Subscriber<sensor_msgs::Image> left_sub(nh, "/zed2i/zed_node/left/image_rect_gray", 1);
    message_filters::Subscriber<sensor_msgs::Image> right_sub(nh, "/zed2i/zed_node/right/image_rect_gray", 1);

    // a policy used by message_filters::sync::Synchronizer to match messages coming on a set of topics
    // can match messages even if they have different time stamps
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;

    // filter synchronizes incoming channels by the timestamps contained in their headers
    // and outputs them in the form of a single callback that takes the same number of channels
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), left_sub, right_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabStereo, &igb, _1, _2));

    clock_t tStart = clock();
    SLAM.Start();
    // processes messages from the callback queue
    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    cout << colouredString("Time taken between SLAM.Start() and SLAM.Shutdown(): ", BLUE, REGULAR) << (double)(clock() - tStart) / CLOCKS_PER_SEC << "s" << endl;

    // visualize the trajectories with evo-traj package
    bool bFileName = (((argc - 4) % 2) == 1);

    string file_name;
    if (bFileName)
    {
        file_name = string(argv[argc - 1]);
        // Save camera trajectory
        const string kf_file = "kf_" + string(argv[argc - 1]) + ".txt";
        const string f_file = "f_" + string(argv[argc - 1]) + ".txt";
        SLAM.SaveTrajectoryTUM(f_file);
        // SLAM.SaveKeyFrameTrajectoryTUM(kf_file);
    }
    else
    {
        SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
        SLAM.SaveTrajectoryTUM("FrameTrajectory_TUM_Format.txt");
    }
    // NOTE ros::spin() 是一个阻塞式函数，它会使节点进入循环等待ROS消息的到来
    // NOTE 在调用 ros::spin() 之前的输出语句和循环体中的代码会在 mapstateCallback 被执行之前立即执行
    ros::spin();
    ros::shutdown();

    return 0;
}