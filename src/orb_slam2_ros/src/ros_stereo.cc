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
//for increasing stack size
//in order to handle large maps
#include <sys/resource.h>

#include "colorutils.h"

//used to measure running time of SLAM system
#include <time.h>

using namespace std;

class ImageGrabber
{
public:

    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight);

    ORB_SLAM2::System* mpSLAM;
    bool do_rectify;
    cv::Mat M1l,M2l,M1r,M2r;
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
int main(int argc, char **argv)
{

    //  ================== Set stack size =============//
    // Fix map saving/loading problem when large Maps are processed
    //该值的含义是设置堆栈大小为 16 * 2014 * 1024 字节，即约为 32 MB。 
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
    //STEP STEREO 1 初始化 ROS
    ros::init(argc, argv, "Stereo");
    ros::start();
    if(argc <= 4)
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
    // 创建了一个 ORB_SLAM2 系统对象 SLAM，使用了自定义的 ROS 系统构建器 ROSSystemBuilder。
    // 构建器接受了一些参数，包括 VOC 文件、设置文件、传感器类型等，并且传递了 ROS 节点句柄 nh。这意味着 ORB_SLAM2 系统将在名为 /orbslamex 的 ROS 节点下运行
    ORB_SLAM2::System SLAM(make_unique<ROSSystemBuilder>(argv[1], argv[2], ORB_SLAM2::System::STEREO, freq, nh));
    // 创建了一个 ImageGrabber 对象 igb，并将之前创建的 ORB_SLAM2 系统对象的指针传递给它，以便它可以使用该系统来抓取图像。
    ImageGrabber igb(&SLAM);

    //Stream class to operate on strings.
    stringstream ss(argv[3]);
    //When the boolalpha format flag is set, bool values are inserted/extracted by their textual representation: either true or false, instead of integral values.
	ss >> boolalpha >> igb.do_rectify;
    if(igb.do_rectify)
    {
        cout << colouredString("igb.do_rectify: ", BLUE, REGULAR) << igb.do_rectify << endl;
        // Load settings related to stereo calibration
        cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);
        if(!fsSettings.isOpened())
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

        if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
                rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0)
        {
            cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
            return -1;
        }

        //Computes the undistortion and rectification transformation map
        //contains cameraMatrix, distCoeffs , R , newCameraMatrix ...
        cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,igb.M1l,igb.M2l);
        cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,igb.M1r,igb.M2r);
    }

    //nh is startup and shutdown of the internal node inside a roscpp program

    message_filters::Subscriber<sensor_msgs::Image> left_sub(nh, "/zed2i/zed_node/left/image_rect_gray", 1);
    message_filters::Subscriber<sensor_msgs::Image> right_sub(nh, "/zed2i/zed_node/right/image_rect_gray", 1);

    //a policy used by message_filters::sync::Synchronizer to match messages coming on a set of topics
    //can match messages even if they have different time stamps
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;

    //filter synchronizes incoming channels by the timestamps contained in their headers
    //and outputs them in the form of a single callback that takes the same number of channels
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), left_sub,right_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabStereo,&igb,_1,_2));


    clock_t tStart = clock();
    SLAM.Start();

    //processes messages from the callback queue
    ros::spin();

    // Stop all threads
    SLAM.Shutdown();
    printf("Time taken between SLAM.Start() and SLAM.Shutdown(): %.2fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);

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
        SLAM.SaveKeyFrameTrajectoryTUM(kf_file);
    }
    else
    {
        SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
        SLAM.SaveTrajectoryTUM("FrameTrajectory_TUM_Format.txt");
    }

    ros::shutdown();

    return 0;
}