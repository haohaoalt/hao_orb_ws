#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include <memory>
#include <string>
#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include<opencv2/core/core.hpp>

#include "System.h"
#include "ROSPublisher.h"
#include "utils.h"

//for increasing stack size
//in order to handle large maps
#include <sys/resource.h>
using namespace std;

struct ImageGrabber
{
    ORB_SLAM2::System* mpSLAM;

    void GrabImage(const sensor_msgs::ImageConstPtr& msg)
    {
        // Copy the ros image message to cv::Mat.
        cv_bridge::CvImageConstPtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvShare(msg);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());
    }
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

    ros::init(argc, argv, "Mono");
    ros::start();

    if(argc <= 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 Mono path_to_vocabulary path_to_settings sequence" << endl;
        ros::shutdown();
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    const double freq = 100.0;
    // Set up a namespace for topics
    ros::NodeHandle nh("/orb_slam2");
    ORB_SLAM2::System SLAM(
	  make_unique<ROSSystemBuilder>(
	    argv[1], argv[2], ORB_SLAM2::System::MONOCULAR, freq, nh));

    ImageGrabber igb {&SLAM};
    ros::NodeHandle nodeHandler;
    //NOTE: MONO topic is /camera/image_raw
    ros::Subscriber sub = nodeHandler.subscribe("/camera/image_raw", 1, &ImageGrabber::GrabImage, &igb);

    SLAM.Start();
    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    bool bFileName = (((argc - 3) % 2) == 1);

    string file_name;
    if (bFileName)
    {
        file_name = string(argv[argc - 1]);
        // Save camera trajectory
        const string kf_file = "kf_" + string(argv[argc - 1]) + ".txt";
        SLAM.SaveKeyFrameTrajectoryTUM(kf_file);
    }
    else
    {
        SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    }

    ros::shutdown();

    return 0;
}
