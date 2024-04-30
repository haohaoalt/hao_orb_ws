#ifndef ORB_SLAM2_ROSPUBLISHER_H
#define ORB_SLAM2_ROSPUBLISHER_H

#include "IPublisherThread.h"
#include "IFrameSubscriber.h"
#include "IMapPublisher.h"
#include "FrameDrawer.h"
#include "System.h"
#include "Map.h"

#include <chrono>
#include <mutex>

#include <ros/ros.h>
#include <ros/time.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <nav_msgs/Odometry.h>

#include <nav_msgs/OccupancyGrid.h>

#include <octomap/OcTree.h>

#include <orb_slam2_ros/ORBState.h>

//将这些类放入命名空间 ORB_SLAM2 中，可以防止与其他库或代码中的同名类发生冲突，并提供了更清晰的代码结构
namespace ORB_SLAM2
{
    class Map;
    class Tracking;
}

class ROSPublisher :
    // Mehrfache Vererbung
    public ORB_SLAM2::IPublisherThread,
    public ORB_SLAM2::IMapPublisher,
    public ORB_SLAM2::IFrameSubscriber
{
public:
    // STEP 成员变量
    static constexpr const char *DEFAULT_MAP_FRAME = "map";
    static constexpr const char *DEFAULT_MAP_FRAME_ADJUSTED = "map_adjusted";
    //TODO： camera frame set 
    static constexpr const char *DEFAULT_CAMERA_FRAME = "orbslamex/frame"; // this is the trajectory guess of the ORB-SLAM2, visualize it in RViz to see trajectory of camera
    static constexpr const char *DEFAULT_BASE_FRAME = "baselink";
    // static constexpr const char *DEFAULT_BASE_FRAME = "rear_axis_middle_ground";
    static constexpr const float DEFAULT_OCTOMAP_RESOLUTION = 0.02; // defines resolution of octomap
    static constexpr const float ORBSTATE_REPUBLISH_RATE = 20;      // re-publish state @ 20 Hz
    static constexpr const float PROJECTION_MIN_HEIGHT = -0.1;      //-10; // minimal voxel-z to consider in projected map
    static constexpr const float GRADIENT_MAX_HEIGHT = 0.;          // maximal voxel-z to consider in gradient-based projection
    static constexpr const int GRADIENT_NB_EROSIONS = 1;            // number of erosions performed before computing gradients
    static constexpr const float GRADIENT_LOW_SLOPE = M_PI / 4.;    // lower bound for a slope being considered obstacle-ish
    static constexpr const float GRADIENT_HIGH_SLOPE = M_PI / 3.;   // lower bound for a slope being considered a full solid obstacle
    static constexpr const float OCTOMAP_RATE = 1;                  // rate of octomap cycles (integrate MapPoints and publish)
    static constexpr const bool OCTOMAP_REBUILD = false;            // clear octomap when tracking is lost and rebuild

    // `frequency` is max amount of messages emitted per second
    // STEP 构造函数
    // explicit 关键字用于声明一个单参数的构造函数，以防止隐式类型转换发生
    // 在这个构造函数中，它表明这个构造函数不会被用于隐式类型转换，而只能用于显式地创建对象
    // 如果没有 explicit 关键字，那么可以通过提供一个 double 类型的参数来隐式地创建一个 ROSPublisher 对象
    // ROSPublisher publisher = 1.0; // 隐式调用构造函数，将 double 类型参数转换为 ROSPublisher 类型
    // 由于构造函数被声明为 explicit，上面的代码将会导致编译错误，需要显式地调用构造函数来创建对象
    // ROSPublisher publisher(1.0); // 显式调用构造函数
    explicit ROSPublisher(
        ORB_SLAM2::Map *map,
        double frequency,
        bool bReuseMap_ = false,
        ros::NodeHandle nh = ros::NodeHandle());

    // Hier wird Run() Methode überschrieben (davor Run()=0)
    // NOTE 类的多态性，允许派生类覆盖（或重写）基类中的相应函数，并根据派生类的实际需求来实现不同的行为。
    // 这个函数被声明为虚函数，并且被标记为 override，表示它是一个重写了基类中同名函数的函数
    // 这个函数的作用是启动类的运行。派生类可以根据具体情况重写这个函数，以实现不同的运行逻辑
    // 在基类中，这个函数可能是一个纯虚函数，即函数声明为 virtual void Run() = 0;，这意味着基类中没有提供具体的实现，派生类必须重写这个函数才能实例化。
    virtual void Run() override;
    // NOTE 重写基类的函数这个函数也是一个虚函数，用于更新跟踪信息
    // 它接受一个 ORB_SLAM2::Tracking* 类型的指针作为参数，表示跟踪过程的信息
    // 派生类可以根据具体的需要重写这个函数，以实现不同的更新逻辑
    // 同样地，这个函数也可能在基类中是一个纯虚函数，需要派生类来提供具体的实现
    virtual void Update(ORB_SLAM2::Tracking *);

protected:
    bool WaitCycleStart();

private:
    ORB_SLAM2::FrameDrawer drawer_;
    bool first_write_; // 标记是否第一次写入
    std::string tum_file_path_; // TUM odom数据集文件路径
    // 在类中声明 ofstream 对象
    std::ofstream tum_file;

    // Important: `nh_` goes before the `*_pub_`, because their construction relies on `nh_`!
    ros::NodeHandle nh_;
    std::string map_frame_name_, camera_frame_name_;
    ros::Publisher map_pub_, map_updates_pub_, image_pub_, odom_pub_, state_pub_, state_desc_pub_, octomap_pub_, projected_map_pub_, gradient_map_pub_;
    tf::TransformBroadcaster camera_tf_pub_;
    ros::Rate pub_rate_;

    ros::Time last_state_publish_time_;
    orb_slam2_ros::ORBState orb_state_;

    int lastBigMapChange_;
    bool octomap_tf_based_;
    octomap::OcTree octomap_;
    tf::Vector3 camera_position_;
    tf::Quaternion orientation;

    tf::Vector3 camera_position_old;
    ros::Time time_old;

    bool octomap_enabled_;
    octomap::Pointcloud pointcloud_map_points_;
    std::mutex pointcloud_map_points_mutex_;
    int pointcloud_chunks_stashed_;
    bool clear_octomap_;
    std::thread octomap_worker_thread_;

    // params for z-plane-based occupancy grid approach
    double projection_min_height_;
    // params for gradient-based approach
    float gradient_max_height_;
    int gradient_nb_erosions_;
    float gradient_low_slope_;
    float gradient_high_slope_;

    tf::TransformListener tf_listener_;

    // NOTE  用于缓存地图点
    void stashMapPoints(bool all_map_points = false);
    // 是一个线程函数，用于处理OctoMap
    void octomapWorker();
    // 其他私有成员函数用于实现地图转换、发布不同类型的地图、发布相机姿态、发布状态等功能
    void updateOctoMap();
    void integrateMapPoints(const std::vector<ORB_SLAM2::MapPoint *> &, const octomap::point3d &, const octomap::pose6d &, octomap::OcTree &);

    void octomapCutToOccupancyGrid(const octomap::OcTree &octree, nav_msgs::OccupancyGrid &map)
    {
        octomapCutToOccupancyGrid(octree, map, -1.0 * std::numeric_limits<double>::max(), std::numeric_limits<double>::max());
    }
    void octomapCutToOccupancyGrid(const octomap::OcTree &octree, nav_msgs::OccupancyGrid &map, const double minZ_, const double maxZ_);

    void octomapGradientToOccupancyGrid(const octomap::OcTree &octree, nav_msgs::OccupancyGrid &map, float max_height = GRADIENT_MAX_HEIGHT, int nb_erosions = GRADIENT_NB_EROSIONS, float low_slope = GRADIENT_LOW_SLOPE, float high_slope = GRADIENT_HIGH_SLOPE);

    void publishMap();
    void publishMapUpdates();
    void publishCameraPose();
    void publishOctomap();
    void publishState(ORB_SLAM2::Tracking *tracking);
    void publishImage(ORB_SLAM2::Tracking *tracking);
    void publishProjectedMap();
    void publishGradientMap();

    // Create tf2odom publisher
    tf::Transform m2c_;
    ros::Publisher odom_publisher_;
    ros::Time stamp;
    void publishOdom(tf::Quaternion orientation);
    int queue_size_;
    tf::TransformListener transformer;
    // /整个类的目的是在ROS环境中发布ORB-SLAM2相关信息，包括地图、相机姿态、状态等，并且通过多线程处理OctoMap，提高了系统的并发性能。
};

// ROSSystemBuilder erbt von GernericBuilder def. in System.cc
class ROSSystemBuilder : public ORB_SLAM2::System::GenericBuilder
{
public:
    // 接受多个参数，包括词汇文件、设置文件、传感器类型、频率等
    ROSSystemBuilder(const std::string &strVocFile,
                     const std::string &strSettingsFile,
                     ORB_SLAM2::System::eSensor sensor,
                     double frequency,
                     ros::NodeHandle nh = ros::NodeHandle(),
                     std::string mutimapfile = "map.bin",
                     std::string map_frame = ROSPublisher::DEFAULT_MAP_FRAME,
                     std::string camera_frame = ROSPublisher::DEFAULT_CAMERA_FRAME);
    // 析构函数 ~ROSSystemBuilder() 是虚析构函数，用于释放在对象销毁时可能分配的资源。
    //在这里，由于 mpPublisher 是一个指向 ROSPublisher 对象的独占指针，当 ROSSystemBuilder 对象被销毁时，会自动释放 mpPublisher 指向的内存。
    virtual ~ROSSystemBuilder();

    // 是一个虚函数，它覆盖了基类中的同名函数。这个函数返回一个 ORB_SLAM2::IPublisherThread 类型的指针，指向 ROSPublisher 对象。
    // 在这里，mpPublisher 指向的对象正是 ROSPublisher 类的一个实例，因此这个函数返回了 mpPublisher.get()，即指向 ROSPublisher 对象的指针。
    virtual ORB_SLAM2::IPublisherThread *GetPublisher() override;

private:
    // 在构造函数中，会初始化 mpPublisher，它是一个指向 ROSPublisher 对象的独占指针。这个指针被设置为一个新创建的 ROSPublisher 对象。
    std::unique_ptr<ROSPublisher> mpPublisher;
};  

#endif // ORB_SLAM2_ROSPUBLISHER_H
