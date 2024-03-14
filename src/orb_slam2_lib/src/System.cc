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

#include <thread>
#include <iomanip>

#include "System.h"
#include "Converter.h"
#include "Tracking.h"
#include "LocalMapping.h"

#include "IFrameSubscriber.h"
#include "IFrameDrawer.h"
#include "IMapPublisher.h"
#include "Failure.h"

#include "utils.h"
#include "colorutils.h"
#include <unistd.h>


static bool has_suffix(const std::string &str, const std::string &suffix)
{
    std::size_t index = str.find(suffix, str.size() - suffix.size());
    return (index != std::string::npos);
}

namespace ORB_SLAM2
{

System::IBuilder::~IBuilder() { }

System::GenericBuilder::GenericBuilder(const std::string &strVocFile,
                                       const std::string &strSettingsFile,
                                       eSensor sensor) :

    mSensor(sensor),
    mSettings(strSettingsFile.c_str(), cv::FileStorage::READ)
{
    //check Settings file
    if (!mSettings.isOpened())
        throw Failure(std::string("Failed to open settings file at: ") + strSettingsFile);


    //Load ORB Vocabulary, based on file extension
    bool bVocLoad = false;

    // 获取字典加载状态
    if (has_suffix(strVocFile, ".txt"))
        bVocLoad = mVocabulary.loadFromTextFile(strVocFile);
    else if(has_suffix(strVocFile, ".bin"))
        bVocLoad = mVocabulary.loadFromBinaryFile(strVocFile);
    else
        bVocLoad = false;

    // 如果加载失败，就输出调试信息
    if(!bVocLoad)
        throw Failure(std::string("Failed to load vocabulary at: ") + strVocFile);
    else
        cout << "Vocabulary loaded!" << endl
             << endl; // 否则则说明加载成功

    cv::FileNode mapfile_node = mSettings["Map.mapfile"];
    bReuseMap = false;
    if (!mapfile_node.empty())
    {
        mapfile = (string)mapfile_node;
    }
    cout << "MapFile name: " << mapfile << endl;

    load_map = mSettings["Map.load_map"];
    save_map = mSettings["Map.save_map"];
    //Do not create new KeyframeDB and Map if
    //there is a saved Map; load the saved one instead
    if (load_map && !mapfile.empty() && LoadMap(mapfile))
    {
        bReuseMap = true;
    }
    else
    {
        // Create KeyFrame Database
        mpKeyFrameDatabase = make_unique<KeyFrameDatabase>(&mVocabulary);
        // Create the Map
        mpMap = make_unique<Map>();
    }

    // NOTE: 在本主进程中初始化追踪线程
    mpTracker = make_unique<Tracking>(&mVocabulary,             // 字典
                                      mpMap.get(),              // get mpMap
                                      mpKeyFrameDatabase.get(), // get mpKeyFrameDatabase
                                      strSettingsFile,          // 设置文件路径
                                      mSensor,                  // 传感器类型iomanip
                                      bReuseMap);
    //NOTE: 初始化局部建图线程并运行
    mpLocalMapper = make_unique<LocalMapping>(mpMap.get(),           // 指定使iomanip
                                              mSensor == MONOCULAR); // TODO 为什么这个要设置成为MONOCULAR？？？
    // NOTE: 初始化闭环检测线程并运行
    mpLoopCloser = make_unique<LoopClosing>(mpMap.get(),              // 地图
                                            mpKeyFrameDatabase.get(), // 关键帧数据库
                                            &mVocabulary,             // ORB字典
                                            mSensor != MONOCULAR);    // 当前的传感器是否是单目
}

System::GenericBuilder::~GenericBuilder() { }
bool              System::GenericBuilder::GetMapSaveOption() {return save_map; }
string            System::GenericBuilder::GetMapfileName() {return mapfile; }
bool              System::GenericBuilder::GetMapReuseOption() {return bReuseMap; }
System::eSensor   System::GenericBuilder::GetSensorType() { return mSensor; }
ORBVocabulary*    System::GenericBuilder::GetVocabulary() { return &mVocabulary; }
KeyFrameDatabase* System::GenericBuilder::GetKeyFrameDatabase() { return mpKeyFrameDatabase.get(); }
Map*              System::GenericBuilder::GetMap() { return mpMap.get(); }
Tracking*         System::GenericBuilder::GetTracker() { return mpTracker.get(); }
LocalMapping*     System::GenericBuilder::GetLocalMapper() { return mpLocalMapper.get(); }
LoopClosing*      System::GenericBuilder::GetLoopCloser() { return mpLoopCloser.get(); }
void              System::GenericBuilder::ResetKeyFrameDatabase(KeyFrameDatabase* pKeyFrameDB) { mpKeyFrameDatabase.reset(pKeyFrameDB); }
void              System::GenericBuilder::ResetMap(Map *pMap) { mpMap.reset(pMap); }



//NOTE 加载地图模式
bool System::GenericBuilder::LoadMap(const string &filename)
{

    //lock MapPoint global Mutex when loading map
    unique_lock<mutex> MapPointGlobal(MapPoint::mGlobalMutex);
    std::ifstream in(filename, std::ios_base::binary);
    if (!in)
    {
        cerr << "Cannot Open Mapfile: " << mapfile << ". No existing map file found!" << std::endl;
        return false;
    }
    cout << "Loading Mapfile: " << mapfile << "....." << std::flush;
    boost::archive::binary_iarchive ia(in, boost::archive::no_header);
    ia >> pMap;
    ia >> pKeyFrameDatabase;

    ResetKeyFrameDatabase(pKeyFrameDatabase);
    ResetMap(pMap);
    pKeyFrameDatabase->SetORBvocabulary(&mVocabulary);

    cout << colouredString("Mapfile loaded succesfully!", GREEN, REGULAR) << std::endl;
    // NOTE std::flush 用于确保输出流中的内容被立即输出
    cout << "Reconstructing Map" << flush;
    vector<ORB_SLAM2::KeyFrame*> vpKFS = pMap->GetAllKeyFrames();
    unsigned long mnFrameId = 0;
    for (auto it:vpKFS) {
        it->SetORBvocabulary(&mVocabulary);
        it->ComputeBoW();
        if (it->mnFrameId > mnFrameId)
            mnFrameId = it->mnFrameId;
    }
    Frame::nNextId = mnFrameId;
    cout << " ...done" << endl;
    in.close();
    return true;
}


/*
Class System is created by taking a Builder as argument. The Builder
will be a GUISystemBuilder (if Pangolin Viewer is wanted)
or a ROSSystemBuilder (if we don't want Pangolin Viewer but ROS functionality)
*/
System::System(std::unique_ptr<IBuilder> builder)

    : mpBuilder(std::move(builder)),
      mSensor(mpBuilder->GetSensorType()), // 初始化传感器类型
      is_save_map(mpBuilder->GetMapSaveOption()),
      mbStarted(false),
      mbReset(false),                     // 无复位标志
      mbActivateLocalizationMode(false),  // 没有这个模式转换标志
      mbDeactivateLocalizationMode(false) // 没有这个模式转换标志
{
    if (!mpBuilder)
    {
      throw std::invalid_argument("builder shouldn't be null");
    }
    // Output welcome message
    cout << endl <<
    "ORB-SLAM2 Copyright (C) 2014-2016 Raul Mur-Artal, University of Zaragoza." << endl <<
    "This program comes with ABSOLUTELY NO WARRANTY;" << endl  <<
    "This is free software, and you are welcome to redistribute it" << endl <<
    "under certain conditions. See LICENSE.txt." << endl << endl;

    cout << "Input sensor was set to: ";

    if(mSensor==MONOCULAR)
        cout << "Monocular" << endl;
    else if(mSensor==STEREO)
        cout << "Stereo" << endl;
    else if(mSensor==RGBD)
        cout << "RGB-D" << endl;

    mapfile = mpBuilder->GetMapfileName();
    bUseSavedMap = mpBuilder->GetMapReuseOption();

    // Load ORB Vocabulary
    cout << endl
         << "Loading ORB Vocabulary. This could take a while..." << endl;
    // 建立一个新的ORB字典
    mpVocabulary = mpBuilder->GetVocabulary();

    mpKeyFrameDatabase = mpBuilder->GetKeyFrameDatabase();
    mpMap = mpBuilder->GetMap();
    mpTracker = mpBuilder->GetTracker();
    mpLocalMapper = mpBuilder->GetLocalMapper();
    mpLoopCloser = mpBuilder->GetLoopCloser();


    //This is a member of ROSSystemBuilder only,
    //not the GenericBuilder
    mpPublisher = mpBuilder->GetPublisher();
    mpTracker->SetPublisherThread(mpPublisher);
    mpPublisher->SetSystem(this);

    // Set pointers between threads
    // 设置进程间的指针
    mpTracker->SetLocalMapper(mpLocalMapper);
    mpTracker->SetLoopClosing(mpLoopCloser);

    mpLocalMapper->SetTracker(mpTracker);
    mpLocalMapper->SetLoopCloser(mpLoopCloser);

    mpLoopCloser->SetTracker(mpTracker);
    mpLoopCloser->SetLocalMapper(mpLocalMapper);

    //If a Map was loaded successfully then start in Localization Mode
    if(bUseSavedMap)
      ActivateLocalizationMode();
}

void System::Start()
{
    if (mbStarted)
        return;

    //thread represents a single thread of execution
    mtLocalMapping = std::thread(&ORB_SLAM2::LocalMapping::Run, mpLocalMapper);
    mtLoopClosing = std::thread(&ORB_SLAM2::LoopClosing::Run, mpLoopCloser);
    mtPublisher = std::thread(&ORB_SLAM2::IPublisherThread::Run, mpPublisher);
    mbStarted = true;
}

// NOTE ORBSLAM2 双目输入时的追踪器接口
cv::Mat System::TrackStereo(const cv::Mat &imLeft,   // 左侧图像
                            const cv::Mat &imRight,  // 右侧图像
                            const double &timestamp) // 时间戳
{
    // 检查输入数据类型是否合法
    if (mSensor != STEREO)
    {
        // 不合法那就退出
        cerr << "ERROR: you called TrackStereo but input sensor was not set to STEREO." << endl;
        std::exit(-1);
    }
    // 检查是否有运行模式的改变
    //  Check mode change
    {
        // 互斥锁，以免mbActivateLocalizationMode、mbDeactivateLocalizationMode被修改
        unique_lock<mutex> lock(mMutexMode);
        // 如果激活定位模式
        if (mbActivateLocalizationMode)
        {
            // 调用局部建图器的请求停止函数
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            // mtLocalMapping.join();
            //NOTE: 不清楚应不应该进行注释 ex没note
            while (!mpLocalMapper->isStopped())
            {
                std::this_thread::sleep_for(std::chrono::microseconds(1000));
            }
            // 运行到这里的时候，局部建图部分就真正地停止了
            // 告知追踪器，现在 只有追踪工作

            mpTracker->InformOnlyTracking(true); // 定位时，只跟踪
            // 同时清除定位标记
            mbActivateLocalizationMode = false; // 防止重复执行
        }
        if (mbDeactivateLocalizationMode)
        {
            // 如果取消定位模式
            // 告知追踪器，现在地图构建部分也要开始工作了
            mpTracker->InformOnlyTracking(false);
            // 局部建图器要开始工作呢
            mpLocalMapper->Release();
            // 清楚标志
            mbDeactivateLocalizationMode = false; // 防止重复执行
        }                                         // 如果取消定位模式
    }                                             // 检查是否有模式的改变

    // Check reset 检查是否有复位的操作
    {
        // 上锁
        unique_lock<mutex> lock(mMutexReset);
        // 是否有复位请求？
        if (mbReset)
        {
            // 有，追踪器复位
            mpTracker->Reset();
            // 清除标志
            mbReset = false;
        }
    }
    // 用矩阵Tcw来保存估计的相机 位姿，运动追踪器的GrabImageStereo函数才是真正进行运动估计的函数
    cv::Mat Tcw = mpTracker->GrabImageStereo(imLeft, imRight, timestamp);
    // 给运动追踪状态上锁
    unique_lock<mutex> lock2(mMutexState);
    // 获取运动追踪状态
    mTrackingState = mpTracker->mState;
    // 获取当前帧追踪到的地图点向量指针
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    // 获取当前帧追踪到的关键帧特征点向量的指针
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
    // 返回获得的相机运动估计
    return Tcw;
}

// // NOTE YOLO双目输入时的追踪器接口
// cv::Mat System::TrackStereo(const cv::Mat &imLeft,   // 左侧图像
//                                 const cv::Mat &imRight,  // 右侧图像
//                                 const double &timestamp, // 时间戳
//                                 const vector<std::pair<vector<double>, int>> &detect_result)
// {
//     // 检查输入数据类型是否合法
//     if (mSensor != STEREO)
//     {
//         // 不合法那就退出
//         cerr << "ERROR: you called TrackStereo but input sensor was not set to STEREO." << endl;
//         std::exit(-1);
//     }
//     // 检查是否有运行模式的改变
//     //  Check mode change
//     {
//         // 互斥锁，以免mbActivateLocalizationMode、mbDeactivateLocalizationMode被修改
//         unique_lock<mutex> lock(mMutexMode);
//         // 如果激活定位模式
//         if (mbActivateLocalizationMode)
//         {
//             // 调用局部建图器的请求停止函数
//             mpLocalMapper->RequestStop();

//             // Wait until Local Mapping has effectively stopped
//             // mtLocalMapping.join();
//             while (!mpLocalMapper->isStopped())
//             {
//                 std::this_thread::sleep_for(std::chrono::microseconds(1000));
//             }
//             // 运行到这里的时候，局部建图部分就真正地停止了
//             // 告知追踪器，现在 只有追踪工作

//             mpTracker->InformOnlyTracking(true); // 定位时，只跟踪
//             // 同时清除定位标记
//             mbActivateLocalizationMode = false; // 防止重复执行
//         }
//         if (mbDeactivateLocalizationMode)
//         {
//             // 如果取消定位模式
//             // 告知追踪器，现在地图构建部分也要开始工作了
//             mpTracker->InformOnlyTracking(false);
//             // 局部建图器要开始工作呢
//             mpLocalMapper->Release();
//             // 清楚标志
//             mbDeactivateLocalizationMode = false; // 防止重复执行
//         }                                         // 如果取消定位模式
//     }                                             // 检查是否有模式的改变

//     // Check reset 检查是否有复位的操作
//     {
//         // 上锁
//         unique_lock<mutex> lock(mMutexReset);
//         // 是否有复位请求？
//         if (mbReset)
//         {
//             // 有，追踪器复位
//             mpTracker->Reset();
//             // 清除标志
//             mbReset = false;
//         }
//     }
//     // 用矩阵Tcw来保存估计的相机 位姿，运动追踪器的GrabImageStereo函数才是真正进行运动估计的函数
//     cv::Mat Tcw = mpTracker->GrabImageStereo(imLeft, imRight, timestamp);
//     // 给运动追踪状态上锁
//     unique_lock<mutex> lock2(mMutexState);
//     // 获取运动追踪状态
//     mTrackingState = mpTracker->mState;
//     // 获取当前帧追踪到的地图点向量指针
//     mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
//     // 获取当前帧追踪到的关键帧特征点向量的指针
//     mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
//     // 返回获得的相机运动估计
//     return Tcw;
// }
cv::Mat System::TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp)
{
    if (mSensor != RGBD)
    {
        cerr << "ERROR: you called TrackRGBD but input sensor was not set to RGBD." << endl;
        std::exit(-1);
    }

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if (mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
	     //NOTE: 以下按照双目模式进行更改
            // mtLocalMapping.join();
            while (!mpLocalMapper->isStopped())
            {
                std::this_thread::sleep_for(std::chrono::microseconds(1000));
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if (mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
        unique_lock<mutex> lock(mMutexReset);
        if (mbReset)
        {
            mpTracker->Reset();
            mbReset = false;
        }
    }
    cv::Mat Tcw = mpTracker->GrabImageRGBD(im, depthmap, timestamp);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
    return Tcw;
}
//  TODO: 添加参数const vector<std::pair<vector<double>, int>>& detect_result
// cv::Mat System::TrackRGBD(const cv::Mat &im,
//                               const cv::Mat &depthmap,
//                               const double &timestamp,
//                               const vector<std::pair<vector<double>, int>> &detect_result)
// {
//     // 判断输入数据类型是否合法
//     if (mSensor != RGBD)
//     {
//         cerr << "ERROR: you called TrackRGBD but input sensor was not set to RGBD." << endl;
//         exit(-1);
//     }

//     // Check mode change
//     // 检查模式改变
//     {
//         unique_lock<mutex> lock(mMutexMode);
//         if (mbActivateLocalizationMode)
//         {
//             mpLocalMapper->RequestStop();

//             // Wait until Local Mapping has effectively stopped
//             //NOTE unknown note
//             // while (!mpLocalMapper->isStopped())
//             // {
//             //     usleep(1000);
//             // }

//             mpTracker->InformOnlyTracking(true);
//             mbActivateLocalizationMode = false;
//         }
//         if (mbDeactivateLocalizationMode)
//         {
//             mpTracker->InformOnlyTracking(false);
//             mpLocalMapper->Release();
//             mbDeactivateLocalizationMode = false;
//         }
//     }

//     // Check reset
//     // 检查是否有复位请求
//     {
//         unique_lock<mutex> lock(mMutexReset);
//         if (mbReset)
//         {
//             mpTracker->Reset();
//             mbReset = false;
//         }
//     }

//     // NOTE  获得相机位姿的估计
//     cv::Mat Tcw = mpTracker->GrabImageRGBD(im, depthmap, timestamp, detect_result);

//     unique_lock<mutex> lock2(mMutexState);
//     mTrackingState = mpTracker->mState;
//     mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
//     mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
//     return Tcw;
// }


// 1、判断传感器的类型是否为单目模式，如果不是，则表示设置错误，函数直接返回
// 2、上锁 模式锁(mMutexMode):
// 	(1)如果目前需要激活定位模式，则请求停止局部建图，并且等待局部建图线程停止，设置为仅追踪模式。
// 	(2)如果目前需要取消定位模式，则通知局部建图可以工作了，关闭仅追踪模式  
// 3、上锁 复位锁(mMutexReset): 检查是否存在复位请求，如果有，则进行复位操作
// 4、核心部分: 根据输入的图像获得相机位姿态（其中包含了特征提取匹配，地图初始化，关键帧查询等操作）
// 5、进行数据更新，如追踪状态、当前帧的地图点、当前帧矫正之后的关键点等。
cv::Mat System::TrackMonocular(const cv::Mat &im, const double &timestamp)
{
    if(mSensor!=MONOCULAR)
    {
        cerr << "ERROR: you called TrackMonocular but input sensor was not set to Monocular." << endl;
        std::exit(-1);
    }

    // Check mode change
    {
        // 独占锁，主要是为了mbActivateLocalizationMode和mbDeactivateLocalizationMode不会发生混乱
        unique_lock<mutex> lock(mMutexMode);
        // mbActivateLocalizationMode为true会关闭局部地图线程
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();
            // Wait until Local Mapping has effectively stopped
            mtLocalMapping.join();

            // 局部地图关闭以后，只进行追踪的线程，只计算相机的位姿，没有对局部地图进行更新
            // 设置mbOnlyTracking为真
            mpTracker->InformOnlyTracking(true);
            // 关闭线程可以使得别的线程得到更多的资源
            mbActivateLocalizationMode = false;
        }
        // 如果mbDeactivateLocalizationMode是true，局部地图线程就被释放, 关键帧从局部地图中删除.
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
    unique_lock<mutex> lock(mMutexReset);
    if(mbReset)
    {
        mpTracker->Reset();
        mbReset = false;
    }
    }

    // 获取相机位姿的估计结果
    cv::Mat Tcw = mpTracker->GrabImageMonocular(im,timestamp);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;

    return Tcw;
}

void System::ActivateLocalizationMode()
{
    if(!mbActivateLocalizationMode)
    {
      unique_lock<mutex> lock(mMutexMode);
      mbActivateLocalizationMode = true;
    }
}

void System::DeactivateLocalizationMode()
{
    unique_lock<mutex> lock(mMutexMode);
    mbDeactivateLocalizationMode = true;
}

bool System::MapChanged()
{
    static int n=0;
    int curn = mpMap->GetLastBigChangeIdx();
    if(n<curn)
    {
        n=curn;
        return true;
    }
    else
        return false;
}

void System::Reset()
{
    unique_lock<mutex> lock(mMutexReset);
    mbReset = true;
}

void System::Shutdown()
{
    mpLocalMapper->RequestFinish();
    mpLoopCloser->RequestFinish();
    mpPublisher->RequestFinish();

    // Wait until all thread have effectively stopped
    while(!mpLocalMapper->isFinished() || !mpLoopCloser->isFinished()  ||
          !mpPublisher->isFinished()      || mpLoopCloser->isRunningGBA())
    {
        std::this_thread::sleep_for(std::chrono::microseconds(5000));
    }
    //Save Map only in SLAM mode
    if (!bUseSavedMap && is_save_map)
        SaveMap(mapfile);
}
//NOTE 保存轨迹SaveTrajectoryTUM
void System::SaveTrajectoryTUM(const string &filename)
{
    cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
    // 只有在传感器为双目或者RGBD时才可以工作
    if(mSensor==MONOCULAR)
    {
        cerr << "ERROR: SaveTrajectoryTUM cannot be used for monocular." << endl;
        return;
    }

    // 从地图中获取所有的关键帧
    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();

    // 根据关键帧生成的先后顺序（id）进行排序
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    // 到原点的转换，获取这个转换矩阵
    cv::Mat Two = vpKFs[0]->GetPoseInverse();

    // 文件写入的准备工作
    ofstream f;
    // NOTE 可以理解为，在输出浮点数的时候使用0.3141592654这样的方式而不是使用科学计数法
    f.open(filename.c_str()); 
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<ORB_SLAM2::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    list<bool>::iterator lbL = mpTracker->mlbLost.begin();
    for(list<cv::Mat>::iterator lit=mpTracker->mlRelativeFramePoses.begin(),
        lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++, lbL++)
    {
        if(*lbL)
            continue;

        KeyFrame* pKF = *lRit;

        cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

        // If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
        while(pKF->isBad())
        {
            Trw = Trw*pKF->mTcp;
            pKF = pKF->GetParent();
        }

        Trw = Trw*pKF->GetPose()*Two;

        cv::Mat Tcw = (*lit)*Trw;
        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

        vector<float> q = Converter::toQuaternion(Rwc);

        f << setprecision(6) << *lT << " " <<  setprecision(9) << twc.at<float>(0) << " " << twc.at<float>(1) << " " << twc.at<float>(2) << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
    }
    f.close();
    cout << endl << "trajectory saved!" << endl;
}

// 保存轨迹SaveTrajectoryKeyFrameTUM
void System::SaveKeyFrameTrajectoryTUM(const string &filename)
{
    cout << endl << "Saving keyframe trajectory to " << filename << " ..." << endl;

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    //cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];

       // pKF->SetPose(pKF->GetPose()*Two);

        if(pKF->isBad())
            continue;

        cv::Mat R = pKF->GetRotation().t();
        vector<float> q = Converter::toQuaternion(R);
        cv::Mat t = pKF->GetCameraCenter();
        f << setprecision(6) << pKF->mTimeStamp << setprecision(7) << " " << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2)
          << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;

    }

    f.close();
    cout << endl << "trajectory saved!" << endl;
}
// 保存轨迹SaveTrajectoryKITTI
void System::SaveTrajectoryKITTI(const string &filename)
{
    cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
    if(mSensor==MONOCULAR)
    {
        cerr << "ERROR: SaveTrajectoryKITTI cannot be used for monocular." << endl;
        return;
    }

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<ORB_SLAM2::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    for(list<cv::Mat>::iterator lit=mpTracker->mlRelativeFramePoses.begin(), lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++)
    {
        ORB_SLAM2::KeyFrame* pKF = *lRit;

        cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

        while(pKF->isBad())
        {
          //  cout << "bad parent" << endl;
            Trw = Trw*pKF->mTcp;
            pKF = pKF->GetParent();
        }

        Trw = Trw*pKF->GetPose()*Two;

        cv::Mat Tcw = (*lit)*Trw;

        // 定义绕x轴旋转90度的旋转矩阵
        // cv::Mat Rx = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 0, -1, 0, 1, 0);

        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

        f << setprecision(9) << Rwc.at<float>(0,0) << " " << Rwc.at<float>(0,1)  << " " << Rwc.at<float>(0,2) << " "  << twc.at<float>(0) << " " <<
             Rwc.at<float>(1,0) << " " << Rwc.at<float>(1,1)  << " " << Rwc.at<float>(1,2) << " "  << twc.at<float>(1) << " " <<
             Rwc.at<float>(2,0) << " " << Rwc.at<float>(2,1)  << " " << Rwc.at<float>(2,2) << " "  << twc.at<float>(2) << endl;
    }
    f.close();
    cout << endl << "trajectory saved!" << endl;
}

int System::GetTrackingState()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackingState;
}

vector<MapPoint*> System::GetTrackedMapPoints()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackedMapPoints;
}

vector<cv::KeyPoint> System::GetTrackedKeyPointsUn()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackedKeyPointsUn;
}
void System::SaveMap(const string &filename)
{
    //lock MapPoint global Mutex when saving map
    unique_lock<mutex> MapPointGlobal(MapPoint::mGlobalMutex);
    std::ofstream out(filename, std::ios_base::binary);
    if (!out)
    {
        cerr << "Cannot Write to Mapfile: " << mapfile << std::endl;
        exit(-1);
    }
    // NOTE std::flush 用于确保输出流中的内容被立即输出，而不是等到程序结束或缓冲区填满时才输出。这在某些需要实时显示或立即将输出内容传递到目标设备的情况下非常有用
    cout << "Saving Mapfile: " << mapfile << " ...." << std::flush;
    boost::archive::binary_oarchive oa(out, boost::archive::no_header);
    //Saving an instance of Map / KeyFrameDatabase here invokes the relevant serialize function which saves
    //all the Map class member variables (mostly private). We just have to apply an archive operator (<<)
    //to the root item mpMap / mpKeyFrameDatabase.
    oa << mpMap;
    oa << mpKeyFrameDatabase;
    cout << colouredString("Mapfile saved succesfully!", GREEN, REGULAR) << std::endl;
    out.close();
}

} //namespace ORB_SLAM
