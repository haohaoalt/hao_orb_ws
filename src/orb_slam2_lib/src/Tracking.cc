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
// 当跟踪状态不正常的时候, 使用重定位跟踪进行挽回.
//  Step Tracking.cc 流程
//  Step 1: 计算当前帧特征点的词袋向量
// Step 2: 用词袋找到与当前帧相似的候选关键帧
// Step 3: 遍历所有的候选关键帧, 通过词袋进行快速匹配, 用匹配结果初始化PnP Solver.
// Step 4: 通过一系列操作, 直到找到能够匹配上的关键帧.
// Step : 通过EPnP算法估计位姿
// Step : 通过PoseOptimization对位姿进行优化
// Step : 如果内点较少, 则通过投影的方式对之前未匹配的点进行匹配, 再进行优化

#include "Tracking.h"
#include <opencv2/core/types_c.h>

#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "ORBmatcher.h"
#include "FrameDrawer.h"
#include "Converter.h"
#include "Initializer.h"
#include "Optimizer.h"
#include "PnPsolver.h"
#include "IMapPublisher.h"
#include "System.h"
#include "Viewer.h"
#include "IPublisherThread.h"

#include <unistd.h>
#include "colorutils.h"
using namespace std;

// 程序中变量名的第一个字母如果为"m"则表示为类中的成员变量，member
// 第一个、第二个字母:
// "p"表示指针数据类型
// "n"表示int类型
// "b"表示bool类型
// "s"表示set类型
// "v"表示vector数据类型
// 'l'表示list数据类型
// "KF"表示KeyFrame数据类型
namespace ORB_SLAM2
{

    Tracking::Tracking(ORBVocabulary *pVoc,                     // BOW字典
                       Map *pMap,                               // 地图句柄
                       KeyFrameDatabase *pKFDB,                 // 关键帧产生的词袋数据库
                       const string &strSettingPath,            // 配置文件路径
                       int sensor,                              // 传感器类型
                       bool bReuseMap) : mState(NO_IMAGES_YET), // 当前系统还没有准备好
                                         mSensor(sensor),       // 传感器类型
                                         mbOnlyTracking(false), // 处于SLAM模式
                                         mbVO(false),           // 当处于纯跟踪模式的时候，这个变量表示了当前跟踪状态的好坏
                                         mpORBVocabulary(pVoc),
                                         mpKeyFrameDB(pKFDB), 
                                         mpInitializer(nullptr), // 暂时给地图初始化器设置为空指针
                                         mpSystem(nullptr),
                                         mpFrameSubscriber(nullptr),
                                         mpMapPublisher(nullptr), 
                                         mpMap(pMap), 
                                         mnLastRelocFrameId(0) // 恢复为0,没有进行这个过程的时候的默认值
    {
        // Load camera parameters from settings file
        cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
        float fx = fSettings["Camera.fx"];
        float fy = fSettings["Camera.fy"];
        float cx = fSettings["Camera.cx"];
        float cy = fSettings["Camera.cy"];

        // Fabian's way to enter Localization mode
        // Not needed with new functionality
        // int OnlyTracking = fSettings["OnlyTracking"];
        // mbOnlyTracking = OnlyTracking;
        //     |fx  0   cx|
        // K = |0   fy  cy|
        //     |0   0   1 |
        // 构造相机内参矩阵
        cv::Mat K = cv::Mat::eye(3, 3, CV_32F);
        K.at<float>(0, 0) = fx;
        K.at<float>(1, 1) = fy;
        K.at<float>(0, 2) = cx;
        K.at<float>(1, 2) = cy;
        K.copyTo(mK);

        // 图像矫正系数
        // [k1 k2 p1 p2 k3]
        cv::Mat DistCoef(4, 1, CV_32F);
        DistCoef.at<float>(0) = fSettings["Camera.k1"];
        DistCoef.at<float>(1) = fSettings["Camera.k2"];
        DistCoef.at<float>(2) = fSettings["Camera.p1"];
        DistCoef.at<float>(3) = fSettings["Camera.p2"];
        const float k3 = fSettings["Camera.k3"];
        // 有些相机的畸变系数中会没有k3项
        if (k3 != 0)
        {
            DistCoef.resize(5);
            DistCoef.at<float>(4) = k3;
        }
        DistCoef.copyTo(mDistCoef);

        // 双目摄像头baseline * fx 50
        mbf = fSettings["Camera.bf"];

        float fps = fSettings["Camera.fps"];
        if (fps == 0)
            fps = 30;

        // Max/Min Frames to insert keyframes and to check relocalization
        mMinFrames = 0;
        mMaxFrames = fps;
        cout << colouredString("Tracking Constructor", GREEN, BOLD) << endl;
        cout << endl
             << "Camera Parameters: " << endl;
        cout << "- fx: " << fx << endl;
        cout << "- fy: " << fy << endl;
        cout << "- cx: " << cx << endl;
        cout << "- cy: " << cy << endl;
        cout << "- k1: " << DistCoef.at<float>(0) << endl;
        cout << "- k2: " << DistCoef.at<float>(1) << endl;
        if (DistCoef.rows == 5)
            cout << "- k3: " << DistCoef.at<float>(4) << endl;
        cout << "- p1: " << DistCoef.at<float>(2) << endl;
        cout << "- p2: " << DistCoef.at<float>(3) << endl;
        cout << "- fps: " << fps << endl;

        int nRGB = fSettings["Camera.RGB"];
        mbRGB = nRGB;

        if (mbRGB)
            cout << "- color order: RGB (ignored if grayscale)" << endl;
        else
            cout << "- color order: BGR (ignored if grayscale)" << endl;

        // Load ORB parameters

        // 每一帧提取的特征点数
        int nFeatures = fSettings["ORBextractor.nFeatures"];
        // 图像建立金字塔时的变化尺度 1.2
        float fScaleFactor = fSettings["ORBextractor.scaleFactor"];
        // 尺度金字塔的层数 8
        int nLevels = fSettings["ORBextractor.nLevels"];
        // 提取fast特征点的默认阈值 20
        int fIniThFAST = fSettings["ORBextractor.iniThFAST"];
        // 如果默认阈值提取不出足够fast特征点，则使用最小阈值 8
        int fMinThFAST = fSettings["ORBextractor.minThFAST"];

        // tracking过程都会用到mpORBextractorLeft作为特征点提取器
        mpORBextractorLeft = new ORBextractor(nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);

        // 如果是双目，tracking过程中还会用用到mpORBextractorRight作为右目特征点提取器
        if (sensor == System::STEREO)
            mpORBextractorRight = new ORBextractor(nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);

        // 在单目初始化的时候，会用mpIniORBextractor来作为特征点提取器
        if (sensor == System::MONOCULAR)
            mpIniORBextractor = new ORBextractor(2 * nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);

        cout << endl
             << "ORB Extractor Parameters: " << endl;
        cout << "- Number of Features: " << nFeatures << endl;
        cout << "- Scale Levels: " << nLevels << endl;
        cout << "- Scale Factor: " << fScaleFactor << endl;
        cout << "- Initial Fast Threshold: " << fIniThFAST << endl;
        cout << "- Minimum Fast Threshold: " << fMinThFAST << endl;

        if (sensor == System::STEREO || sensor == System::RGBD)
        {
            // 判断一个3D点远/近的阈值 mbf * 35 / fx
            // ThDepth其实就是表示基线长度的多少倍
            mThDepth = mbf * (float)fSettings["ThDepth"] / fx;
            cout << endl
                 << "IF STEREO OR RGBD Depth Threshold (Close/Far Points): " << mThDepth << endl;
        }

        if (sensor == System::RGBD)
        {
            // 深度相机disparity转化为depth时的因子
            mDepthMapFactor = fSettings["DepthMapFactor"];
            if (fabs(mDepthMapFactor) < 1e-5)
                mDepthMapFactor = 1;
            else
                mDepthMapFactor = 1.0f / mDepthMapFactor;
        }
        //NOTE 逻辑上是若使用重用地图的方式，那么mState就是LOST
        if (bReuseMap)
        {
            mState = LOST;
        }
    }
    // 设置局部建图器
    void Tracking::SetLocalMapper(LocalMapping *pLocalMapper)
    {
        mpLocalMapper = pLocalMapper;
    }
    // 设置回环检测器
    void Tracking::SetLoopClosing(LoopClosing *pLoopClosing)
    {
        mpLoopClosing = pLoopClosing;
    }

    void Tracking::SetPublisherThread(IPublisherThread *pPubThread)
    {
        mpPublisherThread = pPubThread;
    }

    //NOTE：对双目图像的处理
    cv::Mat Tracking::GrabImageStereo(const cv::Mat &imRectLeft, const cv::Mat &imRectRight, const double &timestamp)
    {
        mImGray = imRectLeft;
        cv::Mat imGrayRight = imRectRight;
        //STEP 1 ：将RGB或RGBA图像转为灰度图像
        if (mImGray.channels() == 3)
        {
            if (mbRGB)
            {
                cvtColor(mImGray, mImGray, cv::COLOR_RGB2GRAY);
                cvtColor(imGrayRight, imGrayRight, cv::COLOR_RGB2GRAY);
            }
            else
            {
                cvtColor(mImGray, mImGray, cv::COLOR_BGR2GRAY);
                cvtColor(imGrayRight, imGrayRight, cv::COLOR_BGR2GRAY);
            }
        }
        else if (mImGray.channels() == 4)
        {
            if (mbRGB)
            {
                cvtColor(mImGray, mImGray, cv::COLOR_RGB2GRAY);
                cvtColor(imGrayRight, imGrayRight, cv::COLOR_RGB2GRAY);
            }
            else
            {
                cvtColor(mImGray, mImGray, cv::COLOR_RGB2GRAY);
                cvtColor(imGrayRight, imGrayRight, cv::COLOR_RGB2GRAY);
            }
        }
        // STEP 2 ：构造Frame
        mCurrentFrame = Frame(mImGray,             // 左目图像
                              imGrayRight,         // 右目图像
                              timestamp,           // 时间戳
                              mpORBextractorLeft,  // 左目特征提取器
                              mpORBextractorRight, // 右目特征提取器
                              mpORBVocabulary,     // 字典
                              mK,                  // 内参矩阵
                              mDistCoef,           // 去畸变参数
                              mbf,                 // 基线长度
                              mThDepth);           // 远点,近点的区分阈值
        // STEP 3 ：跟踪
        Track();

        return mCurrentFrame.mTcw.clone();
    }
    // NOTE Tracking::GrabImageRGBD
    //  输入左目RGB或RGBA图像和深度图
    //  1、将图像转为mImGray和imDepth并初始化mCurrentFrame
    //  2、进行tracking过程
    //  输出世界坐标系到该帧相机坐标系的变换矩阵

    cv::Mat Tracking::GrabImageRGBD(const cv::Mat &imRGB, const cv::Mat &imD, const double &timestamp)
    {
        mImGray = imRGB;
        cv::Mat imDepth = imD;

        // STEP 1 ：将RGB或RGBA图像转为灰度图像
        if (mImGray.channels() == 3)
        {
            if (mbRGB)
                cvtColor(mImGray, mImGray, cv::COLOR_RGB2GRAY);
            else
                cvtColor(mImGray, mImGray, cv::COLOR_BGR2GRAY);
        }
        else if (mImGray.channels() == 4)
        {
            if (mbRGB)
                cvtColor(mImGray, mImGray, cv::COLOR_RGBA2GRAY);
            else
                cvtColor(mImGray, mImGray, cv::COLOR_RGB2GRAY);
        }

        if ((fabs(mDepthMapFactor - 1.0f) > 1e-5) || imDepth.type() != CV_32F)
            imDepth.convertTo(imDepth, CV_32F, mDepthMapFactor);
        // STEP 2 ：构造Frame
        mCurrentFrame = Frame(mImGray, imDepth, timestamp, mpORBextractorLeft, mpORBVocabulary, mK, mDistCoef, mbf, mThDepth);

        // STEP 3 ：跟踪
        Track();

        return mCurrentFrame.mTcw.clone();
    }
    // 输入左目RGB或RGBA图像和深度图
    // 1、将图像转为mImGray和imDepth并初始化mCurrentFrame
    // 2、进行tracking过程
    // 输出世界坐标系到该帧相机坐标系的变换矩阵
    // cv::Mat Tracking::GrabImageRGBD(
    //     const cv::Mat &imRGB, // 彩色图像
    //     const cv::Mat &imD,   // 深度图像
    //     const double &timestamp,
    //     const vector<std::pair<vector<double>, int>> &detect_result) // 时间戳
    // {
    //     mImGray = imRGB;
    //     cv::Mat imDepth = imD;
    //     // step 1：将RGB或RGBA图像转为灰度图像
    //     if (mImGray.channels() == 3)
    //     {
    //         if (mbRGB)
    //             cvtColor(mImGray, mImGray, CV_RGB2GRAY);
    //         else
    //             cvtColor(mImGray, mImGray, CV_BGR2GRAY);
    //     }
    //     else if (mImGray.channels() == 4)
    //     {
    //         if (mbRGB)
    //             cvtColor(mImGray, mImGray, CV_RGBA2GRAY);
    //         else
    //             cvtColor(mImGray, mImGray, CV_BGRA2GRAY);
    //     }
    //     // step 2 ：将深度相机的disparity转为Depth , 也就是转换成为真正尺度下的深度
    //     // 这里的判断条件感觉有些尴尬
    //     // 前者和后者满足一个就可以了
    //     // 满足前者意味着,mDepthMapFactor 相对1来讲要足够大
    //     // 满足后者意味着,如果深度图像不是浮点型? 才会执行
    //     // 意思就是说,如果读取到的深度图像是浮点型,就不执行这个尺度的变换操作了呗? TODO
    //     if ((fabs(mDepthMapFactor - 1.0f) > 1e-5) || imDepth.type() != CV_32F)
    //         imDepth.convertTo(  // 将图像转换成为另外一种数据类型,具有可选的数据大小缩放系数
    //             imDepth,        // 输出图像
    //             CV_32F,         // 输出图像的数据类型
    //             mDepthMapFactor // 缩放系数
    //         );
    //     // 步骤3：构造Frame
    //     mCurrentFrame = Frame(
    //         mImGray,            // 灰度图像
    //         imDepth,            // 深度图像
    //         timestamp,          // 时间戳
    //         mpORBextractorLeft, // ORB特征提取器
    //         mpORBVocabulary,    // 词典
    //         mK,                 // 相机内参矩阵
    //         mDistCoef,          // 相机的去畸变参数
    //         mbf,                // 相机基线*相机焦距
    //         mThDepth,
    //         detect_result); // 内外点区分深度阈值

    //     Track();

    //     return mCurrentFrame.mTcw.clone();
    // }

    cv::Mat Tracking::GrabImageMonocular(const cv::Mat &im, const double &timestamp)
    {
        mImGray = im;

        //STEP 1: 如果输入的图像不为灰度图，则转换为灰度图。
        if (mImGray.channels() == 3)
        {
            if (mbRGB)
                cvtColor(mImGray, mImGray, cv::COLOR_RGB2GRAY);
            else
                cvtColor(mImGray, mImGray, cv::COLOR_BGR2GRAY);
        }
        else if (mImGray.channels() == 4)
        {
            if (mbRGB)
                cvtColor(mImGray, mImGray, cv::COLOR_RGBA2GRAY);
            else
                cvtColor(mImGray, mImGray, cv::COLOR_RGBA2GRAY);
        }

        //STEP 2 ：构造Frame
        // 判断当前是否进行了初始化，如果当前为第一帧，则 mState==NO_IMAGES_YET，表示没有进行初始化
        if (mState == NOT_INITIALIZED || mState == NO_IMAGES_YET)
            mCurrentFrame = Frame(mImGray, timestamp, mpIniORBextractor, // 初始化ORB特征点提取器会提取2倍的指定特征点数目
                                  mpORBVocabulary, mK, mDistCoef, mbf, mThDepth);
        else
            mCurrentFrame = Frame(mImGray, timestamp, mpORBextractorLeft, // 正常运行的时的ORB特征点提取器，提取指定数目特征点
                                  mpORBVocabulary, mK, mDistCoef, mbf, mThDepth);

        //STEP 3 ：跟踪
        Track();

        return mCurrentFrame.mTcw.clone();
    }

    void Tracking::Track()
    {
        if (mState == NO_IMAGES_YET)
        {
            mState = NOT_INITIALIZED;
        }

        mLastProcessedState = mState;

        // Get Map Mutex -> Map cannot be changed
        unique_lock<mutex> lock(mpMap->mMutexMapUpdate);

        if (mState == NOT_INITIALIZED)
        {
            if (mSensor == System::STEREO || mSensor == System::RGBD)
                StereoInitialization();
            else
                MonocularInitialization();

            if (mpFrameSubscriber)
                mpFrameSubscriber->Update(this);

            if (mState != OK)
                return;
        }
        else
        {
            // System is initialized. Track Frame.
            bool bOK;
            // Initial camera pose estimation using motion model or relocalization (if tracking is lost)
            if (!mbOnlyTracking)
            {
                // Local Mapping is activated. This is the normal behaviour, unless
                // you explicitly activate the "only tracking" mode.

                if (mState == OK)
                {
                    // Local Mapping might have changed some MapPoints tracked in last frame
                    CheckReplacedInLastFrame();

                    if (mVelocity.empty() || mCurrentFrame.mnId < mnLastRelocFrameId + 2)
                    {
                        bOK = TrackReferenceKeyFrame();
                    }
                    else
                    {
                        bOK = TrackWithMotionModel();
                        if (!bOK)
                            bOK = TrackReferenceKeyFrame();
                    }
                }
                else
                {
                    // 如果跟踪状态不成功,那么就只能重定位了
                    // 重定位，从之前的关键帧中找出与当前帧之间拥有充足匹配点的候选帧，利用Ransac迭代，通过PnP求解位姿。
                    cout << "!mbOnlyTracking + mState != OK =Ransac PnP" << endl;
                    bOK = Relocalization();
                }
            }
            else
            {
                // NOTE  Localization Mode: Local Mapping is deactivated
                cout << colouredString("mbOnlyTracking = ", PURPLE, BOLD) << mbOnlyTracking << endl;
                cout << mCurrentFrame.mTcw << endl;
                cout << "=====================================================================" << endl;
                // 纯追踪模式：不插入新的关键帧，不添加新的地图点，局部地图线程不工作，而且回环检测线程也不会工作，只会追踪地图中现有的地图点。
                // Step 2：只进行跟踪tracking，局部地图不工作
                if (mState == LOST)
                {
                    // Step 2.1 如果跟丢了，只能重定位
                    cout << colouredString("RRRRRRRRRRRRRRRRRRRRRRRRRRRR mbOnlyTracking + mState == LOST = Relocalization",RED,BOLD) << endl;
                    bOK = Relocalization();
                }
                else
                {
                    // mbVO是mbOnlyTracking为true时的才有的一个变量
                    // mbVO为false表示此帧匹配了很多的MapPoints，跟踪很正常 (注意有点反直觉)
                    // mbVO为true表明此帧匹配了很少的MapPoints，少于10个，要跪的节奏
                    // cout << colouredString("mbVO is : ", RED, REGULAR) << mbVO << endl;

                    if (!mbVO)
                    {
                        // In last frame we tracked enough MapPoints in the map
                        // Step 2.2 如果跟踪正常，使用恒速模型 或 参考关键帧跟踪
                        cout << colouredString("mVelocity is : ", RED, REGULAR) << mVelocity << endl;
                        //[0.999735, 0.0067194519, -0.022020478, -0.0009441562;
                        //  -0.0068219034, 0.9999662, -0.0045828605, 0.00057902932;
                        //  0.021988939, 0.004731793, 0.99974698, 0.00028265268;
                        //  0, 0, 0, 1]
                        if (!mVelocity.empty())
                        {
                            bOK = TrackWithMotionModel();
                            // 为了和前面模式统一，这个地方是不是应该加上
                            // if(!bOK)
                            //    bOK = TrackReferenceKeyFrame();
                        }
                        else
                        {
                            // 如果恒速模型不被满足,那么就只能够通过参考关键帧来定位
                            bOK = TrackReferenceKeyFrame();
                        }
                    }
                    else //**mbVO为true表明此帧匹配了很少的MapPoints，少于10个，要跪的节奏
                    {
                        // In last frame we tracked mainly "visual odometry" points.

                        // We compute two camera poses, one from motion model and one doing relocalization.
                        // If relocalization is sucessfull we choose that solution, otherwise we retain
                        // the "visual odometry" solution.
                        // mbVO为true，表明此帧匹配了很少（小于10）的地图点，要跪的节奏，既做跟踪又做重定位

                        // MM=Motion Model,通过运动模型进行跟踪的结果

                        bool bOKMM = false;
                        // 通过重定位方法来跟踪的结果
                        bool bOKReloc = false;
                        // 运动模型中构造的地图点
                        vector<MapPoint *> vpMPsMM;
                        // 在追踪运动模型后发现的外点
                        vector<bool> vbOutMM;
                        // 运动模型得到的位姿
                        cv::Mat TcwMM;


                        // Step 2.3 当运动模型有效的时候,根据运动模型计算位姿
                        if (!mVelocity.empty())
                        {
                            cout << colouredString("================mState != LOST mbVO is true, TrackWithMotionModel",GREEN,BOLD) << endl;
                            bOKMM = TrackWithMotionModel();
                            // 将恒速模型跟踪结果暂存到这几个变量中，因为后面重定位会改变这些变量
                            vpMPsMM = mCurrentFrame.mvpMapPoints;
                            vbOutMM = mCurrentFrame.mvbOutlier;
                            TcwMM = mCurrentFrame.mTcw.clone();
                        }
                        // Step 2.4 使用重定位的方法来得到当前帧的位姿
                        cout << colouredString("=====================mState != LOST mbVO is true, Relocalization", YELLOW, BOLD) << endl;
                        bOKReloc = Relocalization();

                        // Step 2.5 根据前面的恒速模型、重定位结果来更新状态
                        if (bOKMM && !bOKReloc)
                        {
                            cout << "bOKMM && !bOKReloc" << endl;
                            mCurrentFrame.SetPose(TcwMM);
                            mCurrentFrame.mvpMapPoints = vpMPsMM;
                            mCurrentFrame.mvbOutlier = vbOutMM;

                            if (mbVO)
                            {
                                for (int i = 0; i < mCurrentFrame.N; i++)
                                {
                                    if (mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i])
                                    {
                                        mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                                    }
                                }
                            }
                        }
                        else if (bOKReloc)
                        {
                            // 只要重定位成功整个跟踪过程正常进行（重定位与跟踪，更相信重定位）
                            mbVO = false;
                        }
                        // 有一个成功我们就认为执行成功了
                        cout << colouredString("bOK =bOKMM || bOKReloc", YELLOW, REGULAR) << bOK << "CHANGE TO" << (bOKReloc || bOKMM) << endl;
                        bOK = bOKReloc || bOKMM;
                    }
                }
            }
            // 将最新的关键帧作为当前帧的参考关键帧
            mCurrentFrame.mpReferenceKF = mpReferenceKF;

            // If we have an initial estimation of the camera pose and matching. Track the local map.
            // Step 3：在跟踪得到当前帧初始姿态后，现在对local map进行跟踪得到更多的匹配，并优化当前位姿
            // 前面只是跟踪一帧得到初始位姿，这里搜索局部关键帧、局部地图点，和当前帧进行投影匹配，得到更多匹配的MapPoints后进行Pose优化
            if (!mbOnlyTracking)
            {
                if (bOK) // 作用：通过投影，从已经生成的地图点中找到更多的对应关系，精确结果
                    bOK = TrackLocalMap();
            }
            else
            {
                // mbVO true means that there are few matches to MapPoints in the map. We cannot retrieve
                // a local map and therefore we do not perform TrackLocalMap(). Once the system relocalizes
                // the camera we will use the local map again.
                // 重定位成功
                if (bOK && !mbVO)
                    bOK = TrackLocalMap();
            }

            // 根据上面的操作来判断是否追踪成功
            if (bOK)
                mState = OK;
            else
                mState = LOST;

            // Update drawer
            if (mpFrameSubscriber)
                // Step 4：更新显示线程中的图像、特征点、地图点等信息
                mpFrameSubscriber->Update(this);

            // If tracking were good, check if we insert a keyframe
            if (bOK)
            {
                // Update motion model
                // Step 5：跟踪成功，更新恒速运动模型
                if (!mLastFrame.mTcw.empty())
                {
                    cv::Mat LastTwc = cv::Mat::eye(4, 4, CV_32F);
                    mLastFrame.GetRotationInverse().copyTo(LastTwc.rowRange(0, 3).colRange(0, 3));
                    mLastFrame.GetCameraCenter().copyTo(LastTwc.rowRange(0, 3).col(3));
                    mVelocity = mCurrentFrame.mTcw * LastTwc;
                }
                else
                    mVelocity = cv::Mat(); // 否则速度为空

                // 更新显示中的位姿
                if (mpMapPublisher)
                    mpMapPublisher->SetCurrentCameraPose(mCurrentFrame.mTcw);

                // NOTE: Add code in Tracking.cc
                // mpMap->EraseAllObjCenters();
                // // cv::Mat center3d;
                // int a = 0;
                // for (int k = 0; k < mCurrentFrame.objects_cur_.size(); ++k)
                // {
                //     // cv::Mat obj_3d;
                //     if (mCurrentFrame.objects_cur_[k]->ndetect_class == 1)
                //     {
                //         mCurrentFrame.UnprojectObjectCenter(k, mCurrentFrame.imDepth);
                //     }
                // }

                // for (int a = 0; a < mCurrentFrame.vMcenter_static.size(); ++a)
                // {
                //     // cout << "mCurrentFrame.vMcenter_static[a] is : " << endl << mCurrentFrame.vMcenter_static[a] << endl;
                //     MapPoint *pNewMP = new MapPoint(mCurrentFrame.vMcenter_static[a], mpMap, &mCurrentFrame, a);
                //     // vStaticObj_Center.push_back(pNewMP);
                //     mpMap->AddObjCenters(pNewMP);
                //     // cout << "!1" << endl;
                // }
                // NOTE: End code in Tracking.cc

                // Clean VO matches
                // Step 6：清除观测不到的地图点
                for (int i = 0; i < mCurrentFrame.N; i++)
                {
                    MapPoint *pMP = mCurrentFrame.mvpMapPoints[i];
                    if (pMP)
                        if (pMP->Observations() < 1)
                        {
                            mCurrentFrame.mvbOutlier[i] = false;
                            mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
                        }
                }

                // Delete temporal MapPoints
                // Step 7：清除恒速模型跟踪中 UpdateLastFrame中为当前帧临时添加的MapPoints（仅双目和rgbd）
                for (list<MapPoint *>::iterator lit = mlpTemporalPoints.begin(), lend = mlpTemporalPoints.end(); lit != lend; lit++)
                {
                    MapPoint *pMP = *lit;
                    delete pMP;
                }
                mlpTemporalPoints.clear();

                // Check if we need to insert a new keyframe
                // Step 8：检测并插入关键帧，对于双目或RGB-D会产生新的地图点
                if (NeedNewKeyFrame())
                    CreateNewKeyFrame();

                // We allow points with high innovation (considererd outliers by the Huber Function)
                // pass to the new keyframe, so that bundle adjustment will finally decide
                // if they are outliers or not. We don't want next frame to estimate its position
                // with those points so we discard them in the frame.
                //  Step 9 删除那些在bundle adjustment中检测为outlier的地图点
                for (int i = 0; i < mCurrentFrame.N; i++)
                {
                    if (mCurrentFrame.mvpMapPoints[i] && mCurrentFrame.mvbOutlier[i])
                        mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
                }
            }

            // Reset if the camera get lost soon after initialization
            // Step 10 如果初始化后不久就跟踪失败，并且relocation也没有搞定，只能重新Reset
            if (mState == LOST)
            {
                if (mpMap->KeyFramesInMap() <= 5)
                {
                    cout << "Track lost soon after initialisation, reseting..." << endl;
                    if (mpSystem)
                        mpSystem->Reset();
                    return;
                }
            }

            if (!mCurrentFrame.mpReferenceKF)
                mCurrentFrame.mpReferenceKF = mpReferenceKF;

            mLastFrame = Frame(mCurrentFrame);
        }

        // Store frame pose information to retrieve the complete camera trajectory afterwards.
        // STEP 11：记录位姿信息，用于最后保存所有的轨迹
        if (!mCurrentFrame.mTcw.empty())
        {
            cv::Mat Tcr = mCurrentFrame.mTcw * mCurrentFrame.mpReferenceKF->GetPoseInverse();
            mlRelativeFramePoses.push_back(Tcr);
            mlpReferences.push_back(mpReferenceKF);
            mlFrameTimes.push_back(mCurrentFrame.mTimeStamp);
            mlbLost.push_back(mState == LOST);
        }
        else
        {
            // This can happen if tracking is lost
            if (!mlRelativeFramePoses.empty())
                mlRelativeFramePoses.push_back(mlRelativeFramePoses.back());
            mlpReferences.push_back(mlpReferences.back());
            mlFrameTimes.push_back(mlFrameTimes.back());
            mlbLost.push_back(mState == LOST);
        }
    }

    //NOTE 双目初始化
    void Tracking::StereoInitialization()
    {
        cout << colouredString("Stereo and RGBD init in StereoInitialization", GREEN, REGULAR) << endl;
        if (mCurrentFrame.N > 500)
        {
            // Set Frame pose to the origin
            //双目初始化依旧是单位阵作为初始位姿
            mCurrentFrame.SetPose(cv::Mat::eye(4, 4, CV_32F));

            // Create KeyFrame
            KeyFrame *pKFini = new KeyFrame(mCurrentFrame, mpMap, mpKeyFrameDB);

            // Insert KeyFrame in the map
            mpMap->AddKeyFrame(pKFini);

            // Create MapPoints and asscoiate to KeyFrame
            for (int i = 0; i < mCurrentFrame.N; i++)
            {
                float z = mCurrentFrame.mvDepth[i];
                if (z > 0)
                {
                    cv::Mat x3D = mCurrentFrame.UnprojectStereo(i);
                    MapPoint *pNewMP = new MapPoint(x3D, pKFini, mpMap);
                    pNewMP->AddObservation(pKFini, i);
                    pKFini->AddMapPoint(pNewMP, i);
                    pNewMP->ComputeDistinctiveDescriptors();
                    pNewMP->UpdateNormalAndDepth();
                    mpMap->AddMapPoint(pNewMP);

                    mCurrentFrame.mvpMapPoints[i] = pNewMP;
                }
            }

            cout << colouredString("Stereo Initialization New map created with ", GREEN, REGULAR) << mpMap->MapPointsInMap() << " points" << endl;

            mpLocalMapper->InsertKeyFrame(pKFini);

            mLastFrame = Frame(mCurrentFrame);
            mnLastKeyFrameId = mCurrentFrame.mnId;
            mpLastKeyFrame = pKFini;

            mvpLocalKeyFrames.push_back(pKFini);
            mvpLocalMapPoints = mpMap->GetAllMapPoints();
            mpReferenceKF = pKFini;
            mCurrentFrame.mpReferenceKF = pKFini;

            mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

            mpMap->mvpKeyFrameOrigins.push_back(pKFini);

            if (mpMapPublisher)
                mpMapPublisher->SetCurrentCameraPose(mCurrentFrame.mTcw);

            mState = OK;
        }
    }

    /*
     * @brief 单目的地图初始化
     *
     * 并行地计算基础矩阵和单应性矩阵，选取其中一个模型，恢复出最开始两帧之间的相对姿态以及点云
     * 得到初始两帧的匹配、相对运动、初始MapPoints
     *
     * Step 1：（未创建）得到用于初始化的第一帧，初始化需要两帧
     * Step 2：（已创建）如果当前帧特征点数大于100，则得到用于单目初始化的第二帧
     * Step 3：在mInitialFrame与mCurrentFrame中找匹配的特征点对
     * Step 4：如果初始化的两帧之间的匹配点太少，重新初始化
     * Step 5：通过H模型或F模型进行单目初始化，得到两帧间相对运动、初始MapPoints
     * Step 6：删除那些无法进行三角化的匹配点
     * Step 7：将三角化得到的3D点包装成MapPoints
     */
    //NOTE 单目初始化
    void Tracking::MonocularInitialization()
    {

        // STEP 1 如果单目初始器还没有被创建，则创建。后面如果重新初始化时会清掉这个
        if (!mpInitializer)
        {
            // Set Reference Frame
            // 单目初始帧的特征点数必须大于100
            if (mCurrentFrame.mvKeys.size() > 100)
            {
                // 初始化需要两帧，分别是mInitialFrame，mCurrentFrame
                mInitialFrame = Frame(mCurrentFrame);
                // 用当前帧更新上一帧
                mLastFrame = Frame(mCurrentFrame);
                // mvbPrevMatched  记录"上一帧"所有特征点
                mvbPrevMatched.resize(mCurrentFrame.mvKeysUn.size());
                for (size_t i = 0; i < mCurrentFrame.mvKeysUn.size(); i++)
                    mvbPrevMatched[i] = mCurrentFrame.mvKeysUn[i].pt;
                // 删除前判断一下，来避免出现段错误。不过在这里是多余的判断
                // 不过在这里是多余的判断，因为前面已经判断过了
                // if (mpInitializer)
                delete mpInitializer;

                // 由当前帧构造初始器 sigma:1.0 iterations:200
                mpInitializer = new Initializer(mCurrentFrame, 1.0, 200);

                // 初始化为-1 表示没有任何匹配。这里面存储的是匹配的点的id
                fill(mvIniMatches.begin(), mvIniMatches.end(), -1);

                return;
            }
        }
        else
        {
            // Try to initialize
            // STEP 2 如果当前帧特征点数太少（不超过100），则重新构造初始器
            // NOTICE 只有连续两帧的特征点个数都大于100时，才能继续进行初始化过程
            if ((int)mCurrentFrame.mvKeys.size() <= 100)
            {
                delete mpInitializer;
                mpInitializer = static_cast<Initializer *>(NULL);
                fill(mvIniMatches.begin(), mvIniMatches.end(), -1);
                return;
            }

            // Find correspondences
            // STEP 3 在mInitialFrame与mCurrentFrame中找匹配的特征点对
            ORBmatcher matcher(0.9,   // 最佳的和次佳特征点评分的比值阈值，这里是比较宽松的，跟踪时一般是0.7
                               true); // 检查特征点的方向

            // 对 mInitialFrame,mCurrentFrame 进行特征点匹配
            // mvbPrevMatched为参考帧的特征点坐标，初始化存储的是mInitialFrame中特征点坐标，匹配后存储的是匹配好的当前帧的特征点坐标
            // mvIniMatches 保存参考帧F1中特征点是否匹配上，index保存是F1对应特征点索引，值保存的是匹配好的F2特征点索引
            int nmatches = matcher.SearchForInitialization(mInitialFrame, mCurrentFrame, // 初始化时的参考帧和当前帧
                                                           mvbPrevMatched,               // 在初始化参考帧中提取得到的特征点
                                                           mvIniMatches,                 // 保存匹配关系
                                                           100);                         // 搜索窗口大小

            // Check if there are enough correspondences
            // STEP 4 验证匹配结果，如果初始化的两帧之间的匹配点太少，重新初始化
            if (nmatches < 100)
            {
                delete mpInitializer;
                mpInitializer = static_cast<Initializer *>(NULL);
                return;
            }

            cv::Mat Rcw;                 // Current Camera Rotation
            cv::Mat tcw;                 // Current Camera Translation
            vector<bool> vbTriangulated; // Triangulated Correspondences (mvIniMatches)

            // STEP5 通过H模型或F模型进行单目初始化，得到两帧间相对运动、初始MapPoints
            if (mpInitializer->Initialize(mCurrentFrame, mvIniMatches, Rcw, tcw, mvIniP3D, vbTriangulated))
            {
                // STEP 6 初始化成功后，删除那些无法进行三角化的匹配点
                for (size_t i = 0, iend = mvIniMatches.size(); i < iend; i++)
                {
                    if (mvIniMatches[i] >= 0 && !vbTriangulated[i])
                    {
                        mvIniMatches[i] = -1;
                        nmatches--;
                    }
                }

                // Set Frame Poses
                // STEP 7 将初始化的第一帧作为世界坐标系，因此第一帧变换矩阵为单位矩阵
                mInitialFrame.SetPose(cv::Mat::eye(4, 4, CV_32F));
                cv::Mat Tcw = cv::Mat::eye(4, 4, CV_32F);
                Rcw.copyTo(Tcw.rowRange(0, 3).colRange(0, 3));
                tcw.copyTo(Tcw.rowRange(0, 3).col(3));
                mCurrentFrame.SetPose(Tcw);

                CreateInitialMapMonocular();
            }
        }
    }

    void Tracking::CreateInitialMapMonocular()
    {
        // Create KeyFrames
        KeyFrame *pKFini = new KeyFrame(mInitialFrame, mpMap, mpKeyFrameDB);
        KeyFrame *pKFcur = new KeyFrame(mCurrentFrame, mpMap, mpKeyFrameDB);

        pKFini->ComputeBoW();
        pKFcur->ComputeBoW();

        // Insert KFs in the map
        mpMap->AddKeyFrame(pKFini);
        mpMap->AddKeyFrame(pKFcur);

        // Create MapPoints and asscoiate to keyframes
        for (size_t i = 0; i < mvIniMatches.size(); i++)
        {
            if (mvIniMatches[i] < 0)
                continue;

            // Create MapPoint.
            cv::Mat worldPos(mvIniP3D[i]);

            MapPoint *pMP = new MapPoint(worldPos, pKFcur, mpMap);

            pKFini->AddMapPoint(pMP, i);
            pKFcur->AddMapPoint(pMP, mvIniMatches[i]);

            pMP->AddObservation(pKFini, i);
            pMP->AddObservation(pKFcur, mvIniMatches[i]);

            pMP->ComputeDistinctiveDescriptors();
            pMP->UpdateNormalAndDepth();

            // Fill Current Frame structure
            mCurrentFrame.mvpMapPoints[mvIniMatches[i]] = pMP;
            mCurrentFrame.mvbOutlier[mvIniMatches[i]] = false;

            // Add to Map
            mpMap->AddMapPoint(pMP);
        }

        // Update Connections
        pKFini->UpdateConnections();
        pKFcur->UpdateConnections();

        // Bundle Adjustment
        cout << colouredString("CreateInitialMapMonocular New Map created with ",GREEN,REGULAR) << mpMap->MapPointsInMap() << " points" << endl;

        Optimizer::GlobalBundleAdjustemnt(mpMap, 20);

        // Set median depth to 1
        float medianDepth = pKFini->ComputeSceneMedianDepth(2);
        float invMedianDepth = 1.0f / medianDepth;

        if (medianDepth < 0 || pKFcur->TrackedMapPoints(1) < 100)
        {
            cout << "Wrong initialization (median depth = " << medianDepth
                 << ", #tracked map points = " << pKFcur->TrackedMapPoints(1)
                 << "), reseting..." << endl;
            Reset();
            return;
        }

        // Scale initial baseline
        cv::Mat Tc2w = pKFcur->GetPose();
        Tc2w.col(3).rowRange(0, 3) = Tc2w.col(3).rowRange(0, 3) * invMedianDepth;
        pKFcur->SetPose(Tc2w);

        // Scale points
        vector<MapPoint *> vpAllMapPoints = pKFini->GetMapPointMatches();
        for (size_t iMP = 0; iMP < vpAllMapPoints.size(); iMP++)
        {
            if (vpAllMapPoints[iMP])
            {
                MapPoint *pMP = vpAllMapPoints[iMP];
                pMP->SetWorldPos(pMP->GetWorldPos() * invMedianDepth);
            }
        }

        mpLocalMapper->InsertKeyFrame(pKFini);
        mpLocalMapper->InsertKeyFrame(pKFcur);

        mCurrentFrame.SetPose(pKFcur->GetPose());
        mnLastKeyFrameId = mCurrentFrame.mnId;
        mpLastKeyFrame = pKFcur;

        mvpLocalKeyFrames.push_back(pKFcur);
        mvpLocalKeyFrames.push_back(pKFini);
        mvpLocalMapPoints = mpMap->GetAllMapPoints();
        mpReferenceKF = pKFcur;
        mCurrentFrame.mpReferenceKF = pKFcur;

        mLastFrame = Frame(mCurrentFrame);

        mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

        if (mpMapPublisher)
            mpMapPublisher->SetCurrentCameraPose(pKFcur->GetPose());

        mpMap->mvpKeyFrameOrigins.push_back(pKFini);

        mState = OK;
    }

    void Tracking::CheckReplacedInLastFrame()
    {
        for (int i = 0; i < mLastFrame.N; i++)
        {
            MapPoint *pMP = mLastFrame.mvpMapPoints[i];

            if (pMP)
            {
                MapPoint *pRep = pMP->GetReplaced();
                if (pRep)
                {
                    mLastFrame.mvpMapPoints[i] = pRep;
                }
            }
        }
    }

    bool Tracking::TrackReferenceKeyFrame()
    {
        // cout << "-----------------enter TrackReferenceKeyFrame--------------------------" << endl;
        //  Compute Bag of Words vector
        mCurrentFrame.ComputeBoW();

        // We perform first an ORB matching with the reference keyframe
        // If enough matches are found we setup a PnP solver
        ORBmatcher matcher(0.7, true);
        vector<MapPoint *> vpMapPointMatches;

        int nmatches = matcher.SearchByBoW(mpReferenceKF, mCurrentFrame, vpMapPointMatches);

        if (nmatches < 15)
            return false;

        mCurrentFrame.mvpMapPoints = vpMapPointMatches;
        mCurrentFrame.SetPose(mLastFrame.mTcw);

        Optimizer::PoseOptimization(&mCurrentFrame);

        // Discard outliers
        int nmatchesMap = 0;
        for (int i = 0; i < mCurrentFrame.N; i++)
        {
            if (mCurrentFrame.mvpMapPoints[i])
            {
                if (mCurrentFrame.mvbOutlier[i])
                {
                    MapPoint *pMP = mCurrentFrame.mvpMapPoints[i];

                    mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
                    mCurrentFrame.mvbOutlier[i] = false;
                    pMP->mbTrackInView = false;
                    pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                    nmatches--;
                }
                else if (mCurrentFrame.mvpMapPoints[i]->Observations() > 0)
                    nmatchesMap++;
            }
        }
        // cout << "-----------------end TrackReferenceKeyFrame--------------------------" << endl;
        return nmatchesMap >= 10;
    }

    void Tracking::UpdateLastFrame()
    {
        // Update pose according to reference keyframe
        KeyFrame *pRef = mLastFrame.mpReferenceKF;
        cv::Mat Tlr = mlRelativeFramePoses.back();

        mLastFrame.SetPose(Tlr * pRef->GetPose());

        if (mnLastKeyFrameId == mLastFrame.mnId || mSensor == System::MONOCULAR || !mbOnlyTracking)
            return;

        // Create "visual odometry" MapPoints
        // We sort points according to their measured depth by the stereo/RGB-D sensor
        vector<pair<float, int>> vDepthIdx;
        vDepthIdx.reserve(mLastFrame.N);
        for (int i = 0; i < mLastFrame.N; i++)
        {
            float z = mLastFrame.mvDepth[i];
            if (z > 0)
            {
                vDepthIdx.push_back(make_pair(z, i));
            }
        }

        if (vDepthIdx.empty())
            return;

        sort(vDepthIdx.begin(), vDepthIdx.end());

        // We insert all close points (depth<mThDepth)
        // If less than 100 close points, we insert the 100 closest ones.
        int nPoints = 0;
        for (size_t j = 0; j < vDepthIdx.size(); j++)
        {
            int i = vDepthIdx[j].second;

            bool bCreateNew = false;

            MapPoint *pMP = mLastFrame.mvpMapPoints[i];
            if (!pMP)
                bCreateNew = true;
            else if (pMP->Observations() < 1)
            {
                bCreateNew = true;
            }

            if (bCreateNew)
            {
                cv::Mat x3D = mLastFrame.UnprojectStereo(i);
                MapPoint *pNewMP = new MapPoint(x3D, mpMap, &mLastFrame, i);

                mLastFrame.mvpMapPoints[i] = pNewMP;

                mlpTemporalPoints.push_back(pNewMP);
                nPoints++;
            }
            else
            {
                nPoints++;
            }

            if (vDepthIdx[j].first > mThDepth && nPoints > 100)
                break;
        }
    }

    bool Tracking::TrackWithMotionModel()
    {
        // cout << colouredString("enter TrackWithMotionModel",GREEN,BOLD) << endl;
        // 最小距离 < 0.9*次小距离 匹配成功，检查旋转
        ORBmatcher matcher(0.9, true);

        // Update last frame pose according to its reference keyframe
        // Create "visual odometry" points if in Localization Mode
        // STEP 1：更新上一帧的位姿；对于双目或RGB-D相机，还会根据深度值生成临时地图点
        //TODO onlyTracking是不更新上一帧的位姿
        UpdateLastFrame();

        // STEP 2：根据之前估计的速度，用恒速模型得到当前帧的初始位姿。
        mCurrentFrame.SetPose(mVelocity * mLastFrame.mTcw);

        // 清空当前帧的地图点
        fill(mCurrentFrame.mvpMapPoints.begin(), mCurrentFrame.mvpMapPoints.end(), static_cast<MapPoint *>(NULL));

        // Project points seen in previous frame
        // 设置特征匹配过程中的搜索半径
        int th;
        if (mSensor != System::STEREO)
            th = 15;
        else
            th = 7;

        // STEP 3：用上一帧地图点进行投影匹配，如果匹配点不够，则扩大搜索半径再来一次
        int nmatches = matcher.SearchByProjection(mCurrentFrame, mLastFrame, th, mSensor == System::MONOCULAR);

        // If few matches, uses a wider window search
        // 如果匹配点太少，则扩大搜索半径再来一次
        if (nmatches < 20)
        {
            fill(mCurrentFrame.mvpMapPoints.begin(), mCurrentFrame.mvpMapPoints.end(), static_cast<MapPoint *>(NULL));
            nmatches = matcher.SearchByProjection(mCurrentFrame, mLastFrame, 2 * th, mSensor == System::MONOCULAR);
        }

        // 如果还是不能够获得足够的匹配点,那么就认为跟踪失败
        if (nmatches < 20)
            return false;

        // Optimize frame pose with all matches
        // STEP 4：利用3D-2D投影关系，优化当前帧位姿
        Optimizer::PoseOptimization(&mCurrentFrame);

        // Discard outliers
        // Step 5：剔除地图点中外点
        int nmatchesMap = 0;
        for (int i = 0; i < mCurrentFrame.N; i++)
        {
            if (mCurrentFrame.mvpMapPoints[i])
            {
                if (mCurrentFrame.mvbOutlier[i])
                {
                    // 如果优化后判断某个地图点是外点，清除它的所有关系
                    MapPoint *pMP = mCurrentFrame.mvpMapPoints[i];

                    mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
                    mCurrentFrame.mvbOutlier[i] = false;
                    pMP->mbTrackInView = false;
                    pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                    nmatches--;
                }
                else if (mCurrentFrame.mvpMapPoints[i]->Observations() > 0)
                    // 累加成功匹配到的地图点数目
                    nmatchesMap++;
            }
        }
        // cout << "nmatchesMap is : " << nmatchesMap << "and mbOnlyTracking" << mbOnlyTracking  << endl;
        if (mbOnlyTracking)
        {
            // 纯定位模式下：如果成功追踪的地图点非常少,那么这里的mbVO标志就会置位
            mbVO = nmatchesMap < 10;
            return nmatches > 20;
        }

        return nmatchesMap >= 10;
    }

    bool Tracking::TrackLocalMap()
    {
        // We have an estimation of the camera pose and some map points tracked in the frame.
        // We retrieve the local map and try to find matches to points in the local map.

        UpdateLocalMap();

        SearchLocalPoints();

        // Optimize Pose
        Optimizer::PoseOptimization(&mCurrentFrame);
        mnMatchesInliers = 0;

        // Update MapPoints Statistics
        for (int i = 0; i < mCurrentFrame.N; i++)
        {
            if (mCurrentFrame.mvpMapPoints[i])
            {
                if (!mCurrentFrame.mvbOutlier[i])
                {
                    mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                    if (!mbOnlyTracking)
                    {
                        if (mCurrentFrame.mvpMapPoints[i]->Observations() > 0)
                            mnMatchesInliers++;
                    }
                    else
                        mnMatchesInliers++;
                }
                else if (mSensor == System::STEREO)
                    mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
            }
        }

        // Decide if the tracking was succesful
        // More restrictive if there was a relocalization recently
        if (mCurrentFrame.mnId < mnLastRelocFrameId + mMaxFrames && mnMatchesInliers < 50)
            return false;

        if (mnMatchesInliers < 30)
            return false;
        else
            return true;
    }

    bool Tracking::NeedNewKeyFrame()
    {
        if (mbOnlyTracking)
            return false;

        // If Local Mapping is freezed by a Loop Closure do not insert keyframes
        if (mpLocalMapper->isStopped() || mpLocalMapper->stopRequested())
            return false;

        const int nKFs = mpMap->KeyFramesInMap();

        // Do not insert keyframes if not enough frames have passed from last relocalisation
        if (mCurrentFrame.mnId < mnLastRelocFrameId + mMaxFrames && nKFs > mMaxFrames)
            return false;

        // Tracked MapPoints in the reference keyframe
        int nMinObs = 3;
        if (nKFs <= 2)
            nMinObs = 2;
        int nRefMatches = mpReferenceKF->TrackedMapPoints(nMinObs);

        // Local Mapping accept keyframes?
        bool bLocalMappingIdle = mpLocalMapper->AcceptKeyFrames();

        // Check how many "close" points are being tracked and how many could be potentially created.
        int nNonTrackedClose = 0;
        int nTrackedClose = 0;
        if (mSensor != System::MONOCULAR)
        {
            for (int i = 0; i < mCurrentFrame.N; i++)
            {
                if (mCurrentFrame.mvDepth[i] > 0 && mCurrentFrame.mvDepth[i] < mThDepth)
                {
                    if (mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i])
                        nTrackedClose++;
                    else
                        nNonTrackedClose++;
                }
            }
        }

        bool bNeedToInsertClose = (nTrackedClose < 100) && (nNonTrackedClose > 70);

        // Thresholds
        float thRefRatio = 0.75f;
        if (nKFs < 2)
            thRefRatio = 0.4f;

        if (mSensor == System::MONOCULAR)
            thRefRatio = 0.9f;

        // Condition 1a: More than "MaxFrames" have passed from last keyframe insertion
        const bool c1a = mCurrentFrame.mnId >= mnLastKeyFrameId + mMaxFrames;
        // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
        const bool c1b = (mCurrentFrame.mnId >= mnLastKeyFrameId + mMinFrames && bLocalMappingIdle);
        // Condition 1c: tracking is weak
        const bool c1c = mSensor != System::MONOCULAR && (mnMatchesInliers < nRefMatches * 0.25 || bNeedToInsertClose);
        // Condition 2: Few tracked points compared to reference keyframe. Lots of visual odometry compared to map matches.
        const bool c2 = ((mnMatchesInliers < nRefMatches * thRefRatio || bNeedToInsertClose) && mnMatchesInliers > 15);

        if ((c1a || c1b || c1c) && c2)
        {
            // If the mapping accepts keyframes, insert keyframe.
            // Otherwise send a signal to interrupt BA
            if (bLocalMappingIdle)
            {
                return true;
            }
            else
            {
                mpLocalMapper->InterruptBA();
                if (mSensor != System::MONOCULAR)
                {
                    if (mpLocalMapper->KeyframesInQueue() < 3)
                        return true;
                    else
                        return false;
                }
                else
                    return false;
            }
        }
        else
            return false;
    }

    void Tracking::CreateNewKeyFrame()
    {
        if (!mpLocalMapper->SetNotStop(true))
            return;

        KeyFrame *pKF = new KeyFrame(mCurrentFrame, mpMap, mpKeyFrameDB);

        mpReferenceKF = pKF;
        mCurrentFrame.mpReferenceKF = pKF;

        if (mSensor != System::MONOCULAR)
        {
            mCurrentFrame.UpdatePoseMatrices();

            // We sort points by the measured depth by the stereo/RGBD sensor.
            // We create all those MapPoints whose depth < mThDepth.
            // If there are less than 100 close points we create the 100 closest.
            vector<pair<float, int>> vDepthIdx;
            vDepthIdx.reserve(mCurrentFrame.N);
            for (int i = 0; i < mCurrentFrame.N; i++)
            {
                float z = mCurrentFrame.mvDepth[i];
                if (z > 0)
                {
                    vDepthIdx.push_back(make_pair(z, i));
                }
            }

            if (!vDepthIdx.empty())
            {
                sort(vDepthIdx.begin(), vDepthIdx.end());

                int nPoints = 0;
                for (size_t j = 0; j < vDepthIdx.size(); j++)
                {
                    int i = vDepthIdx[j].second;

                    bool bCreateNew = false;

                    MapPoint *pMP = mCurrentFrame.mvpMapPoints[i];
                    if (!pMP)
                        bCreateNew = true;
                    else if (pMP->Observations() < 1)
                    {
                        bCreateNew = true;
                        mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
                    }

                    if (bCreateNew)
                    {
                        cv::Mat x3D = mCurrentFrame.UnprojectStereo(i);
                        MapPoint *pNewMP = new MapPoint(x3D, pKF, mpMap);
                        pNewMP->AddObservation(pKF, i);
                        pKF->AddMapPoint(pNewMP, i);
                        pNewMP->ComputeDistinctiveDescriptors();
                        pNewMP->UpdateNormalAndDepth();
                        mpMap->AddMapPoint(pNewMP);

                        mCurrentFrame.mvpMapPoints[i] = pNewMP;
                        nPoints++;
                    }
                    else
                    {
                        nPoints++;
                    }

                    if (vDepthIdx[j].first > mThDepth && nPoints > 100)
                        break;
                }
            }
        }

        mpLocalMapper->InsertKeyFrame(pKF);

        mpLocalMapper->SetNotStop(false);

        mnLastKeyFrameId = mCurrentFrame.mnId;
        mpLastKeyFrame = pKF;
    }

    void Tracking::SearchLocalPoints()
    {
        // Do not search map points already matched
        for (vector<MapPoint *>::iterator vit = mCurrentFrame.mvpMapPoints.begin(), vend = mCurrentFrame.mvpMapPoints.end(); vit != vend; vit++)
        {
            MapPoint *pMP = *vit;
            if (pMP)
            {
                if (pMP->isBad())
                {
                    *vit = static_cast<MapPoint *>(NULL);
                }
                else
                {
                    pMP->IncreaseVisible();
                    pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                    pMP->mbTrackInView = false;
                }
            }
        }

        int nToMatch = 0;

        // Project points in frame and check its visibility
        for (vector<MapPoint *>::iterator vit = mvpLocalMapPoints.begin(), vend = mvpLocalMapPoints.end(); vit != vend; vit++)
        {
            MapPoint *pMP = *vit;
            if (pMP->mnLastFrameSeen == mCurrentFrame.mnId)
                continue;
            if (pMP->isBad())
                continue;
            // Project (this fills MapPoint variables for matching)
            if (mCurrentFrame.isInFrustum(pMP, 0.5))
            {
                pMP->IncreaseVisible();
                nToMatch++;
            }
        }

        if (nToMatch > 0)
        {
            ORBmatcher matcher(0.8);
            int th = 1;
            if (mSensor == System::RGBD)
                th = 3;
            // If the camera has been relocalised recently, perform a coarser search
            if (mCurrentFrame.mnId < mnLastRelocFrameId + 2)
                th = 5;
            matcher.SearchByProjection(mCurrentFrame, mvpLocalMapPoints, th);
        }
    }

    void Tracking::UpdateLocalMap()
    {
        // This is for visualization
        mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

        // Update
        UpdateLocalKeyFrames();
        UpdateLocalPoints();
    }

    void Tracking::UpdateLocalPoints()
    {
        mvpLocalMapPoints.clear();

        for (vector<KeyFrame *>::const_iterator itKF = mvpLocalKeyFrames.begin(), itEndKF = mvpLocalKeyFrames.end(); itKF != itEndKF; itKF++)
        {
            KeyFrame *pKF = *itKF;
            const vector<MapPoint *> vpMPs = pKF->GetMapPointMatches();

            for (vector<MapPoint *>::const_iterator itMP = vpMPs.begin(), itEndMP = vpMPs.end(); itMP != itEndMP; itMP++)
            {
                MapPoint *pMP = *itMP;
                if (!pMP)
                    continue;
                if (pMP->mnTrackReferenceForFrame == mCurrentFrame.mnId)
                    continue;
                if (!pMP->isBad())
                {
                    mvpLocalMapPoints.push_back(pMP);
                    pMP->mnTrackReferenceForFrame = mCurrentFrame.mnId;
                }
            }
        }
    }

    void Tracking::UpdateLocalKeyFrames()
    {
        // Each map point vote for the keyframes in which it has been observed
        map<KeyFrame *, int> keyframeCounter;
        for (int i = 0; i < mCurrentFrame.N; i++)
        {
            if (mCurrentFrame.mvpMapPoints[i])
            {
                MapPoint *pMP = mCurrentFrame.mvpMapPoints[i];
                if (!pMP->isBad())
                {
                    const map<KeyFrame *, size_t> observations = pMP->GetObservations();
                    for (map<KeyFrame *, size_t>::const_iterator it = observations.begin(), itend = observations.end(); it != itend; it++)
                        keyframeCounter[it->first]++;
                }
                else
                {
                    mCurrentFrame.mvpMapPoints[i] = NULL;
                }
            }
        }

        if (keyframeCounter.empty())
            return;

        int max = 0;
        KeyFrame *pKFmax = static_cast<KeyFrame *>(NULL);

        mvpLocalKeyFrames.clear();
        mvpLocalKeyFrames.reserve(3 * keyframeCounter.size());

        // All keyframes that observe a map point are included in the local map. Also check which keyframe shares most points
        for (map<KeyFrame *, int>::const_iterator it = keyframeCounter.begin(), itEnd = keyframeCounter.end(); it != itEnd; it++)
        {
            KeyFrame *pKF = it->first;

            if (pKF->isBad())
                continue;

            if (it->second > max)
            {
                max = it->second;
                pKFmax = pKF;
            }

            mvpLocalKeyFrames.push_back(it->first);
            pKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
        }

        // Include also some not-already-included keyframes that are neighbors to already-included keyframes
        for (vector<KeyFrame *>::const_iterator itKF = mvpLocalKeyFrames.begin(), itEndKF = mvpLocalKeyFrames.end(); itKF != itEndKF; itKF++)
        {
            // Limit the number of keyframes
            if (mvpLocalKeyFrames.size() > 80)
                break;

            KeyFrame *pKF = *itKF;

            const vector<KeyFrame *> vNeighs = pKF->GetBestCovisibilityKeyFrames(10);

            for (vector<KeyFrame *>::const_iterator itNeighKF = vNeighs.begin(), itEndNeighKF = vNeighs.end(); itNeighKF != itEndNeighKF; itNeighKF++)
            {
                KeyFrame *pNeighKF = *itNeighKF;
                if (!pNeighKF->isBad())
                {
                    if (pNeighKF->mnTrackReferenceForFrame != mCurrentFrame.mnId)
                    {
                        mvpLocalKeyFrames.push_back(pNeighKF);
                        pNeighKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
                        break;
                    }
                }
            }

            const set<KeyFrame *> spChilds = pKF->GetChilds();
            for (set<KeyFrame *>::const_iterator sit = spChilds.begin(), send = spChilds.end(); sit != send; sit++)
            {
                KeyFrame *pChildKF = *sit;
                if (!pChildKF->isBad())
                {
                    if (pChildKF->mnTrackReferenceForFrame != mCurrentFrame.mnId)
                    {
                        mvpLocalKeyFrames.push_back(pChildKF);
                        pChildKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
                        break;
                    }
                }
            }

            KeyFrame *pParent = pKF->GetParent();
            if (pParent)
            {
                if (pParent->mnTrackReferenceForFrame != mCurrentFrame.mnId)
                {
                    mvpLocalKeyFrames.push_back(pParent);
                    pParent->mnTrackReferenceForFrame = mCurrentFrame.mnId;
                    break;
                }
            }
        }

        if (pKFmax)
        {
            mpReferenceKF = pKFmax;
            mCurrentFrame.mpReferenceKF = mpReferenceKF;
        }
    }

    // NOTE 重定位Tracking::Relocalization()
    bool Tracking::Relocalization()
    {
        cout << colouredString("RELOCALIZATION", YELLOW, BOLD) << "\n";
        // Compute Bag of Words Vector
        // STEP RELOCALIZATION 1：计算当前帧特征点的词袋向量
        mCurrentFrame.ComputeBoW();

        // Relocalization is performed when tracking is lost
        // Track Lost: Query KeyFrame Database for keyframe candidates for relocalization
        // STEP RELOCALIZATION 2: 用词袋找到与当前帧相似的候选关键帧
        vector<KeyFrame *> vpCandidateKFs = mpKeyFrameDB->DetectRelocalizationCandidates(&mCurrentFrame);
        cout << "vpCandidateKFs.size() is : " << vpCandidateKFs.size() << endl;
        if (vpCandidateKFs.empty())
            return false;
        // NOTE 定义 nKFs 为候选关键帧的数量.
        const int nKFs = vpCandidateKFs.size();

        // We perform first an ORB matching with each candidate
        // If enough matches are found we setup a PnP solver
        // NOTE 定义 matcher 最小距离 < 0.75次小距离, 并检查旋转
        ORBmatcher matcher(0.75, true);

        // NOTE 定义vpPnPsolvers: 记录每一个关键帧的solver
        vector<PnPsolver *> vpPnPsolvers;
        vpPnPsolvers.resize(nKFs);

        // NOTE 定义vvpMapPointMatches: 记录每个关键帧和当前帧中特征点的匹配关系
        vector<vector<MapPoint *>> vvpMapPointMatches;
        vvpMapPointMatches.resize(nKFs);

        // NOTE 定义vbDiscarded: 标记放弃的关键帧

        vector<bool> vbDiscarded;
        vbDiscarded.resize(nKFs);
        // NOTE 定义 nCandidates: 记录有效的候选关键帧数目

        int nCandidates = 0;
        // STEP RELOCALIZATION 3: 遍历所有的候选关键帧, 通过词袋进行快速匹配, 用匹配结果初始化PnP Solver.
        for (int i = 0; i < nKFs; i++)
        {
            KeyFrame *pKF = vpCandidateKFs[i];
            if (pKF->isBad()){
                vbDiscarded[i] = true;
                cout << "pKF is bad" << endl;
            }        
            else
            {
                // NOTE 当前帧和候选关键帧用BoW进行快速匹配, 匹配结果记录在vvpMapPointMatches
                int nmatches = matcher.SearchByBoW(pKF, mCurrentFrame, vvpMapPointMatches[i]);
                cout << colouredString("nmatches is : ", YELLOW, BOLD) << nmatches << endl;
                // NOTE 如果当前帧的匹配数小于15, 那么只能放弃这个关键帧.
                if (nmatches < 15)
                {
                    vbDiscarded[i] = true;
                    cout << "nmatches < 15;" << endl;
                    continue;
                }
                else
                {
                    // NOTE 如果匹配数目够用, 用匹配结果初始化EPnPsolver
                    PnPsolver *pSolver = new PnPsolver(mCurrentFrame, vvpMapPointMatches[i]);
                    pSolver->SetRansacParameters(0.99,   // 用于计算RANSAC迭代次数理论值的概率
                                                 10,     // 最小内点数, 但是要注意在程序中实际上是min(给定最小内点数,最小集,内点数理论值),不一定使用这个
                                                 300,    // 最大迭代次数
                                                 4,      // 最小集(求解这个问题在一次采样中所需要采样的最少的点的个数,对于Sim3是3,EPnP是4),参与到最小内点数的确定过程中
                                                 0.5,    // 这个是表示(最小内点数/样本总数);实际上的RANSAC正常退出的时候所需要的最小内点数其实是根据这个量来计算得到的
                                                 5.991); // 自由度为2的卡方检验的阈值,程序中还会根据特征点所在的图层对这个阈值进行缩放

                    vpPnPsolvers[i] = pSolver;
                    nCandidates++;
                }
            }
        }

        // Alternatively perform some iterations of P4P RANSAC
        // Until we found a camera pose supported by enough inliers
        // NOTE 定义 bMatch: 标记是否已经找到相匹配的关键帧
        bool bMatch = false;
        // NOTE 定义 matcher2 最小距离 < 0.9次小距离, 并检查旋转
        ORBmatcher matcher2(0.9, true);
        // STEP RELOCALIZATION 4: 通过一系列操作, 直到找到能够匹配上的关键帧.
        while (nCandidates > 0 && !bMatch)
        {
            // 遍历当前所有的候选关键帧
            for (int i = 0; i < nKFs; i++)
            {
                // 忽略放弃的
                cout << "vbDiscarded[i] is : " << vbDiscarded[i] << endl;
                if (vbDiscarded[i])
                    continue;

                // Perform 5 Ransac Iterations
                // 定义 vbInliers: 记录内点
                vector<bool> vbInliers;
                // 定义 nInliers: 记录内点数目
                int nInliers;
                // 定义 bNoMore: 若赋值为true, 则表示RANSAC已经没有更多的迭代次数可用
                bool bNoMore;
                // STEP RELOCALIZATION 4.1: 通过EPnP算法估计位姿, 迭代5次
                PnPsolver *pSolver = vpPnPsolvers[i];
                cv::Mat Tcw = pSolver->iterate(5, bNoMore, vbInliers, nInliers);
                cout << "Tcw is : " << Tcw << endl;
                // If Ransac reachs max. iterations discard keyframe
                // 若bNoMore为true, 则表示已经超过RANSAC的最大迭代次数, 放弃当前关键帧.

                if (bNoMore)
                {
                    vbDiscarded[i] = true;
                    nCandidates--;
                }

                // If a Camera Pose is computed, optimize
                // STEP RELOCALIZATION 4.2: 若EPnP计算出了位姿, 则对内点进行BA优化.
                if (!Tcw.empty())
                {
                    cout << "Tcw is not empty" << endl;
                    Tcw.copyTo(mCurrentFrame.mTcw);
                    // 定义sFound: 作为EPnP里RANSAC后的内点的集合
                    set<MapPoint *> sFound;
                    // 遍历所有内点
                    const int np = vbInliers.size();

                    for (int j = 0; j < np; j++)
                    {
                        if (vbInliers[j])
                        {
                            mCurrentFrame.mvpMapPoints[j] = vvpMapPointMatches[i][j];
                            sFound.insert(vvpMapPointMatches[i][j]);
                        }
                        else
                            mCurrentFrame.mvpMapPoints[j] = NULL;
                    }
                    // 只优化位姿, 不优化地图点的坐标, 返回的是内点的个数
                    int nGood = Optimizer::PoseOptimization(&mCurrentFrame);
                    // 如果优化之后的内点数目不多, 则进行下一个候选关键帧的处理.
                    if (nGood < 10)
                        continue;
                    // 删除外点对应的地图点
                    for (int io = 0; io < mCurrentFrame.N; io++)
                        if (mCurrentFrame.mvbOutlier[io])
                            mCurrentFrame.mvpMapPoints[io] = static_cast<MapPoint *>(NULL);

                    // If few inliers, search by projection in a coarse window and optimize again
                    // STEP RELOCALIZATION 4.3 若通过词袋匹配的内点较少, 则通过投影的方式对之前未匹配的点进行匹配, 再通过BA进行优化.
                    if (nGood < 50)
                    {
                        cout << "nGood < 50" << endl;
                        int nadditional = matcher2.SearchByProjection(mCurrentFrame, vpCandidateKFs[i], sFound, 10, 100);
                        // 若通过投影方式匹配了足够的特征点, 则再次使用3D-2D pnp BA优化位姿.
                        if (nadditional + nGood >= 50)
                        {
                            nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                            // If many inliers but still not enough, search by projection again in a narrower window
                            // the camera has been already optimized with many points
                            // STEP RELOCALIZATION 4.4: 如果BA后内点数目在[30-50]之间, 则使用更小的搜索窗口, 再执行一遍4.3
                            if (nGood > 30 && nGood < 50)
                            {
                                sFound.clear();
                                for (int ip = 0; ip < mCurrentFrame.N; ip++)
                                    if (mCurrentFrame.mvpMapPoints[ip])
                                        sFound.insert(mCurrentFrame.mvpMapPoints[ip]);
                                nadditional = matcher2.SearchByProjection(mCurrentFrame, vpCandidateKFs[i], sFound, 3, 64);

                                // Final optimization
                                // 如果匹配数目达到了要求, 最后BA优化一下
                                if (nGood + nadditional >= 50)
                                {
                                    nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                                    for (int io = 0; io < mCurrentFrame.N; io++)
                                        if (mCurrentFrame.mvbOutlier[io])
                                            mCurrentFrame.mvpMapPoints[io] = NULL;
                                }
                            }
                        }
                    }

                    // If the pose is supported by enough inliers stop ransacs and continue
                    // 如果对于当前的候选关键帧已经有足够的内点(大于50个)了, 那么就认为重定位成功.
                    if (nGood >= 50)
                    {
                        cout << colouredString("RELOCALIZATION nGood >=50", YELLOW, BOLD) << "\n";
                        bMatch = true;
                        break;
                    }
                }
            }
        }
        cout << "bMatch is : " << bMatch << endl;
        if (!bMatch)
        {
            // 如果只有一个 就直接用这个关键帧的位姿
            if (0) // nKFs == 1
            {
                cv::Mat Tcwbow;
                Tcwbow = vpCandidateKFs[0]->GetPose();
                cout << "Tcwbow is : " << Tcwbow << endl;
                mCurrentFrame.mTcw = vpCandidateKFs[0]->GetPose();
                // return true;
            }
            mCurrentFrame.mTcw = cv::Mat::zeros(0, 0, CV_32F); // set mTcw back to empty if relocation is failed
            cout << colouredString("RELOCALIZATION FAILED", RED, BOLD) << endl;
            return false;
        }
        else
        {
            // 如果匹配上了,说明当前帧重定位成功了(当前帧已经有了自己的位姿)
            // 记录成功重定位帧的id，防止短时间多次重定位
            mnLastRelocFrameId = mCurrentFrame.mnId;
            return true;
        }
        cout << colouredString("RELOCALIZATION DONE", YELLOW, BOLD) << endl;
    }

    void Tracking::Reset()
    {
        mpPublisherThread->RequestStop();

        cout << "System Reseting" << endl;
        while (!mpPublisherThread->isStopped())
        {
            std::this_thread::sleep_for(std::chrono::microseconds(3000));
        }

        // Reset Local Mapping
        cout << "Reseting Local Mapper...";
        mpLocalMapper->RequestReset();
        cout << " done" << endl;

        // Reset Loop Closing
        cout << "Reseting Loop Closing...";
        mpLoopClosing->RequestReset();
        cout << " done" << endl;

        // Clear BoW Database
        cout << "Reseting Database...";
        mpKeyFrameDB->clear();
        cout << " done" << endl;

        // Clear Map (this erase MapPoints and KeyFrames)
        mpMap->clear();

        KeyFrame::nNextId = 0;
        Frame::nNextId = 0;
        mState = NO_IMAGES_YET;

        if (mpInitializer)
        {
            delete mpInitializer;
            mpInitializer = static_cast<Initializer *>(NULL);
        }

        mlRelativeFramePoses.clear();
        mlpReferences.clear();
        mlFrameTimes.clear();
        mlbLost.clear();

        mpPublisherThread->Release();
    }

    void Tracking::ChangeCalibration(const string &strSettingPath)
    {
        cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
        float fx = fSettings["Camera.fx"];
        float fy = fSettings["Camera.fy"];
        float cx = fSettings["Camera.cx"];
        float cy = fSettings["Camera.cy"];

        cv::Mat K = cv::Mat::eye(3, 3, CV_32F);
        K.at<float>(0, 0) = fx;
        K.at<float>(1, 1) = fy;
        K.at<float>(0, 2) = cx;
        K.at<float>(1, 2) = cy;
        K.copyTo(mK);

        cv::Mat DistCoef(4, 1, CV_32F);
        DistCoef.at<float>(0) = fSettings["Camera.k1"];
        DistCoef.at<float>(1) = fSettings["Camera.k2"];
        DistCoef.at<float>(2) = fSettings["Camera.p1"];
        DistCoef.at<float>(3) = fSettings["Camera.p2"];
        const float k3 = fSettings["Camera.k3"];
        if (k3 != 0)
        {
            DistCoef.resize(5);
            DistCoef.at<float>(4) = k3;
        }
        DistCoef.copyTo(mDistCoef);

        mbf = fSettings["Camera.bf"];

        Frame::mbInitialComputations = true;
    }

    void Tracking::InformOnlyTracking(const bool &flag)
    {
        mbOnlyTracking = flag;
    }

} // namespace ORB_SLAM
