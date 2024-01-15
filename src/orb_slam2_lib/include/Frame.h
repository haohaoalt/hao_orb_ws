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

#ifndef FRAME_H
#define FRAME_H

#include<vector>
#include <memory>

#include "MapPoint.h"
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"
#include "ORBVocabulary.h"
#include "KeyFrame.h"
#include "ORBextractor.h"

#include <opencv2/opencv.hpp>

namespace ORB_SLAM2
{
#define FRAME_GRID_ROWS 48
#define FRAME_GRID_COLS 64

class MapPoint;
class KeyFrame;

class Frame
{
public:
    /**
     * @brief Construct a new Frame object without parameter.
     *
     */
    Frame();

    // Copy constructor.
    // Copy constructor. 拷贝构造函数
    /**
     * @brief 拷贝构造函数
     * @details 复制构造函数, mLastFrame = Frame(mCurrentFrame) \n
     * 如果不是自定以拷贝函数的话，系统自动生成的拷贝函数对于所有涉及分配内存的操作都将是浅拷贝 \n
     * @param[in] frame 引用
     * @note 另外注意，调用这个函数的时候，这个函数中隐藏的this指针其实是指向目标帧的
     */
    Frame(const Frame &frame);

    // Constructor for stereo cameras.
    /**
     * @brief 为双目相机准备的构造函数
     *
     * @param[in] imLeft            左目图像
     * @param[in] imRight           右目图像
     * @param[in] timeStamp         时间戳
     * @param[in] extractorLeft     左目图像特征点提取器句柄
     * @param[in] extractorRight    右目图像特征点提取器句柄
     * @param[in] voc               ORB字典句柄
     * @param[in] K                 相机内参矩阵
     * @param[in] distCoef          相机去畸变参数
     * @param[in] bf                相机基线长度和焦距的乘积
     * @param[in] thDepth           远点和近点的深度区分阈值
     *
     */
    Frame(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timeStamp, ORBextractor* extractorLeft, ORBextractor* extractorRight, ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth);

    // Constructor for RGB-D cameras.
    Frame(const cv::Mat &imGray, const cv::Mat &imDepth, const double &timeStamp, ORBextractor* extractor,ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth);

    /**
     * @brief 为RGBD相机准备的帧构造函数
     *
     * @param[in] imGray        对RGB图像灰度化之后得到的灰度图像
     * @param[in] imDepth       深度图像
     * @param[in] timeStamp     时间戳
     * @param[in] extractor     特征点提取器句柄
     * @param[in] voc           ORB特征点词典的句柄
     * @param[in] K             相机的内参数矩阵
     * @param[in] distCoef      相机的去畸变参数
     * @param[in] bf            baseline*bf
     * @param[in] thDepth       远点和近点的深度区分阈值
     */
    // Frame(const cv::Mat &imGray,
    //       const cv::Mat &imDepth,
    //       const long double &timeStamp,
    //       ORBextractor *extractor,
    //       ORBVocabulary *voc,
    //       cv::Mat &K,
    //       cv::Mat &distCoef,
    //       const float &bf,
    //       const float &thDepth,
    //       const vector<std::pair<vector<double>, int>> &detect_result);

    // Constructor for Monocular cameras.
    Frame(const cv::Mat &imGray, const double &timeStamp, ORBextractor* extractor,ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth);

    // Extract ORB on the image. 0 for left image and 1 for right image.
    void ExtractORB(int flag, const cv::Mat &im);

    // Compute Bag of Words representation.
    void ComputeBoW();

    // Set the camera pose.
    void SetPose(cv::Mat Tcw);

    // Computes rotation, translation and camera center matrices from the camera pose.
    void UpdatePoseMatrices();

    // Returns the camera center.
    inline cv::Mat GetCameraCenter(){
        return mOw.clone();
    }

    // Returns inverse of rotation
    inline cv::Mat GetRotationInverse(){
        return mRwc.clone();
    }

    // Check if a MapPoint is in the frustum of the camera
    // and fill variables of the MapPoint to be used by the tracking
    bool isInFrustum(MapPoint* pMP, float viewingCosLimit);

    // Compute the cell of a keypoint (return false if outside the grid)
    bool PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY);

    vector<size_t> GetFeaturesInArea(const float &x, const float  &y, const float  &r, const int minLevel=-1, const int maxLevel=-1) const;

    // Search a match for each keypoint in the left image to a keypoint in the right image.
    // If there is a match, depth is computed and the right coordinate associated to the left keypoint is stored.
    void ComputeStereoMatches();

    // Associate a "right" coordinate to a keypoint if there is valid depth in the depthmap.
    void ComputeStereoFromRGBD(const cv::Mat &imDepth);

    // Backprojects a keypoint (if stereo/depth info available) into 3D world coordinates.
    /**
     * @brief 当某个特征点的深度信息或者双目信息有效时,将它反投影到三维世界坐标系中
     *
     * @param[in] i     特征点的ID
     * @return cv::Mat  反投影后得到的特征点的反投影点的世界坐标
     */
    cv::Mat UnprojectStereo(const int &i);

    // 反投影物体中心点
    // void UnprojectObjectCenter(const int &box_id, const cv::Mat &imDepth);
    //*************
    // 判断特征点是否在物体框内
    // bool IsInBox(const int &i, int &box_id);
    // 判断特征点是否在动态物体(人)内
    // bool IsInDynamic(const int &i);
    // bool IsInStatic(const int &i);

public:
    // Vocabulary used for relocalization.
    ORBVocabulary* mpORBvocabulary;

    // Feature extractor. The right is used only in the stereo case.
    ORBextractor* mpORBextractorLeft, *mpORBextractorRight;

    // Frame timestamp.
    double mTimeStamp;

    // Calibration matrix and OpenCV distortion parameters.
    cv::Mat mK;
    static float fx;        // x轴方向焦距
    static float fy;        // y轴方向焦距
    static float cx;        // x轴方向光心偏移
    static float cy;        // y轴方向光心偏移
    static float invfx;     // x轴方向焦距的逆
    static float invfy;     // x轴方向焦距的逆
    // TODO 目测是opencv提供的图像去畸变参数矩阵的，但是其具体组成未知
    /// 去畸变参数
    cv::Mat mDistCoef;

    // Stereo baseline multiplied by fx.
    float mbf;

    // Stereo baseline in meters.
    float mb;

    // Threshold close/far points. Close points are inserted from 1 view.
    // Far points are inserted as in the monocular case from 2 views.
    float mThDepth;

    // Number of KeyPoints.
    int N;

    // Vector of keypoints (original for visualization) and undistorted (actually used by the system).
    // In the stereo case, mvKeysUn is redundant as images must be rectified.
    // In the RGB-D case, RGB images can be distorted.
    // 原始左图像提取出的特征点（未校正） 原始右图像提取出的特征点（未校正）
    std::vector<cv::KeyPoint> mvKeys, mvKeysRight;
    // 校正mvKeys后的特征点
    std::vector<cv::KeyPoint> mvKeysUn;

    ///@note 之所以对于双目摄像头只保存左图像矫正后的特征点,是因为对于双目摄像头,一般得到的图像都是矫正好的,这里再矫正一次有些多余.\n
    /// 校正操作是在帧的构造函数中进行的。

    // Corresponding stereo coordinate and depth for each keypoint.
    // "Monocular" keypoints have a negative value.
    // 对于双目，mvuRight存储了左目像素点在右目中的对应点的横坐标 （因为纵坐标是一样的）
    // mvDepth对应的深度
    // 单目摄像头，这两个容器中存的都是-1

    ///@note 对于单目摄像头，这两个容器中存的都是-1
    /// 对于双目相机,存储左目像素点在右目中的对应点的横坐标 （因为纵坐标是一样的）

    // Corresponding stereo coordinate and depth for each keypoint.
    // "Monocular" keypoints have a negative value.
    std::vector<float> mvuRight;
    std::vector<float> mvDepth;

    // Bag of Words Vector structures.
    DBoW2::BowVector mBowVec;
    DBoW2::FeatureVector mFeatVec;

    // ORB descriptor, each row associated to a keypoint.
    cv::Mat mDescriptors, mDescriptorsRight;

    // MapPoints associated to keypoints, NULL pointer if no association.
    std::vector<MapPoint*> mvpMapPoints;

    // Flag to identify outlier associations.
    std::vector<bool> mvbOutlier;

    // Keypoints are assigned to cells in a grid to reduce matching complexity when projecting MapPoints.
    static float mfGridElementWidthInv;
    static float mfGridElementHeightInv;
    std::vector<std::size_t> mGrid[FRAME_GRID_COLS][FRAME_GRID_ROWS];

    // Camera pose.
    cv::Mat mTcw; //< 相机姿态 世界坐标系到相机坐标坐标系的变换矩阵,是我们常规理解中的相机位姿

    // Current and Next Frame id.
    // 类的静态成员变量，这些变量则是在整个系统开始执行的时候被初始化的——它在全局区被初始化
    static long unsigned int nNextId;
    long unsigned int mnId;

    // Reference Keyframe.
    // 普通帧与自己共视程度最高的关键帧作为参考关键帧
    KeyFrame* mpReferenceKF;

    // Scale pyramid info.
    int mnScaleLevels;
    float mfScaleFactor;
    float mfLogScaleFactor;
    vector<float> mvScaleFactors;
    vector<float> mvInvScaleFactors;
    vector<float> mvLevelSigma2;
    vector<float> mvInvLevelSigma2;

    // hayden matches_in_box in Frame.h
    // cv::Mat imDepth;
    //  vector<std::shared_ptr<Object>> objects_cur_detect_;//用于光流的物体
    // vector<std::shared_ptr<Object>> objects_cur_; // 存储帧中的object
    // map<int, int> matches_out_box;                // 在物体框外的匹配点

    // map<int, std::pair<int, int>> matches_in_box; // 在bounding_box框内的匹配
                                                  // CurrentFrame.matches_in_box.emplace(bestIdx2, make_pair(i, box_id));
                                                  // bestIdx2是当前帧的候选匹配点中的最佳匹配特征点的序列,i是前一帧的序列,boxid是所在物体的序列
    // vector<vector<int>> points_in_box; // 框内的特征点.内vector储存一系列点,外vector代表一系列物体
    // map<int, std::pair<int, int>> matches_in_dynamic;

    // vector<bool> vbInDynamic_mvKeys; // 位于动态物体区域内的特征点(未做动态检验)
    // vector<bool> vbInStatic_mvKeys;
    // std::vector<cv::KeyPoint> mvKeys_after;
    
    // vector<cv::Mat> vMcenter_static;

    // Undistorted Image Bounds (computed once).
    /**
     * @name 用于确定画格子时的边界
     * @note（未校正图像的边界，只需要计算一次，因为是类的静态成员变量）
     * @{
     */
    static float mnMinX;
    static float mnMaxX;
    static float mnMinY;
    static float mnMaxY;

    static bool mbInitialComputations;


private:

    // Undistort keypoints given OpenCV distortion parameters.
    // Only for the RGB-D case. Stereo must be already rectified!
    // (called in the constructor).
    void UndistortKeyPoints();

    // Computes image bounds for the undistorted image (called in the constructor).
    void ComputeImageBounds(const cv::Mat &imLeft);

    // Assign keypoints to the grid for speed up feature matching (called in the constructor).
    void AssignFeaturesToGrid();

    // Rotation, translation and camera center
    cv::Mat mRcw;
    cv::Mat mtcw;
    cv::Mat mRwc;
    cv::Mat mOw; //==mtwc
};

}// namespace ORB_SLAM

#endif // FRAME_H
