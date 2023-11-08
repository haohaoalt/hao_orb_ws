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

#include "Frame.h"
#include "Objectyolo.h"
#include "Converter.h"
#include "ORBmatcher.h"
#include <thread>

namespace ORB_SLAM2
{

    long unsigned int Frame::nNextId = 0;
    bool Frame::mbInitialComputations = true;
    float Frame::cx, Frame::cy, Frame::fx, Frame::fy, Frame::invfx, Frame::invfy;
    float Frame::mnMinX, Frame::mnMinY, Frame::mnMaxX, Frame::mnMaxY;
    float Frame::mfGridElementWidthInv, Frame::mfGridElementHeightInv;

    Frame::Frame()
    {
    }

    // Copy Constructor
    /** @details 另外注意，调用这个函数的时候，这个函数中隐藏的this指针其实是指向目标帧的
     */
    Frame::Frame(const Frame &frame)
        : mpORBvocabulary(frame.mpORBvocabulary),
          mpORBextractorLeft(frame.mpORBextractorLeft),
          mpORBextractorRight(frame.mpORBextractorRight),
          mTimeStamp(frame.mTimeStamp),
          mK(frame.mK.clone()),               // 深拷贝
          mDistCoef(frame.mDistCoef.clone()), // 深拷贝
          mbf(frame.mbf),
          mb(frame.mb),
          mThDepth(frame.mThDepth),
          N(frame.N), mvKeys(frame.mvKeys),
          mvKeysRight(frame.mvKeysRight),
          mvKeysUn(frame.mvKeysUn),
          mvuRight(frame.mvuRight),
          mvDepth(frame.mvDepth),
          mBowVec(frame.mBowVec),
          mFeatVec(frame.mFeatVec),
          mDescriptors(frame.mDescriptors.clone()),
          mDescriptorsRight(frame.mDescriptorsRight.clone()),
          mvpMapPoints(frame.mvpMapPoints),
          mvbOutlier(frame.mvbOutlier),
          mnId(frame.mnId),
          mpReferenceKF(frame.mpReferenceKF),
          mnScaleLevels(frame.mnScaleLevels),
          mfScaleFactor(frame.mfScaleFactor),
          mfLogScaleFactor(frame.mfLogScaleFactor),
          mvScaleFactors(frame.mvScaleFactors),
          mvInvScaleFactors(frame.mvInvScaleFactors),
          mvLevelSigma2(frame.mvLevelSigma2),
          mvInvLevelSigma2(frame.mvInvLevelSigma2),
          objects_cur_(frame.objects_cur_) // 深拷贝
    {
        for (int i = 0; i < FRAME_GRID_COLS; i++)
            for (int j = 0; j < FRAME_GRID_ROWS; j++)
                mGrid[i][j] = frame.mGrid[i][j];

        if (!frame.mTcw.empty())
            SetPose(frame.mTcw);
    }

    //TODO Frame need to do something

    Frame::Frame(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timeStamp, ORBextractor *extractorLeft, ORBextractor *extractorRight, ORBVocabulary *voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth)
        : mpORBvocabulary(voc), mpORBextractorLeft(extractorLeft), mpORBextractorRight(extractorRight), mTimeStamp(timeStamp), mK(K.clone()), mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth),
          mpReferenceKF(static_cast<KeyFrame *>(NULL))
    {

        // Frame ID
        // Step 1 双目 帧的ID 自增
        mnId = nNextId++;

        // Scale Level Info
        mnScaleLevels = mpORBextractorLeft->GetLevels();
        mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
        mfLogScaleFactor = log(mfScaleFactor);
        mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
        mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
        mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
        mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

        // ORB extraction
        thread threadLeft(&Frame::ExtractORB, this, 0, imLeft);
        thread threadRight(&Frame::ExtractORB, this, 1, imRight);
        threadLeft.join();
        threadRight.join();

        // 获取特征点的个数
        N = mvKeys.size();

        // 如果这一帧没有能够提取出特征点，那么就直接返回了
        if (mvKeys.empty())
            return;


        UndistortKeyPoints();

        ComputeStereoMatches();

        // 初始化本帧的地图点
        mvpMapPoints = vector<MapPoint *>(N, static_cast<MapPoint *>(NULL));
        // 记录地图点是否为外点，初始化均为外点false
        mvbOutlier = vector<bool>(N, false);

        // This is done only for the first Frame (or after a change in the calibration)
        if (mbInitialComputations)
        {
            ComputeImageBounds(imLeft);

            mfGridElementWidthInv = static_cast<float>(FRAME_GRID_COLS) / (mnMaxX - mnMinX);
            mfGridElementHeightInv = static_cast<float>(FRAME_GRID_ROWS) / (mnMaxY - mnMinY);

            fx = K.at<float>(0, 0);
            fy = K.at<float>(1, 1);
            cx = K.at<float>(0, 2);
            cy = K.at<float>(1, 2);
            invfx = 1.0f / fx;
            invfy = 1.0f / fy;

            mbInitialComputations = false;
        }

        mb = mbf / fx;

        AssignFeaturesToGrid();
    }

    Frame::Frame(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timeStamp, ORBextractor *extractorLeft, ORBextractor *extractorRight, ORBVocabulary *voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth, const vector<std::pair<vector<double>, int>> &detect_result)
        : mpORBvocabulary(voc), mpORBextractorLeft(extractorLeft), mpORBextractorRight(extractorRight), mTimeStamp(timeStamp), mK(K.clone()), mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth),
          mpReferenceKF(static_cast<KeyFrame *>(NULL))
    {
        vector<double> vobject_box;
        int nobject_class;
        for (int k = 0; k < detect_result.size(); ++k)
        {
            vobject_box = detect_result[k].first; // 把所有物体检测存在了vobject_box和vobject_class中
            nobject_class = detect_result[k].second;
            std::shared_ptr<Object> obj = make_shared<Object>(vobject_box, nobject_class);
            objects_cur_.push_back(obj);
        }
        // Frame ID
        // Step 1 双目 帧的ID 自增
        mnId = nNextId++;

        // Scale Level Info
        mnScaleLevels = mpORBextractorLeft->GetLevels();
        mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
        mfLogScaleFactor = log(mfScaleFactor);
        mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
        mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
        mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
        mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

        // ORB extraction
        thread threadLeft(&Frame::ExtractORB, this, 0, imLeft);
        thread threadRight(&Frame::ExtractORB, this, 1, imRight);
        threadLeft.join();
        threadRight.join();

        // 获取特征点的个数
        N = mvKeys.size();

        // 如果这一帧没有能够提取出特征点，那么就直接返回了
        if (mvKeys.empty())
            return;

        //*******直接检验特征点是否在动态物体的框内
        // cout << "mvKeys.size() is " << mvKeys.size() << endl;
        for (int k = 0; k < mvKeys.size(); ++k)
        {
            if (IsInDynamic(k) == true && IsInStatic(k) == false)
            { // 在动态物体框内,但不在静态物体框内
                vbInDynamic_mvKeys.push_back(true);
                // cout << "true" << endl;
                mvKeys[k] = cv::KeyPoint(-1, -1, -1);
            }
            else
            {
                vbInDynamic_mvKeys.push_back(false);
                // cout << "false" << endl;
            }
        }

        UndistortKeyPoints();

        ComputeStereoMatches();

        // 初始化本帧的地图点
        mvpMapPoints = vector<MapPoint *>(N, static_cast<MapPoint *>(NULL));
        // 记录地图点是否为外点，初始化均为外点false
        mvbOutlier = vector<bool>(N, false);

        // This is done only for the first Frame (or after a change in the calibration)
        if (mbInitialComputations)
        {
            ComputeImageBounds(imLeft);

            mfGridElementWidthInv = static_cast<float>(FRAME_GRID_COLS) / (mnMaxX - mnMinX);
            mfGridElementHeightInv = static_cast<float>(FRAME_GRID_ROWS) / (mnMaxY - mnMinY);

            fx = K.at<float>(0, 0);
            fy = K.at<float>(1, 1);
            cx = K.at<float>(0, 2);
            cy = K.at<float>(1, 2);
            invfx = 1.0f / fx;
            invfy = 1.0f / fy;

            mbInitialComputations = false;
        }

        mb = mbf / fx;

        AssignFeaturesToGrid();
    }
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
    Frame::Frame(const cv::Mat &imGray, const cv::Mat &imDepth, const double &timeStamp, ORBextractor *extractor, ORBVocabulary *voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth)
        : mpORBvocabulary(voc), mpORBextractorLeft(extractor), mpORBextractorRight(static_cast<ORBextractor *>(NULL)),
          mTimeStamp(timeStamp), mK(K.clone()), mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth)
    {
        // Frame ID
        mnId = nNextId++;

        // Scale Level Info
        mnScaleLevels = mpORBextractorLeft->GetLevels();
        mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
        mfLogScaleFactor = log(mfScaleFactor);
        mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
        mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
        mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
        mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

        // ORB extraction
        ExtractORB(0, imGray);

        N = mvKeys.size();

        if (mvKeys.empty())
            return;

        UndistortKeyPoints();

        ComputeStereoFromRGBD(imDepth);

        mvpMapPoints = vector<MapPoint *>(N, static_cast<MapPoint *>(NULL));
        mvbOutlier = vector<bool>(N, false);

        // This is done only for the first Frame (or after a change in the calibration)
        if (mbInitialComputations)
        {
            ComputeImageBounds(imGray);

            mfGridElementWidthInv = static_cast<float>(FRAME_GRID_COLS) / static_cast<float>(mnMaxX - mnMinX);
            mfGridElementHeightInv = static_cast<float>(FRAME_GRID_ROWS) / static_cast<float>(mnMaxY - mnMinY);

            fx = K.at<float>(0, 0);
            fy = K.at<float>(1, 1);
            cx = K.at<float>(0, 2);
            cy = K.at<float>(1, 2);
            invfx = 1.0f / fx;
            invfy = 1.0f / fy;

            mbInitialComputations = false;
        }

        mb = mbf / fx;

        AssignFeaturesToGrid();
    }

    // hayden: yolo Frame 使用不同参数重载
    Frame::Frame(const cv::Mat &imGray,
                 const cv::Mat &imDepth,
                 const long double &timeStamp,
                 ORBextractor *extractor,
                 ORBVocabulary *voc,
                 cv::Mat &K,
                 cv::Mat &distCoef,
                 const float &bf,
                 const float &thDepth,
                 const vector<std::pair<vector<double>, int>> &detect_result)
        : mpORBvocabulary(voc), mpORBextractorLeft(extractor), mpORBextractorRight(static_cast<ORBextractor *>(NULL)),
          mTimeStamp(timeStamp), mK(K.clone()), mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth)
    {
        //*****step0.5 把检测框添加进Frame中,包含了所有的物体.
        vector<double> vobject_box;
        int nobject_class;

        for (int k = 0; k < detect_result.size(); ++k)
        {
            vobject_box = detect_result[k].first; // 把所有物体检测存在了vobject_box和vobject_class中
            nobject_class = detect_result[k].second;
            std::shared_ptr<Object> obj = make_shared<Object>(vobject_box, nobject_class);
            objects_cur_.push_back(obj);
        }

        //*****

        // Step 1 RGBD 帧的ID 自增
        mnId = nNextId++;

        // Step 2 计算图像金字塔的参数
        // 获取图像金字塔的层数
        mnScaleLevels = mpORBextractorLeft->GetLevels();
        // 获取每层的缩放因子
        mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
        // 计算每层缩放因子的自然对数
        mfLogScaleFactor = log(mfScaleFactor);
        // 获取各层图像的缩放因子
        mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
        // 获取各层图像的缩放因子的倒数
        mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
        // TODO 也是获取这个不知道有什么实际含义的sigma^2
        mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
        // 计算上面获取的sigma^2的倒数
        mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

        /** 3. 提取彩色图像(其实现在已经灰度化成为灰度图像了)的特征点 \n Frame::ExtractORB() */

        // ORB extraction
        // Step 3 对图像进行提取特征点, 第一个参数0-左图，1-右图
        ExtractORB(0, imGray); //!!!提取特征点!!!

        // 获取特征点的个数
        N = mvKeys.size();

        // 如果这一帧没有能够提取出特征点，那么就直接返回了
        if (mvKeys.empty())
            return;

        //*******直接检验特征点是否在动态物体的框内
        // cout << "mvKeys.size() is " << mvKeys.size() << endl;
        for (int k = 0; k < mvKeys.size(); ++k)
        {
            if (IsInDynamic(k) == true && IsInStatic(k) == false)
            { // 在动态物体框内,但不在静态物体框内
                vbInDynamic_mvKeys.push_back(true);
                // cout << "true" << endl;
                mvKeys[k] = cv::KeyPoint(-1, -1, -1);
            }
            else
            {
                vbInDynamic_mvKeys.push_back(false);
                // cout << "false" << endl;
            }
        }

        // Step 4 用OpenCV的矫正函数、内参对提取到的特征点进行矫正
        UndistortKeyPoints();

        //********************
        // Step 5 获取图像的深度，并且根据这个深度推算其右图中匹配的特征点的视差
        ComputeStereoFromRGBD(imDepth);

        // 初始化本帧的地图点
        mvpMapPoints = vector<MapPoint *>(N, static_cast<MapPoint *>(NULL));
        // 记录地图点是否为外点，初始化均为外点false
        mvbOutlier = vector<bool>(N, false);

        // This is done only for the first Frame (or after a change in the calibration)
        //  Step 5 计算去畸变后图像边界，将特征点分配到网格中。这个过程一般是在第一帧或者是相机标定参数发生变化之后进行
        if (mbInitialComputations)
        {
            // 计算去畸变后图像的边界
            ComputeImageBounds(imGray);

            // 表示一个图像像素相当于多少个图像网格列（宽）
            mfGridElementWidthInv = static_cast<float>(FRAME_GRID_COLS) / static_cast<float>(mnMaxX - mnMinX);
            // 表示一个图像像素相当于多少个图像网格行（高）
            mfGridElementHeightInv = static_cast<float>(FRAME_GRID_ROWS) / static_cast<float>(mnMaxY - mnMinY);

            // 给类的静态成员变量复制
            fx = K.at<float>(0, 0);
            fy = K.at<float>(1, 1);
            cx = K.at<float>(0, 2);
            cy = K.at<float>(1, 2);
            // 猜测是因为这种除法计算需要的时间略长，所以这里直接存储了这个中间计算结果
            invfx = 1.0f / fx;
            invfy = 1.0f / fy;

            // 特殊的初始化过程完成，标志复位
            mbInitialComputations = false;
        }

        // 计算假想的基线长度 baseline= mbf/fx
        // 后面要对从RGBD相机输入的特征点,结合相机基线长度,焦距,以及点的深度等信息来计算其在假想的"右侧图像"上的匹配点
        mb = mbf / fx;

        // 将特征点分配到图像网格中
        AssignFeaturesToGrid();
    }

    Frame::Frame(const cv::Mat &imGray, const double &timeStamp, ORBextractor *extractor, ORBVocabulary *voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth)
        : mpORBvocabulary(voc), mpORBextractorLeft(extractor), mpORBextractorRight(static_cast<ORBextractor *>(NULL)),
          mTimeStamp(timeStamp), mK(K.clone()), mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth)
    {
        // Frame ID
        mnId = nNextId++;

        // Scale Level Info
        mnScaleLevels = mpORBextractorLeft->GetLevels();
        mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
        mfLogScaleFactor = log(mfScaleFactor);
        mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
        mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
        mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
        mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

        // ORB extraction
        ExtractORB(0, imGray);

        N = mvKeys.size();

        if (mvKeys.empty())
            return;

        UndistortKeyPoints();

        // Set no stereo information
        mvuRight = vector<float>(N, -1);
        mvDepth = vector<float>(N, -1);

        mvpMapPoints = vector<MapPoint *>(N, static_cast<MapPoint *>(NULL));
        mvbOutlier = vector<bool>(N, false);

        // This is done only for the first Frame (or after a change in the calibration)
        if (mbInitialComputations)
        {
            ComputeImageBounds(imGray);

            mfGridElementWidthInv = static_cast<float>(FRAME_GRID_COLS) / static_cast<float>(mnMaxX - mnMinX);
            mfGridElementHeightInv = static_cast<float>(FRAME_GRID_ROWS) / static_cast<float>(mnMaxY - mnMinY);

            fx = K.at<float>(0, 0);
            fy = K.at<float>(1, 1);
            cx = K.at<float>(0, 2);
            cy = K.at<float>(1, 2);
            invfx = 1.0f / fx;
            invfy = 1.0f / fy;

            mbInitialComputations = false;
        }

        mb = mbf / fx;

        AssignFeaturesToGrid();
    }

    void Frame::AssignFeaturesToGrid()
    {
        int nReserve = 0.5f * N / (FRAME_GRID_COLS * FRAME_GRID_ROWS);
        for (unsigned int i = 0; i < FRAME_GRID_COLS; i++)
            for (unsigned int j = 0; j < FRAME_GRID_ROWS; j++)
                mGrid[i][j].reserve(nReserve);

        for (int i = 0; i < N; i++)
        {
            const cv::KeyPoint &kp = mvKeysUn[i];

            int nGridPosX, nGridPosY;
            if (PosInGrid(kp, nGridPosX, nGridPosY))
                mGrid[nGridPosX][nGridPosY].push_back(i);
        }
    }

    void Frame::ExtractORB(int flag, const cv::Mat &im)
    {
        if (flag == 0)
            (*mpORBextractorLeft)(im, cv::Mat(), mvKeys, mDescriptors);
        else
            (*mpORBextractorRight)(im, cv::Mat(), mvKeysRight, mDescriptorsRight);
    }

    void Frame::SetPose(cv::Mat Tcw)
    {
        mTcw = Tcw.clone();
        UpdatePoseMatrices();
    }

    void Frame::UpdatePoseMatrices()
    {
        mRcw = mTcw.rowRange(0, 3).colRange(0, 3);
        mRwc = mRcw.t();
        mtcw = mTcw.rowRange(0, 3).col(3);
        mOw = -mRcw.t() * mtcw;
    }

    bool Frame::isInFrustum(MapPoint *pMP, float viewingCosLimit)
    {
        pMP->mbTrackInView = false;

        // 3D in absolute coordinates
        cv::Mat P = pMP->GetWorldPos();

        // 3D in camera coordinates
        const cv::Mat Pc = mRcw * P + mtcw;
        const float &PcX = Pc.at<float>(0);
        const float &PcY = Pc.at<float>(1);
        const float &PcZ = Pc.at<float>(2);

        // Check positive depth
        if (PcZ < 0.0f)
            return false;

        // Project in image and check it is not outside
        const float invz = 1.0f / PcZ;
        const float u = fx * PcX * invz + cx;
        const float v = fy * PcY * invz + cy;

        if (u < mnMinX || u > mnMaxX)
            return false;
        if (v < mnMinY || v > mnMaxY)
            return false;

        // Check distance is in the scale invariance region of the MapPoint
        const float maxDistance = pMP->GetMaxDistanceInvariance();
        const float minDistance = pMP->GetMinDistanceInvariance();
        const cv::Mat PO = P - mOw;
        const float dist = cv::norm(PO);

        if (dist < minDistance || dist > maxDistance)
            return false;

        // Check viewing angle
        cv::Mat Pn = pMP->GetNormal();

        const float viewCos = PO.dot(Pn) / dist;

        if (viewCos < viewingCosLimit)
            return false;

        // Predict scale in the image
        const int nPredictedLevel = pMP->PredictScale(dist, this);

        // Data used by the tracking
        pMP->mbTrackInView = true;
        pMP->mTrackProjX = u;
        pMP->mTrackProjXR = u - mbf * invz;
        pMP->mTrackProjY = v;
        pMP->mnTrackScaleLevel = nPredictedLevel;
        pMP->mTrackViewCos = viewCos;

        return true;
    }

    vector<size_t> Frame::GetFeaturesInArea(const float &x, const float &y, const float &r, const int minLevel, const int maxLevel) const
    {
        vector<size_t> vIndices;
        vIndices.reserve(N);

        const int nMinCellX = max(0, (int)floor((x - mnMinX - r) * mfGridElementWidthInv));
        if (nMinCellX >= FRAME_GRID_COLS)
            return vIndices;

        const int nMaxCellX = min((int)FRAME_GRID_COLS - 1, (int)ceil((x - mnMinX + r) * mfGridElementWidthInv));
        if (nMaxCellX < 0)
            return vIndices;

        const int nMinCellY = max(0, (int)floor((y - mnMinY - r) * mfGridElementHeightInv));
        if (nMinCellY >= FRAME_GRID_ROWS)
            return vIndices;

        const int nMaxCellY = min((int)FRAME_GRID_ROWS - 1, (int)ceil((y - mnMinY + r) * mfGridElementHeightInv));
        if (nMaxCellY < 0)
            return vIndices;

        const bool bCheckLevels = (minLevel > 0) || (maxLevel >= 0);

        for (int ix = nMinCellX; ix <= nMaxCellX; ix++)
        {
            for (int iy = nMinCellY; iy <= nMaxCellY; iy++)
            {
                const vector<size_t> vCell = mGrid[ix][iy];
                if (vCell.empty())
                    continue;

                for (size_t j = 0, jend = vCell.size(); j < jend; j++)
                {
                    const cv::KeyPoint &kpUn = mvKeysUn[vCell[j]];
                    if (bCheckLevels)
                    {
                        if (kpUn.octave < minLevel)
                            continue;
                        if (maxLevel >= 0)
                            if (kpUn.octave > maxLevel)
                                continue;
                    }

                    const float distx = kpUn.pt.x - x;
                    const float disty = kpUn.pt.y - y;

                    if (fabs(distx) < r && fabs(disty) < r)
                        vIndices.push_back(vCell[j]);
                }
            }
        }

        return vIndices;
    }

    bool Frame::PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY)
    {
        posX = round((kp.pt.x - mnMinX) * mfGridElementWidthInv);
        posY = round((kp.pt.y - mnMinY) * mfGridElementHeightInv);

        // Keypoint's coordinates are undistorted, which could cause to go out of the image
        if (posX < 0 || posX >= FRAME_GRID_COLS || posY < 0 || posY >= FRAME_GRID_ROWS)
            return false;

        return true;
    }

    void Frame::ComputeBoW()
    {
        if (mBowVec.empty())
        {
            vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(mDescriptors);
            mpORBvocabulary->transform(vCurrentDesc, mBowVec, mFeatVec, 4);
        }
    }

    void Frame::UndistortKeyPoints()
    {
        if (mDistCoef.at<float>(0) == 0.0)
        {
            mvKeysUn = mvKeys;
            return;
        }

        // Fill matrix with points
        cv::Mat mat(N, 2, CV_32F);
        for (int i = 0; i < N; i++)
        {
            mat.at<float>(i, 0) = mvKeys[i].pt.x;
            mat.at<float>(i, 1) = mvKeys[i].pt.y;
        }

        // Undistort points
        mat = mat.reshape(2);
        cv::undistortPoints(mat, mat, mK, mDistCoef, cv::Mat(), mK);
        mat = mat.reshape(1);

        // Fill undistorted keypoint vector
        mvKeysUn.resize(N);
        for (int i = 0; i < N; i++)
        {
            cv::KeyPoint kp = mvKeys[i];
            kp.pt.x = mat.at<float>(i, 0);
            kp.pt.y = mat.at<float>(i, 1);
            mvKeysUn[i] = kp;
        }
    }

    void Frame::ComputeImageBounds(const cv::Mat &imLeft)
    {
        if (mDistCoef.at<float>(0) != 0.0)
        {
            cv::Mat mat(4, 2, CV_32F);
            mat.at<float>(0, 0) = 0.0;
            mat.at<float>(0, 1) = 0.0;
            mat.at<float>(1, 0) = imLeft.cols;
            mat.at<float>(1, 1) = 0.0;
            mat.at<float>(2, 0) = 0.0;
            mat.at<float>(2, 1) = imLeft.rows;
            mat.at<float>(3, 0) = imLeft.cols;
            mat.at<float>(3, 1) = imLeft.rows;

            // Undistort corners
            mat = mat.reshape(2);
            cv::undistortPoints(mat, mat, mK, mDistCoef, cv::Mat(), mK);
            mat = mat.reshape(1);

            mnMinX = min(mat.at<float>(0, 0), mat.at<float>(2, 0));
            mnMaxX = max(mat.at<float>(1, 0), mat.at<float>(3, 0));
            mnMinY = min(mat.at<float>(0, 1), mat.at<float>(1, 1));
            mnMaxY = max(mat.at<float>(2, 1), mat.at<float>(3, 1));
        }
        else
        {
            mnMinX = 0.0f;
            mnMaxX = imLeft.cols;
            mnMinY = 0.0f;
            mnMaxY = imLeft.rows;
        }
    }

    void Frame::ComputeStereoMatches()
    {
        mvuRight = vector<float>(N, -1.0f);
        mvDepth = vector<float>(N, -1.0f);

        const int thOrbDist = (ORBmatcher::TH_HIGH + ORBmatcher::TH_LOW) / 2;

        const int nRows = mpORBextractorLeft->mvImagePyramid[0].rows;

        // Assign keypoints to row table
        vector<vector<size_t>> vRowIndices(nRows, vector<size_t>());

        for (int i = 0; i < nRows; i++)
            vRowIndices[i].reserve(200);

        const int Nr = mvKeysRight.size();

        for (int iR = 0; iR < Nr; iR++)
        {
            const cv::KeyPoint &kp = mvKeysRight[iR];
            const float &kpY = kp.pt.y;
            const float r = 2.0f * mvScaleFactors[mvKeysRight[iR].octave];
            const int maxr = ceil(kpY + r);
            const int minr = floor(kpY - r);

            for (int yi = minr; yi <= maxr; yi++)
                vRowIndices[yi].push_back(iR);
        }

        // Set limits for search
        const float minZ = mb;
        const float minD = 0;
        const float maxD = mbf / minZ;

        // For each left keypoint search a match in the right image
        vector<pair<int, int>> vDistIdx;
        vDistIdx.reserve(N);

        for (int iL = 0; iL < N; iL++)
        {
            const cv::KeyPoint &kpL = mvKeys[iL];
            const int &levelL = kpL.octave;
            const float &vL = kpL.pt.y;
            const float &uL = kpL.pt.x;

            const vector<size_t> &vCandidates = vRowIndices[vL];

            if (vCandidates.empty())
                continue;

            const float minU = uL - maxD;
            const float maxU = uL - minD;

            if (maxU < 0)
                continue;

            int bestDist = ORBmatcher::TH_HIGH;
            size_t bestIdxR = 0;

            const cv::Mat &dL = mDescriptors.row(iL);

            // Compare descriptor to right keypoints
            for (size_t iC = 0; iC < vCandidates.size(); iC++)
            {
                const size_t iR = vCandidates[iC];
                const cv::KeyPoint &kpR = mvKeysRight[iR];

                if (kpR.octave < levelL - 1 || kpR.octave > levelL + 1)
                    continue;

                const float &uR = kpR.pt.x;

                if (uR >= minU && uR <= maxU)
                {
                    const cv::Mat &dR = mDescriptorsRight.row(iR);
                    const int dist = ORBmatcher::DescriptorDistance(dL, dR);

                    if (dist < bestDist)
                    {
                        bestDist = dist;
                        bestIdxR = iR;
                    }
                }
            }

            // Subpixel match by correlation
            if (bestDist < thOrbDist)
            {
                // coordinates in image pyramid at keypoint scale
                const float uR0 = mvKeysRight[bestIdxR].pt.x;
                const float scaleFactor = mvInvScaleFactors[kpL.octave];
                const float scaleduL = round(kpL.pt.x * scaleFactor);
                const float scaledvL = round(kpL.pt.y * scaleFactor);
                const float scaleduR0 = round(uR0 * scaleFactor);

                // sliding window search
                const int w = 5;
                cv::Mat IL = mpORBextractorLeft->mvImagePyramid[kpL.octave].rowRange(scaledvL - w, scaledvL + w + 1).colRange(scaleduL - w, scaleduL + w + 1);
                IL.convertTo(IL, CV_32F);
                IL = IL - IL.at<float>(w, w) * cv::Mat::ones(IL.rows, IL.cols, CV_32F);

                int bestDist = INT_MAX;
                int bestincR = 0;
                const int L = 5;
                vector<float> vDists;
                vDists.resize(2 * L + 1);

                const float iniu = scaleduR0 + L - w;
                const float endu = scaleduR0 + L + w + 1;
                if (iniu < 0 || endu >= mpORBextractorRight->mvImagePyramid[kpL.octave].cols)
                    continue;

                for (int incR = -L; incR <= +L; incR++)
                {
                    cv::Mat IR = mpORBextractorRight->mvImagePyramid[kpL.octave].rowRange(scaledvL - w, scaledvL + w + 1).colRange(scaleduR0 + incR - w, scaleduR0 + incR + w + 1);
                    IR.convertTo(IR, CV_32F);
                    IR = IR - IR.at<float>(w, w) * cv::Mat::ones(IR.rows, IR.cols, CV_32F);

                    float dist = cv::norm(IL, IR, cv::NORM_L1);
                    if (dist < bestDist)
                    {
                        bestDist = dist;
                        bestincR = incR;
                    }

                    vDists[L + incR] = dist;
                }

                if (bestincR == -L || bestincR == L)
                    continue;

                // Sub-pixel match (Parabola fitting)
                const float dist1 = vDists[L + bestincR - 1];
                const float dist2 = vDists[L + bestincR];
                const float dist3 = vDists[L + bestincR + 1];

                const float deltaR = (dist1 - dist3) / (2.0f * (dist1 + dist3 - 2.0f * dist2));

                if (deltaR < -1 || deltaR > 1)
                    continue;

                // Re-scaled coordinate
                float bestuR = mvScaleFactors[kpL.octave] * ((float)scaleduR0 + (float)bestincR + deltaR);

                float disparity = (uL - bestuR);

                if (disparity >= minD && disparity < maxD)
                {
                    if (disparity <= 0)
                    {
                        disparity = 0.01;
                        bestuR = uL - 0.01;
                    }
                    mvDepth[iL] = mbf / disparity;
                    mvuRight[iL] = bestuR;
                    vDistIdx.push_back(pair<int, int>(bestDist, iL));
                }
            }
        }

        sort(vDistIdx.begin(), vDistIdx.end());
        const float median = vDistIdx[vDistIdx.size() / 2].first;
        const float thDist = 1.5f * 1.4f * median;

        for (int i = vDistIdx.size() - 1; i >= 0; i--)
        {
            if (vDistIdx[i].first < thDist)
                break;
            else
            {
                mvuRight[vDistIdx[i].second] = -1;
                mvDepth[vDistIdx[i].second] = -1;
            }
        }
    }

    // 计算RGBD图像的立体深度信息
    void Frame::ComputeStereoFromRGBD(const cv::Mat &imDepth)
    {
        /** 主要步骤如下:.对于彩色图像中的每一个特征点:<ul>  */
        // mvDepth直接由depth图像读取`
        // 这里是初始化这两个存储“右图”匹配特征点横坐标和存储特征点深度值的vector
        mvuRight = vector<float>(N, -1);
        mvDepth = vector<float>(N, -1);

        // 开始遍历彩色图像中的所有特征点
        for (int i = 0; i < N; i++)
        {
            /** <li> 从<b>未矫正的特征点</b>提供的坐标来读取深度图像拿到这个点的深度数据 </li> */
            // 获取校正前和校正后的特征点
            const cv::KeyPoint &kp = mvKeys[i];
            const cv::KeyPoint &kpU = mvKeysUn[i];

            // 获取其横纵坐标，注意 NOTICE 是校正前的特征点的
            const float &v = kp.pt.y;
            const float &u = kp.pt.x;
            // 从深度图像中获取这个特征点对应的深度点
            // NOTE 从这里看对深度图像进行去畸变处理是没有必要的,我们依旧可以直接通过未矫正的特征点的坐标来直接拿到深度数据
            if(v < 0 || u < 0) continue;
            const float d = imDepth.at<float>(v, u);

            //
            /** <li> 如果获取到的深度点合法(d>0), 那么就保存这个特征点的深度,并且计算出等效的\在假想的右图中该特征点所匹配的特征点的横坐标 </li>
             * \n 这个横坐标的计算是 x-mbf/d
             * \n 其中的x使用的是<b>矫正后的</b>特征点的图像坐标
             */
            if (d > 0)
            {
                // 那么就保存这个点的深度
                mvDepth[i] = d;
                // 根据这个点的深度计算出等效的、在假想的右图中的该特征点的横坐标
                // TODO 话说为什么要计算这个嘞，计算出来之后有什么用?可能是为了保持计算一致
                mvuRight[i] = kpU.pt.x - mbf / d;
            } // 如果获取到的深度点合法
        }     // 开始遍历彩色图像中的所有特征点
    }

    cv::Mat Frame::UnprojectStereo(const int &i)
    {
        // KeyFrame::UnprojectStereo
        // 貌似这里普通帧的反投影函数操作过程和关键帧的反投影函数操作过程有一些不同：
        // mvDepth是在ComputeStereoMatches函数中求取的
        // TODO 验证下面的这些内容. 虽然现在我感觉是理解错了,但是不确定;如果确定是真的理解错了,那么就删除下面的内容
        // mvDepth对应的校正前的特征点，可这里却是对校正后特征点反投影
        // KeyFrame::UnprojectStereo中是对校正前的特征点mvKeys反投影
        // 在ComputeStereoMatches函数中应该对校正后的特征点求深度？？ (wubo???)
        // NOTE 不过我记得好像上面的ComputeStereoMatches函数就是对于双目相机设计的，而双目相机的图像默认都是经过了校正的啊

        /** 步骤如下: <ul> */

        /** <li> 获取这个特征点的深度（这里的深度可能是通过双目视差得出的，也可能是直接通过深度图像的出来的） </li> */
        const float z = mvDepth[i];
        /** <li> 判断这个深度是否合法 </li> <ul> */
        // （其实这里也可以不再进行判断，因为在计算或者生成这个深度的时候都是经过检查了的_不行,RGBD的不是）
        if (z > 0)
        {
            /** <li> 如果合法,就利用<b></b>矫正后的特征点的坐标 Frame::mvKeysUn 和相机的内参数,通过反投影和位姿变换得到空间点的坐标 </li> */
            // 获取像素坐标，注意这里是矫正后的特征点的坐标
            const float u = mvKeysUn[i].pt.x;
            const float v = mvKeysUn[i].pt.y;
            // 计算在当前相机坐标系下的坐标
            const float x = (u - cx) * z * invfx;
            const float y = (v - cy) * z * invfy;
            // 生成三维点（在当前相机坐标系下）
            cv::Mat x3Dc = (cv::Mat_<float>(3, 1) << x, y, z);
            // 然后计算这个点在世界坐标系下的坐标，这里是对的，但是公式还是要斟酌一下。首先变换成在没有旋转的相机坐标系下，最后考虑相机坐标系相对于世界坐标系的平移
            return mRwc * x3Dc + mOw;
        }
        else
            /** <li> 如果深度值不合法，那么就返回一个空矩阵,表示计算失败 </li> */
            return cv::Mat();
    }

    // 反投影物体中心点
    void Frame::UnprojectObjectCenter(const int &box_id, const cv::Mat &imDepth)
    {
        vector<float> obj_center;
        cv::Mat x3Dc = (cv::Mat_<float>(3, 1) << -1, -1, -1);
        cv::Mat matCenter3d;

        float u_center = (objects_cur_[box_id]->vdetect_parameter[0] + objects_cur_[box_id]->vdetect_parameter[2]) / 2.0;
        float v_center = (objects_cur_[box_id]->vdetect_parameter[1] + objects_cur_[box_id]->vdetect_parameter[3]) / 2.0;
        float z_center = imDepth.at<float>(v_center, u_center);

        if (z_center > 0)
        {
            const float x = (u_center - cx) * z_center * invfx;
            const float y = (v_center - cy) * z_center * invfy;

            // 生成三维点（在当前相机坐标系下）
            x3Dc = (cv::Mat_<float>(3, 1) << x, y, z_center);

            matCenter3d = mRwc * x3Dc + mOw;
            // cout << "matCenter3d:  " << endl << matCenter3d << endl;
            vMcenter_static.push_back(matCenter3d);
            // return matCenter3d;
        }
        // else
        //     return cv::Mat();

        // 计算在当前相机坐标系下的坐标 guiyihua multi z(where is my chinese input??)
    }

    
    bool Frame::IsInBox(const int &i, int &box_id)
    { // 判断kp是否在bounding_box_(txt读入的数据)内. objects_cur_存储帧中的object,由frame.cc引入
        const cv::KeyPoint &kp = mvKeysUn[i];
        float kp_u = kp.pt.x;
        float kp_v = kp.pt.y;
        bool in_box = false;
        for (int k = 0; k < objects_cur_.size(); ++k)
        { // 遍历所有物体
            vector<double> box = objects_cur_[k]->vdetect_parameter;
            double left = box[0];
            double top = box[1];
            double right = box[2];
            double bottom = box[3];
            if (kp_u > left - 2 && kp_u < right + 2 && kp_v > top - 2 && kp_v < bottom + 2)
            {
                in_box = true;
                box_id = k;
                break;
            }
        }
        return in_box;
    }

    bool Frame::IsInDynamic(const int &i)
    { // 判断kp是否在bounding_box_(txt读入的数据)内. objects_cur_存储帧中的object,由frame.cc引入
        const cv::KeyPoint &kp = mvKeys[i];
        float kp_u = kp.pt.x;
        float kp_v = kp.pt.y;
        bool in_dynamic = false;

        for (int k = 0; k < objects_cur_.size(); ++k)
        { // 遍历所有物体
            int obj_class = objects_cur_[k]->ndetect_class;

            if (obj_class == 3)
            {
                vector<double> box = objects_cur_[k]->vdetect_parameter; // 获取物体框
                double left = box[0];
                double top = box[1];
                double right = box[2];
                double bottom = box[3];

                if (kp_u > left - 2 && kp_u < right + 2 && kp_v > top - 2 && kp_v < bottom + 2)
                {
                    in_dynamic = true;
                }
            }
        }
        return in_dynamic;
    }

    bool Frame::IsInStatic(const int &i)
    { // 判断kp是否在bounding_box_(txt读入的数据)内. objects_cur_存储帧中的object,由frame.cc引入
        const cv::KeyPoint &kp = mvKeys[i];
        float kp_u = kp.pt.x;
        float kp_v = kp.pt.y;
        bool in_static = false;

        for (int k = 0; k < objects_cur_.size(); ++k)
        { // 遍历所有物体
            int obj_class = objects_cur_[k]->ndetect_class;

            if (obj_class == 1)
            {
                vector<double> box = objects_cur_[k]->vdetect_parameter; // 获取物体框
                double left = box[0];
                double top = box[1];
                double right = box[2];
                double bottom = box[3];

                if (kp_u > left - 2 && kp_u < right + 2 && kp_v > top - 2 && kp_v < bottom + 2)
                {
                    in_static = true;
                }
            }
        }
        return in_static;
    }

} // namespace ORB_SLAM
