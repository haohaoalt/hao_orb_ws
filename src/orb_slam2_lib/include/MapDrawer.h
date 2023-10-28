/*
 * @Author: https://github.com/haohaoalt
 * @Date: 2023-10-26 15:10:38
 * @LastEditors: haohaoalt haohaoalt@163.com
 * @LastEditTime: 2023-10-27 22:17:34
 * @FilePath: /hao_orb_ws/src/orb_slam2_lib/include/MapDrawer.h
 * @Description: 
 * Copyright (c) 2023 by haohaoalt@163.com, All Rights Reserved. 
 */
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

#ifndef MAPDRAWER_H
#define MAPDRAWER_H

#include"Map.h"
#include"MapPoint.h"
#include"KeyFrame.h"
#include "IMapPublisher.h"
#include<pangolin/pangolin.h>

namespace ORB_SLAM2
{

class MapDrawer : public IMapPublisher
{
public:
    /**
     * @brief 构造函数
     * @param[in] pMap              地图句柄
     * @param[in] strSettingPath    配置文件的路径
     */
    MapDrawer(Map* pMap, const string &strSettingPath);

    // 地图句柄
    Map *mpMap;

    /** @brief 绘制地图点 */
    void DrawMapPoints();

    /**
     * @brief 绘制关键帧
     *
     * @param[in] bDrawKF       是否绘制关键帧
     * @param[in] bDrawGraph    是否绘制共视图
     */
    void DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph);

    /**
     * @brief 绘制当前相机
     *
     * @param[in] Twc 相机的位姿矩阵
     */
    void DrawCurrentCamera(pangolin::OpenGlMatrix &Twc);

    void DrawObjStatic();

    void DrawCurrentCamera(pangolin::OpenGlMatrix &Twc);
    /**
     * @brief 设置当前帧的相机位姿
     *
     * @param[in] Tcw 位姿矩阵
     */
    void SetCurrentCameraPose(const cv::Mat &Tcw);
    void SetReferenceKeyFrame(KeyFrame *pKF);
    void GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M);

private:
    // 绘制这些部件的参数
    /// 关键帧-大小
    float mKeyFrameSize;
    // 关键帧-线宽
    float mKeyFrameLineWidth;
    // 共视图的线宽
    float mGraphLineWidth;

    // 地图点的大小
    float mPointSize;

    // 绘制的相机的大小
    float mCameraSize;

    // 绘制相机的线宽
    float mCameraLineWidth;
};

} //namespace ORB_SLAM

#endif // MAPDRAWER_H
