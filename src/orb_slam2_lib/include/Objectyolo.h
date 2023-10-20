/*
 * @Author: https://github.com/haohaoalt
 * @Date: 2023-10-19 16:11:12
 * @LastEditors: haohaoalt haohaoalt@163.com
 * @LastEditTime: 2023-10-19 16:21:17
 * @FilePath: /hao_orb_ws/src/orb_slam2_lib/include/Objectyolo.h
 * @Description: 
 * Copyright (c) 2023 by haohaoalt@163.com, All Rights Reserved. 
 */
#ifndef MYSLAM_OBJECT_YOLO_H
#define MYSLAM_OBJECT_YOLO_H

#include <fstream>
#include <mutex>
#include <set>
#include "MapPoint.h"
using namespace std;

namespace ORB_SLAM2 {
    class MapPoint;
    class Object{
        public:
        Object();
        Object(vector<double> vdetect_parameter_,int ndetect_class_);
        ~Object();

        public:
        enum classname{
            person = 3,
            car = 1
        };

        public:
        vector<double> vdetect_parameter;//检测框的四个参数
        int ndetect_class;
        bool bdynamic_;

        public:

        vector<double> GetDetectParameter();
        int GetDetectClass();

    };
}

#endif 