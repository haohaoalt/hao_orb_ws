/*
 * @Author: https://github.com/haohaoalt
 * @Date: 2023-10-19 16:15:18
 * @LastEditors: haohaoalt haohaoalt@163.com
 * @LastEditTime: 2023-10-19 16:21:31
 * @FilePath: /hao_orb_ws/src/orb_slam2_lib/src/Object.cpp
 * @Description: 
 * Copyright (c) 2023 by haohaoalt@163.com, All Rights Reserved. 
 */

#include "Objectyolo.h"
#include "vector"
#include "math.h"
#define PI 3.14159265

namespace ORB_SLAM2 {
    Object::Object() {
        bdynamic_ = false;
        vdetect_parameter = {0,0,0,0};
        ndetect_class = -1;
    }

    Object::Object(vector<double> vdetect_parameter_,int ndetect_class_):
    vdetect_parameter(vdetect_parameter_),ndetect_class(ndetect_class_){}

    Object::~Object(){}

    vector<double> Object::GetDetectParameter(){       
        return vdetect_parameter;
    }

    int Object::GetDetectClass(){
        return ndetect_class;
    }

}