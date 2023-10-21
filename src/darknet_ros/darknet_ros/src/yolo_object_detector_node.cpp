/*
 * @Author: https://github.com/haohaoalt
 * @Date: 2023-10-16 15:26:56
 * @LastEditors: haohaoalt haohaoalt@163.com
 * @LastEditTime: 2023-10-16 17:56:39
 * @FilePath: /1016_ws/src/darknet_ros/darknet_ros/src/yolo_object_detector_node.cpp
 * @Description: 
 * Copyright (c) 2023 by haohaoalt@163.com, All Rights Reserved. 
 */
/*
 * yolo_object_detector_node.cpp
 *
 *  Created on: Dec 19, 2016
 *      Author: Marko Bjelonic
 *   Institute: ETH Zurich, Robotic Systems Lab
 */

#include <ros/ros.h>
#include <darknet_ros/YoloObjectDetector.hpp>

int main(int argc, char** argv) {
  ros::init(argc, argv, "darknet_ros1");
  ros::NodeHandle nodeHandle("~");
  darknet_ros::YoloObjectDetector yoloObjectDetector(nodeHandle);

  ros::spin();
  return 0;
}
