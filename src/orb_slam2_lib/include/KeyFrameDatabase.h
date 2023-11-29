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

#ifndef KEYFRAMEDATABASE_H
#define KEYFRAMEDATABASE_H

#include <vector>
#include <list>
#include <set>

#include "KeyFrame.h"
#include "Frame.h"
#include "ORBVocabulary.h"
#include "Serialize.h"

#include<mutex>

namespace ORB_SLAM2
{

class KeyFrame;
class Frame;

//hayden： 关键帧数据库
class KeyFrameDatabase
{
public:
    //hayden： voc词袋模型的字典
    KeyFrameDatabase(ORBVocabulary *voc);
    //KeyFrameDatabase(const ORBVocabulary &voc);

   void add(KeyFrame* pKF);

   void erase(KeyFrame* pKF);

   void clear();

   // Loop Detection
   //hayden：在闭环检测中找到与该关键帧可能闭环的关键帧;
   //param pKF需要闭环的关键帧 minScore相似性分数最低要求， return可能闭环关键帧
   std::vector<KeyFrame *> DetectLoopCandidates(KeyFrame* pKF, float minScore);

   // Relocalization
   std::vector<KeyFrame*> DetectRelocalizationCandidates(Frame* F);


public:
    //Default constructor for serialization
    KeyFrameDatabase() {}
    void SetORBvocabulary(ORBVocabulary *porbv) {mpVoc=porbv;}
private:
    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive &ar, const unsigned int version);


protected:

  // Associated vocabulary
  //const ORBVocabulary* mpVoc;
  ORBVocabulary* mpVoc;
  // Inverted file
  std::vector<list<KeyFrame*> > mvInvertedFile;

  // Mutex
  std::mutex mMutex;
};

} //namespace ORB_SLAM

#endif
