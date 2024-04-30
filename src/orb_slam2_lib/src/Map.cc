#include "Map.h"

#include<mutex>

namespace ORB_SLAM2
{
    // 构造函数,地图点中最大关键帧id归0
    Map::Map() : mnMaxKFid(0), mnBigChangeIdx(0)
    {
}
/*
 * @brief Insert KeyFrame in the map
 * @param pKF KeyFrame
 */
// 在地图中插入关键帧,同时更新关键帧的最大id
void Map::AddKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexMap);
    mspKeyFrames.insert(pKF);
    if(pKF->mnId>mnMaxKFid)
        mnMaxKFid=pKF->mnId;
}
/*
 * @brief Insert MapPoint in the map
 * @param pMP MapPoint
 */
// 向地图中插入地图点
void Map::AddMapPoint(MapPoint *pMP)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapPoints.insert(pMP);
}

// NOTE 向地图中插入ObjCenters
// void Map::AddObjCenters(MapPoint *pMP)
// {
//     unique_lock<mutex> lock(mMutexMap);
//     mspObjCenters.insert(pMP);
//     // cout << "mspObjCenters size is : " << mspObjCenters.size() << endl;
// }

// void Map::EraseAllObjCenters()
// {
//     unique_lock<mutex> lock(mMutexMap);

//     mspObjCenters.erase(mspObjCenters.begin(), mspObjCenters.end());
// }

// vector<MapPoint *> Map::GetAllObjCenters()
// {
//     unique_lock<mutex> lock(mMutexMap);
//     return vector<MapPoint *>(mspObjCenters.begin(), mspObjCenters.end());
// }
/**
 * @brief 从地图中删除地图点,但是其实这个地图点所占用的内存空间并没有被释放
 *
 * @param[in] pMP
 */
void Map::EraseMapPoint(MapPoint *pMP)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapPoints.erase(pMP);
    // 下面是作者加入的注释. 实际上只是从std::set中删除了地图点的指针, 原先地图点
    // 占用的内存区域并没有得到释放
    //  TODO: This only erase the pointer.
    //  Delete the MapPoint
}

/**
 * @brief Erase KeyFrame from the map
 * @param pKF KeyFrame
 */
void Map::EraseKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexMap);
    mspKeyFrames.erase(pKF);

    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

/*
 * @brief 设置参考MapPoints，将用于DrawMapPoints函数画图
 * @param vpMPs Local MapPoints
 */
// 设置参考地图点用于绘图显示局部地图点（红色）
void Map::SetReferenceMapPoints(const vector<MapPoint *> &vpMPs)
{
    unique_lock<mutex> lock(mMutexMap);
    mvpReferenceMapPoints = vpMPs;
}

void Map::InformNewBigChange()
{
    unique_lock<mutex> lock(mMutexMap);
    mnBigChangeIdx++;
}

int Map::GetLastBigChangeIdx()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnBigChangeIdx;
}

vector<KeyFrame*> Map::GetAllKeyFrames()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<KeyFrame*>(mspKeyFrames.begin(),mspKeyFrames.end());
}

vector<MapPoint*> Map::GetAllMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<MapPoint*>(mspMapPoints.begin(),mspMapPoints.end());
}

long unsigned int Map::MapPointsInMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mspMapPoints.size();
}

long unsigned int Map::KeyFramesInMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mspKeyFrames.size();
}

vector<MapPoint*> Map::GetReferenceMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return mvpReferenceMapPoints;
}

long unsigned int Map::GetMaxKFid()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnMaxKFid;
}

void Map::clear()
{
    for(set<MapPoint*>::iterator sit=mspMapPoints.begin(), send=mspMapPoints.end(); sit!=send; sit++)
        delete *sit;

    for(set<KeyFrame*>::iterator sit=mspKeyFrames.begin(), send=mspKeyFrames.end(); sit!=send; sit++)
        delete *sit;

    mspMapPoints.clear();
    mspKeyFrames.clear();
    mnMaxKFid = 0;
    mvpReferenceMapPoints.clear();
    mvpKeyFrameOrigins.clear();
}

// NOTE Map::serialize
template<class Archive>
void Map::serialize(Archive &ar, const unsigned int version)
{
    unique_lock<mutex> lock_MapUpdate(mMutexMapUpdate);
    unique_lock<mutex> lock_Map(mMutexMap);
    ar & mspMapPoints;
    ar & mvpKeyFrameOrigins;
    ar & mspKeyFrames;
    ar & mvpReferenceMapPoints;
    ar & mnMaxKFid & mnBigChangeIdx;
}
template void Map::serialize(boost::archive::binary_iarchive&, const unsigned int);
template void Map::serialize(boost::archive::binary_oarchive&, const unsigned int);

} //namespace ORB_SLAM
