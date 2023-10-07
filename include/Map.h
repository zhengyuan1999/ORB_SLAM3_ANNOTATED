/**
 * This file is part of ORB-SLAM3
 *
 * Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 * Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 *
 * ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
 * the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with ORB-SLAM3.
 * If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef MAP_H
#define MAP_H

#include "MapPoint.h"
#include "KeyFrame.h"

#include <set>
#include <pangolin/pangolin.h>
#include <mutex>

#include <boost/serialization/base_object.hpp>

namespace ORB_SLAM3
{

class MapPoint;
class KeyFrame;
class Atlas;
class KeyFrameDatabase;

class Map
{
    friend class boost::serialization::access;

    template <class Archive>
    void serialize(Archive &ar, const unsigned int version)
    {
        ar & mnId;
        ar & mnInitKFid;
        ar & mnMaxKFid;
        ar & mnBigChangeIdx;

        // Save/load a set structure, the set structure is broken in libboost 1.58 for ubuntu 16.04, a vector is serializated
        // ar & mspKeyFrames;
        // ar & mspMapPoints;
        ar & mvpBackupKeyFrames;
        ar & mvpBackupMapPoints;

        ar & mvBackupKeyFrameOriginsId;

        ar & mnBackupKFinitialID;
        ar & mnBackupKFlowerID;

        ar & mbImuInitialized;
        ar & mbIsInertial;
        ar & mbIMU_BA1;
        ar & mbIMU_BA2;
    }

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Map();
    Map(int initKFid);
    ~Map();

    // 向本地图中插入关键帧
    void AddKeyFrame(KeyFrame *pKF);

    // mspMapPoints.insert(pMP);
    void AddMapPoint(MapPoint *pMP);

    // mspMapPoints.erase(pMP);
    void EraseMapPoint(MapPoint *pMP);

    // 从本地图中删除关键帧
    void EraseKeyFrame(KeyFrame *pKF);

    // mvpReferenceMapPoints = vpMPs;
    void SetReferenceMapPoints(const std::vector<MapPoint *> &vpMPs);

    // mnBigChangeIdx++;
    void InformNewBigChange();

    // return mnBigChangeIdx;
    int GetLastBigChangeIdx();

    // return vector<KeyFrame *>(mspKeyFrames.begin(), mspKeyFrames.end());
    std::vector<KeyFrame *> GetAllKeyFrames();

    // return vector<MapPoint *>(mspMapPoints.begin(), mspMapPoints.end());
    std::vector<MapPoint *> GetAllMapPoints();

    // return mvpReferenceMapPoints;
    std::vector<MapPoint *> GetReferenceMapPoints();

    // return mspMapPoints.size();
    long unsigned int MapPointsInMap();

    // return mspKeyFrames.size();
    long unsigned KeyFramesInMap();

    // return mnId;
    long unsigned int GetId();

    // return mnInitKFid;
    long unsigned int GetInitKFid();

    // void SetInitKFid(long unsigned int initKFif); // 没用到

    // return mnMaxKFid;
    long unsigned int GetMaxKFid();

    // return mpKFinitial;
    KeyFrame *GetOriginKF();

    // mIsInUse = true;
    void SetCurrentMap();

    // mIsInUse = false;
    void SetStoredMap();

    // bool HasThumbnail(); // 没用到

    // return mIsInUse;
    bool IsInUse();

    // mbBad = true;
    void SetBad();

    // return mbBad;
    bool IsBad();

    void clear();

    // return mnMapChange;
    int GetMapChangeIndex();

    // mnMapChange++;
    void IncreaseChangeIndex();

    // return mnMapChangeNotified;
    int GetLastMapChange();

    // mnMapChangeNotified = currentChangeId;
    void SetLastMapChange(int currentChangeId);

    // mbImuInitialized = true;
    void SetImuInitialized();

    // return mbImuInitialized;
    bool isImuInitialized();

    /**
     * @brief 恢复尺度及重力方向
     * 
     * @param[in] T Tgw 世界坐标系到重力坐标系的变换矩阵（重力方向沿着重力坐标系 Z 轴负方向）
     * @param[in] s 尺度因子
     * @param[in] bScaledVel 是否将尺度更新到速度
    */
    void ApplyScaledRotation(const Sophus::SE3f &T, const float s, const bool bScaledVel = false);

    // mbIsInertial = true;
    void SetInertialSensor();

    // return mbIsInertial;
    bool IsInertial();

    // mbIMU_BA1 = true;
    void SetIniertialBA1();

    // mbIMU_BA2 = true;
    void SetIniertialBA2();

    // return mbIMU_BA1;
    bool GetIniertialBA1();

    // return mbIMU_BA2;
    bool GetIniertialBA2();

/** // 没用到
    void PrintEssentialGraph();
    bool CheckEssentialGraph();
*/

    // mnId = nId;
    void ChangeId(long unsigned int nId);

    // return mpKFlowerID->mnId;
    unsigned int GetLowerKFID();

    void PreSave(std::set<GeometricCamera *> &spCams);
    void PostLoad(KeyFrameDatabase *pKFDB, ORBVocabulary *pORBVoc /*, map<long unsigned int, KeyFrame*>& mpKeyFrameId*/, map<unsigned int, GeometricCamera *> &mpCams);

    // void printReprojectionError(list<KeyFrame *> &lpLocalWindowKFs, KeyFrame *mpCurrentKF, string &name, string &name_folder); // 没用到

    vector<KeyFrame *> mvpKeyFrameOrigins;
    vector<unsigned long int> mvBackupKeyFrameOriginsId;
    // KeyFrame *mpFirstRegionKF; // 没用
    std::mutex mMutexMapUpdate; // 地图更新锁

    // This avoid that two points are created simultaneously in separate threads (id conflict)
    std::mutex mMutexPointCreation;

    // bool mbFail; // 没用到

/** // 没用到
    // Size of the thumbnail (always in power of 2)
    static const int THUMB_WIDTH = 512;
    static const int THUMB_HEIGHT = 512;
*/

    static long unsigned int nNextId;

    // DEBUG: show KFs which are used in LBA
    std::set<long unsigned int> msOptKFs;
    std::set<long unsigned int> msFixedKFs;

protected:
    long unsigned int mnId; // 本地图 ID（可能在 LoopClosing::MergeLocal() 中被改变）

    std::set<MapPoint *> mspMapPoints; // 保存本地图所有路标点
    std::set<KeyFrame *> mspKeyFrames; // 保存本地图所有关键帧

    // Save/load, the set structure is broken in libboost 1.58 for ubuntu 16.04, a vector is serializated
    std::vector<MapPoint *> mvpBackupMapPoints;
    std::vector<KeyFrame *> mvpBackupKeyFrames;

    KeyFrame *mpKFinitial;
    KeyFrame *mpKFlowerID; // 在关键帧集合中拥有最小 KFID 的关键帧指针

    unsigned long int mnBackupKFinitialID;
    unsigned long int mnBackupKFlowerID;

    std::vector<MapPoint *> mvpReferenceMapPoints; // 局部地图的所有路标点

    bool mbImuInitialized;

    int mnMapChange;
    int mnMapChangeNotified;

    long unsigned int mnInitKFid; // 本地图初始关键帧的 KFID
    long unsigned int mnMaxKFid;  // 本地图关键帧中最大的 KFID
    // long unsigned int mnLastLoopKFid;

    int mnBigChangeIdx; // Index related to a big change in the map (loop closure, global BA)

    // GLubyte *mThumbnail; // 没用到（View of the map in aerial sight (for the AtlasViewer)）

    bool mIsInUse; // 本地图是否为活跃地图

    // bool mHasTumbnail; // 没用到

    bool mbBad = false;

    bool mbIsInertial;
    bool mbIMU_BA1;
    bool mbIMU_BA2;

    std::mutex mMutexMap; // 地图锁（Mutex）
};

} // namespace ORB_SLAM3

#endif // MAP_H
