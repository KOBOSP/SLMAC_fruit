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

#include "Map.h"

#include <mutex>

namespace ORB_SLAM3 {

    long unsigned int Map::nNextId = 0;

    Map::Map()
            : mnMaxKFid(0), mbImuInitialized(false), mbRtkInitialized(false), mfRtkToLocalDist(1e10),
              mnMapChangeIdx(0), mpFirstRegionKF(static_cast<KeyFrame *>(NULL)),
              mbFail(false), mIsInUse(false), mbBad(false), mnLastMapChangeIdx(0),
              mbIMU_BA1(false), mbIMU_BA2(false) {
        mnId = nNextId++;
        mThumbnail = static_cast<GLubyte *>(NULL);
    }

    Map::Map(int initKFid)
            : mnInitKFId(initKFid), mnMaxKFid(initKFid), mIsInUse(false),
              mbBad(false), mbImuInitialized(false), mbRtkInitialized(false), mfRtkToLocalDist(1e10),
              mpFirstRegionKF(static_cast<KeyFrame *>(NULL)),
              mnMapChangeIdx(0), mbFail(false), mnLastMapChangeIdx(0), mbIMU_BA1(false),
              mbIMU_BA2(false) {
        mnId = nNextId++;
        mThumbnail = static_cast<GLubyte *>(NULL);
    }

    Map::~Map() {
        // TODO: erase all points from memory
        mspMapPoints.clear();

        // TODO: erase all keyframes from memory
        mspKeyFrames.clear();

        if (mThumbnail)
            delete mThumbnail;
        mThumbnail = static_cast<GLubyte *>(NULL);

        mvpReferenceMapPoints.clear();
        mvpInitKeyFrames.clear();
    }

// 在地图中插入关键帧,同时更新关键帧的最大id
    void Map::AddKeyFrame(KeyFrame *pKF) {
        unique_lock<mutex> lock(mMutexMap);
        if (mspKeyFrames.empty()) {
            cout << "First KF:" << pKF->mnId << "; Map init KF:" << mnInitKFId << endl;
            mnInitKFId = pKF->mnId;
            mpKFinitial = pKF;
            mpKFlowerID = pKF;
        }
        mspKeyFrames.insert(pKF);
        if (pKF->mnId > mnMaxKFid) {
            mnMaxKFid = pKF->mnId;
        }
        if (pKF->mnId < mpKFlowerID->mnId) {
            mpKFlowerID = pKF;
        }
    }


    void Map::AddMapPoint(MapPoint *pMP) {
        unique_lock<mutex> lock(mMutexMap);
        mspMapPoints.insert(pMP);
    }

    void Map::SetImuIniertialBA1() {
        unique_lock<mutex> lock(mMutexMap);
        mbIMU_BA1 = true;
    }

    void Map::SetImuIniertialBA2() {
        unique_lock<mutex> lock(mMutexMap);
        mbIMU_BA2 = true;
    }

    bool Map::GetImuIniertialBA1() {
        unique_lock<mutex> lock(mMutexMap);
        return mbIMU_BA1;
    }

    bool Map::GetImuIniertialBA2() {
        unique_lock<mutex> lock(mMutexMap);
        return mbIMU_BA2;
    }

    void Map::SetImuInitialized() {
        unique_lock<mutex> lock(mMutexMap);
        mbImuInitialized = true;
    }

    bool Map::GetImuInitialized() {
        unique_lock<mutex> lock(mMutexMap);
        return mbImuInitialized;
    }

    void Map::SetRtkInitialized(bool bInited) {
        unique_lock<mutex> lock(mMutexRtkUpdate);
        mbRtkInitialized = bInited;

    }

    bool Map::GetRtkInitialized() {
        unique_lock<mutex> lock(mMutexRtkUpdate);
        return mbRtkInitialized;
    }

    void Map::GetSim3FRtkToLocal(Eigen::Matrix<float, 4, 4> &Sim3lr, Eigen::Matrix3f &Rlr, Eigen::Vector3f &tlr, float &slr) {
        unique_lock<mutex> lock(mMutexRtkUpdate);
        Rlr = mRlr;
        tlr = mtlr;
        slr = mslr;
        Sim3lr = mSim3lr;
    }

    void Map::SetSim3FRtkToLocal(Eigen::Matrix<float, 4, 4> Sim3lr, Eigen::Matrix3f Rlr, Eigen::Vector3f tlr, float slr) {
        unique_lock<mutex> lock(mMutexRtkUpdate);
        mRlr = Rlr;
        mtlr = tlr;
        mslr = slr;
        mSim3lr = Sim3lr;
    }

    void Map::EraseMapPoint(MapPoint *pMP) {
        unique_lock<mutex> lock(mMutexMap);
        mspMapPoints.erase(pMP);

        // 下面是作者加入的注释. 实际上只是从std::set中删除了地图点的指针, 原先地图点
        // 占用的内存区域并没有得到释放
        // TODO: This only erase the pointer.
        // Delete the MapPoint
    }

    void Map::EraseKeyFrame(KeyFrame *pKF) {
        unique_lock<mutex> lock(mMutexMap);
        mspKeyFrames.erase(pKF);
        if (mspKeyFrames.size() > 0) {
            if (pKF->mnId == mpKFlowerID->mnId) {
                vector<KeyFrame *> vpKFs = vector<KeyFrame *>(mspKeyFrames.begin(), mspKeyFrames.end());
                sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);
                mpKFlowerID = vpKFs[0];
            }
        } else {
            mpKFlowerID = 0;
        }

        // TODO: This only erase the pointer.
        // Delete the MapPoint
    }

/*
 * @brief 设置参考MapPoints，将用于DrawMapPoints函数画图
 * 设置参考地图点用于绘图显示局部地图点（红色）
 * @param vpMPs Local MapPoints
 */
    void Map::SetReferenceMapPoints(const std::vector<MapPoint *> &vpMPs) {
        unique_lock<mutex> lock(mMutexMap);
        mvpReferenceMapPoints = vpMPs;
    }

    void Map::SetReferenceKeyFrames(const std::vector<KeyFrame *> &vpKFs) {
        unique_lock<mutex> lock(mMutexMap);
        mvpReferenceKeyFrames = vpKFs;
    }


// 获取地图中的所有关键帧
    vector<KeyFrame *> Map::GetAllKeyFrames() {
        unique_lock<mutex> lock(mMutexMap);
        return vector<KeyFrame *>(mspKeyFrames.begin(), mspKeyFrames.end());
    }

// 获取地图中的所有地图点
    vector<MapPoint *> Map::GetAllMapPoints() {
        unique_lock<mutex> lock(mMutexMap);
        return vector<MapPoint *>(mspMapPoints.begin(), mspMapPoints.end());
    }

// 获取地图点数目
    long unsigned int Map::GetMapPointsNumInMap() {
        unique_lock<mutex> lock(mMutexMap);
        return mspMapPoints.size();
    }

// 获取地图中的关键帧数目
    long unsigned int Map::GetKeyFramesNumInMap() {
        unique_lock<mutex> lock(mMutexMap);
        return mspKeyFrames.size();
    }

// 获取参考地图点
    vector<MapPoint *> Map::GetReferenceMapPoints() {
        unique_lock<mutex> lock(mMutexMap);
        return mvpReferenceMapPoints;
    }

    vector<KeyFrame *> Map::GetReferenceKeyFrames() {
        unique_lock<mutex> lock(mMutexMap);
        return mvpReferenceKeyFrames;
    }

    long unsigned int Map::GetId() {
        return mnId;
    }

    long unsigned int Map::GetInitKFId() {
        unique_lock<mutex> lock(mMutexMap);
        return mnInitKFId;
    }

    void Map::SetInitKFId(long unsigned int InitKFId) {
        unique_lock<mutex> lock(mMutexMap);
        mnInitKFId = InitKFId;
    }

    long unsigned int Map::GetMaxKFId() {
        unique_lock<mutex> lock(mMutexMap);
        return mnMaxKFid;
    }

    KeyFrame *Map::GetOriginKF() {
        return mpKFinitial;
    }

    void Map::SetCurrentMap() {
        mIsInUse = true;
    }

    void Map::SetStoredMap() {
        mIsInUse = false;
    }

    void Map::clear() {
        //    for(set<MapPoint*>::iterator sit=mspMapPoints.begin(), send=mspMapPoints.end(); sit!=send; sit++)
        //        delete *sit;

        for (set<KeyFrame *>::iterator sit = mspKeyFrames.begin(), send = mspKeyFrames.end(); sit != send; sit++) {
            KeyFrame *pKF = *sit;
            pKF->UpdateMap(static_cast<Map *>(NULL));
            //        delete *sit;
        }

        mspMapPoints.clear();
        mspKeyFrames.clear();
        mnMaxKFid = mnInitKFId;
        mbImuInitialized = false;
        mbRtkInitialized = false;
        mvpReferenceMapPoints.clear();
        mvpInitKeyFrames.clear();
        mbIMU_BA1 = false;
        mbIMU_BA2 = false;
        mfRtkToLocalDist = 1e10;
    }

    bool Map::IsInUse() {
        return mIsInUse;
    }

    void Map::SetBad() {
        mbBad = true;
    }

    bool Map::IsBad() {
        return mbBad;
    }

// 恢复尺度及重力方向
/** imu在localmapping中初始化，LocalMapping::InitializeIMU中使用，误差包含三个残差与两个偏置
 * 地图融合时也会使用
 * @param R 初始化时为Rgw
 * @param s 尺度
 * @param bScaledToVel 将尺度更新到速度
 * @param t 默认cv::Mat::zeros(cv::Size(1,3),CV_32F)
 */
    void Map::ApplyScaledRotation(const Sophus::SE3f &T, const float s, const bool bScaledToVel) {
        unique_lock<mutex> lock(mMutexMap);

        // Body position (IMU) of first keyframe is fixed to (0,0,0)
        Sophus::SE3f Tyw = T;
        Eigen::Matrix3f Ryw = Tyw.rotationMatrix();
        Eigen::Vector3f tyw = Tyw.translation();

        for (set<KeyFrame *>::iterator sit = mspKeyFrames.begin(); sit != mspKeyFrames.end(); sit++) {
            // 更新关键帧位姿
            /**
             * | Rw2w1  tw2w1 |   *   | Rw1c  s*tw1c  |     =    |  Rw2c     s*Rw2w1*tw1c + tw2w1  |
             * |   0      1   |       |  0       1    |          |   0                1            |
             * 这么做比正常乘在旋转上少了个s，后面不需要这个s了，因为所有mp在下面已经全部转到了w2坐标系下，不存在尺度变化了
             *
             * | s*Rw2w1  tw2w1 |   *   | Rw1c    tw1c  |     =    |  s*Rw2c     s*Rw2w1*tw1c + tw2w1  |
             * |   0        1   |       |  0       1    |          |     0                1            |
             */
            KeyFrame *pKF = *sit;
            Sophus::SE3f Twc = pKF->GetPoseInverse();
            Twc.translation() *= s;

            // |  Ryc     s*Ryw*twc + tyw  |
            // |   0           1           |
            Sophus::SE3f Tyc = Tyw * Twc;
            Sophus::SE3f Tcy = Tyc.inverse();
            pKF->SetPose(Tcy);
            // 更新关键帧速度
            Eigen::Vector3f Vw = pKF->GetVelocity();
            if (!bScaledToVel)
                pKF->SetVelocity(Ryw * Vw);
            else
                pKF->SetVelocity(Ryw * Vw * s);
        }
        for (set<MapPoint *>::iterator sit = mspMapPoints.begin(); sit != mspMapPoints.end(); sit++) {
            // 更新每一个mp在世界坐标系下的坐标
            MapPoint *pMP = *sit;
            pMP->SetWorldPos(s * Ryw * pMP->GetWorldPos() + tyw);
            pMP->UpdateNormalAndDepth();
        }
        mnMapChangeIdx++;
    }


    void Map::ChangeId(long unsigned int nId) {
        mnId = nId;
    }

    unsigned int Map::GetLowerKFID() {
        unique_lock<mutex> lock(mMutexMap);
        if (mpKFlowerID) {
            return mpKFlowerID->mnId;
        }
        return 0;
    }

    int Map::GetMapChangeIdx() {
        unique_lock<mutex> lock(mMutexMap);
        return mnMapChangeIdx;
    }

    void Map::IncreaseChangeIdx() {
        unique_lock<mutex> lock(mMutexMap);
        mnMapChangeIdx++;
    }

    int Map::GetLastMapChangeIdx() {
        unique_lock<mutex> lock(mMutexMap);
        return mnLastMapChangeIdx;
    }

    void Map::SetLastMapChangeIdx(int currentChangeId) {
        unique_lock<mutex> lock(mMutexMap);
        mnLastMapChangeIdx = currentChangeId;
    }

/** 预保存，也就是把想保存的信息保存到备份的变量中
 * @param spCams 相机
 */
    void Map::PreSave(std::set<GeometricCamera *> &spCams) {
        int nMPWithoutObs = 0;  // 统计用
        // 1. 剔除一下无效观测
        for (MapPoint *pMPi : mspMapPoints) {
            if (!pMPi || pMPi->isBad())
                continue;

            if (pMPi->GetObsKFAndLRIdx().size() == 0) {
                nMPWithoutObs++;
            }
            map<KeyFrame *, std::tuple<int, int>> mpObs = pMPi->GetObsKFAndLRIdx();
            for (map<KeyFrame *, std::tuple<int, int>>::iterator it = mpObs.begin(), end = mpObs.end();
                 it != end; ++it) {
                if (it->first->GetMap() != this || it->first->isBad()) {
                    pMPi->EraseObservation(it->first);
                }
            }
        }

        // Saves the id of KF origins
        // 2. 保存最开始的帧的id，貌似一个map的mvpKeyFrameOrigins里面只有一个，可以验证一下
        mvBackupKeyFrameOriginsId.clear();
        mvBackupKeyFrameOriginsId.reserve(mvpInitKeyFrames.size());
        for (int i = 0, numEl = mvpInitKeyFrames.size(); i < numEl; ++i) {
            mvBackupKeyFrameOriginsId.emplace_back(mvpInitKeyFrames[i]->mnId);
        }

        // Backup of MapPoints
        // 3. 保存一下对应的mp
        mvpBackupMapPoints.clear();
        for (MapPoint *pMPi : mspMapPoints) {
            if (!pMPi || pMPi->isBad())
                continue;

            mvpBackupMapPoints.emplace_back(pMPi);
            pMPi->PreSave(mspKeyFrames, mspMapPoints);
        }

        // Backup of KeyFrames
        // 4. 保存一下对应的KF
        mvpBackupKeyFrames.clear();
        for (KeyFrame *pKFi : mspKeyFrames) {
            if (!pKFi || pKFi->isBad())
                continue;

            mvpBackupKeyFrames.emplace_back(pKFi);
            pKFi->PreSave(mspKeyFrames, mspMapPoints, spCams);
        }

        // 保存一些id
        mnBackupKFinitialID = -1;
        if (mpKFinitial) {
            mnBackupKFinitialID = mpKFinitial->mnId;
        }

        mnBackupKFlowerID = -1;
        if (mpKFlowerID) {
            mnBackupKFlowerID = mpKFlowerID->mnId;
        }
    }

/** 后加载，也就是把备份的变量恢复到正常变量中
 * @param spCams 相机
 */
    void
    Map::PostLoad(KeyFrameDatabase *pKFDB, ORBVocabulary *pORBVoc /*, map<long unsigned int, KeyFrame*>& mpKeyFrameId*/,
                  map<unsigned int, GeometricCamera *> &mpCams) {
        std::copy(mvpBackupMapPoints.begin(), mvpBackupMapPoints.end(),
                  std::inserter(mspMapPoints, mspMapPoints.begin()));
        std::copy(mvpBackupKeyFrames.begin(), mvpBackupKeyFrames.end(),
                  std::inserter(mspKeyFrames, mspKeyFrames.begin()));

        // 1. 恢复map中的mp，注意此时mp中只恢复了保存的量
        map<long unsigned int, MapPoint *> mpMapPointId;
        for (MapPoint *pMPi : mspMapPoints) {
            if (!pMPi || pMPi->isBad())
                continue;

            pMPi->UpdateMap(this);
            mpMapPointId[pMPi->mnId] = pMPi;
        }

        // 2. 恢复map中的kf，注意此时kf中只恢复了保存的量
        map<long unsigned int, KeyFrame *> mpKeyFrameId;
        for (KeyFrame *pKFi : mspKeyFrames) {
            if (!pKFi || pKFi->isBad())
                continue;

            pKFi->UpdateMap(this);
            pKFi->SetORBVocabulary(pORBVoc);
            pKFi->SetKeyFrameDatabase(pKFDB);
            mpKeyFrameId[pKFi->mnId] = pKFi;
        }

        // References reconstruction between different instances
        // 3. 使用mp中的备份变量恢复正常变量
        for (MapPoint *pMPi : mspMapPoints) {
            if (!pMPi || pMPi->isBad())
                continue;

            pMPi->PostLoad(mpKeyFrameId, mpMapPointId);
        }

        // 4. 使用kf中的备份变量恢复正常变量
        for (KeyFrame *pKFi : mspKeyFrames) {
            if (!pKFi || pKFi->isBad())
                continue;

            pKFi->PostLoad(mpKeyFrameId, mpMapPointId, mpCams);
            pKFDB->add(pKFi);
        }

        // 5. 恢复ID
        if (mnBackupKFinitialID != -1) {
            mpKFinitial = mpKeyFrameId[mnBackupKFinitialID];
        }

        if (mnBackupKFlowerID != -1) {
            mpKFlowerID = mpKeyFrameId[mnBackupKFlowerID];
        }

        mvpInitKeyFrames.clear();
        mvpInitKeyFrames.reserve(mvBackupKeyFrameOriginsId.size());
        for (int i = 0; i < mvBackupKeyFrameOriginsId.size(); ++i) {
            mvpInitKeyFrames.emplace_back(mpKeyFrameId[mvBackupKeyFrameOriginsId[i]]);
        }

        mvpBackupMapPoints.clear();
    }

} // namespace ORB_SLAM3
