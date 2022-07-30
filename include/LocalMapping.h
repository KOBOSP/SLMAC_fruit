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


#ifndef LOCALMAPPING_H
#define LOCALMAPPING_H

#include "KeyFrame.h"
#include "Atlas.h"
#include "LoopClosing.h"
#include "Tracking.h"
#include "KeyFrameDatabase.h"
#include "Settings.h"

#include <mutex>


namespace ORB_SLAM3
{

class System;
class Tracking;
class LoopClosing;
class Atlas;

class LocalMapping
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    LocalMapping(System* pSys, Atlas* pAtlas, const float bMonocular, bool bHaveIm, Settings *settings);

    void SetLoopCloser(LoopClosing* pLoopCloser);

    void SetTracker(Tracking* pTracker);

    // Main function
    void Run();

    void InsertKeyFrame(KeyFrame* pKF);
    void EmptyQueue();

    // Thread Synch
    void RequestPause();
    void RequestReset();
    void RequestResetActiveMap(Map* pMap);
    void CancelPause();
    bool CheckPaused();
    bool CheckRequestPause();
    bool AcceptKeyFrames();
    void SetAcceptKeyFrames(bool flag);
    bool RequestNotPauseOrFinish(bool flag);
    bool CheckNotPauseOrFinish();
    void InterruptBA();

    void RequestFinish();
    bool CheckFinished();

    int KeyframesInQueue(){
        unique_lock<std::mutex> lock(mMutexNewKFs);
        return mlNewKeyFrames.size();
    }

    bool IsInitializing();
    double GetCurrKFTime();

    Eigen::Matrix3d mRwg;
    Eigen::Vector3d mbg;
    Eigen::Vector3d mba;
    double mfScale;

    double mFirstTs;
    bool mbBadImu;
    // not consider far points (clouds)
    bool mbFarPoints;
    float mfThFarPoints;
protected:

    bool HaveNewKeyFrames();
    void ProcessNewKeyFrame();
    void CreateNewMapPoints();

    void MapPointCulling();
    void FuseMapPointsInNeighbors();
    void KeyFrameCulling();

    System *mpSystem;

    bool mbHaveMono;
    bool mbHaveImu;

    void ResetIfRequested();
    bool mbResetRequested;
    bool mbResetRequestedActiveMap;
    std::mutex mMutexReset;

    bool CheckFinishRequest();
    void SetFinished();
    bool mbRequestFinish;
    bool mbFinished;
    std::mutex mMutexFinish;

    Atlas* mpAtlas;

    LoopClosing* mpLoopCloser;
    Tracking* mpTracker;

    std::list<KeyFrame*> mlNewKeyFrames;

    KeyFrame* mpCurKF;

    std::list<MapPoint*> mlpRecentAddedMapPoints;

    std::mutex mMutexNewKFs;

    bool mbAbortBA;

    bool mbPaused;
    bool mbRequestPause;
    bool mbNotPauseOrFinish;
    std::mutex mMutexStop;

    bool mbAcceptKeyFrames;
    std::mutex mMutexAccept;

    void InitializeImu(float GInfo = 1e2, float AInfo = 1e6, bool bNeedGBA = false);

    bool bInitializing;

    Eigen::MatrixXd nImuInfo;
    float mfCullKFRedundantTh;
    float mTimeFirstToCur;
    int mnWeakCovisTh, mnStrongCovisTh, mnSingleMaxCullKFsNum;

    };

} //namespace ORB_SLAM

#endif // LOCALMAPPING_H
