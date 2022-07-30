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


#ifndef TRACKING_H
#define TRACKING_H

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "Viewer.h"
#include "FrameDrawer.h"
#include "Atlas.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "Frame.h"
#include "ORBVocabulary.h"
#include "KeyFrameDatabase.h"
#include "ORBextractor.h"
#include "MapDrawer.h"
#include "System.h"
#include "ImuTypes.h"
#include "Settings.h"

#include "GeometricCamera.h"

#include <mutex>
#include <unordered_set>

namespace ORB_SLAM3
{

class Viewer;
class FrameDrawer;
class Atlas;
class LocalMapping;
class LoopClosing;
class System;
class Settings;

class Tracking
{  

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Tracking(System* pSys, ORBVocabulary* pVoc, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, Atlas* pAtlas,
             KeyFrameDatabase* pKFDB, const string &strSettingPath, const int sensor, Settings* settings);

    ~Tracking();



    Sophus::SE3f GrabImageStereo(const cv::Mat &imRectLeft, const cv::Mat &imRectRight, Eigen::Matrix<float, 3, 1> trw, const double &dTimestamp);
    void GrabImuData(const IMU::Point &ImuMeasure);

    void SetLocalMapper(LocalMapping* pLocalMapper);
    void SetLoopClosing(LoopClosing* pLoopClosing);
    void SetViewer(Viewer* pViewer);
    void SetStepByStep(bool bSet);
    bool GetStepByStep();


    void InformOnlyTracking(const bool &flag);

    void UpdateLastAndCurFrameIMU(const float s, const IMU::Bias &b, KeyFrame* pCurrentKeyFrame);
    KeyFrame* GetLastKeyFrame()
    {
        return mpLastKeyFrame;
    }

    void SaveAndCreateMapInAtlas();
    int GetMatchNumInLM();
    float GetImageScale();
    void ResetActiveMap(bool bLocMap = false);

public:

    // Tracking states
    enum eTrackingState{
        SYSTEM_NOT_READY=-1,
        NO_IMAGES_YET=0,
        NOT_INITIALIZED=1,
        OK=2,
        RECENTLY_LOST=3,
        LOST=4,
        OK_KLT=5
    };

    eTrackingState mState;
    eTrackingState mStateForViewer;
    int mnTrackMethod;//0:Lost, 1:Imu, 2:Motion, 3:RefKF, 4:Reloc,

    int mSensor;

    Frame mCurFrame;
    Frame mLastFrame;
    Frame mInitFrame;
    std::vector<int> mvIniMatchesForViewer;
    bool mbDoNext;
    int mnLMInFMatchNum;
    bool mbOnlyTracking;
    double mdTrackFps,mdExtraFps;

    cv::Mat mImgLeftToViewer;
    cv::Mat mImgRightToViewer;
protected:

    // Main tracking function. It is independent of the input sensor.
    void Track();

    // Map initialization for stereo and RGB-D
    void StereoInitialization();

    void CheckReplacedInLastFrame();
    bool TrackReferenceKeyFrame();
    void UpdateLastFramePose();
    bool TrackWithMotionModel();
    bool TrackWithIMU();

    bool Relocalization();

    void UpdateKFsAndMPsInLocal();
    void UpdateLocalMapPoints();
    void UpdateLocalKeyFrames();

    bool TrackLocalMap();
    int MatchLocalMPsToCurFrame();

    bool NeedNewKeyFrame();
    void CreateNewKeyFrame();

    // Perform preintegration from last frame
    void PreintegrateIMU();

    void LoadParameter(Settings* settings);


    // System
    LocalMapping* mpLocalMapping;
    LoopClosing* mpLoopClosing;
    ORBextractor* mpORBextractorLeft, *mpORBextractorRight;
    ORBVocabulary* mpORBVocabulary;
    KeyFrameDatabase* mpKeyFrameDB;
    System* mpSystem;
    Viewer* mpViewer;
    FrameDrawer* mpFrameDrawer;
    MapDrawer* mpMapDrawer;
    Atlas* mpAtlas;
    GeometricCamera* mpCamera;


    Sophus::SE3f mTcFrKF;
    bool mbHaveCreatedMap;
    std::vector<KeyFrame*> mvpLocalKeyFrames;
    std::vector<MapPoint*> mvpLocalMapPoints;
    KeyFrame* mpLastKeyFrame;
    KeyFrame* mpReferenceKF;
    unsigned int mnFrameIdLastReloc;
    unsigned int mnFrameIdLastRecLost;
    unsigned int mnFrameIdLastLost;


    bool bStepByStep;
    bool mbMapUpdated;

    //Calibration matrix
    cv::Mat mCvK;
    float mfBaselineFocal;
    float mImageScale;
    int mMinFrames;
    int mnMaxFrames;
    int mnFrameNumDurRefLoc;
    int mnFrameNumDurRecLost;
    int mnFrameNumDurLost;
    float mfThDepth;

    std::mutex mMutexImuQueue;
    float mImuFreq;
    double mImuInterval;
    bool mInsertKFsLost;
    IMU::Preintegrated *mpImuPreintegratedFromLastKF;
    std::list<IMU::Point> mlImuFromLastFrame;
    std::vector<IMU::Point> mvImuFromLastFrame;//filled by PreintegrateIMU)
    IMU::Calib *mpImuCalib;
    IMU::Bias mLastBias;


    bool mbVelocity{false};
    Sophus::SE3f mVelocity;

};

} //namespace ORB_SLAM

#endif // TRACKING_H
