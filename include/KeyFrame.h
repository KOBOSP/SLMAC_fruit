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


#ifndef KEYFRAME_H
#define KEYFRAME_H

#include "MapPoint.h"
#include "DBoW2/BowVector.h"
#include "DBoW2/FeatureVector.h"
#include "ORBVocabulary.h"
#include "ORBextractor.h"
#include "Frame.h"
#include "KeyFrameDatabase.h"
#include "ImuTypes.h"

#include "GeometricCamera.h"
#include "SerializationUtils.h"

#include <mutex>

#include <boost/serialization/base_object.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/map.hpp>


namespace ORB_SLAM3 {

    class Map;

    class MapPoint;

    class Frame;

    class KeyFrameDatabase;

    class GeometricCamera;

    class KeyFrame {
        friend class boost::serialization::access;

        template<class Archive>
        void serialize(Archive &ar, const unsigned int version) {

        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        KeyFrame();

        KeyFrame(Frame &F, Map *pMap, KeyFrameDatabase *pKFDB);

        // Pose functions
        void SetPose(const Sophus::SE3f &Tcw);

        void SetRtkTrans(Eigen::Matrix<float, 3, 1> trw);

        Eigen::Matrix<float, 3, 1> GetRtkTransF();

        void SetVelocity(const Eigen::Vector3f &Vw_);

        Sophus::SE3f GetPose();

        Sophus::SE3f GetPoseInverse();

        Eigen::Vector3f GetCameraCenter();

        Eigen::Vector3f GetImuPosition();

        Eigen::Matrix3f GetImuRotation();

        Sophus::SE3f GetImuPose();

        Eigen::Matrix3f GetRotation();

        Eigen::Vector3f GetTranslation();

        Eigen::Vector3f GetVelocity();

        bool isVelocitySet();

        // Bag of Words Representation
        void ComputeBoW();

        // Covisibility graph functions
        void AddConnection(KeyFrame *pKF, const int &weight);

        void EraseConnection(KeyFrame *pKF);

        void UpdateCovisGraph(bool bShow = true);

        void UpdateBestCovisibles();

        std::set<KeyFrame *> GetConnectedKeyFrames();

        std::vector<KeyFrame *> GetVectorCovisibleKeyFrames();

        std::vector<KeyFrame *> GetBestCovisibilityKeyFrames(const int &N);

        std::vector<KeyFrame *> GetCovisiblesByWeight(const int &w);

        int GetWeightBetweenKF(KeyFrame *pKF);

        // Spanning tree functions
        void AddChildKF(KeyFrame *pKF);

        void EraseChild(KeyFrame *pKF);

        void ChangeParent(KeyFrame *pKF);

        std::set<KeyFrame *> GetChilds();

        KeyFrame *GetParent();

        bool hasChild(KeyFrame *pKF);

        void SetFirstConnection(bool bFirst);

        // Loop Edges
        void AddLoopEdge(KeyFrame *pKF);

        std::set<KeyFrame *> GetLoopEdges();

        // Merge Edges
        void AddMergeEdge(KeyFrame *pKF);

        set<KeyFrame *> GetMergeEdges();

        // MapPoint observation functions
        int GetNumberMPs();

        void AddMapPoint(MapPoint *pMP, const size_t &idx);

        void EraseMapPointMatch(const int &idx);

        void EraseMapPointMatch(MapPoint *pMP);

        void ReplaceMapPointMatch(const int &idx, MapPoint *pMP);

        std::set<MapPoint *> GetMapPoints();

        std::vector<MapPoint *> GetVectorMapPointsInKF();

        int TrackedMapPoints(const int &minObs);

        MapPoint *GetIdxMapPoint(const size_t &idx);

        // KeyPoint functions
        std::vector<size_t>
        GetFeaturesInArea(const float &x, const float &y, const float &r, const bool bRight = false) const;

        bool UnprojectStereo(int i, Eigen::Vector3f &x3D);

        // Image
        bool IsInImage(const float &x, const float &y) const;

        // Enable/Disable bad flag changes
        void SetNotErase();

        void SetCanErase();

        // Set/check bad flag
        void SetBadFlag();

        bool isBad();

        // Compute Scene Depth (q=2 median). Used in monocular.
        float ComputeSceneMedianDepth(const int q);

        static bool weightComp(int a, int b) {
            return a > b;
        }

        static bool lId(KeyFrame *pKF1, KeyFrame *pKF2) {
            return pKF1->mnId < pKF2->mnId;
        }

        Map *GetMap();

        void UpdateMap(Map *pMap);

        void SetNewBias(const IMU::Bias &b);

        Eigen::Vector3f GetGyroBias();

        Eigen::Vector3f GetAccBias();

        IMU::Bias GetImuBias();


        void PreSave(set<KeyFrame *> &spKF, set<MapPoint *> &spMP, set<GeometricCamera *> &spCam);

        void PostLoad(map<long unsigned int, KeyFrame *> &mpKFid, map<long unsigned int, MapPoint *> &mpMPid,
                      map<unsigned int, GeometricCamera *> &mpCamId);


        void SetORBVocabulary(ORBVocabulary *pORBVoc);

        void SetKeyFrameDatabase(KeyFrameDatabase *pKFDB);

        bool bImu;

        // The following variables are accesed from only 1 thread or never change (no mutex needed).
    public:

        static long unsigned int nNextId;
        long unsigned int mnId;
        const long unsigned int mnFrameId;
        static int mnStrongCovisTh;
        const double mdTimestamp;

        // Grid (to speed up feature matching)
        const int mnGridCols;
        const int mnGridRows;
        const float mfGridElementWidthInv;
        const float mfGridElementHeightInv;

        // Variables used by the tracking
        long unsigned int mnTrackReferenceForFrame;
        long unsigned int mnFuseFlagInLocalMapping;

        // Variables used by the local mapping
        long unsigned int mnBAOptFlagInLM;
        long unsigned int mnBAFixFlagInLM;

        // Variables used by the keyframe database
        long unsigned int mnLoopQuery;
        int mnLoopWords;
        float mLoopScore;
        long unsigned int mnRelocQuery;
        int mnRelocWords;
        float mRelocScore;
        long unsigned int mnMergeQuery;
        int mnMergeWords;
        float mMergeScore;
        long unsigned int mnRecognitionFlagInLoopClosing;
        int mnRecognitionCommonWords;
        float mPlaceRecognitionScore;

        // Variables used by loop closing
        Sophus::SE3f mLMGBATcw;
        Sophus::SE3f mBefGBATcw;
        Eigen::Vector3f mLMGBAVwb;
        IMU::Bias mLMGBABias;
        long unsigned int mnLMGBAFlag;

        // Variables used by merging
        Sophus::SE3f mTcwMerge;
        Sophus::SE3f mTcwBefMerge;
        Sophus::SE3f mTwcBefMerge;
        Eigen::Vector3f mVwbMerge;
        long unsigned int mnBALocalForMerge;

        float mfScale;

        // Calibration parameters
        const float fx, fy, cx, cy, invfx, invfy, mfBaselineFocal, mfBaseline, mfThDepth;

        // Number of KeyPoints
        const int mnKPsLeftNum;

        // KeyPoints, stereo coordinate and descriptors (all associated by an index)
        const std::vector<cv::KeyPoint> mvKPsLeft;
        const std::vector<cv::KeyPoint> mvKPsUn;
        const std::vector<float> mvfXInRight; // negative value for monocular points
        const std::vector<float> mvfMPDepth; // negative value for monocular points
        const cv::Mat mDescriptors;

        //BoW
        DBoW2::BowVector mBowVec;
        DBoW2::FeatureVector mFeatVec;

        // Pose relative to parent (this is computed when bad flag is activated)
        Sophus::SE3f mTcp;

        // Scale
        const int mnScaleLevels;
        const float mfScaleFactor;
        const float mfLogScaleFactor;
        const std::vector<float> mvScaleFactors;
        const std::vector<float> mvfLevelSigma2;
        const std::vector<float> mvfInvLevelSigma2;

        // Image bounds and calibration
        const int mnMinX;
        const int mnMinY;
        const int mnMaxX;
        const int mnMaxY;

        // Preintegrated IMU measurements from previous keyframe
        KeyFrame *mPrevKF;
        KeyFrame *mNextKF;
        IMU::Preintegrated *mpImuPreintegrated;
        IMU::Calib mImuCalib;
        unsigned int mnOriginMapId;

        //bool mbHasHessian;
        //cv::Mat mHessianPose;

        // The following variables need to be accessed trough a mutex to be thread safe.
    protected:
        // sophus poses
        Sophus::SE3<float> mTcw;
        Eigen::Matrix3f mRcw;
        Sophus::SE3<float> mTwc;
        Eigen::Matrix3f mRwc;
        Eigen::Matrix<float, 3, 1> mtrw;
        // IMU position
        Eigen::Vector3f mOwb;
        // Velocity (Only used for inertial SLAM)
        Eigen::Vector3f mVw;
        bool mbHasVelocity;

        //Transformation matrix between cameras in stereo fisheye
        Sophus::SE3<float> mTlr;
        Sophus::SE3<float> mTrl;

        // Imu bias
        IMU::Bias mImuBias;

        // MapPoints associated to keypoints
        std::vector<MapPoint *> mvpMapPoints;
        // For save relation without pointer, this is necessary for save/load function
        std::vector<long long int> mvBackupMapPointsId;

        // BoW
        KeyFrameDatabase *mpKeyFrameDB;
        ORBVocabulary *mpORBvocabulary;

        // Grid over the image to speed up feature matching
        std::vector<std::vector<std::vector<size_t> > > mGrid;

        std::map<KeyFrame *, int> mConnectedKFAndWeights;
        std::vector<KeyFrame *> mvpOrderedConnectedKFs;
        std::vector<int> mvOrderedWeights;
        // For save relation without pointer, this is necessary for save/load function
        std::map<long unsigned int, int> mBackupConnectedKeyFrameIdWeights;

        // Spanning Tree and Loop Edges
        bool mbFirstConnection;
        KeyFrame *mpParentKF;
        std::set<KeyFrame *> mspChildKF;
        std::set<KeyFrame *> mspLoopEdges;
        std::set<KeyFrame *> mspMergeEdges;
        // For save relation without pointer, this is necessary for save/load function
        long long int mBackupParentId;
        std::vector<long unsigned int> mvBackupChildrensId;
        std::vector<long unsigned int> mvBackupLoopEdgesId;
        std::vector<long unsigned int> mvBackupMergeEdgesId;

        // Bad flags
        bool mbNotErase;
        bool mbToBeErased;
        bool mbBad;

        float mHalfBaseline; // Only for visualization

        Map *mpMap;

        // Backup variables for inertial
        long long int mBackupPrevKFId;
        long long int mBackupNextKFId;
        IMU::Preintegrated mBackupImuPreintegrated;

        // Backup for Cameras
        unsigned int mnBackupIdCamera;

        // Calibration
        Eigen::Matrix3f mEigenK;

        // Mutex
        std::mutex mMutexPose; // for pose, velocity and biases
        std::mutex mMutexConnections;
        std::mutex mMutexFeatures;
        std::mutex mMutexMap;

    public:
        GeometricCamera *mpCamera;

        Sophus::SE3f GetRelativePoseTrl();

        Sophus::SE3f GetRelativePoseTlr();

        //KeyPoints in the right image (for stereo fisheye, coordinates are needed)
        const std::vector<cv::KeyPoint> mvKPsRight;

        std::vector<std::vector<std::vector<size_t> > > mGridRight;

        Sophus::SE3<float> GetRightPose();

        Sophus::SE3<float> GetRightPoseInverse();

        Eigen::Vector3f GetRightCameraCenter();

        Eigen::Matrix<float, 3, 3> GetRightRotation();

        Eigen::Vector3f GetRightTranslation();

        void PrintPointDistribution() {
            int left = 0, right = 0;
            int Nlim = mnKPsLeftNum;
            for (int i = 0; i < mnKPsLeftNum; i++) {
                if (mvpMapPoints[i]) {
                    if (i < Nlim) left++;
                    else right++;
                }
            }
            cout << "Point distribution in KeyFrame: left-> " << left << " --- right-> " << right << endl;
        }


    };

} //namespace ORB_SLAM

#endif // KEYFRAME_H
