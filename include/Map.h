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


namespace ORB_SLAM3 {

    class MapPoint;

    class KeyFrame;

    class Atlas;

    class KeyFrameDatabase;

    class Map {
        friend class boost::serialization::access;

        template<class Archive>
        void serialize(Archive &ar, const unsigned int version) {
        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        Map();

        Map(int initKFid);

        ~Map();

        void AddKeyFrame(KeyFrame *pKF);

        void AddMapPoint(MapPoint *pMP);

        void EraseMapPoint(MapPoint *pMP);

        void EraseKeyFrame(KeyFrame *pKF);

        std::vector<KeyFrame *> GetAllKeyFrames();

        std::vector<MapPoint *> GetAllMapPoints();

        long unsigned int GetMapPointsNumInMap();

        long unsigned GetKeyFramesNumInMap();

        std::vector<MapPoint *> GetReferenceMapPoints();

        std::vector<KeyFrame *> GetReferenceKeyFrames();

        void SetReferenceMapPoints(const std::vector<MapPoint *> &vpMPs);

        void SetReferenceKeyFrames(const std::vector<KeyFrame *> &vpKFs);


        long unsigned int GetId();

        long unsigned int GetInitKFId();

        void SetInitKFId(long unsigned int initKFif);

        long unsigned int GetMaxKFId();

        void ChangeId(long unsigned int nId);

        unsigned int GetLowerKFID();

        KeyFrame *GetOriginKF();

        void SetCurrentMap();

        void SetStoredMap();

        bool IsInUse();

        void SetBad();

        bool IsBad();

        void clear();

        int GetMapChangeIdx();

        void IncreaseChangeIdx();

        int GetLastMapChangeIdx();

        void SetLastMapChangeIdx(int currentChangeId);

        void SetImuInitialized();

        bool GetImuInitialized();

        void SetRtkInitialized(bool bInited);

        bool GetRtkInitialized();

        void SetSim3FRtkToLocal(Eigen::Matrix<float, 4, 4> Sim3lr, Eigen::Matrix3f Rlr, Eigen::Vector3f tlr, float slr);
        void GetSim3FRtkToLocal(Eigen::Matrix<float, 4, 4> &Sim3lr, Eigen::Matrix3f &Rlr, Eigen::Vector3f &tlr, float &slr);

        void SetImuIniertialBA1();

        void SetImuIniertialBA2();

        bool GetImuIniertialBA1();

        bool GetImuIniertialBA2();

        void ApplyScaledRotation(const Sophus::SE3f &T, const float s, const bool bScaledToVel = false);



        void PreSave(std::set<GeometricCamera *> &spCams);

        void
        PostLoad(KeyFrameDatabase *pKFDB, ORBVocabulary *pORBVoc/*, map<long unsigned int, KeyFrame*>& mpKeyFrameId*/,
                 map<unsigned int, GeometricCamera *> &mpCams);


        vector<KeyFrame *> mvpInitKeyFrames;
        vector<unsigned long int> mvBackupKeyFrameOriginsId;
        KeyFrame *mpFirstRegionKF;
        std::mutex mMutexMapUpdate;

        // This avoid that two points are created simultaneously in separate threads (id conflict)
        std::mutex mMutexPointCreation;
        std::mutex mMutexRtkUpdate;
        bool mbFail;

        // Size of the thumbnail (always in power of 2)
        static const int THUMB_WIDTH = 512;
        static const int THUMB_HEIGHT = 512;

        static long unsigned int nNextId;

        // DEBUG: show KFs which are used in LBA
        std::set<long unsigned int> msOptVisKFs;
        std::set<long unsigned int> msOptImuKFs;
        std::set<long unsigned int> msFixedKFs;

    protected:


        long unsigned int mnId;
        std::set<MapPoint *> mspMapPoints;
        std::set<KeyFrame *> mspKeyFrames;

        // Save/load, the set structure is broken in libboost 1.58 for ubuntu 16.04, a vector is serializated
        std::vector<MapPoint *> mvpBackupMapPoints;
        std::vector<KeyFrame *> mvpBackupKeyFrames;

        KeyFrame *mpKFinitial;
        KeyFrame *mpKFlowerID;

        unsigned long int mnBackupKFinitialID;
        unsigned long int mnBackupKFlowerID;

        std::vector<MapPoint *> mvpReferenceMapPoints;
        std::vector<KeyFrame *> mvpReferenceKeyFrames;

        bool mbIMU_BA1;
        bool mbIMU_BA2;
        bool mbImuInitialized;
        bool mbRtkInitialized;
        Eigen::Matrix3f mRlr;
        Eigen::Vector3f mtlr;
        float mslr, mfRtkToLocalDist;
        Eigen::Matrix<float, 4, 4> mSim3lr;

        int mnMapChangeIdx;
        int mnLastMapChangeIdx;

        long unsigned int mnInitKFId;
        long unsigned int mnMaxKFid;

        // View of the map in aerial sight (for the AtlasViewer)
        GLubyte *mThumbnail;

        bool mIsInUse;
        bool mbBad = false;

        // Mutex
        std::mutex mMutexMap;

    };

} //namespace ORB_SLAM3

#endif // MAP_H
