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


#ifndef LOOPCLOSING_H
#define LOOPCLOSING_H

#include "KeyFrame.h"
#include "LocalMapping.h"
#include "Atlas.h"
#include "ORBVocabulary.h"
#include "Tracking.h"

#include "KeyFrameDatabase.h"

#include <boost/algorithm/string.hpp>
#include <thread>
#include <mutex>
#include "g2o/types/types_seven_dof_expmap.h"

namespace ORB_SLAM3
{

class Tracking;
class LocalMapping;
class KeyFrameDatabase;
class Map;


class LoopClosing
{
public:

    typedef map<KeyFrame*,g2o::Sim3,std::less<KeyFrame*>,
        Eigen::aligned_allocator<std::pair<KeyFrame* const, g2o::Sim3> > > KFAndPose;

public:

    LoopClosing(Atlas* pAtlas, KeyFrameDatabase* pDB, ORBVocabulary* pVoc,const bool bFixScale, const bool bActiveLC, Settings *settings);

    void SetTracker(Tracking* pTracker);
    void SetLocalMapper(LocalMapping* pLocalMapper);
    void SetViewer(Viewer *pViewer);

    // Main function
    void Run();
    void InitializeRtk();
    void InsertKeyFrame(KeyFrame *pKF);

    void RequestReset();
    void RequestResetActiveMap(Map* pMap);

    // This function will run in a separate thread
    void RunGlobalBundleAdjustment(Map* pActiveMap, unsigned long nGBAId);
    void UpdateKFsAndMPsAfterBA(list<KeyFrame *> &lpKFstoUpdate, vector<MapPoint *> &vpMPsToUpdate, unsigned long nGBAId);

    bool CheckRunningGBA(){
        unique_lock<std::mutex> lock(mMutexGBA);
        return mbRunningGBA;
    }
    bool isFinishedGBA(){
        unique_lock<std::mutex> lock(mMutexGBA);
        return mbFinishedGBA;
    }   

    void RequestFinish();

    bool CheckFinished();


    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

protected:

    bool CheckNewKeyFrames();


    //Methods to implement the new place recognition algorithm
    bool DetectCommonRegionsExist();
    bool MatchKFsByProAndOptSim3AndPro(KeyFrame* pCurrentKF, KeyFrame* pCandidKF, g2o::Sim3 &gScw, int &nNumMatches,
                                       std::vector<MapPoint*> &vpCanCovMPs, std::vector<MapPoint*> &vpMatchedMPs);
    bool MatchKFsByBowAndIterSim3AndMatchKFAndMPsByProAndOptSIm3AndPro(std::vector<KeyFrame*> &vpCanKFs, KeyFrame* &pMatchedKF, KeyFrame* &pLastCurrentKF, g2o::Sim3 &g2oScw,
                                                                       int &nNumCoincidences, std::vector<MapPoint*> &vpCanMPs, std::vector<MapPoint*> &vpMatchedMPs);
    bool VerifyKFsByPro(KeyFrame* pCurrentKF, KeyFrame* pCandidKF, g2o::Sim3 &gScw, int &nNumProjMatches,
                        std::vector<MapPoint*> &vpMPs, std::vector<MapPoint*> &vpMatchedMPs);
    int FindMatchesByProjection(KeyFrame* pCurrentKF, KeyFrame* pCandidKFw, g2o::Sim3 &g2oScw,
                                vector<MapPoint*> &vpCanCovMPs,
                                vector<MapPoint*> &vpMatchedMPs);


    void FuseBetweenKFsAndMPsWithPose(const KFAndPose &CorrectedKFAndPose, vector<MapPoint*> &vpMapPoints);
    void FuseBetweenKFsAndMPsWithoutPose(const vector<KeyFrame*> &vConectedKFs, vector<MapPoint*> &vpMapPoints);

    void CorrectLoop();
    void MergeLocalWithImu();


    void ResetIfRequested();
    bool mbResetRequested;
    bool mbResetActiveMapRequested;
    Map* mpMapToReset;
    std::mutex mMutexReset;

    bool CheckRequestFinish();
    void SetFinished();
    bool mbRequestFinish;
    bool mbFinished;
    std::mutex mMutexFinish;

    Atlas* mpAtlas;
    KeyFrameDatabase* mpKeyFrameDB;
    ORBVocabulary* mpORBVocabulary;
    Tracking* mpTracker;
    LocalMapping *mpLocalMapper;
    Viewer* mpViewer;

    std::list<KeyFrame*> mlpKFQueueInLC;
    std::mutex mMutexNewKFQueue;

    // Variables related to Global Bundle Adjustment
    bool mbRunningGBA;
    bool mbFinishedGBA;
    bool mbStopGBA;
    std::mutex mMutexGBA;
    std::thread* mpThreadGBA;

    bool mbFixScale;
    bool mbActiveLC;

    // Loop detector parameters
    int mnThContiCoinSuccess;
    int mnThContiCoinGiveup;
    int mnThOriProjMatches;
    int mnThBoWMatches;
    int mnThIterInliers;
    int mnThOptInliers;
    int mnThIterProjMatches;
    int mnThOptProjMatches;

    // Loop detector variables
    KeyFrame* mpCurrentKF;
    Map* mpMapInCurKF;

    bool mbLoopCoinSuccess;
    int mnLoopContiCoinDetect;
    int mnLoopContiCoinFail;
    KeyFrame* mpLoopLastCoinKF;
    g2o::Sim3 mg2oLoopSlw;
    g2o::Sim3 mg2oLoopScw;
    KeyFrame* mpLoopMatchedKF;
    std::vector<MapPoint*> mvpLoopCandidMPs;
    std::vector<MapPoint*> mvpLoopMatchedMPs;

    bool mbMergeCoinSuccess;
    int mnMergeContiCoinDetect;
    int mnMergeContiCoinFail;
    KeyFrame* mpMergeLastCurrentKF;
    g2o::Sim3 mg2oMergeSlw;
    g2o::Sim3 mg2oMergeScw;
    KeyFrame* mpMergeMatchedKF;
    std::vector<MapPoint*> mvpMergeCandidMPs;
    std::vector<MapPoint*> mvpMergeMatchedMPs;
    std::vector<KeyFrame*> mvpMergeConnectedKFs;

    g2o::Sim3 mSold_new;
    //-------


};

} //namespace ORB_SLAM

#endif // LOOPCLOSING_H
