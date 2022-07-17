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


#ifndef ORBMATCHER_H
#define ORBMATCHER_H

#include<vector>
#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>
#include"sophus/sim3.hpp"

#include"MapPoint.h"
#include"KeyFrame.h"
#include"Frame.h"


namespace ORB_SLAM3
{

    class ORBmatcher
    {
    public:

        ORBmatcher(float nnratio=0.6, bool checkOri=true);

        // Computes the Hamming distance between two ORB descriptors
        static int GetDescriptorDistance(const cv::Mat &a, const cv::Mat &b);


        // TrackWithMotionModel()||invzc,uv,bForward,bestDist,GetObsTimes,er,mbCheckOrientation
        int SearchFrameAndFrameByProject(Frame &CurrentFrame, const Frame &LastFrame, const float th, const bool bMono);
        // TrackReferenceKeyFrame() and Relocalization()||BestDist,secondDist,mbCheckOrientation
        int SearchMatchFrameAndKFByBoW(KeyFrame *pKF, Frame &F, std::vector<MapPoint*> &vpMPMatches);
        // Relocalization()||uv,Dist3D,bestDist,mbCheckOrientation
        int SearchFrameAndKFByProject(Frame &CurrentFrame, KeyFrame* pKF, const std::set<MapPoint*> &sAlreadyFound, const float th, const int ORBdist);
        //MatchLocalMPsToCurFrame()||bFarPoints,bestDist,secondDist,GetObsTimes,er
        int SearchReplaceFrameAndMPsByProject(Frame &F, const std::vector<MapPoint*> &vpMapPoints, const float nThProjRad= 3, const bool bFarPoints = false, const float thFarPoints = 50.0f);



        // CreateNewMapPoints()
        int SearchKFAndKFByTriangulation(KeyFrame *pKF1, KeyFrame* pKF2, std::vector<pair<size_t, size_t> > &vMatchedPairs, const bool bSkipExistStereoMP, const bool bCoarse = false);

        // FuseMapPointsInNeighbors()
        int SearchReplaceKFAndMPsByProjectInLocalMap(KeyFrame* pKF, const vector<MapPoint *> &vpMapPoints, const float nThProjRad=3.0, const bool bRight = false);
        // FuseBetweenKFs() and FuseBetweenKFAndMPs()
        int SearchReplaceKFAndMPsByProjectInGlobalMap(KeyFrame* pKF, Sophus::SE3f &Tcw, const std::vector<MapPoint*> &vpCandidMPs, float nThProjRad, vector<MapPoint *> &vpReplacePoint);
        // DetectCommonRegionsByBoWSearchAndProjectVerify() and FindMatchesByProjection()
        int SearchMatchKFAndMPsByProject(KeyFrame* pKF, Sophus::Sim3<float> &Scw, const std::vector<MapPoint*> &vpCandidMPs, std::vector<MapPoint*> &vpMatchedMPs, int th, float ratioHamming= 1.0);
        // DetectCommonRegionsByBoWSearchAndProjectVerify()
        int SearchMatchKFAndKFByBoW(KeyFrame *pKF1, KeyFrame* pKF2, std::vector<MapPoint*> &vpMatches12);

    public:

        static const int TH_LOW;
        static const int TH_HIGH;
        static const int HISTO_LENGTH;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    protected:
        float RadiusByViewingCos(const float &viewCos);
        void ComputeThreeMaxima(std::vector<int>* histo, const int L, int &ind1, int &ind2, int &ind3);
        bool GetBestKPIdxInKFToMP(KeyFrame *pKF, Sophus::SE3f &Tcw, MapPoint *pMP, float th,
                                  int &bestDist, int &bestIdx, bool bProjError);
        float mfNNratio;
        bool mbCheckOrientation;
    };

}// namespace ORB_SLAM

#endif // ORBMATCHER_H