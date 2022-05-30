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

#include "FrameDrawer.h"
#include "Tracking.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include<mutex>

namespace ORB_SLAM3 {

    FrameDrawer::FrameDrawer(Atlas *pAtlas) : mpAtlas(pAtlas) {
        mState = Tracking::SYSTEM_NOT_READY;
        mImgLeft = cv::Mat(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));
        mImgRight = cv::Mat(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));
    }

    cv::Mat FrameDrawer::DrawKeyFrame(float imageScale, bool bFrameBoth) {
        cv::Mat ImgLeft, ImgRight, ImgTmp;
        vector<int> vMatches; // Initialization: correspondeces with reference keypoints
        vector<bool> vbVO, vbMap; // Tracked MapPoints in current frame
        int State;
        vector<cv::KeyPoint> vIniKPsLeft, vCurKPsLeft, vCurKPsRight; // KeyPoints in current frame
        cv::Scalar ColorA(0, 255, 0);
        cv::Scalar ColorB(255, 0, 0);

        //Copy variables within scoped mutex
        {
            unique_lock<mutex> lock(mMutex);
            State = mState;
            if (mState == Tracking::SYSTEM_NOT_READY)
                mState = Tracking::NO_IMAGES_YET;
            mImgLeft.copyTo(ImgLeft);
            mImgRight.copyTo(ImgRight);
            vCurKPsLeft = mvCurKPsLeft;
            vIniKPsLeft = mvIniKPsLeft;
            vCurKPsRight = mvCurKPsRight;
            if (mState == Tracking::NOT_INITIALIZED) {
                vMatches = mvIniMatches;
            } else if (mState == Tracking::OK) {
                vbVO = mvbVO;
                vbMap = mvbMap;
            }
        }

        if (ImgLeft.channels()<3){
            cvtColor(ImgLeft, ImgLeft, cv::COLOR_GRAY2BGR);
        }
        if (ImgRight.channels()<3){
            cvtColor(ImgRight, ImgRight, cv::COLOR_GRAY2BGR);
        }
        //Draw
        if (State == Tracking::NOT_INITIALIZED) {
            for (unsigned int i = 0; i < vMatches.size(); i++) {
                if (vMatches[i] >= 0) {
                    cv::Point2f pt1, pt2;
                    pt1 = vIniKPsLeft[i].pt / imageScale;
                    pt2 = vCurKPsLeft[vMatches[i]].pt / imageScale;
                    cv::line(ImgLeft, pt1, pt2, ColorA);
                }
            }
        } else if (State == Tracking::OK){
            mnTrackedMap = 0;
            mnTrackedVO = 0;
            const float r = 5;
            int n = vCurKPsLeft.size();
            for (int i = 0; i < n; i++) {
                if (vbVO[i] || vbMap[i]) {
                    cv::Point2f pt1, pt2;
                    cv::Point2f point;
                    point = vCurKPsLeft[i].pt / imageScale;
                    float px = vCurKPsLeft[i].pt.x / imageScale;
                    float py = vCurKPsLeft[i].pt.y / imageScale;
                    pt1.x = px - r;
                    pt1.y = py - r;
                    pt2.x = px + r;
                    pt2.y = py + r;
                    if (vbMap[i]) {
                        cv::rectangle(ImgLeft, pt1, pt2, ColorA);
                        cv::circle(ImgLeft, point, 2, ColorA, -1);
                        mnTrackedMap++;
                    } else {
                        cv::rectangle(ImgLeft, pt1, pt2, ColorB);
                        cv::circle(ImgLeft, point, 2, ColorB, -1);
                        mnTrackedVO++;
                    }
                }
            }
        }

        ImgLeft.copyTo(ImgTmp);
        if(bFrameBoth){
            cv::resize(ImgRight, ImgRight, cv::Size(ImgLeft.cols, ImgLeft.rows));
            cv::hconcat(ImgLeft, ImgRight, ImgTmp);
        }
        DrawTextInfo(ImgTmp, State, ImgLeft);
        return ImgLeft;
    }



    void FrameDrawer::DrawTextInfo(cv::Mat &ImgOri, int nState, cv::Mat &ImgWithText) {
        stringstream sShow;
        if (nState == Tracking::NO_IMAGES_YET)
            sShow << " WAITING FOR IMAGES";
        else if (nState == Tracking::NOT_INITIALIZED)
            sShow << " TRYING TO INITIALIZE ";
        else if (nState == Tracking::OK) {
            if (!mbOnlyTracking)
                sShow << "SLAM MODE | ";
            else
                sShow << "LOCALIZATION | ";
            int nMaps = mpAtlas->CountMaps();
            int nKFs = mpAtlas->KeyFramesInMap();
            int nMPs = mpAtlas->MapPointsInMap();
            sShow << "Maps: " << nMaps << ", KFs: " << nKFs << ", MPs: " << nMPs << ", TMap: " << mnTrackedMap << " , TVO: " << mnTrackedVO;
        } else if (nState == Tracking::LOST) {
            sShow << " TRACK LOST. TRYING TO RELOCALIZE ";
        } else if (nState == Tracking::SYSTEM_NOT_READY) {
            sShow << " LOADING ORB VOCABULARY. PLEASE WAIT...";
        }

        int nBaseline = 0;
        cv::Size TextSize = cv::getTextSize(sShow.str(), cv::FONT_HERSHEY_PLAIN, 1, 1, &nBaseline);
        ImgWithText = cv::Mat(ImgOri.rows + TextSize.height + 10, ImgOri.cols, ImgOri.type());
        ImgOri.copyTo(ImgWithText.rowRange(0, ImgOri.rows).colRange(0, ImgOri.cols));
        ImgWithText.rowRange(ImgOri.rows, ImgWithText.rows) = cv::Mat::zeros(TextSize.height + 10, ImgOri.cols, ImgOri.type());
        cv::putText(ImgWithText, sShow.str(), cv::Point(5, ImgWithText.rows - 5), cv::FONT_HERSHEY_PLAIN, 1,
                    cv::Scalar(255, 255, 255), 1, 8);

    }

    void FrameDrawer::Update(Tracking *pTracker, bool bFrameBoth) {
        unique_lock<mutex> lock(mMutex);
        pTracker->mImgLeft.copyTo(mImgLeft);
        mvCurKPsLeft = pTracker->mCurFrame.mvKPsLeft;
        mvIniKPsLeft = pTracker->mInitialFrame.mvKPsLeft;
        mnCurKPsLeft = mvCurKPsLeft.size();
        if (bFrameBoth) {
            pTracker->mImgRight.copyTo(mImgRight);
            mvCurKPsRight = pTracker->mCurFrame.mvKPsRight;
        }

        mvbVO = vector<bool>(mnCurKPsLeft, false);
        mvbMap = vector<bool>(mnCurKPsLeft, false);
        mbOnlyTracking = pTracker->mbOnlyTracking;

        if (pTracker->mLastProcessedState == Tracking::NOT_INITIALIZED) {
            mvIniMatches = pTracker->mvIniMatches;
        } else if (pTracker->mLastProcessedState == Tracking::OK) {
            for (int i = 0; i < mnCurKPsLeft; i++) {
                MapPoint *pMP = pTracker->mCurFrame.mvpMPs[i];
                if (pMP) {
                    if (!pTracker->mCurFrame.mvbOutlier[i]) {
                        if (pMP->Observations() > 2)
                            mvbMap[i] = true;
                        else
                            mvbVO[i] = true;
                    }
                }
            }
        }
        mState = static_cast<int>(pTracker->mLastProcessedState);
    }

} //namespace ORB_SLAM
