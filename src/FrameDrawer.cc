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

    FrameDrawer::FrameDrawer(Atlas *pAtlas, int nCN) : mpAtlas(pAtlas) {
        mState = Tracking::SYSTEM_NOT_READY;
        mImgLeft = cv::Mat(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));
        mImgRight = cv::Mat(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));
        mnColorNum = nCN;
        for (int i = 0; i < nCN; i++) {
            vRandColorSet.push_back(cv::Scalar(rand() % 255, rand() % 255, rand() % 255));
        }
    }

    cv::Mat FrameDrawer::DrawKeyFrame(float imageScale, bool bFrameBoth) {
        cv::Mat ImgLeft, ImgRight, ImgTmp;
        vector<int> vMatches; // Initialization: correspondeces with reference keypoints
        int State, nTrackMethod;
        vector<cv::KeyPoint> vIniKPsLeft, vCurKPsLeft; // KeyPoints in current frame
        vector<float> vfCurXInRight;
        std::vector<bool> vbOutlier;
        //Copy variables within scoped mutex
        {
            unique_lock<mutex> lock(mMutex);
            State = mState;
            nTrackMethod = mnTrackMethod;
            mImgLeft.copyTo(ImgLeft);
            mImgRight.copyTo(ImgRight);
            vCurKPsLeft = mvCurKPsLeft;
            vIniKPsLeft = mvIniKPsLeft;
            vfCurXInRight = mvfCurXInRight;
            vbOutlier = mvbOutlier;
            vMatches = mvIniMatches;
        }

        if (ImgLeft.channels() < 3) {
            cvtColor(ImgLeft, ImgLeft, cv::COLOR_GRAY2BGR);
        }
        if (ImgRight.channels() < 3) {
            cvtColor(ImgRight, ImgRight, cv::COLOR_GRAY2BGR);
        }
        //Draw
        if (bFrameBoth) {
            cv::hconcat(ImgLeft, ImgRight, ImgTmp);
        } else {
            ImgLeft.copyTo(ImgTmp);
        }
        if (State == Tracking::NOT_INITIALIZED) {
            for (unsigned int i = 0; i < vMatches.size(); i++) {
                if (vMatches[i] >= 0) {
                    cv::Point2f pt1, pt2;
                    pt1 = vIniKPsLeft[i].pt / imageScale;
                    pt2 = vCurKPsLeft[vMatches[i]].pt / imageScale;
                    cv::line(ImgTmp, pt1, pt2, vRandColorSet[i % mnColorNum]);
                }
            }
        } else if (State == Tracking::OK) {
            const float fRectR = 5;
            const float fCircleR = 3;
            int n = vCurKPsLeft.size();
            for (int i = 0; i < n; i++) {
                cv::Point2f pt1, pt2, pt3;
                pt1 = vCurKPsLeft[i].pt / imageScale;

                if ((vfCurXInRight[i] > 0)) {
                    //left KPs
                    cv::circle(ImgTmp, pt1, fCircleR, vRandColorSet[i % mnColorNum], fCircleR / 2);
                    //right KPs
                    if (bFrameBoth) {
                        pt2.x = (ImgLeft.cols + vfCurXInRight[i]) / imageScale;
                        pt2.y = pt1.y;
                        cv::circle(ImgTmp, pt2, fCircleR, vRandColorSet[i % mnColorNum], fCircleR / 2);
                        if (i % 4 == 0) {
                            cv::line(ImgTmp, pt1, pt2, vRandColorSet[i % mnColorNum]);
                        }
                    }
                }

                if (!vbOutlier[i]) {
                    pt2.x = pt1.x - fRectR;
                    pt2.y = pt1.y - fRectR;
                    pt3.x = pt1.x + fRectR;
                    pt3.y = pt1.y + fRectR;
                    cv::rectangle(ImgTmp, pt2, pt3, vRandColorSet[i % mnColorNum]);
                }
            }
        }

        DrawTextInfo(ImgTmp, State, nTrackMethod, ImgLeft);
        return ImgLeft;
    }


    void FrameDrawer::DrawTextInfo(cv::Mat &ImgOri, int nState, int nTrackMethod, cv::Mat &ImgWithText) {
        stringstream sShow;
        if (nState == Tracking::NO_IMAGES_YET) {
            sShow << " Wait ";
        } else if (nState == Tracking::NOT_INITIALIZED) {
            sShow << " Init";
        } else if (nState == Tracking::SYSTEM_NOT_READY) {
            sShow << " Load ";
        } else if (nState == Tracking::OK) {
            if (!mbOnlyTracking) {
                sShow << "SLAM ";
            } else {
                sShow << " Track ";
            }
        } else if (nState == Tracking::RECENTLY_LOST) {
            sShow << " Rece ";
        } else if (nState == Tracking::LOST) {
            sShow << " Lost ";
        }

        switch (nTrackMethod) {
            case (0): {
                sShow << "NoWay ";
                break;
            }
            case (1): {
                sShow << "Imu ";
                break;
            }
            case (2): {
                sShow << "Motion ";
                break;
            }
            case (3): {
                sShow << "RefKF ";
                break;
            }
            case (4): {
                sShow << "Reloc ";
                break;
            }
        }
        sShow << "| EFps:" << mnExtraFps << " TFps:" << mnTrackFps << " | ";
        int nMaps = mpAtlas->CountMaps();
        int nKFs = mpAtlas->KeyFramesInMap();
        int nMPs = mpAtlas->MapPointsInMap();
        sShow << "Maps: " << nMaps << ", KFs: " << nKFs << ", MPs: " << nMPs << ", TMap: " << mnTMap << " , MStero: "
              << mnMStereo;

        int nBaseline = 0;
        cv::Size TextSize = cv::getTextSize(sShow.str(), cv::FONT_HERSHEY_PLAIN, 1, 1, &nBaseline);
        ImgWithText = cv::Mat(ImgOri.rows + TextSize.height + 10, ImgOri.cols, ImgOri.type());
        ImgOri.copyTo(ImgWithText.rowRange(0, ImgOri.rows).colRange(0, ImgOri.cols));
        ImgWithText.rowRange(ImgOri.rows, ImgWithText.rows) = cv::Mat::zeros(TextSize.height + 10, ImgOri.cols,
                                                                             ImgOri.type());
        cv::putText(ImgWithText, sShow.str(), cv::Point(5, ImgWithText.rows - 5), cv::FONT_HERSHEY_PLAIN, 1,
                    cv::Scalar(255, 255, 255), 1, 8);

    }

    void FrameDrawer::Update(Tracking *pTracker, bool bFrameBoth) {
        unique_lock<mutex> lock(mMutex);
        pTracker->mImgLeft.copyTo(mImgLeft);
        if (bFrameBoth) {
            pTracker->mImgRight.copyTo(mImgRight);
        }

        if (pTracker->mLastProcessedState == Tracking::NOT_INITIALIZED) {
            mvIniMatches = pTracker->mvIniMatches;
        } else if (pTracker->mLastProcessedState == Tracking::OK) {
            mnTrackMethod = pTracker->mnTrackMethod;
            mvbOutlier = pTracker->mCurFrame.mvbOutlier;
            mvfCurXInRight = pTracker->mCurFrame.mvfXInRight;
        }

        mvCurKPsLeft = pTracker->mCurFrame.mvKPsLeft;
        mvIniKPsLeft = pTracker->mInitialFrame.mvKPsLeft;

        mState = static_cast<int>(pTracker->mLastProcessedState);
        mbOnlyTracking = pTracker->mbOnlyTracking;
        mnTrackFps = pTracker->mdTrackFps;
        mnExtraFps = pTracker->mdExtraFps;
        mnTMap = pTracker->mnLMInFMatchNum;
        mnMStereo = pTracker->mCurFrame.mnMStereo;
    }

} //namespace ORB_SLAM
