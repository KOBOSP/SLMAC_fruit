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


#ifndef FRAMEDRAWER_H
#define FRAMEDRAWER_H

#include "Tracking.h"
#include "MapPoint.h"
#include "Atlas.h"

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include<mutex>
#include <unordered_set>


namespace ORB_SLAM3
{

class Tracking;
class Viewer;

class FrameDrawer
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    FrameDrawer(Atlas* pAtlas);

    // Update info from the last processed frame.
    void Update(Tracking *pTracker, bool bFrameBoth);

    // Draw last processed frame.
    cv::Mat DrawKeyFrame(float imageScale, bool bFrameBoth);


protected:

    void DrawTextInfo(cv::Mat &ImgOri, int nState, cv::Mat &ImgWithText);

    // Info of the frame to be drawn
    cv::Mat mImgLeft, mImgRight;
    int mnCurKPsLeft;
    vector<cv::KeyPoint> mvCurKPsLeft, mvCurKPsRight, mvIniKPsLeft;
    vector<bool> mvbMap, mvbVO;
    bool mbOnlyTracking;
    int mnTrackedMap, mnTrackedVO;
    vector<int> mvIniMatches;
    int mState;

    Atlas* mpAtlas;

    std::mutex mMutex;

};

} //namespace ORB_SLAM

#endif // FRAMEDRAWER_H
