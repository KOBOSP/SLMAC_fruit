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

#ifndef ORB_SLAM3_SETTINGS_H
#define ORB_SLAM3_SETTINGS_H


#include "CameraModels/GeometricCamera.h"

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>

namespace ORB_SLAM3 {

    class System;

    //TODO: change to double instead of float

    class Settings {
    public:
        /*
         * Enum for the different camera types implemented
         */
        enum CameraType {
            PinHole = 0,
            Rectified = 1,
            KannalaBrandt = 2
        };

        /*
         * Delete default constructor
         */
        Settings() = delete;

        /*
         * Constructor from file
         */
        Settings(const std::string &configFile, const int &sensor);

        /*
         * Ostream operator overloading to dump settings to the terminal
         */
        friend std::ostream &operator<<(std::ostream &output, const Settings &s);


        cv::Mat GetCamera1DistoCoef() {
            return cv::Mat(mvfPinHoleDistorsion1.size(), 1, CV_32F, mvfPinHoleDistorsion1.data());
        }

        cv::Mat GetCamera2DistoCoef() {
            return cv::Mat(mvfPinHoleDistorsion2.size(), 1, CV_32F, mvfPinHoleDistorsion1.data());
        }


        template<typename T>
        T ReadParameter(cv::FileStorage &fSettings, const std::string &name, bool &found, const bool required = true) {
            cv::FileNode node = fSettings[name];
            if (node.empty()) {
                if (required) {
                    std::cerr << name << " required parameter does not exist, aborting..." << std::endl;
                    exit(-1);
                } else {
                    std::cerr << name << " optional parameter does not exist..." << std::endl;
                    found = false;
                    return T();
                }

            } else {
                found = true;
                return (T) node;
            }
        }

        void ReadCamera(cv::FileStorage &fSettings);

        void ReadImageInfo(cv::FileStorage &fSettings);

        void ReadIMU(cv::FileStorage &fSettings);

        void ReadORB(cv::FileStorage &fSettings);

        void ReadViewer(cv::FileStorage &fSettings);

        void ReadSystem(cv::FileStorage &fSettings);

        void PrecomputeRectificationMaps();

        int mSensor;
        CameraType mCameraType;     //Camera type

        /*
         * Visual stuff
         */
        GeometricCamera *mCalibration1, *mCalibration2;   //Camera calibration
        std::vector<float> mvfPinHoleDistorsion1, mvfPinHoleDistorsion2;

        cv::Size mOriginalImSize, mImgSize;
        float mfImgFps;
        bool mbRGB;

        bool mbNeedToRectify;
        bool mbNeedToResize;

        Sophus::SE3f mTlr;
        float mfThDepth;
        float mfBaselineFocal, mfBaseline;

        /*
         * Rectification stuff
         */
        cv::Mat Map1X, Map1Y;
        cv::Mat Map2X, Map2Y;

        /*
         * Inertial stuff
         */
        float mGyrNoise, mAccNoise;
        float mGyrWalk, mAccWalk;
        float mImuFreq;
        Sophus::SE3f mTbc;
        bool mbInsertKFsWhenLost;


        /*
         * ORB stuff
         */
        int mnFeatures;
        float mfScaleFactor;
        int mnLevels;
        int mnInitThFAST, mnMinThFAST;

        /*
         * Viewer stuff
         */
        float mfKeyFrameSize;
        float mfKeyFrameLineWidth;
        float mfGraphLineWidth;
        float mfPointSize;
        float mfCameraSize;
        float mfCameraLineWidth;
        float mfViewPointX, mfViewPointY, mfViewPointZ, mfViewPointF;
        float mfImageFrameScale;
        float mfMapWidth, mfMapHeight;
        int mnColorNum;
        /*
         * Save & load maps
         */
        std::string msLoadFrom, msSaveTo;
        float mfThFarPoints;
        bool mbActivateLC;
        float mfCullKFRedundantTh;
        int mnWeakCovisTh, mnStrongCovisTh, mnSingleMaxCullKFsNum;
        int mnThOriProjMatches, mnThBoWMatches, mnThIterInliers, mnThOptInliers, mnThIterProjMatches, mnThOptProjMatches;
        int mnThContiCoinSuccess;
        int mnThContiCoinGiveup;
        float mfThTimeRescueLost;
    };
};


#endif //ORB_SLAM3_SETTINGS_H
