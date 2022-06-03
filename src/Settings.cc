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

#include "Settings.h"

#include "CameraModels/Pinhole.h"
#include "CameraModels/KannalaBrandt8.h"

#include "System.h"

#include <opencv2/core/persistence.hpp>
#include <opencv2/core/eigen.hpp>

#include <iostream>

using namespace std;

namespace ORB_SLAM3 {

    template<>
    float Settings::ReadParameter<float>(cv::FileStorage& fSettings, const std::string& name, bool& found, const bool required){
        cv::FileNode node = fSettings[name];
        if(node.empty()){
            if(required){
                std::cerr << name << " required parameter does not exist, aborting..." << std::endl;
                exit(-1);
            }
            else{
                std::cerr << name << " optional parameter does not exist..." << std::endl;
                found = false;
                return 0.0f;
            }
        }
        else if(!node.isReal()){
            std::cerr << name << " parameter must be a real number, aborting..." << std::endl;
            exit(-1);
        }
        else{
            found = true;
            return node.real();
        }
    }

    template<>
    int Settings::ReadParameter<int>(cv::FileStorage& fSettings, const std::string& name, bool& found, const bool required){
        cv::FileNode node = fSettings[name];
        if(node.empty()){
            if(required){
                std::cerr << name << " required parameter does not exist, aborting..." << std::endl;
                exit(-1);
            }
            else{
                std::cerr << name << " optional parameter does not exist..." << std::endl;
                found = false;
                return 0;
            }
        }
        else if(!node.isInt()){
            std::cerr << name << " parameter must be an integer number, aborting..." << std::endl;
            exit(-1);
        }
        else{
            found = true;
            return node.operator int();
        }
    }

    template<>
    string Settings::ReadParameter<string>(cv::FileStorage& fSettings, const std::string& name, bool& found, const bool required){
        cv::FileNode node = fSettings[name];
        if(node.empty()){
            if(required){
                std::cerr << name << " required parameter does not exist, aborting..." << std::endl;
                exit(-1);
            }
            else{
                std::cerr << name << " optional parameter does not exist..." << std::endl;
                found = false;
                return string();
            }
        }
        else if(!node.isString()){
            std::cerr << name << " parameter must be a string, aborting..." << std::endl;
            exit(-1);
        }
        else{
            found = true;
            return node.string();
        }
    }

    template<>
    cv::Mat Settings::ReadParameter<cv::Mat>(cv::FileStorage& fSettings, const std::string& name, bool& found, const bool required){
        cv::FileNode node = fSettings[name];
        if(node.empty()){
            if(required){
                std::cerr << name << " required parameter does not exist, aborting..." << std::endl;
                exit(-1);
            }
            else{
                std::cerr << name << " optional parameter does not exist..." << std::endl;
                found = false;
                return cv::Mat();
            }
        }
        else{
            found = true;
            return node.mat();
        }
    }

    Settings::Settings(const std::string &configFile, const int& sensor) : mbNeedToRectify(false), mbNeedToResize(false) {
        mSensor = sensor;
        //Open settings file
        cv::FileStorage fSettings(configFile, cv::FileStorage::READ);
        if (!fSettings.isOpened()) {
            cerr << "[ERROR]: could not open configuration file at: " << configFile << endl;
            cerr << "Aborting..." << endl;
            exit(-1);
        }
        else{
            cout << "Loading settings from " << configFile << endl;
        }
        ReadCamera(fSettings);
        cout << "\t-Loaded camera 1 and 2 settings" << endl;
        ReadImageInfo(fSettings);
        cout << "\t-Loaded image settings" << endl;
        ReadIMU(fSettings);
        cout << "\t-Loaded IMU settings" << endl;
        ReadORB(fSettings);
        cout << "\t-Loaded ORB settings" << endl;
        ReadViewer(fSettings);
        cout << "\t-Loaded viewer settings" << endl;
        ReadSystem(fSettings);
        cout << "\t-Loaded System settings" << endl;

        if(mbNeedToRectify){
            PrecomputeRectificationMaps();
            cout << "\t-Computed rectification maps" << endl;
        }

        cout << "----------------------------------" << endl;
    }

    void Settings::ReadCamera(cv::FileStorage &fSettings) {
        bool found;
        vector<float> vCalibration;
        mCameraType = PinHole;
        mbNeedToRectify = true;

        //Read intrinsic parameters
        float fx = ReadParameter<float>(fSettings, "Camera1.fx", found);
        float fy = ReadParameter<float>(fSettings, "Camera1.fy", found);
        float cx = ReadParameter<float>(fSettings, "Camera1.cx", found);
        float cy = ReadParameter<float>(fSettings, "Camera1.cy", found);
        vCalibration = {fx, fy, cx, cy};
        mCalibration1 = new Pinhole(vCalibration);
        mvfPinHoleDistorsion1.resize(4);
        mvfPinHoleDistorsion1[0] = ReadParameter<float>(fSettings, "Camera1.k1", found);
        mvfPinHoleDistorsion1[1] = ReadParameter<float>(fSettings, "Camera1.k2", found);
        mvfPinHoleDistorsion1[2] = ReadParameter<float>(fSettings, "Camera1.p1", found);
        mvfPinHoleDistorsion1[3] = ReadParameter<float>(fSettings, "Camera1.p2", found);

        //Read intrinsic parameters
        fx = ReadParameter<float>(fSettings, "Camera2.fx", found);
        fy = ReadParameter<float>(fSettings, "Camera2.fy", found);
        cx = ReadParameter<float>(fSettings, "Camera2.cx", found);
        cy = ReadParameter<float>(fSettings, "Camera2.cy", found);
        vCalibration = {fx, fy, cx, cy};
        mCalibration2 = new Pinhole(vCalibration);
        mvfPinHoleDistorsion2.resize(4);
        mvfPinHoleDistorsion2[0] = ReadParameter<float>(fSettings, "Camera2.k1", found);
        mvfPinHoleDistorsion2[1] = ReadParameter<float>(fSettings, "Camera2.k2", found);
        mvfPinHoleDistorsion2[2] = ReadParameter<float>(fSettings, "Camera2.p1", found);
        mvfPinHoleDistorsion2[3] = ReadParameter<float>(fSettings, "Camera2.p2", found);

        //Load stereo extrinsic calibration
        cv::Mat cvTlr = ReadParameter<cv::Mat>(fSettings, "Stereo.T_c1_c2", found);
        mTlr = Converter::toSophus(cvTlr);
        mfBaseline = mTlr.translation().norm();
        mfBaselineFocal = mfBaseline * mCalibration1->GetParameter(0);
        mfThDepth = ReadParameter<float>(fSettings, "Stereo.ThDepth", found);
    }

    void Settings::ReadImageInfo(cv::FileStorage &fSettings) {
        bool found;
        //Read original and desired image dimensions
        int nOriginalRows = ReadParameter<int>(fSettings, "Camera.height", found);
        int nOriginalCols = ReadParameter<int>(fSettings, "Camera.width", found);
        mOriginalImSize.width = nOriginalCols;
        mOriginalImSize.height = nOriginalRows;
        mImgSize = mOriginalImSize;
        mfImgFps = ReadParameter<int>(fSettings, "Camera.fps", found);
        mbRGB = (bool) ReadParameter<int>(fSettings, "Camera.RGB", found);
    }

    void Settings::ReadIMU(cv::FileStorage &fSettings) {
        bool found;
        mGyrNoise = ReadParameter<float>(fSettings, "IMU.NoiseGyro", found);
        mAccNoise = ReadParameter<float>(fSettings, "IMU.NoiseAcc", found);
        mGyrWalk = ReadParameter<float>(fSettings, "IMU.GyroWalk", found);
        mAccWalk = ReadParameter<float>(fSettings, "IMU.AccWalk", found);
        mImuFreq = ReadParameter<float>(fSettings, "IMU.Frequency", found);
        cv::Mat cvTbc = ReadParameter<cv::Mat>(fSettings, "IMU.T_b_c1", found);
        mTbc = Converter::toSophus(cvTbc);
        mbInsertKFsWhenLost = (bool) ReadParameter<int>(fSettings, "IMU.InsertKFsWhenLost", found);
    }

    void Settings::ReadORB(cv::FileStorage &fSettings) {
        bool found;
        mnFeatures = ReadParameter<int>(fSettings, "ORBextractor.nFeatures", found);
        mfScaleFactor = ReadParameter<float>(fSettings, "ORBextractor.fScaleFactor", found);
        mnLevels = ReadParameter<int>(fSettings, "ORBextractor.nLevels", found);
        mnInitThFAST = ReadParameter<int>(fSettings, "ORBextractor.nIniThFAST", found);
        mnMinThFAST = ReadParameter<int>(fSettings, "ORBextractor.nMinThFAST", found);
    }

    void Settings::ReadViewer(cv::FileStorage &fSettings) {
        bool found;
        mfKeyFrameSize = ReadParameter<float>(fSettings, "Viewer.KeyFrameSize", found);
        mfKeyFrameLineWidth = ReadParameter<float>(fSettings, "Viewer.KeyFrameLineWidth", found);
        mfGraphLineWidth = ReadParameter<float>(fSettings, "Viewer.GraphLineWidth", found);
        mfPointSize = ReadParameter<float>(fSettings, "Viewer.PointSize", found);
        mfCameraSize = ReadParameter<float>(fSettings, "Viewer.CameraSize", found);
        mfCameraLineWidth = ReadParameter<float>(fSettings, "Viewer.CameraLineWidth", found);
        mfViewPointX = ReadParameter<float>(fSettings, "Viewer.ViewpointX", found);
        mfViewPointY = ReadParameter<float>(fSettings, "Viewer.ViewpointY", found);
        mfViewPointZ = ReadParameter<float>(fSettings, "Viewer.ViewpointZ", found);
        mfViewPointF = ReadParameter<float>(fSettings, "Viewer.ViewpointF", found);
        mfImageFrameScale = ReadParameter<float>(fSettings, "Viewer.imageViewScale", found);
        mfMapWidth = ReadParameter<float>(fSettings, "Viewer.MapWidth", found);
        mfMapHeight = ReadParameter<float>(fSettings, "Viewer.MapHeight", found);
    }

    void Settings::ReadSystem(cv::FileStorage &fSettings) {
        bool found;
        msLoadFrom = ReadParameter<string>(fSettings, "System.LoadAtlasFromFile", found, false);
        msSaveTo = ReadParameter<string>(fSettings, "System.SaveAtlasToFile", found, false);
        mfThFarPoints = ReadParameter<float>(fSettings, "System.thFarPoints", found);
        mbOpenLoop = ReadParameter<int>(fSettings, "LoopClosing.OpenLoop", found);
        mfCullKFRedundantTh = ReadParameter<float>(fSettings, "LocalMapping.CullKFRedundantTh", found);
        mnStrongCovisTh = ReadParameter<int>(fSettings, "LocalMapping.StrongCovisTh", found);
        mnWeakCovisTh = ReadParameter<int>(fSettings, "LocalMapping.WeakCovisTh", found);

    }

    void Settings::PrecomputeRectificationMaps() {
        //Precompute rectification maps, new calibrations, ...
        cv::Mat K1 = static_cast<Pinhole*>(mCalibration1)->toK();
        K1.convertTo(K1,CV_64F);
        cv::Mat K2 = static_cast<Pinhole*>(mCalibration2)->toK();
        K2.convertTo(K2,CV_64F);

        cv::Mat cvTlr;
        cv::eigen2cv(mTlr.inverse().matrix3x4(), cvTlr);
        cv::Mat R12 = cvTlr.rowRange(0,3).colRange(0,3);
        R12.convertTo(R12,CV_64F);
        cv::Mat t12 = cvTlr.rowRange(0,3).col(3);
        t12.convertTo(t12,CV_64F);

        cv::Mat R_r1_u1, R_r2_u2;
        cv::Mat P1, P2, Q;
        //https://docs.opencv.org/3.2.0/d9/d0c/group__calib3d.html#ga617b1685d4059c6040827800e72ad2b6
        //The function computes the rotation matrices for each camera that (virtually) make mbFrameBoth camera image planes the same plane.
        // Consequently, this makes all the epipolar lines parallel and thus simplifies the dense stereo correspondence problem.
        // The function takes the matrices computed by stereoCalibrate as input.
        // As output, it provides two rotation matrices and also two projection matrices in the new coordinates.
        //Horizontal stereo: the first and the second camera views are shifted relative to each other mainly along the x axis (with possible small vertical shift).
        // In the rectified images, the corresponding epipolar lines in the left and right cameras are horizontal and have the same y-coordinate.
        cv::stereoRectify(K1, GetCamera1DistoCoef(),
                          K2, GetCamera2DistoCoef(),
                          mImgSize, R12, t12,
                          R_r1_u1, R_r2_u2, P1, P2, Q,
                          cv::CALIB_ZERO_DISPARITY, -1, mImgSize);

        cv::initUndistortRectifyMap(K1, GetCamera1DistoCoef(), R_r1_u1, P1.rowRange(0, 3).colRange(0, 3),
                                    mImgSize, CV_32F, Map1X, Map1Y);
        cv::initUndistortRectifyMap(K2, GetCamera2DistoCoef(), R_r2_u2, P2.rowRange(0, 3).colRange(0, 3),
                                    mImgSize, CV_32F, Map2X, Map2Y);

        //Update calibration
        mCalibration1->setParameter(P1.at<double>(0, 0), 0);
        mCalibration1->setParameter(P1.at<double>(1, 1), 1);
        mCalibration1->setParameter(P1.at<double>(0, 2), 2);
        mCalibration1->setParameter(P1.at<double>(1, 2), 3);

        //Update bf
        mfBaselineFocal = mfBaseline * P1.at<double>(0, 0);

        //Update relative pose between camera 1 and IMU if necessary
        Eigen::Matrix3f eigenR_r1_u1;
        cv::cv2eigen(R_r1_u1,eigenR_r1_u1);
        Sophus::SE3f T_r1_u1(eigenR_r1_u1,Eigen::Vector3f::Zero());
        mTbc = mTbc * T_r1_u1.inverse();
    }

    ostream &operator<<(std::ostream& output, const Settings& settings){
        output << "SLAM settings: " << endl;

        output << "\t-Camera 1 parameters: [";
        for(size_t i = 0; i < settings.mCalibration1->size(); i++){
            output << " " << settings.mCalibration1->GetParameter(i);
        }
        output << " ]" << endl;
        output << "\t-Camera 1 distortion parameters: [ ";
        for(float d : settings.mvfPinHoleDistorsion1){
            output << " " << d;
        }
        output << " ]" << endl;

        output << "\t-Camera 2 parameters: [";
        for(size_t i = 0; i < settings.mCalibration2->size(); i++){
            output << " " << settings.mCalibration2->GetParameter(i);
        }
        output << " ]" << endl;
        output << "\t-Camera 2 distortion parameters: [ ";
        for(float d : settings.mvfPinHoleDistorsion2){
            output << " " << d;
        }
        output << " ]" << endl;

        output << "\t-Original image size: [ " << settings.mOriginalImSize.width << " , " << settings.mOriginalImSize.height << " ]" << endl;
        output << "\t-Current image size: [ " << settings.mImgSize.width << " , " << settings.mImgSize.height << " ]" << endl;


        output << "\t-Camera 1 parameters after rectification: [ ";
        for(size_t i = 0; i < settings.mCalibration1->size(); i++){
            output << " " << settings.mCalibration1->GetParameter(i);
        }
        output << " ]" << endl;


        output << "\t-Sequence FPS: " << settings.mfImgFps << endl;
        output << "\t-Stereo baseline: " << settings.mfBaseline << endl;
        output << "\t-Stereo depth threshold : " << settings.mfThDepth << endl;


        output << "\t-Gyro noise: " << settings.mGyrNoise << endl;
        output << "\t-Accelerometer noise: " << settings.mAccNoise << endl;
        output << "\t-Gyro walk: " << settings.mGyrWalk << endl;
        output << "\t-Accelerometer walk: " << settings.mAccWalk << endl;
        output << "\t-IMU frequency: " << settings.mImuFreq << endl;


        output << "\t-Features per image: " << settings.mnFeatures << endl;
        output << "\t-ORB scale factor: " << settings.mfScaleFactor << endl;
        output << "\t-ORB number of scales: " << settings.mnLevels << endl;
        output << "\t-Initial FAST threshold: " << settings.mnInitThFAST << endl;
        output << "\t-Min FAST threshold: " << settings.mnMinThFAST << endl;

        return output;
    }
};
