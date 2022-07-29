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



#include "System.h"
#include "Converter.h"
#include <thread>
#include <pangolin/pangolin.h>
#include <iomanip>
#include <openssl/md5.h>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/string.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>

namespace ORB_SLAM3 {

    Verbose::eLevel Verbose::th = Verbose::VERBOSITY_NORMAL;

/**
 * @brief 系统的构造函数，将会启动其他的线程
 * @param sVocFile 词袋文件所在路径
 * @param sSettingFile 配置文件所在路径
 * @param Sensor 传感器类型
 * @param bUseViewer 是否使用可视化界面
 * @param nFrameIdInit initFr表示初始化帧的id,开始设置为0
 * @param sSeqName 序列名,在跟踪线程和局部建图线程用得到
 */
    System::System(const string &sVocFile, const string &sSettingFile, const eSensor Sensor,
                   const bool bUseViewer, const string &sSeqName) :
            mSensor(Sensor), mpViewer(static_cast<Viewer *>(NULL)), mbRequestResetActiveMap(false),
            mbRequestOnlyTrackingMode(false), mbRequestSLAMMode(false), mbShutDowned(false) {
        cout << "Input Sensor was set to: Stereo-Inertial" << endl;       // 双目 + imu
        mSettings = new Settings(sSettingFile, mSensor);
        // 保存及加载地图的名字
        msLoadAtlasFromFile = mSettings->msLoadFrom;
        msSaveAtlasToFile = mSettings->msSaveTo;
        msSaveFAndKFSeqName = sSeqName;
        mbRGB = mSettings->mbRGB;
        cout << (*mSettings) << endl;

        msVocabularyFilePath = sVocFile;
        cout << endl << "Loading ORB Vocabulary. This could take a while..." << endl;
        mpVocabulary = new ORBVocabulary();
        bool bVocLoad = mpVocabulary->loadFromTextFile(sVocFile);
        if (!bVocLoad) {
            cerr << "Wrong path to vocabulary. " << endl;
            cerr << "Falied to open at: " << sVocFile << endl;
            exit(-1);
        }
        cout << "Vocabulary loaded!" << endl << endl;
        mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);


        if (msLoadAtlasFromFile.empty()) {
            cout << "Initialization of Atlas from scratch " << endl;
            mpAtlas = new Atlas(0);
        } else {
            cout << "Initialization of Atlas from file: " << msLoadAtlasFromFile << endl;
            mpAtlas->CreateNewMap();
        }
        KeyFrame::mnStrongCovisTh = mSettings->mnStrongCovisTh;

        mpFrameDrawer = new FrameDrawer(mpAtlas, mSettings->mnColorNum);
        mpMapDrawer = new MapDrawer(mpAtlas, sSettingFile, mSettings);
        cout << "Seq. Name: " << sSeqName << endl;
        mpTracker = new Tracking(this, mpVocabulary, mpFrameDrawer, mpMapDrawer,
                                 mpAtlas, mpKeyFrameDatabase, sSettingFile, mSensor, mSettings);
        mpLocalMapper = new LocalMapping(this, mpAtlas, false, true, mSettings);
        mptLocalMapping = new thread(&ORB_SLAM3::LocalMapping::Run, mpLocalMapper);
        mpLoopCloser = new LoopClosing(mpAtlas, mpKeyFrameDatabase, mpVocabulary, true, mSettings->mbActivateLC,
                                       mSettings);
        mptLoopClosing = new thread(&ORB_SLAM3::LoopClosing::Run, mpLoopCloser);

        mpTracker->SetLocalMapper(mpLocalMapper);
        mpTracker->SetLoopClosing(mpLoopCloser);
        mpLocalMapper->SetTracker(mpTracker);
        mpLocalMapper->SetLoopCloser(mpLoopCloser);
        mpLoopCloser->SetTracker(mpTracker);
        mpLoopCloser->SetLocalMapper(mpLocalMapper);


        if (bUseViewer) {
            mpViewer = new Viewer(this, mpFrameDrawer, mpMapDrawer, mpTracker, sSettingFile, mSettings);
            mptViewer = new thread(&Viewer::Run, mpViewer);
            mpTracker->SetViewer(mpViewer);
            mpLoopCloser->SetViewer(mpViewer);
            mpViewer->mbFrameBoth = true;
        }
        // Fix verbosity
//        Verbose::SetTh(Verbose::VERBOSITY_QUIET);
        Verbose::SetTh(Verbose::VERBOSITY_DEBUG);
    }

    Sophus::SE3f System::CalibAndTrack(const cv::Mat &ImgLeft, const cv::Mat &ImgRight, Eigen::Matrix<float, 3, 1> trw,
                                       const double &dTimestamp,
                                       const vector<IMU::Point> &vImuMeas) {
        // Check mode change
        {
            unique_lock<mutex> lock(mMutexMode);
            if (mbRequestOnlyTrackingMode) {
                mpLocalMapper->RequestPause();
                // Wait until Local Mapping has effectively stopped
                while (!mpLocalMapper->CheckPaused()) {
                    usleep(5000);
                }
                mpTracker->InformOnlyTracking(true);
                mbRequestOnlyTrackingMode = false;
            }
            if (mbRequestSLAMMode) {
                mpTracker->InformOnlyTracking(false);
                mpLocalMapper->CancelPause();
                mbRequestSLAMMode = false;
            }
        }
        // Check reset
        {
            unique_lock<mutex> lock(mMutexReset);
            if (mbRequestResetActiveMap) {
                mpTracker->ResetActiveMap();
                mbRequestResetActiveMap = false;
            } else if (mbRequestShutDown) {
                ShutDownSystem();
                mbShutDowned = true;
                return Sophus::SE3<float>();
            }
        }
        cv::Mat ImgLeftToTrack, ImgRightToTrack;
        if (mSettings && mSettings->mbNeedToRectify) {
            cv::Mat MXL = mSettings->Map1X;
            cv::Mat MYL = mSettings->Map1Y;
            cv::Mat MXR = mSettings->Map2X;
            cv::Mat MYR = mSettings->Map2Y;
            cv::remap(ImgLeft, ImgLeftToTrack, MXL, MYL, cv::INTER_LINEAR);
            cv::remap(ImgRight, ImgRightToTrack, MXR, MYR, cv::INTER_LINEAR);
        } else if (mSettings && mSettings->mbNeedToResize) {
            cv::resize(ImgLeft, ImgLeftToTrack, mSettings->mImgSize);
            cv::resize(ImgRight, ImgRightToTrack, mSettings->mImgSize);
        } else {
            ImgLeftToTrack = ImgLeft.clone();
            ImgRightToTrack = ImgRight.clone();
        }
        ImgLeftToTrack.copyTo(mpTracker->mImgLeftToViewer);
        ImgRightToTrack.copyTo(mpTracker->mImgRightToViewer);
        if (ImgLeftToTrack.channels() == 3) {
            if (mbRGB) {
                cvtColor(ImgLeftToTrack, ImgLeftToTrack, cv::COLOR_RGB2GRAY);
                cvtColor(ImgRightToTrack, ImgRightToTrack, cv::COLOR_RGB2GRAY);
            }
        } else if (ImgLeftToTrack.channels() == 4) {
            if (mbRGB) {
                cvtColor(ImgLeftToTrack, ImgLeftToTrack, cv::COLOR_RGBA2GRAY);
                cvtColor(ImgRightToTrack, ImgRightToTrack, cv::COLOR_RGBA2GRAY);
            }
        }
        for (size_t nImu = 0; nImu < vImuMeas.size(); nImu++) {
            mpTracker->GrabImuData(vImuMeas[nImu]);
        }

        Sophus::SE3f Tcw = mpTracker->GrabImageStereo(ImgLeftToTrack, ImgRightToTrack, trw, dTimestamp);

        return Tcw;
    }


    void System::ActivateLocalizationMode() {
        unique_lock<mutex> lock(mMutexMode);
        mbRequestOnlyTrackingMode = true;
    }

    void System::DeactivateLocalizationMode() {
        unique_lock<mutex> lock(mMutexMode);
        mbRequestSLAMMode = true;
    }



    void System::RequestResetActiveMap() {
        unique_lock<mutex> lock(mMutexReset);
        mbRequestResetActiveMap = true;
    }

    void System::RequestRequestShutDown() {
        unique_lock<mutex> lock(mMutexReset);
        mbRequestShutDown = true;
    }


    void System::ShutDownSystem() {
        mpLocalMapper->RequestFinish();
        mpLoopCloser->RequestFinish();
        // Wait until all thread have effectively stopped
        while (!mpLocalMapper->CheckFinished() || !mpLoopCloser->CheckFinished() || mpLoopCloser->CheckRunningGBA()) {
            if (!mpLocalMapper->CheckFinished()) {
                cout << "mpLocalMapping is not finished" << endl;
            }
            if (!mpLoopCloser->CheckFinished()) {
                cout << "mpLoopCloser is not finished" << endl;
            }
            if (mpLoopCloser->CheckRunningGBA()) {
                cout << "mpLoopCloser is running GBA" << endl;
            }
            usleep(5000);
        }

        if (mpViewer) {
            mpViewer->RequestFinish();
            while (!mpViewer->CheckFinished()) {
                usleep(5000);
            }
        }

        if (!msSaveAtlasToFile.empty()) {
            Verbose::PrintMess("Atlas saving to file " + msSaveAtlasToFile, Verbose::VERBOSITY_NORMAL);
        }
        cout << "System Finished" << endl;
    }

    bool System::CheckShutDowned() {
        unique_lock<mutex> lock(mMutexReset);
        return mbShutDowned;
    }


    void System::SaveKeyFrameTrajectoryEuRoC() {
        const string filename = "KF" + msSaveFAndKFSeqName;
        cout << endl << "Saving KeyFrame Trajectory to " << filename << " ..." << endl;

        vector<Map *> vpMaps = mpAtlas->GetAllMaps();
        Map *pBiggerMap;
        int numMaxKFs = 0;
        for (Map *pMap: vpMaps) {
            if (pMap && pMap->GetKeyFramesNumInMap() > numMaxKFs) {
                numMaxKFs = pMap->GetKeyFramesNumInMap();
                pBiggerMap = pMap;
            }
        }

        if (!pBiggerMap) {
            std::cout << "There is not a map!!" << std::endl;
            return;
        }

        vector<KeyFrame *> vpKFs = pBiggerMap->GetAllKeyFrames();
        sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

        // Transform all keyframes so that the first keyframe is at the origin.
        // After a loop closure the first keyframe might not be at the origin.
        ofstream fKFPose;
        fKFPose.open(filename.c_str());
        fKFPose << fixed;

        for (size_t i = 0; i < vpKFs.size(); i++) {
            KeyFrame *pKF = vpKFs[i];

            // pKF->SetPose(pKF->GetPose()*Two);

            if (!pKF || pKF->isBad())
                continue;
            Sophus::SE3f Twb = pKF->GetImuPose();
            Eigen::Quaternionf q = Twb.unit_quaternion();
            Eigen::Vector3f twb = Twb.translation();
            fKFPose << setprecision(6) << 1e9 * pKF->mdTimestamp << " " << setprecision(9)
                    << twb(0) << " " << twb(1) << " " << twb(2) << " "
                    << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
        }
        fKFPose.close();
        cout << endl << "Done" << endl;
    }

    void System::ChangeDataset() {
        if (mpAtlas->GetCurrentMap()->GetKeyFramesNumInMap() < 12) {
            mpTracker->ResetActiveMap();
        } else {
            mpTracker->CreateMapInAtlas();
        }
    }
} //namespace ORB_SLAM

