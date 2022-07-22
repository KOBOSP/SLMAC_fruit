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
            mSensor(Sensor), mpViewer(static_cast<Viewer *>(NULL)), mbResetThread(false), mbResetActiveMap(false),
            mbActivateLocalizationMode(false), mbDeactivateLocalizationMode(false), mbShutDowned(false) {
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
            bool isRead = LoadAtlas(FileType::BINARY_FILE);
            if (!isRead) {
                cout << "Error to load the file, please try with other session file or vocabulary file" << endl;
                exit(-1);
            }
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
            if (mbActivateLocalizationMode) {
                mpLocalMapper->RequestPause();
                // Wait until Local Mapping has effectively stopped
                while (!mpLocalMapper->CheckPaused()) {
                    usleep(5000);
                }
                mpTracker->InformOnlyTracking(true);
                mbActivateLocalizationMode = false;
            }
            if (mbDeactivateLocalizationMode) {
                mpTracker->InformOnlyTracking(false);
                mpLocalMapper->CancelPause();
                mbDeactivateLocalizationMode = false;
            }
        }
        // Check reset
        {
            unique_lock<mutex> lock(mMutexReset);
            if (mbResetThread) {
                mpTracker->ResetThread();
                mbResetThread = false;
                mbResetActiveMap = false;
            } else if (mbResetActiveMap) {
                mpTracker->ResetActiveMap();
                mbResetActiveMap = false;
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
        ImgLeftToTrack.copyTo(mpTracker->mImgLeft);
        ImgRightToTrack.copyTo(mpTracker->mImgRight);
        if (mImgLeftToViewer.channels() == 3) {
            if (mbRGB) {
                cvtColor(ImgLeftToTrack, ImgLeftToTrack, cv::COLOR_RGB2GRAY);
                cvtColor(ImgRightToTrack, ImgRightToTrack, cv::COLOR_RGB2GRAY);
            }
        } else if (mImgLeftToViewer.channels() == 4) {
            if (mbRGB) {
                cvtColor(ImgLeftToTrack, ImgLeftToTrack, cv::COLOR_RGBA2GRAY);
                cvtColor(ImgRightToTrack, ImgRightToTrack, cv::COLOR_RGBA2GRAY);
            }
        }
        for (size_t nImu = 0; nImu < vImuMeas.size(); nImu++) {
            mpTracker->GrabImuData(vImuMeas[nImu]);
        }
        // std::cout << "start GrabImageStereo" << std::endl;
        Sophus::SE3f Tcw = mpTracker->GrabImageStereo(ImgLeftToTrack, ImgRightToTrack, trw, dTimestamp);
        // std::cout << "out grabber" << std::endl;
        {
            unique_lock<mutex> lock2(mMutexState);
            mTrackingState = mpTracker->mState;
            mTrackedMPs = mpTracker->mCurFrame.mvpMPs;
            mTrackedKPsUn = mpTracker->mCurFrame.mvKPsUn;
        }
        return Tcw;
    }


    void System::ActivateLocalizationMode() {
        unique_lock<mutex> lock(mMutexMode);
        mbActivateLocalizationMode = true;
    }

    void System::DeactivateLocalizationMode() {
        unique_lock<mutex> lock(mMutexMode);
        mbDeactivateLocalizationMode = true;
    }

    bool System::MapChanged() {
        static int n = 0;
        int curn = mpAtlas->GetLastBigChangeIdx();
        if (n < curn) {
            n = curn;
            return true;
        } else
            return false;
    }

    void System::ResetThread() {
        unique_lock<mutex> lock(mMutexReset);
        mbResetThread = true;
    }

    void System::ResetActiveMap() {
        unique_lock<mutex> lock(mMutexReset);
        mbResetActiveMap = true;
    }

    void System::RequestShutDown() {
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
            SaveAtlas(FileType::BINARY_FILE);
        }
        cout << "System Finished" << endl;
    }

    bool System::CheckShutDowned() {
        unique_lock<mutex> lock(mMutexReset);
        return mbShutDowned;
    }


    void System::SaveFrameTrajectoryEuRoC() {

        const string filename = "Frame" + msSaveFAndKFSeqName;
        cout << endl << "Saving Frame Trajectory to " << filename << " ..." << endl;

        vector<Map *> vpMaps = mpAtlas->GetAllMaps();
        int numMaxKFs = 0;
        Map *pBiggerMap;
        std::cout << "There are " << std::to_string(vpMaps.size()) << " maps in the atlas" << std::endl;
        for (Map *pMap: vpMaps) {
            std::cout << "  Map " << std::to_string(pMap->GetId()) << " has "
                      << std::to_string(pMap->GetKeyFramesNumInMap()) << " KFs" << std::endl;
            if (pMap->GetKeyFramesNumInMap() > numMaxKFs) {
                numMaxKFs = pMap->GetKeyFramesNumInMap();
                pBiggerMap = pMap;
            }
        }

        vector<KeyFrame *> vpKFs = pBiggerMap->GetAllKeyFrames();
        sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

        // Transform all keyframes so that the first keyframe is at the origin.
        // After a loop closure the first keyframe might not be at the origin.
        Sophus::SE3f Twb; // Can be word to cam0 or world to mBiasOri depending on IMU or not.
        Twb = vpKFs[0]->GetImuPose();

        ofstream fFPose;
        fFPose.open(filename.c_str());
        // cout << "file open" << endl;
        fFPose << fixed;

        // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
        // We need to get first the keyframe pose and then concatenate the relative transformation.
        // Frames not localized (tracking failure) are not saved.

        // For each frame we have a reference keyframe (lRKFit), the timestamp (lFT) and a flag
        // which is true when tracking failed (lbL).
        list<ORB_SLAM3::KeyFrame *>::iterator lRKFit = mpTracker->mlpRefKFs.begin();
        list<double>::iterator lFT = mpTracker->mlTimestamp.begin();
        list<bool>::iterator lbL = mpTracker->mlbLost.begin();

        //cout << "ParameterSize mlpRefKFs: " << mpTracker->mlpRefKFs.size() << endl;
        //cout << "size mlRelativeFramePoses: " << mpTracker->mlRelativeFramePoses.ParameterSize() << endl;
        //cout << "size mpTracker->mlTimestamp: " << mpTracker->mlTimestamp.ParameterSize() << endl;
        //cout << "ParameterSize mpTracker->mlbLost: " << mpTracker->mlbLost.size() << endl;


        for (auto lit = mpTracker->mlRelativeFramePoses.begin(),
                     lend = mpTracker->mlRelativeFramePoses.end(); lit != lend; lit++, lRKFit++, lFT++, lbL++) {
            if (*lbL)
                continue;
            KeyFrame *pKF = *lRKFit;
            //cout << "KF: " << pKF->mnId << endl;

            Sophus::SE3f Trw;

            // If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
            if (!pKF)
                continue;

            while (pKF->isBad()) {
                Trw = Trw * pKF->mTcp;
                pKF = pKF->GetParent();
//                cout << "--Parent KF: " << pKF->mnId << endl;
            }

            if (!pKF || pKF->GetMap() != pBiggerMap) {
//                cout << "--Parent KF is from another map" << endl;
                continue;
            }
            Trw = Trw * pKF->GetPose() * Twb; // Tcp*Tpw*Twb0=Tcb0 where b0 is the new world reference

            Sophus::SE3f Twb = (pKF->mImuCalib.mTbc * (*lit) * Trw).inverse();
            Eigen::Quaternionf q = Twb.unit_quaternion();
            Eigen::Vector3f twb = Twb.translation();
            fFPose << setprecision(6) << 1e9 * (*lFT) << " " << setprecision(9) << twb(0) << " " << twb(1) << " "
                   << twb(2)
                   << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
        }
        fFPose.close();
        cout << endl << "Done" << endl;
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


    int System::GetTrackingState() {
        unique_lock<mutex> lock(mMutexState);
        return mTrackingState;
    }

    vector<MapPoint *> System::GetTrackedMapPoints() {
        unique_lock<mutex> lock(mMutexState);
        return mTrackedMPs;
    }

    vector<cv::KeyPoint> System::GetTrackedKeyPointsUn() {
        unique_lock<mutex> lock(mMutexState);
        return mTrackedKPsUn;
    }

    double System::GetTimeFromIMUInit() {
        double aux = mpLocalMapper->GetCurrKFTime() - mpLocalMapper->mFirstTs;
        if ((aux > 0.) && mpAtlas->GetImuInitialized())
            return mpLocalMapper->GetCurrKFTime() - mpLocalMapper->mFirstTs;
        else
            return 0.f;
    }

    bool System::isLost() {
        if (!mpAtlas->GetImuInitialized())
            return false;
        else {
            if ((mpTracker->mState == Tracking::LOST)) //||(mpTracker->mState==Tracking::RECENTLY_LOST))
                return true;
            else
                return false;
        }
    }


    bool System::isFinished() {
        return (GetTimeFromIMUInit() > 0.1);
    }

    void System::ChangeDataset() {
        if (mpAtlas->GetCurrentMap()->GetKeyFramesNumInMap() < 12) {
            mpTracker->ResetActiveMap();
        } else {
            mpTracker->CreateMapInAtlas();
        }
    }

    float System::GetImageScale() {
        return mpTracker->GetImageScale();
    }

/**
 * @brief 保存地图
 * @param type 保存类型
 */
    void System::SaveAtlas(int type) {
        if (msSaveAtlasToFile.empty()) {
            return;
        }
        // 1. 预保存想要保存的数据
        mpAtlas->PreSave();
        // 2. 确定文件名字
        string pathSaveFileName = "./";
        pathSaveFileName = pathSaveFileName.append(msSaveAtlasToFile);
        pathSaveFileName = pathSaveFileName.append(".osa");

        // 3. 保存词典的校验结果及名字
        string strVocabularyChecksum = CalculateCheckSum(msVocabularyFilePath, TEXT_FILE);
        std::size_t found = msVocabularyFilePath.find_last_of("/\\");
        string strVocabularyName = msVocabularyFilePath.substr(found + 1);

        if (type == TEXT_FILE) {
            cout << "Starting to write the save text file " << endl;
            std::remove(pathSaveFileName.c_str());
            std::ofstream ofs(pathSaveFileName, std::ios::binary);
            boost::archive::text_oarchive oa(ofs);

            oa << strVocabularyName;
            oa << strVocabularyChecksum;
            oa << mpAtlas;
            cout << "End to write the save text file" << endl;
        } else if (type == BINARY_FILE) {
            cout << "Starting to write the save binary file" << endl;
            std::remove(pathSaveFileName.c_str());
            std::ofstream ofs(pathSaveFileName, std::ios::binary);
            boost::archive::binary_oarchive oa(ofs);
            oa << strVocabularyName;
            oa << strVocabularyChecksum;
            oa << mpAtlas;
            cout << "End to write save binary file" << endl;
        }
    }

/**
 * @brief 加载地图
 * @param type 保存类型
 */
    bool System::LoadAtlas(int type) {
        // 1. 加载地图文件
        string sFileVoc, sVocChecksum;
        bool isRead = false;

        string sPathLoadFileName = "./";
        sPathLoadFileName = sPathLoadFileName.append(msLoadAtlasFromFile);
        sPathLoadFileName = sPathLoadFileName.append(".osa");

        if (type == TEXT_FILE) {
            cout << "Starting to read the save text file " << endl;
            std::ifstream ifs(sPathLoadFileName, std::ios::binary);
            if (!ifs.good()) {
                cout << "Load file not found" << endl;
                return false;
            }
            boost::archive::text_iarchive ia(ifs);
            ia >> sFileVoc;
            ia >> sVocChecksum;
            ia >> mpAtlas;
            cout << "End to load the save text file " << endl;
            isRead = true;
        } else if (type == BINARY_FILE) // File binary
        {
            cout << "Starting to read the save binary file" << endl;
            std::ifstream ifs(sPathLoadFileName, std::ios::binary);
            if (!ifs.good()) {
                cout << "Load file not found" << endl;
                return false;
            }
            boost::archive::binary_iarchive ia(ifs);
            ia >> sFileVoc;
            ia >> sVocChecksum;
            ia >> mpAtlas;
            cout << "End to load the save binary file" << endl;
            isRead = true;
        }

        // 2. 如果加载成功
        if (isRead) {
            //Check if the vocabulary is the same
            string sInputVocabularyChecksum = CalculateCheckSum(msVocabularyFilePath, TEXT_FILE);
            if (sInputVocabularyChecksum.compare(sVocChecksum) != 0) {
                cout << "The vocabulary load isn'mTs the same which the load session was created " << endl;
                cout << "-Vocabulary name: " << sFileVoc << endl;
                return false; // Both are differents
            }
            mpAtlas->SetKeyFrameDababase(mpKeyFrameDatabase);
            mpAtlas->SetORBVocabulary(mpVocabulary);
            mpAtlas->PostLoad();
            return true;
        }
        return false;
    }

// 校验词典文件，哈希出一个值，两个哈希值一样表示是同一文件
    string System::CalculateCheckSum(string filename, int type) {
        string checksum = "";
        unsigned char c[MD5_DIGEST_LENGTH];
        std::ios_base::openmode flags = std::ios::in;
        if (type == BINARY_FILE) // Binary file
            flags = std::ios::in | std::ios::binary;

        ifstream f(filename.c_str(), flags);
        if (!f.is_open()) {
            cout << "[E] Unable to open the in file " << filename << " for Md5 hash." << endl;
            return checksum;
        }
        MD5_CTX md5Context;
        char buffer[1024];
        MD5_Init(&md5Context);
        while (int count = f.readsome(buffer, sizeof(buffer))) {
            MD5_Update(&md5Context, buffer, count);
        }
        f.close();
        MD5_Final(c, &md5Context);
        for (int i = 0; i < MD5_DIGEST_LENGTH; i++) {
            char aux[10];
            sprintf(aux, "%02x", c[i]);
            checksum = checksum + aux;
        }
        return checksum;
    }

} //namespace ORB_SLAM

