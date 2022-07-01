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

#include<iostream>
#include<chrono>
#include <ctime>

#include <opencv2/core/core.hpp>


#include <System.h>
#include "ImuTypes.h"

using namespace std;

void LoadImages(const string &sCam0Path, const string &sCam1Path, const string &sImageTimeStampPath,
                vector<string> &vsImageLeftPath, vector<string> &vsImageRightPath, vector<double> &vdImageTimestamp);

void
LoadIMU(const string &sImuPath, vector<double> &vdImuTimestamp, vector<cv::Point3f> &vAcc, vector<cv::Point3f> &vGyr);

void LoadRtk(const string &sRtkPath, vector<double> &vdRtkTimestamp, vector<cv::Point3f> &vtrw);

//./Examples/Stereo-Inertial/stereo_inertial_euroc ./../ORB3Vocabulary/ORBvoc.txt ./Examples/Stereo-Inertial/EuRoC.yaml /media/kobosp/POCKET2/EuRoc/V1_03_difficult ./Examples/Stereo-Inertial/EuRoC_TimeStamps/V103.txt V103
///home/kobosp/SLMAC/ORB3Vocabulary/ORBvoc.txt /home/kobosp/SLMAC/ORB3_Vd1/Examples/Stereo-Inertial/EuRoC.yaml /media/kobosp/POCKET2/EuRoc/MH_01_easy /home/kobosp/SLMAC/ORB3_Vd1/Examples/Stereo-Inertial/EuRoC_TimeStamps/MH01.txt MH01
int main(int argc, char **argv) {
    if (argc != 6) {
        cerr << endl
             << "Usage: ./stereo_inertial_euroc path_to_vocabulary path_to_settings path_to_sequence_folder_1 path_to_times_file_1 (path_to_image_folder_2 path_to_times_file_2 ... path_to_image_folder_N path_to_times_file_N) "
             << endl;
        return 1;
    }

    const int nSeqNum = (argc - 3) / 2;
    const int nEarthRadius = 6378137;
    cout << "nSeqNum = " << nSeqNum << endl;

    // Load all sequences:
    int nSeqId;
    vector<vector<string> > vvsImageLeftPath;
    vector<vector<string> > vvsImageRightPath;
    vector<vector<double> > vvdImgTimestamp;
    vector<vector<double> > vvdImuTimestamp;
    vector<vector<double> > vvdRtkTimestamp;
    vector<vector<cv::Point3f> > vvtrw;
    vector<vector<cv::Point3f> > vvAcc, vvGyr;
    vector<int> nImageNumInSeq;
    vector<int> nImuNumInSeq;
    vector<int> nRtkNumInSeq;
    vector<int> nFirstImuInSeq(nSeqNum, 0);
    vector<int> nFirstRtkInSeq(nSeqNum, 0);

    vvsImageLeftPath.resize(nSeqNum);
    vvsImageRightPath.resize(nSeqNum);
    vvdImgTimestamp.resize(nSeqNum);
    vvdImuTimestamp.resize(nSeqNum);
    vvdRtkTimestamp.resize(nSeqNum);
    vvAcc.resize(nSeqNum);
    vvGyr.resize(nSeqNum);
    vvtrw.resize(nSeqNum);
    nImageNumInSeq.resize(nSeqNum);
    nImuNumInSeq.resize(nSeqNum);
    nRtkNumInSeq.resize(nSeqNum);

    int nTotImageNum = 0;
    for (nSeqId = 0; nSeqId < nSeqNum; nSeqId++) {
        string sDatasetPath(argv[(2 * nSeqId) + 3]);
        string sImageTimeStampPath(argv[(2 * nSeqId) + 4]);
        string sCam0Path = sDatasetPath + "/mav0/cam0/data";
        string sCam1Path = sDatasetPath + "/mav0/cam1/data";
        string sImuPath = sDatasetPath + "/mav0/imu0/data.csv";
        string sRtkPath = sDatasetPath + "/mav0/state_groundtruth_estimate0/data.csv";

        cout << "Loading Image Form Sequence " << nSeqId << "...";
        LoadImages(sCam0Path, sCam1Path, sImageTimeStampPath, vvsImageLeftPath[nSeqId], vvsImageRightPath[nSeqId],
                   vvdImgTimestamp[nSeqId]);
        cout << "LOADED!" << endl;

        cout << "Loading IMU Form Sequence " << nSeqId << "...";
        LoadIMU(sImuPath, vvdImuTimestamp[nSeqId], vvAcc[nSeqId], vvGyr[nSeqId]);
        cout << "LOADED!" << endl;

        cout << "Loading Rtk Form Sequence " << nSeqId << "...";
        LoadRtk(sRtkPath, vvdRtkTimestamp[nSeqId], vvtrw[nSeqId]);
        cout << "LOADED!" << endl;

        nImageNumInSeq[nSeqId] = vvsImageLeftPath[nSeqId].size();
        nImuNumInSeq[nSeqId] = vvdImuTimestamp[nSeqId].size();
        nRtkNumInSeq[nSeqId] = vvdRtkTimestamp[nSeqId].size();
        nTotImageNum += nImageNumInSeq[nSeqId];

        if ((nImageNumInSeq[nSeqId] <= 0) || (nImuNumInSeq[nSeqId] <= 0)) {
            cerr << "ERROR: Failed to load images or IMU for sequence" << nSeqId << endl;
            return 1;
        }
        // Find first imu to be considered, supposing imu measurements start first
        while (vvdImuTimestamp[nSeqId][nFirstImuInSeq[nSeqId]] <= vvdImgTimestamp[nSeqId][0])
            nFirstImuInSeq[nSeqId]++;
        while (vvdRtkTimestamp[nSeqId][nFirstRtkInSeq[nSeqId]] <= vvdImgTimestamp[nSeqId][0])
            nFirstRtkInSeq[nSeqId]++;
        nFirstImuInSeq[nSeqId]--;
        nFirstRtkInSeq[nSeqId]--;
    }

    // Read rectification parameters
    cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);
    if (!fsSettings.isOpened()) {
        cerr << "ERROR: Wrong path to settings" << endl;
        return -1;
    }

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nTotImageNum);

    cout << endl << "----------------" << endl;
    cout.precision(17);//整个数的位数（包括整数部分）为val位

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::IMU_STEREO, true, argv[argc - 1]);

    cv::Mat ImgLeft, ImgRight;
    Eigen::Matrix<float, 3, 1> ttrw;
    for (nSeqId = 0; nSeqId < nSeqNum; nSeqId++) {        // Seq loop
        vector<ORB_SLAM3::IMU::Point> vImuIntervalSet;
        cout << "Img.size() " << vvdImgTimestamp[nSeqId].size() << " Imu.size() " << vvdImuTimestamp[nSeqId].size()
             << " Rtk.size() " << vvdRtkTimestamp[nSeqId].size() << endl;
        for (int ni = 0; ni < nImageNumInSeq[nSeqId]; ni++) {
            // Read left and right images from file
            ImgLeft = cv::imread(vvsImageLeftPath[nSeqId][ni], cv::IMREAD_UNCHANGED);
            ImgRight = cv::imread(vvsImageRightPath[nSeqId][ni], cv::IMREAD_UNCHANGED);
            if (ImgLeft.empty()) {
                cerr << endl << "Failed to load left image at: "
                     << string(vvsImageLeftPath[nSeqId][ni]) << endl;
                return 1;
            }
            if (ImgRight.empty()) {
                cerr << endl << "Failed to load right image at: "
                     << string(vvsImageRightPath[nSeqId][ni]) << endl;
                return 1;
            }
            double dImgTimestamp = vvdImgTimestamp[nSeqId][ni];
            // Load imu measurements from previous frame
            vImuIntervalSet.clear();
            if (ni > 0) {
                while (vvdImuTimestamp[nSeqId][nFirstImuInSeq[nSeqId]] < dImgTimestamp) {
                    vImuIntervalSet.emplace_back(ORB_SLAM3::IMU::Point(vvAcc[nSeqId][nFirstImuInSeq[nSeqId]].x,
                                                                       vvAcc[nSeqId][nFirstImuInSeq[nSeqId]].y,
                                                                       vvAcc[nSeqId][nFirstImuInSeq[nSeqId]].z,
                                                                       vvGyr[nSeqId][nFirstImuInSeq[nSeqId]].x,
                                                                       vvGyr[nSeqId][nFirstImuInSeq[nSeqId]].y,
                                                                       vvGyr[nSeqId][nFirstImuInSeq[nSeqId]].z,
                                                                       vvdImuTimestamp[nSeqId][nFirstImuInSeq[nSeqId]]));
                    nFirstImuInSeq[nSeqId]++;
                }
                while (vvdRtkTimestamp[nSeqId][nFirstRtkInSeq[nSeqId]] < dImgTimestamp) {
                    nFirstRtkInSeq[nSeqId]++;
                }
                ttrw << vvtrw[nSeqId][nFirstRtkInSeq[nSeqId]].x, vvtrw[nSeqId][nFirstRtkInSeq[nSeqId]].y, vvtrw[nSeqId][nFirstRtkInSeq[nSeqId]].z;
            }
            std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
            SLAM.CalibAndTrack(ImgLeft, ImgRight, ttrw, dImgTimestamp, vImuIntervalSet);
            std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
            double dTrackTimePass = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
            vTimesTrack[ni] = dTrackTimePass;
            if (SLAM.CheckShutDowned()) {
                break;
            }
        }
        if (SLAM.CheckShutDowned()) {
            break;
        }
        if (nSeqId < nSeqNum - 1) {
            cout << "Changing the dataset" << endl;
            SLAM.ChangeDataset();
        }
    }
    cv::waitKey(0);
    if (!SLAM.CheckShutDowned()) {
        SLAM.ShutDownSystem();
    }

    // Save camera trajectory
    string sSaveFileName = string(argv[argc - 1]);
    const string FrameFile = "Frame" + sSaveFileName + ".txt";
    const string KeyFrameFile = "KeyFrame" + sSaveFileName + ".txt";
    SLAM.SaveFrameTrajectoryEuRoC(FrameFile);
    SLAM.SaveKeyFrameTrajectoryEuRoC(KeyFrameFile);

    return 0;
}


void LoadImages(const string &sCam0Path, const string &sCam1Path, const string &sImageTimeStampPath,
                vector<string> &vsImageLeftPath, vector<string> &vsImageRightPath, vector<double> &vdImageTimestamp) {
    ifstream fTimes;
    fTimes.open(sImageTimeStampPath.c_str());
    vdImageTimestamp.reserve(5000);
    vsImageLeftPath.reserve(5000);
    vsImageRightPath.reserve(5000);
    while (!fTimes.eof()) {
        string sTmp;
        getline(fTimes, sTmp);
        if (!sTmp.empty()) {
            stringstream ss;
            ss << sTmp;
            vsImageLeftPath.emplace_back(sCam0Path + "/" + ss.str() + ".png");
            vsImageRightPath.emplace_back(sCam1Path + "/" + ss.str() + ".png");
            double t;
            ss >> t;
            vdImageTimestamp.emplace_back(t / 1e9);
        }
    }
}


void
LoadIMU(const string &sImuPath, vector<double> &vdImuTimestamp, vector<cv::Point3f> &vAcc, vector<cv::Point3f> &vGyr) {
    ifstream fImu;
    fImu.open(sImuPath.c_str());
    vdImuTimestamp.reserve(5000);
    vAcc.reserve(5000);
    vGyr.reserve(5000);
    while (!fImu.eof()) {
        string sTmp;
        getline(fImu, sTmp);
        if (sTmp[0] == '#')
            continue;
        double vData[7];
        if (!sTmp.empty()) {
            string sValue;
            size_t nCnt = 0;
            int nPos = 0;
            while ((nCnt = sTmp.find(',')) != string::npos) {//Maximum value for size_t该值表示直到字符串结尾
                sValue = sTmp.substr(0, nCnt);
                vData[nPos++] = stod(sValue);
                sTmp.erase(0, nCnt + 1);
            }
            sValue = sTmp.substr(0, nCnt);
            vData[6] = stod(sValue);
            vdImuTimestamp.emplace_back(vData[0] / 1e9);
            vAcc.emplace_back(cv::Point3f(vData[4], vData[5], vData[6]));
            vGyr.emplace_back(cv::Point3f(vData[1], vData[2], vData[3]));
        }
    }
}

void LoadRtk(const string &sRtkPath, vector<double> &vdRtkTimestamp, vector<cv::Point3f> &vtrw) {
    ifstream fRtk;
    fRtk.open(sRtkPath.c_str());
    vdRtkTimestamp.reserve(5000);
    vtrw.reserve(5000);
    while (!fRtk.eof()) {
        string sTmp;
        getline(fRtk, sTmp);
        if (sTmp[0] == '#')
            continue;
        if (!sTmp.empty()) {
            string sValue;
            size_t nCnt = 0;
            double vData[16];
            int nPos = 0;
            while ((nCnt = sTmp.find(',')) != string::npos) {//Maximum value for size_t该值表示直到字符串结尾
                sValue = sTmp.substr(0, nCnt);
                vData[nPos++] = stod(sValue);
                sTmp.erase(0, nCnt + 1);
            }
            vdRtkTimestamp.emplace_back(vData[0] / 1e9);
            vtrw.emplace_back(cv::Point3f(-1 * vData[1], 1 * vData[2], -1 * vData[3]));
        }
    }
}