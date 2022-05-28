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


int main(int argc, char **argv) {
    if (argc != 6) {
        cerr << endl
             << "Usage: ./stereo_inertial_euroc path_to_vocabulary path_to_settings path_to_sequence_folder_1 path_to_times_file_1 (path_to_image_folder_2 path_to_times_file_2 ... path_to_image_folder_N path_to_times_file_N) "
             << endl;
        return 1;
    }

    const int nSeqNum = (argc - 3) / 2;
    cout << "nSeqNum = " << nSeqNum << endl;

    // Load all sequences:
    int nSeqId;
    vector<vector<string> > vvsImageLeftPath;
    vector<vector<string> > vvsImageRightPath;
    vector<vector<double> > vvdImgTimestamp;
    vector<vector<cv::Point3f> > vvAcc, vvGyr;
    vector<vector<double> > vvdImuTimestamp;
    vector<int> nImageNumInSeq;
    vector<int> nImuNumInSeq;
    vector<int> nFirstImuInSeq(nSeqNum, 0);

    vvsImageLeftPath.resize(nSeqNum);
    vvsImageRightPath.resize(nSeqNum);
    vvdImgTimestamp.resize(nSeqNum);
    vvAcc.resize(nSeqNum);
    vvGyr.resize(nSeqNum);
    vvdImuTimestamp.resize(nSeqNum);
    nImageNumInSeq.resize(nSeqNum);
    nImuNumInSeq.resize(nSeqNum);

    int nTotImageNum = 0;
    for (nSeqId = 0; nSeqId < nSeqNum; nSeqId++) {
        string sDatasetPath(argv[(2 * nSeqId) + 3]);
        string sImageTimeStampPath(argv[(2 * nSeqId) + 4]);
        string sCam0Path = sDatasetPath + "/mav0/cam0/data";
        string sCam1Path = sDatasetPath + "/mav0/cam1/data";
        string sImuPath = sDatasetPath + "/mav0/imu0/data.csv";

        cout << "Loading Image Form Sequence " << nSeqId << "...";
        LoadImages(sCam0Path, sCam1Path, sImageTimeStampPath, vvsImageLeftPath[nSeqId], vvsImageRightPath[nSeqId], vvdImgTimestamp[nSeqId]);
        cout << "LOADED!" << endl;

        cout << "Loading IMU Form Sequence " << nSeqId << "...";
        LoadIMU(sImuPath, vvdImuTimestamp[nSeqId], vvAcc[nSeqId], vvGyr[nSeqId]);
        cout << "LOADED!" << endl;

        nImageNumInSeq[nSeqId] = vvsImageLeftPath[nSeqId].size();
        nImuNumInSeq[nSeqId] = vvdImuTimestamp[nSeqId].size();
        nTotImageNum += nImageNumInSeq[nSeqId];

        if ((nImageNumInSeq[nSeqId] <= 0) || (nImuNumInSeq[nSeqId] <= 0)) {
            cerr << "ERROR: Failed to load images or IMU for sequence" << nSeqId << endl;
            return 1;
        }
        // Find first imu to be considered, supposing imu measurements start first
        while (vvdImuTimestamp[nSeqId][nFirstImuInSeq[nSeqId]] <= vvdImgTimestamp[nSeqId][0])
            nFirstImuInSeq[nSeqId]++;
        nFirstImuInSeq[nSeqId]--; // first imu measurement to be considered
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
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::IMU_STEREO, true);

    cv::Mat ImgLeft, ImgRight;
    for (nSeqId = 0; nSeqId < nSeqNum; nSeqId++) {        // Seq loop
        vector<ORB_SLAM3::IMU::Point> vImuIntervalSet;
        for (int ni = 0; ni < nImageNumInSeq[nSeqId]; ni++) {
            // Read left and right images from file
            ImgLeft = cv::imread(vvsImageLeftPath[nSeqId][ni], cv::IMREAD_UNCHANGED);
            ImgRight = cv::imread(vvsImageRightPath[nSeqId][ni], cv::IMREAD_UNCHANGED);
            if (ImgLeft.empty()) {
                cerr << endl << "Failed to load image at: "
                     << string(vvsImageLeftPath[nSeqId][ni]) << endl;
                return 1;
            }
            if (ImgRight.empty()) {
                cerr << endl << "Failed to load image at: "
                     << string(vvsImageRightPath[nSeqId][ni]) << endl;
                return 1;
            }
            double dImgTimestamp = vvdImgTimestamp[nSeqId][ni];
            // Load imu measurements from previous frame
            vImuIntervalSet.clear();
            if (ni > 0){
                while (vvdImuTimestamp[nSeqId][nFirstImuInSeq[nSeqId]] <= vvdImgTimestamp[nSeqId][ni]){
                    vImuIntervalSet.push_back(ORB_SLAM3::IMU::Point(vvAcc[nSeqId][nFirstImuInSeq[nSeqId]].x,
                                                                    vvAcc[nSeqId][nFirstImuInSeq[nSeqId]].y,
                                                                    vvAcc[nSeqId][nFirstImuInSeq[nSeqId]].z,
                                                                    vvGyr[nSeqId][nFirstImuInSeq[nSeqId]].x,
                                                                    vvGyr[nSeqId][nFirstImuInSeq[nSeqId]].y,
                                                                    vvGyr[nSeqId][nFirstImuInSeq[nSeqId]].z,
                                                                    vvdImuTimestamp[nSeqId][nFirstImuInSeq[nSeqId]]));
                    nFirstImuInSeq[nSeqId]++;
                }
            }
            std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
            SLAM.CalibAndTrack(ImgLeft, ImgRight, dImgTimestamp, vImuIntervalSet);
            std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
            double dTrackTimePass = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
            vTimesTrack[ni] = dTrackTimePass;
            if(SLAM.CheckShutDowned()){
                break;
            }
        }
        if(SLAM.CheckShutDowned()){
            break;
        }
        if (nSeqId < nSeqNum - 1) {
            cout << "Changing the dataset" << endl;
            SLAM.ChangeDataset();
        }
    }
    if(!SLAM.CheckShutDowned()){
        SLAM.ShutDownSystem();
    }

    // Save camera trajectory
    string sSaveFileName = string(argv[argc - 1]);
    cout << "sSaveFileName: " << sSaveFileName << endl;
    const string kf_file = "Camera" + sSaveFileName + ".txt";
    const string f_file = "KeyFrame" + sSaveFileName + ".txt";
    SLAM.SaveTrajectoryEuRoC(f_file);
    SLAM.SaveKeyFrameTrajectoryEuRoC(kf_file);

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
            vsImageLeftPath.push_back(sCam0Path + "/" + ss.str() + ".png");
            vsImageRightPath.push_back(sCam1Path + "/" + ss.str() + ".png");
            double t;
            ss >> t;
            vdImageTimestamp.push_back(t / 1e9);
        }
    }
}


void LoadIMU(const string &sImuPath, vector<double> &vdImuTimestamp, vector<cv::Point3f> &vAcc, vector<cv::Point3f> &vGyr) {
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

        if (!sTmp.empty()) {
            string sValue;
            size_t nCnt = 0;
            double vData[7];
            int nPos = 0;
            while ((nCnt = sTmp.find(',')) != string::npos) {//Maximum value for size_t该值表示直到字符串结尾
                sValue = sTmp.substr(0, nCnt);
                vData[nPos++] = stod(sValue);
                sTmp.erase(0, nCnt + 1);
            }
            sValue = sTmp.substr(0, nCnt);
            vData[6] = stod(sValue);
            vdImuTimestamp.push_back(vData[0] / 1e9);
            vAcc.push_back(cv::Point3f(vData[4], vData[5], vData[6]));
            vGyr.push_back(cv::Point3f(vData[1], vData[2], vData[3]));
        }
    }
}
