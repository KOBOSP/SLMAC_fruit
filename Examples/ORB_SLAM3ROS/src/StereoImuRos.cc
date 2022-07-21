/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
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
#include<algorithm>
#include<fstream>
#include<chrono>
#include<vector>
#include<queue>
#include<thread>
#include<mutex>

#include<ros/ros.h>
#include<cv_bridge/cv_bridge.h>
#include<sensor_msgs/Imu.h>

#include<opencv2/core/core.hpp>

#include"System.h"
#include"ImuTypes.h"

using namespace std;

class ImuGrabber {
public:
    ImuGrabber() {};

    void GrabImu(const sensor_msgs::ImuConstPtr &imu_msg);

    queue <sensor_msgs::ImuConstPtr> mqImuBuf;
    std::mutex mBufMutex;
};

class GrabAndSync {
public:
    GrabAndSync(ORB_SLAM3::System *pSLAM, ImuGrabber *pImuGb, const bool bClahe) : mpSLAM(pSLAM),
                                                                                    mpImuGb(pImuGb),
                                                                                    mbClahe(bClahe) {}

    void GrabImageLeft(const sensor_msgs::ImageConstPtr &msg);

    void GrabImageRight(const sensor_msgs::ImageConstPtr &msg);

    cv::Mat GetImageConvert(const sensor_msgs::ImageConstPtr &img_msg);

    void SyncWithImu();

    queue <sensor_msgs::ImageConstPtr> mqImgLeftBuf, mqImgRightBuf;
    std::mutex mBufMutexLeft, mBufMutexRight;
    ORB_SLAM3::System *mpSLAM;
    ImuGrabber *mpImuGb;
    const bool mbClahe;
    cv::Ptr<cv::CLAHE> mClahe = cv::createCLAHE(3.0, cv::Size(8, 8));
};


int main(int argc, char **argv) {
    ros::init(argc, argv, "StereoImuRos");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    bool bEqual = false;
    if (argc < 3 || argc > 4) {
        cerr << endl
             << "Usage: rosrun ORB_SLAM3 StereoImuRos path_to_vocabulary path_to_settings [do_equalize]"
             << endl;
        ros::shutdown();
        return 1;
    }
    if (argc == 4) {
        std::string sbEqual(argv[3]);
        if (sbEqual == "true")
            bEqual = true;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::IMU_STEREO, true, std::string("StereoImuRos"));

    ImuGrabber imugb;
    GrabAndSync GrabSync(&SLAM, &imugb, bEqual);

    // Maximum delay, 5 seconds
    ros::Subscriber sub_imu = n.subscribe("/mavros/imu/data", 400, &ImuGrabber::GrabImu, &imugb);
    ros::Subscriber sub_img_left = n.subscribe("/camera/left/image_raw", 120, &GrabAndSync::GrabImageLeft, &GrabSync);
    ros::Subscriber sub_img_right = n.subscribe("/camera/right/image_raw", 120, &GrabAndSync::GrabImageRight, &GrabSync);

    std::thread sync_thread(&GrabAndSync::SyncWithImu, &GrabSync);

    ros::spin();

    return 0;
}


void GrabAndSync::GrabImageLeft(const sensor_msgs::ImageConstPtr &img_msg) {
    mBufMutexLeft.lock();
    if (!mqImgLeftBuf.empty())
        mqImgLeftBuf.pop();
    mqImgLeftBuf.push(img_msg);
    mBufMutexLeft.unlock();
}

void GrabAndSync::GrabImageRight(const sensor_msgs::ImageConstPtr &img_msg) {
    mBufMutexRight.lock();
    if (!mqImgRightBuf.empty())
        mqImgRightBuf.pop();
    mqImgRightBuf.push(img_msg);
    mBufMutexRight.unlock();
}

cv::Mat GrabAndSync::GetImageConvert(const sensor_msgs::ImageConstPtr &img_msg) {
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvShare(img_msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }

    if (cv_ptr->image.type() == 0) {
        return cv_ptr->image.clone();
    } else {
        std::cout << "Error type" << std::endl;
        return cv_ptr->image.clone();
    }
}

void GrabAndSync::SyncWithImu() {
    const double maxTimeDiff = 0.01;
    while (true) {
        cv::Mat imLeft, imRight;
        double tImLeft = 0, tImRight = 0;
        if (!mqImgLeftBuf.empty() && !mqImgRightBuf.empty() && !mpImuGb->mqImuBuf.empty()) {
            tImLeft = mqImgLeftBuf.front()->header.stamp.toSec();
            tImRight = mqImgRightBuf.front()->header.stamp.toSec();

            this->mBufMutexRight.lock();
            while ((tImLeft - tImRight) > maxTimeDiff && mqImgRightBuf.size() > 1) {
                mqImgRightBuf.pop();
                tImRight = mqImgRightBuf.front()->header.stamp.toSec();
            }
            this->mBufMutexRight.unlock();

            this->mBufMutexLeft.lock();
            while ((tImRight - tImLeft) > maxTimeDiff && mqImgLeftBuf.size() > 1) {
                mqImgLeftBuf.pop();
                tImLeft = mqImgLeftBuf.front()->header.stamp.toSec();
            }
            this->mBufMutexLeft.unlock();

            if ((tImLeft - tImRight) > maxTimeDiff || (tImRight - tImLeft) > maxTimeDiff) {
                 std::cout << "big time difference" << std::endl;
                continue;
            }
            if (tImLeft > mpImuGb->mqImuBuf.back()->header.stamp.toSec())
                continue;

            this->mBufMutexLeft.lock();
            imLeft = GetImageConvert(mqImgLeftBuf.front());
            mqImgLeftBuf.pop();
            this->mBufMutexLeft.unlock();

            this->mBufMutexRight.lock();
            imRight = GetImageConvert(mqImgRightBuf.front());
            mqImgRightBuf.pop();
            this->mBufMutexRight.unlock();

            vector <ORB_SLAM3::IMU::Point> vImuMeas;
            mpImuGb->mBufMutex.lock();
            if (!mpImuGb->mqImuBuf.empty()) {
                // Load imu measurements from buffer
                vImuMeas.clear();
                while (!mpImuGb->mqImuBuf.empty() && mpImuGb->mqImuBuf.front()->header.stamp.toSec() <= tImLeft) {
                    double t = mpImuGb->mqImuBuf.front()->header.stamp.toSec();
                    cv::Point3f acc(mpImuGb->mqImuBuf.front()->linear_acceleration.x,
                                    mpImuGb->mqImuBuf.front()->linear_acceleration.y,
                                    mpImuGb->mqImuBuf.front()->linear_acceleration.z);
                    cv::Point3f gyr(mpImuGb->mqImuBuf.front()->angular_velocity.x,
                                    mpImuGb->mqImuBuf.front()->angular_velocity.y,
                                    mpImuGb->mqImuBuf.front()->angular_velocity.z);
                    vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc, gyr, t));
                    mpImuGb->mqImuBuf.pop();
                }
            }
            mpImuGb->mBufMutex.unlock();
            if (mbClahe) {
                mClahe->apply(imLeft, imLeft);
                mClahe->apply(imRight, imRight);
            }
            Eigen::Matrix<float, 3, 1> ttrw(0,0,0);
            mpSLAM->CalibAndTrack(imLeft, imRight, ttrw, tImLeft, vImuMeas);
            std::chrono::milliseconds tSleep(1);
            std::this_thread::sleep_for(tSleep);
        }
        if (!mpSLAM->CheckShutDowned()) {
            mpSLAM->ShutDownSystem();
        }
    }
}

void ImuGrabber::GrabImu(const sensor_msgs::ImuConstPtr &imu_msg) {
    mBufMutex.lock();
    mqImuBuf.push(imu_msg);
    mBufMutex.unlock();
    return;
}


