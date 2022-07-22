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


#include "Tracking.h"

#include "ORBmatcher.h"
#include "FrameDrawer.h"
#include "Converter.h"
#include "G2oTypes.h"
#include "Optimizer.h"
#include "Pinhole.h"
#include "KannalaBrandt8.h"
#include "MLPnPsolver.h"
#include "GeometricTools.h"

#include <iostream>

#include <mutex>
#include <chrono>


using namespace std;

// 程序中变量名的第一个字母如果为"m"则表示为类中的成员变量，member
// 第一个、第二个字母:
// "p"表示指针数据类型
// "n"表示int类型
// "mBiasOri"表示bool类型
// "s"表示set类型
// "v"表示vector数据类型
// 'l'表示list数据类型
// "KF"表示KeyFrame数据类型 

namespace ORB_SLAM3 {

/**
 * @brief 跟踪线程构造函数
 * @param pSys 系统类指针
 * @param pVoc 词典
 * @param pFrameDrawer 画图像的
 * @param pMapDrawer 画地图的
 * @param pAtlas atlas
 * @param pKFDB 关键帧词典数据库
 * @param strSettingPath 参数文件路径
 * @param sensor 传感器类型
 * @param settings 参数类
 * @param _strSeqName 序列名字，没用到
 */
    Tracking::Tracking(System *pSys, ORBVocabulary *pVoc, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer,
                       Atlas *pAtlas, KeyFrameDatabase *pKFDB, const string &strSettingPath, const int sensor,
                       Settings *settings)
            : mState(NO_IMAGES_YET), mSensor(sensor), mTrackedFrame(0), mbDoNext(false),
              mbOnlyTracking(false), mbMapUpdated(false), mpORBVocabulary(pVoc), mpKeyFrameDB(pKFDB),
              mpSystem(pSys), mpViewer(NULL), bStepByStep(false),
              mpFrameDrawer(pFrameDrawer), mpMapDrawer(pMapDrawer), mpAtlas(pAtlas), mnFrameIdLastReloc(0),
              mnInitialFrameId(0), mbCreatedMap(false), mnFirstFrameId(0),
              mpLastKeyFrame(static_cast<KeyFrame *>(NULL)) {
        // Load camera parameters from settings file
        // Step 1 从配置文件中加载相机参数
        LoadParameter(settings);
        // 遍历下地图中的相机，然后打印出来了
        vector<GeometricCamera *> vpCams = mpAtlas->GetAllCameras();
        std::cout << "There are " << vpCams.size() << " cameras in the atlas" << std::endl;
        for (GeometricCamera *pCam: vpCams) {
            std::cout << "Camera " << pCam->GetId();
            if (pCam->GetType() == GeometricCamera::CAM_PINHOLE) {
                std::cout << " is pinhole" << std::endl;
            } else if (pCam->GetType() == GeometricCamera::CAM_FISHEYE) {
                std::cout << " is fisheye" << std::endl;
            } else {
                std::cout << " is unknown" << std::endl;
            }
        }
    }

    Tracking::~Tracking() {
        //f_track_stats.close();
    }

/**
 * @brief 根据参数类读取参数，可快速略过不看
 * @param settings 参数类
 */
    void Tracking::LoadParameter(Settings *settings) {
        // 1. 读取相机1
        mpCamera = settings->mCalibration1;
        mpCamera = mpAtlas->AddCamera(mpCamera);


        //TODO: missing image scaling and rectification
        mImageScale = 1.0f;

        mCvK = cv::Mat::eye(3, 3, CV_32F);
        mCvK.at<float>(0, 0) = mpCamera->GetParameter(0);
        mCvK.at<float>(1, 1) = mpCamera->GetParameter(1);
        mCvK.at<float>(0, 2) = mpCamera->GetParameter(2);
        mCvK.at<float>(1, 2) = mpCamera->GetParameter(3);

        mEigenK.setIdentity();
        mEigenK(0, 0) = mpCamera->GetParameter(0);
        mEigenK(1, 1) = mpCamera->GetParameter(1);
        mEigenK(0, 2) = mpCamera->GetParameter(2);
        mEigenK(1, 2) = mpCamera->GetParameter(3);


        // 读取双目
        mfBaselineFocal = settings->mfBaselineFocal;
        mfThDepth = settings->mfBaseline * settings->mfThDepth;


        mMinFrames = 0;
        mnMaxFrames = settings->mfImgFps;
        mnFrameNumDurRefLoc = settings->mfImgFps;
        mnFrameNumDurRecLost = settings->mfImgFps;
        mnFrameNumDurLost = settings->mfImgFps;


        //ORB parameters
        // 2. 读取特征点参数
        int nFeatures = settings->mnFeatures;
        int nLevels = settings->mnLevels;
        int fIniThFAST = settings->mnInitThFAST;
        int fMinThFAST = settings->mnMinThFAST;
        float fScaleFactor = settings->mfScaleFactor;

        mpORBextractorLeft = new ORBextractor(nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);
        mpORBextractorRight = new ORBextractor(nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);

        //IMU parameters
        // 3. 读取imu参数
        Sophus::SE3f Tbc = settings->mTbc;
        mInsertKFsLost = settings->mbInsertKFsWhenLost;
        mImuFreq = settings->mImuFreq;
        mImuInterval = 1.0 / (double) mImuFreq;
        float fGyrNoise = settings->mGyrNoise;
        float fAccNoise = settings->mAccNoise;
        float fGyrWalk = settings->mGyrWalk;
        float fAccWalk = settings->mAccWalk;

        const float sf = sqrt(mImuFreq);
        mpImuCalib = new IMU::Calib(Tbc, fGyrNoise * sf, fAccNoise * sf, fGyrWalk / sf, fAccWalk / sf);
        mpImuPreintegratedFromLastKF = new IMU::Preintegrated(IMU::Bias(), *mpImuCalib);
    }

/**
 * @brief 设置局部建图器
 * @param pLocalMapper 局部地图类
 */
    void Tracking::SetLocalMapper(LocalMapping *pLocalMapper) {
        mpLocalMapping = pLocalMapper;
    }

/**
 * @brief 设置回环器
 * @param pLoopClosing 回环类
 */
    void Tracking::SetLoopClosing(LoopClosing *pLoopClosing) {
        mpLoopClosing = pLoopClosing;
    }

/**
 * @brief 设置显示器
 * @param pViewer 显示类
 */
    void Tracking::SetViewer(Viewer *pViewer) {
        mpViewer = pViewer;
    }

    void Tracking::SetStepByStep(bool bSet) {
        bStepByStep = bSet;
    }

    bool Tracking::GetStepByStep() {
        return bStepByStep;
    }


/**
 * @brief 输入左右目图像，可以为RGB、BGR、RGBA、GRAY
 * 1、将图像转为mImGray和imGrayRight并初始化mCurrentFrame
 * 2、进行tracking过程
 * 输出世界坐标系到该帧相机坐标系的变换矩阵
 * @param imRectLeft 左图
 * @param imRectRight 右图
 * @param dTimestamp 时间戳
 * @param sFileName 文件名字，貌似调试用的
 */
    Sophus::SE3f
    Tracking::GrabImageStereo(const cv::Mat &imRectLeft, const cv::Mat &imRectRight, Eigen::Matrix<float, 3, 1> trw,
                              const double &dTimestamp) {
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        mCurFrame = Frame(imRectLeft, imRectRight, trw, dTimestamp, mpORBextractorLeft, mpORBextractorRight,
                          mpORBVocabulary, mCvK, mfBaselineFocal, mfThDepth, mpCamera, &mLastFrame,
                          *mpImuCalib);
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        mdExtraFps = 1.0 / std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        if (bStepByStep) {
            std::cout << "Tracking: Waiting to the next step" << std::endl;
            while (!mbDoNext && bStepByStep)
                usleep(5000);
            mbDoNext = false;
        }

        t1 = std::chrono::steady_clock::now();
        Track();
        t2 = std::chrono::steady_clock::now();
        mdTrackFps = 1.0 / std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
        return mCurFrame.GetPose();
    }


/**
 * @brief 将imu数据存放在mlQueueImuData的list链表里
 * @param[in] ImuMeasure
 */
    void Tracking::GrabImuData(const IMU::Point &ImuMeasure) {
        unique_lock<mutex> lock(mMutexImuQueue);
        mlQueueImuData.emplace_back(ImuMeasure);
    }

/**
 * @brief 预积分，对于一个帧有两种预积分，一种是相对于上一帧，一种是相对于上一个关键帧
 */
    void Tracking::PreintegrateIMU() {

        // Step 1.拿到两两帧之间待处理的预积分数据，组成一个集合
        // 上一帧不存在,说明两帧之间没有imu数据，不进行预积分
        if (!mCurFrame.mpPrevFrame) {
            Verbose::PrintMess("non prev frame ", Verbose::VERBOSITY_NORMAL);
            mCurFrame.setIntegrated();
            return;
        }
        // 没有imu数据,不进行预积分
        if (mlQueueImuData.size() == 0) {
            Verbose::PrintMess("Not IMU data in mlQueueImuData!!", Verbose::VERBOSITY_NORMAL);
            mCurFrame.setIntegrated();
            return;
        }
        mvImuFromLastFrame.clear();
        mvImuFromLastFrame.reserve(mlQueueImuData.size());
        while (true) {
            // 数据还没有时,会等待一段时间,直到mlQueueImuData中有imu数据.一开始不需要等待
            bool bSleep = false;
            {
                unique_lock<mutex> lock(mMutexImuQueue);
                if (!mlQueueImuData.empty()) {
                    // 拿到第一个imu数据作为起始数据
                    IMU::Point *m = &mlQueueImuData.front();
                    cout.precision(17);
                    // imu起始数据会比当前帧的前一帧时间戳早,如果相差0.001则舍弃这个imu数据
                    if (m->mTs < mCurFrame.mpPrevFrame->mdTimestamp - mImuInterval) {
                        mlQueueImuData.pop_front();
                    } else if (m->mTs < mCurFrame.mdTimestamp - mImuInterval) {
                        mvImuFromLastFrame.emplace_back(*m);
                        mlQueueImuData.pop_front();
                    } else {
                        // 得到两帧间的imu数据放入mvImuFromLastFrame中,得到后面预积分的处理数据
                        mvImuFromLastFrame.emplace_back(*m);
                        break;
                    }
                } else {
                    break;
                }
            }
            if (bSleep)
                usleep(5000);
        }

        // Step 2.对两帧之间进行中值积分处理
        // m个imu组数据会有m-1个预积分量
        const int n = mvImuFromLastFrame.size() - 1;
        if (n < 1) {
            cout << "Empty IMU measurements vector!!!\n";
            return;
        }
        // 构造imu预处理器,并初始化标定数据
        IMU::Preintegrated *pImuPreintegratedFromLastFrame = new IMU::Preintegrated(mLastFrame.mImuBias,
                                                                                    mCurFrame.mImuCalib);
        // 针对预积分位置的不同做不同中值积分的处理

        /**
     *  根据上面imu帧的筛选，IMU与图像帧的时序如下：
     *  Frame---IMU0---IMU1---IMU2---IMU3---IMU4---------------IMUx---Frame---IMUx+1
     *  T_------T0-----T1-----T2-----T3-----T4-----------------Tx-----_T------Tx+1
     *  A_------A0-----A1-----A2-----A3-----A4-----------------Ax-----_T------Ax+1
     *  W_------W0-----W1-----W2-----W3-----W4-----------------Wx-----_T------Wx+1
     *  T_和_T分别表示上一图像帧和当前图像帧的时间戳，A(加速度数据)，W(陀螺仪数据)，同理
     */

        for (int i = 0; i < n; i++) {
            float TDelta;
            Eigen::Vector3f Acc, Gyr;
            // 第一帧数据但不是最后两帧,imu总帧数大于2
            if ((i == 0) && (i < (n - 1))) {
                // 获取相邻两段imu的时间间隔
                float TCN = mvImuFromLastFrame[i + 1].mTs - mvImuFromLastFrame[i].mTs;
                // 获取当前imu到上一帧的时间间隔
                float TPC = mvImuFromLastFrame[i].mTs - mCurFrame.mpPrevFrame->mdTimestamp;
                // 设当前时刻imu的加速度a0，下一时刻加速度a1，时间间隔tab 为t10，TPC t0p
                // 正常情况下时为了求上一帧到当前时刻imu的一个平均加速度，但是imu时间不会正好落在上一帧的时刻，需要做补偿，要求得a0时刻到上一帧这段时间加速度的改变量
                // 有了这个改变量将其加到a0上之后就可以表示上一帧时的加速度了。其中a0 - (a1-a0)*(TPC/TCN) 为上一帧时刻的加速度再加上a1 之后除以2就为这段时间的加速度平均值
                // 其中tstep表示a1到上一帧的时间间隔，a0 - (a1-a0)*(TPC/TCN)这个式子中tini可以是正也可以是负表示时间上的先后，(a1-a0)也是一样，多种情况下这个式子依然成立
                Acc = (mvImuFromLastFrame[i + 1].mAcc + mvImuFromLastFrame[i].mAcc -
                       (mvImuFromLastFrame[i + 1].mAcc - mvImuFromLastFrame[i].mAcc) * (TPC / TCN)) * 0.5f;
                // 计算过程类似加速度
                Gyr = (mvImuFromLastFrame[i + 1].mGyr + mvImuFromLastFrame[i].mGyr -
                       (mvImuFromLastFrame[i + 1].mGyr - mvImuFromLastFrame[i].mGyr) * (TPC / TCN)) * 0.5f;
                TDelta = mvImuFromLastFrame[i + 1].mTs - mCurFrame.mpPrevFrame->mdTimestamp;
            } else if (i < (n - 1)) {
                // 中间的数据不存在帧的干扰，正常计算
                Acc = (mvImuFromLastFrame[i].mAcc + mvImuFromLastFrame[i + 1].mAcc) * 0.5f;
                Gyr = (mvImuFromLastFrame[i].mGyr + mvImuFromLastFrame[i + 1].mGyr) * 0.5f;
                TDelta = mvImuFromLastFrame[i + 1].mTs - mvImuFromLastFrame[i].mTs;
            } else if ((i > 0) && (i == (n - 1))) {// 直到倒数第二个imu时刻时，计算过程跟第一时刻类似，都需要考虑帧与imu时刻的关系
                float tab = mvImuFromLastFrame[i + 1].mTs - mvImuFromLastFrame[i].mTs;
                float tend = mvImuFromLastFrame[i + 1].mTs - mCurFrame.mdTimestamp;
                Acc = (mvImuFromLastFrame[i].mAcc + mvImuFromLastFrame[i + 1].mAcc -
                       (mvImuFromLastFrame[i + 1].mAcc - mvImuFromLastFrame[i].mAcc) * (tend / tab)) * 0.5f;
                Gyr = (mvImuFromLastFrame[i].mGyr + mvImuFromLastFrame[i + 1].mGyr -
                       (mvImuFromLastFrame[i + 1].mGyr - mvImuFromLastFrame[i].mGyr) * (tend / tab)) * 0.5f;
                TDelta = mCurFrame.mdTimestamp - mvImuFromLastFrame[i].mTs;
            } else if ((i == 0) && (i == (n - 1))) {                // 就两个数据时使用第一个时刻的，这种情况应该没有吧，，回头应该试试看
                Acc = mvImuFromLastFrame[i].mAcc;
                Gyr = mvImuFromLastFrame[i].mGyr;
                TDelta = mCurFrame.mdTimestamp - mCurFrame.mpPrevFrame->mdTimestamp;
            }
            // Step 3.依次进行预积分计算
            // 应该是必存在的吧，一个是相对上一关键帧，一个是相对上一帧
            if (!mpImuPreintegratedFromLastKF)
                cout << "mpImuPreintegratedFromLastKF does not exist" << endl;
            mpImuPreintegratedFromLastKF->IntegrateNewMeasurement(Acc, Gyr, TDelta);
            pImuPreintegratedFromLastFrame->IntegrateNewMeasurement(Acc, Gyr, TDelta);
        }
        // 记录当前预积分的图像帧
        mCurFrame.mpImuFromPrevFrame = pImuPreintegratedFromLastFrame;
        mCurFrame.mpImuFromPrevKF = mpImuPreintegratedFromLastKF;
        mCurFrame.mpPrevKeyFrame = mpLastKeyFrame;
        mCurFrame.setIntegrated();
        //Verbose::PrintMess("Preintegration is finished!! ", Verbose::VERBOSITY_DEBUG);
    }

/**
 * @brief 跟踪不成功的时候，用初始化好的imu数据做跟踪处理，通过IMU预测状态
 * 两个地方用到：
 * 1. 匀速模型计算速度,但并没有给当前帧位姿赋值；
 * 2. 跟踪丢失时不直接判定丢失，通过这个函数预测当前帧位姿看看能不能拽回来，代替纯视觉中的重定位
 *
 * @return true
 * @return false
 */
    bool Tracking::TrackWithIMU() {
        if (!mCurFrame.mpPrevFrame) {
            Verbose::PrintMess("No last frame", Verbose::VERBOSITY_NORMAL);
            return false;
        }

        // 总结下都在什么时候地图更新，也就是mbMapUpdated为true
        // 1. 回环或融合
        // 2. 局部地图LocalBundleAdjustment
        // 3. IMU三阶段的初始化
        // 下面的代码流程一模一样，只不过计算时相对的帧不同，地图有更新后相对于上一关键帧做的，反之相对于上一帧
        // 地图更新后会更新关键帧与MP，所以相对于关键帧更准
        // 而没更新的话，距离上一帧更近，计算起来误差更小
        // 地图更新时，并且上一个图像关键帧存在
        if (mbMapUpdated && mpLastKeyFrame) {
            const Eigen::Vector3f twb1 = mpLastKeyFrame->GetImuPosition();
            const Eigen::Matrix3f Rwb1 = mpLastKeyFrame->GetImuRotation();
            const Eigen::Vector3f Vwb1 = mpLastKeyFrame->GetVelocity();

            const Eigen::Vector3f Gz(0, 0, -IMU::GRAVITY_VALUE);
            const float TKFToF = mpImuPreintegratedFromLastKF->mfTs;

            // 计算当前帧在世界坐标系的位姿,原理都是用预积分的位姿（预积分的值不会变化）与上一帧的位姿（会迭代变化）进行更新
            // 旋转 R_wb2 = R_wb1 * R_b1b2
            Eigen::Matrix3f Rwb2 = IMU::NormalizeRotation(
                    Rwb1 * mpImuPreintegratedFromLastKF->GetDeltaRotation(mpLastKeyFrame->GetImuBias()));
            // 位移
            Eigen::Vector3f twb2 = twb1 + Vwb1 * TKFToF + 0.5f * TKFToF * TKFToF * Gz +
                                   Rwb1 * mpImuPreintegratedFromLastKF->GetDeltaPosition(mpLastKeyFrame->GetImuBias());
            // 速度
            Eigen::Vector3f Vwb2 = Vwb1 + TKFToF * Gz +
                                   Rwb1 * mpImuPreintegratedFromLastKF->GetDeltaVelocity(mpLastKeyFrame->GetImuBias());
            // 设置当前帧的世界坐标系的相机位姿
            mCurFrame.SetImuPoseVelocity(Rwb2, twb2, Vwb2);

            // 记录bias
            mCurFrame.mImuBias = mpLastKeyFrame->GetImuBias();
            return true;
        }
            // 地图未更新时
        else if (!mbMapUpdated) {
            const Eigen::Vector3f twb1 = mLastFrame.GetImuTcbTranslation();
            const Eigen::Matrix3f Rwb1 = mLastFrame.GetImuTcbRotation();
            const Eigen::Vector3f Vwb1 = mLastFrame.GetVelocity();
            const Eigen::Vector3f Gz(0, 0, -IMU::GRAVITY_VALUE);
            // mpImuPreintegratedFrame是当前帧上一帧，不一定是关键帧
            const float t12 = mCurFrame.mpImuFromPrevFrame->mfTs;

            Eigen::Matrix3f Rwb2 = IMU::NormalizeRotation(
                    Rwb1 * mCurFrame.mpImuFromPrevFrame->GetDeltaRotation(mLastFrame.mImuBias));
            Eigen::Vector3f twb2 = twb1 + Vwb1 * t12 + 0.5f * t12 * t12 * Gz +
                                   Rwb1 * mCurFrame.mpImuFromPrevFrame->GetDeltaPosition(mLastFrame.mImuBias);
            Eigen::Vector3f Vwb2 = Vwb1 + t12 * Gz +
                                   Rwb1 * mCurFrame.mpImuFromPrevFrame->GetDeltaVelocity(mLastFrame.mImuBias);

            mCurFrame.SetImuPoseVelocity(Rwb2, twb2, Vwb2);

            mCurFrame.mImuBias = mLastFrame.mImuBias;
            return true;
        } else {
            cout << "not IMU prediction!!" << endl;
        }
        return false;
    }

    void Tracking::ResetFrameIMU() {
        // TODO To implement...
    }

/**
 * @brief 跟踪过程，包括恒速模型跟踪、参考关键帧跟踪、局部地图跟踪
 * track包含两部分：估计运动、跟踪局部地图
 *
 * Step 1：初始化
 * Step 2：跟踪
 * Step 3：记录位姿信息，用于轨迹复现
 */
    void Tracking::Track() {
        // Step 1 如局部建图里认为IMU有问题，重置当前活跃地图
        if (mpLocalMapping->mbBadImu) {
            cout << "TRACK: CheckRequestReset map because local mapper set the bad imu flag " << endl;
            mpSystem->ResetActiveMap();
            return;
        }
        // 从Atlas中取出当前active的地图
        Map *pCurrentMap = mpAtlas->GetCurrentMap();
        if (!pCurrentMap) {
            cout << "ERROR: There is not an active map in the atlas" << endl;
        }

        // Step 2 处理时间戳异常的情况
        if (mState != NO_IMAGES_YET) {
            if (mLastFrame.mdTimestamp > mCurFrame.mdTimestamp) {
                // 如果当前图像时间戳比前一帧图像时间戳小，说明出错了，清除imu数据，创建新的子地图
                cerr << "ERROR: Frame with a timestamp older than previous frame detected!" << endl;
                unique_lock<mutex> lock(mMutexImuQueue);
                // mlQueueImuData.clear();
                // 创建新地图
                CreateMapInAtlas();
                return;
            } else if (mCurFrame.mdTimestamp > mLastFrame.mdTimestamp + 1.0) {
                // 如果当前图像时间戳和前一帧图像时间戳大于1s，说明时间戳明显跳变了，重置地图后直接返回
                //根据是否是imu模式,进行imu的补偿
                // 如果当前地图imu成功初始化
                if (mpAtlas->GetImuInitialized()) {
                    cout << "Timestamp jump detected. State set to LOST. Reseting IMU integration..." << endl;
                    // IMU完成第3次初始化（在localmapping线程里）
                    if (!pCurrentMap->GetImuIniertialBA2()) {
                        // 如果当前子图中imu没有经过BA2，重置active地图，也就是之前的数据不要了
                        mpSystem->ResetActiveMap();
                    } else {
                        // 如果当前子图中imu进行了BA2，重新创建新的子图，保存当前地图
                        CreateMapInAtlas();
                    }
                } else {
                    // 如果当前子图中imu还没有初始化，重置active地图
                    cout << "Timestamp jump detected, before IMU initialization. Reseting..." << endl;
                    mpSystem->ResetActiveMap();
                }
                return;
            }
        }
        // Step 3 IMU模式下设置IMU的Bias参数,还要保证上一帧存在
        if (mpLastKeyFrame) {
            mCurFrame.SetNewBias(mpLastKeyFrame->GetImuBias());  // 使用上一帧的bias作为当前帧的初值
        }

        if (mState == NO_IMAGES_YET) {
            mState = NOT_INITIALIZED;
        }
        mLastProcessedState = mState;
        // Step 4 IMU模式且没有创建地图的情况下对IMU数据进行预积分
        if (!mbCreatedMap) {
            // IMU数据进行预积分
            PreintegrateIMU();
        }
        mbCreatedMap = false;

        // Get Map Mutex -> Map cannot be changed
        // 地图更新时加锁。保证地图不会发生变化
        // 疑问:这样子会不会影响地图的实时更新?
        // 回答：主要耗时在构造帧中特征点的提取和匹配部分,在那个时候地图是没有被上锁的,有足够的时间更新地图
        unique_lock<mutex> lock(pCurrentMap->mMutexMapUpdate);

        mbMapUpdated = false;
        // 判断地图id是否更新了
        int nCurMapChangeIndex = pCurrentMap->GetMapChangeIdx();
        int nMapChangeIndex = pCurrentMap->GetLastMapChangeIdx();
        if (nCurMapChangeIndex > nMapChangeIndex) {
            // 检测到地图更新了
            pCurrentMap->SetLastMapChangeIdx(nCurMapChangeIndex);
            mbMapUpdated = true;
        }
        // Step 5 初始化
        if (mState == NOT_INITIALIZED) {
            // 双目RGBD相机的初始化共用一个函数
            StereoInitialization();
            //mpFrameDrawer->Update6DoF(this);
            if (mState != OK) // If rightly initialized, mState=OK
            {
                // 如果没有成功初始化，直接返回
                mLastFrame = Frame(mCurFrame);
                return;
            }
            if (mpAtlas->GetAllMaps().size() == 1) {
                // 如果当前地图是第一个地图，记录当前帧id为第一帧
                mnFirstFrameId = mCurFrame.mnId;
            }

        } else { // System is initialized. Track Frame.
            bool bOK = false;
            mnTrackMethod = 0;
            // State OK
            if (mState == OK || mState == RECENTLY_LOST) {
                // Local Mapping might have changed some MapPoints tracked in last frame
                CheckReplacedInLastFrame();

                // Step 6.2 运动模型是空的并且imu未初始化或刚完成重定位，跟踪参考关键帧；否则恒速模型跟踪
                // 第一个条件,如果运动模型为空并且imu未初始化,说明是刚开始第一帧跟踪，或者已经跟丢了。
                // 第二个条件,如果当前帧紧紧地跟着在重定位的帧的后面，我们用重定位帧来恢复位姿
                // mnFrameIdLastReloc 上一次重定位的那一帧
                if ((!mbVelocity && !pCurrentMap->GetImuInitialized()) ||
                    (mCurFrame.mnId < mnFrameIdLastReloc + mnFrameNumDurRefLoc)) {
                    bOK = TrackReferenceKeyFrame();
                } else {
                    // Update6DoF last frame pose according to its reference keyframe
                    UpdateLastFramePose();
                    if (mpAtlas->GetImuInitialized()) {
                        // Predict state with IMU if it is initialized and it doesnt need reset
                        bOK = TrackWithIMU();
                    }
                    if (!bOK) {
                        if (mbVelocity) {
                            bOK = TrackWithMotionModel();
                        }
                    } else if (mnTrackMethod == 0) {
                        mnTrackMethod = 1;//IMU
                    }
                    if (!bOK) {
                        bOK = TrackReferenceKeyFrame();  // 根据恒速模型失败了，只能根据参考关键帧来跟踪
                    } else if (mnTrackMethod == 0) {
                        mnTrackMethod = 2;//Motion
                    }
                }
                if (bOK && mnTrackMethod == 0) {
                    mnTrackMethod = 3;//RefKF
                }
            } else if (mState == LOST) {
                if (mCurFrame.mdTimestamp < mnFrameIdLost + mnFrameNumDurLost) {
                    bOK = Relocalization();
                    if (bOK) {
                        mnTrackMethod = 4;
                    }
                }
            }
            // 将最新的关键帧作为当前帧的参考关键帧
            // mpReferenceKF先是上一时刻的参考关键帧，如果当前为新关键帧则变成当前关键帧，如果不是新的关键帧则先为上一帧的参考关键帧，而后经过更新局部关键帧重新确定
            if (!mCurFrame.mpReferenceKF)
                mCurFrame.mpReferenceKF = mpReferenceKF;
            // Step 7 在跟踪得到当前帧初始姿态后，现在对local map进行跟踪得到更多的匹配，并优化当前位姿
            // 前面只是跟踪一帧得到初始位姿，这里搜索局部关键帧、局部地图点，和当前帧进行投影匹配，得到更多匹配的MapPoints后进行Pose优化
            // 在帧间匹配得到初始的姿态后，现在对local map进行跟踪得到更多的匹配，并优化当前位姿
            // local map:当前帧、当前帧的MapPoints、当前关键帧与其它关键帧共视关系
            // 前面主要是两两跟踪（恒速模型跟踪上一帧、跟踪参考帧），这里搜索局部关键帧后搜集所有局部MapPoints，
            // 然后将局部MapPoints和当前帧进行投影匹配，得到更多匹配的MapPoints后进行Pose优化
            // If we have an initial estimation of the camera pose and matching. Track the local map.
            if (bOK) {
                // 局部地图跟踪
                bOK = TrackLocalMap();
            }

            if (bOK) {
                mState = OK;
            } else if (mState == OK) {  // 由上图可知只有当第一阶段跟踪成功，但第二阶段局部地图跟踪失败时执行
                if (pCurrentMap->GetImuInitialized() &&
                    (mCurFrame.mnId > (mnFrameIdLastReloc + mnFrameNumDurRefLoc)) &&
                    (pCurrentMap->GetKeyFramesNumInMap() > mnMaxFrames / 3.0)) {
                    // cout << "KF in map: " << pCurrentMap->GetKeyFramesNumInMap() << endl;
                    // 条件1：当前地图中关键帧数目较多（大于10）
                    // 条件2（隐藏条件）：当前帧距离上次重定位帧超过1s或者非IMU模式
                    // 同时满足条件1，2，则将状态标记为RECENTLY_LOST，后面会结合IMU预测的位姿看看能不能拽回来
                    mState = RECENTLY_LOST;
                    mnFrameIdRecLost = mCurFrame.mnId;
                    cout << "State OK to RECENTLY_LOST" << endl;
                } else {
                    mState = LOST;
                    mnFrameIdLost = mCurFrame.mnId;
                    cout << "State OK to LOST" << endl;
                }
            } else if (mState == RECENTLY_LOST) {
                if ((mCurFrame.mnId > (mnFrameIdRecLost + mnFrameNumDurLost))) {
                    mState = LOST;
                    mnFrameIdLost = mCurFrame.mnId;
                    cout << "State RECENTLY_LOST to LOST" << endl;
                }
            }
            // Update6DoF drawer
            mpFrameDrawer->Update(this, mpViewer->mbFrameBoth);
            if (mCurFrame.HasPose())
                mpMapDrawer->SetCurrentCameraPose(mCurFrame.GetPose());

            // Step 9 如果跟踪成功 或 最近刚刚跟丢，更新速度，清除无效地图点，按需创建关键帧
            if (bOK || mState == RECENTLY_LOST) {
                // Update6DoF motion model
                // Step 9.1 更新恒速运动模型 TrackWithMotionModel 中的mVelocity
                if (mLastFrame.HasPose() && mCurFrame.HasPose()) {
                    Sophus::SE3f LastTwc = mLastFrame.GetPose().inverse();
                    // mVelocity = Tcl = Tcw * Twl,表示上一帧到当前帧的变换， 其中 Twl = LastTwc
                    mVelocity = mCurFrame.GetPose() * LastTwc;
                    mbVelocity = true;
                } else {
                    // 否则没有速度
                    mbVelocity = false;
                }
                // 使用IMU积分的位姿显示
                mpMapDrawer->SetCurrentCameraPose(mCurFrame.GetPose());
                // Clean VO matches
                // Step 9.2 清除观测不到的地图点
                for (int i = 0; i < mCurFrame.mnKPsLeftNum; i++) {
                    MapPoint *pMP = mCurFrame.mvpMPs[i];
                    if (pMP)
                        if (pMP->GetObsTimes() < 1) {
                            mCurFrame.mvbOutlier[i] = false;
                            mCurFrame.mvpMPs[i] = static_cast<MapPoint *>(NULL);
                        }
                }
                // Check if we need to insert a new keyframe
                bool bNeedKF = NeedNewKeyFrame();
                if (bNeedKF && (bOK || (mInsertKFsLost && mState == RECENTLY_LOST)))
                    CreateNewKeyFrame();
                // We allow points with high innovation (considererd outliers by the Huber Function)
                // pass to the new keyframe, so that bundle adjustment will finally decide
                // if they are outliers or not. We don'mTs want next frame to estimate its position
                // with those points so we discard them in the frame. Only has effect if lastframe is tracked

                //  Step 9.5 删除那些在BA中检测为外点的地图点
                for (int i = 0; i < mCurFrame.mnKPsLeftNum; i++) {
                    if (mCurFrame.mvpMPs[i] && mCurFrame.mvbOutlier[i]) {
                        mCurFrame.mvpMPs[i] = static_cast<MapPoint *>(NULL);
                    }
                }
            } else if (mState == LOST) {
                if (mCurFrame.mnId > (mnFrameIdLost + mnFrameNumDurLost)) {
                    if (!pCurrentMap->GetImuInitialized() || pCurrentMap->GetKeyFramesNumInMap() <= mnMaxFrames / 3) {
                        mpSystem->ResetActiveMap();
                    } else {
                        CreateMapInAtlas();
                    }
                    // 干掉上一个关键帧
                    if (mpLastKeyFrame) {
                        mpLastKeyFrame = static_cast<KeyFrame *>(NULL);
                    }
                    return;
                }
            }
            // 确保已经设置了参考关键帧
            if (!mCurFrame.mpReferenceKF)
                mCurFrame.mpReferenceKF = mpReferenceKF;
            // 保存上一帧的数据,当前帧变上一帧
            mLastFrame = Frame(mCurFrame);
        }

        if (mState == OK || mState == RECENTLY_LOST) {
            // Store frame pose information to retrieve the complete camera trajectory afterwards.
            // Step 11：记录位姿信息，用于最后保存所有的轨迹
            if (mCurFrame.HasPose()) {
                // 计算相对姿态Tcr = Tcw * Twr, Twr = Trw^-1
                Sophus::SE3f Tcr_ = mCurFrame.GetPose() * mCurFrame.mpReferenceKF->GetPoseInverse();
                mlRelativeFramePoses.emplace_back(Tcr_);
                mlpRefKFs.emplace_back(mCurFrame.mpReferenceKF);
                mlTimestamp.emplace_back(mCurFrame.mdTimestamp);
                mlbLost.emplace_back(mState == LOST);
            } else {
                // This can happen if tracking is lost
                // 如果跟踪失败，则相对位姿使用上一次值
                mlRelativeFramePoses.emplace_back(mlRelativeFramePoses.back());
                mlpRefKFs.emplace_back(mlpRefKFs.back());
                mlTimestamp.emplace_back(mlTimestamp.back());
                mlbLost.emplace_back(mState == LOST);
            }
        }
    }

/*
 * @brief 双目和rgbd的地图初始化，比单目简单很多
 *
 * 由于具有深度信息，直接生成MapPoints
 */
    void Tracking::StereoInitialization() {
        if (mCurFrame.mnKPsLeftNum < 500) {
            cout << "not enough keypoints" << endl;
            return;
        }
        if (!mCurFrame.mpImuFromPrevKF || !mLastFrame.mpImuFromPrevKF) {
            cout << "not IMU meas" << endl;
            return;
        }
        if ((mCurFrame.mpImuFromPrevFrame->mAvgAcc - mLastFrame.mpImuFromPrevFrame->mAvgAcc).norm() <
            0.5) {
            cout << "not enough acceleration" << endl;
            return;
        }

        if (mpImuPreintegratedFromLastKF)
            delete mpImuPreintegratedFromLastKF;
        mpImuPreintegratedFromLastKF = new IMU::Preintegrated(IMU::Bias(), *mpImuCalib);
        mCurFrame.mpImuFromPrevKF = mpImuPreintegratedFromLastKF;

        // Set Frame pose to the origin (In case of inertial SLAM to imu)
        // 设定初始位姿为单位旋转，0平移，imu模式下设置的是相机位姿
        Eigen::Matrix3f Rwb0 = mCurFrame.mImuCalib.mTcb.rotationMatrix();
        Eigen::Vector3f twb0 = mCurFrame.mImuCalib.mTcb.translation();
        Eigen::Vector3f Vwb0;
        Vwb0.setZero();
        mCurFrame.SetImuPoseVelocity(Rwb0, twb0, Vwb0);

        // Create KeyFrame
        // 将当前帧构造为初始关键帧
        KeyFrame *pKFini = new KeyFrame(mCurFrame, mpAtlas->GetCurrentMap(), mpKeyFrameDB);

        // Insert KeyFrame in the map
        // 在地图中添加该初始关键帧
        mpAtlas->AddKeyFrame(pKFini);

        // Create MapPoints and asscoiate to KeyFrame
        for (int i = 0; i < mCurFrame.mnKPsLeftNum; i++) {
            // 只有具有正深度的点才会被构造地图点
            float z = mCurFrame.mvfMPDepth[i];
            if (z > 0) {
                // 通过反投影得到该特征点的世界坐标系下3D坐标
                Eigen::Vector3f x3D;
                mCurFrame.UnprojectStereo(i, x3D);
                MapPoint *pNewMP = new MapPoint(x3D, pKFini, mpAtlas->GetCurrentMap());
                // 为该MapPoint添加属性：
                // a.观测到该MapPoint的关键帧
                // mBiasOri.该MapPoint的描述子
                // c.该MapPoint的平均观测方向和深度范围
                pKFini->AddMapPoint(pNewMP, i);
                pNewMP->AddObsKFAndLRIdx(pKFini, i);
                pNewMP->ComputeDistinctiveDescriptors();
                pNewMP->UpdateNormalAndDepth();
                mpAtlas->AddMapPoint(pNewMP);
                mCurFrame.mvpMPs[i] = pNewMP;
            }
        }


        Verbose::PrintMess("New Map created with " + to_string(mpAtlas->MapPointsInMap()) + " points",
                           Verbose::VERBOSITY_QUIET);

        //cout << "Active map: " << mpAtlas->GetCurrentMap()->GetId() << endl;
        // 在局部地图中添加该初始关键帧
        mpLocalMapping->InsertKeyFrame(pKFini);

        // 更新当前帧为上一帧
        mLastFrame = Frame(mCurFrame);
        mnLastKeyFrameId = mCurFrame.mnId;
        mpLastKeyFrame = pKFini;

        mvpLocalKeyFrames.emplace_back(pKFini);
        mvpLocalMapPoints = mpAtlas->GetAllMapPoints();
        mpReferenceKF = pKFini;
        mCurFrame.mpReferenceKF = pKFini;

        // 把当前（最新的）局部MapPoints作为ReferenceMapPoints
        mpAtlas->SetReferenceMapPoints(mvpLocalMapPoints);
        mpAtlas->GetCurrentMap()->mvpInitKeyFrames.emplace_back(pKFini);
        mpMapDrawer->SetCurrentCameraPose(mCurFrame.GetPose());
        // 追踪成功
        mState = OK;
    }

/**
 * @brief 在Atlas中保存当前地图，创建新地图，所有跟状态相关的变量全部重置
 * 1. 前后两帧对应的时间戳反了
 * 2. imu模式下前后帧超过1s
 * 3. 上一帧为最近丢失且重定位失败时
 * 4. 重定位成功，局部地图跟踪失败
 */
    void Tracking::CreateMapInAtlas() {
        mnLastInitFrameId = mCurFrame.mnId;
        mpAtlas->CreateNewMap();
        mnInitialFrameId = mCurFrame.mnId + 1;
        mState = NO_IMAGES_YET;

        // Restart the variable with information about the last KF
        mbVelocity = false;
        //mnFrameIdLastReloc = mnLastInitFrameId; // The last relocation KF_id is the current id, because it is the new starting point for new map
        Verbose::PrintMess("First frame id in map: " + to_string(mnLastInitFrameId + 1), Verbose::VERBOSITY_NORMAL);
        if (mpImuPreintegratedFromLastKF) {
            delete mpImuPreintegratedFromLastKF;
            mpImuPreintegratedFromLastKF = new IMU::Preintegrated(IMU::Bias(), *mpImuCalib);
        }

        if (mpLastKeyFrame)
            mpLastKeyFrame = static_cast<KeyFrame *>(NULL);

        if (mpReferenceKF)
            mpReferenceKF = static_cast<KeyFrame *>(NULL);

        mLastFrame = Frame();
        mCurFrame = Frame();
        mvIniMatches.clear();
        mlQueueImuData.clear();
        mbCreatedMap = true;
    }

/*
 * @brief 检查上一帧中的地图点是否需要被替换
 *
 * Local Mapping线程可能会将关键帧中某些地图点进行替换，由于tracking中需要用到上一帧地图点，所以这里检查并更新上一帧中被替换的地图点
 * @see LocalMapping::FuseMapPointsInNeighbors()
 */
    void Tracking::CheckReplacedInLastFrame() {
        for (int i = 0; i < mLastFrame.mnKPsLeftNum; i++) {
            MapPoint *pMP = mLastFrame.mvpMPs[i];
            // 如果这个地图点存在
            if (pMP) {
                MapPoint *pRep = pMP->GetReplaced();
                if (pRep) {
                    mLastFrame.mvpMPs[i] = pRep;
                }
            }
        }
    }

/*
 * @brief 用参考关键帧的地图点来对当前普通帧进行跟踪
 *
 * Step 1：将当前普通帧的描述子转化为BoW向量
 * Step 2：通过词袋BoW加速当前帧与参考帧之间的特征点匹配
 * Step 3: 将上一帧的位姿态作为当前帧位姿的初始值
 * Step 4: 通过优化3D-2D的重投影误差来获得位姿
 * Step 5：剔除优化后的匹配点中的外点
 * @return 如果匹配数超10，返回true
 *
 */
    bool Tracking::TrackReferenceKeyFrame() {
        // Compute Bag of Words vector
        // Step 1：将当前帧的描述子转化为BoW向量
        mCurFrame.ComputeBoW();

        // We perform first an ORB matching with the reference keyframe
        // If enough matches are found we setup a PnP solver
        ORBmatcher matcher(0.7, true);
        vector<MapPoint *> vpMapPointMatches;

        // Step 2：通过词袋BoW加速当前帧与参考帧之间的特征点匹配
        int nmatches = matcher.SearchMatchFrameAndKFByBoW(mpReferenceKF, mCurFrame, vpMapPointMatches);

        // 匹配数目小于15，认为跟踪失败
        if (nmatches < 15) {
            cout << "TRACK_REF_KF: Less than 15 matches!!\n";
            return false;
        }

        // Step 3:将上一帧的位姿态作为当前帧位姿的初始值
        mCurFrame.mvpMPs = vpMapPointMatches;
        mCurFrame.SetPose(mLastFrame.GetPose());

        // cout << " TrackReferenceKeyFrame mLastFrame.mTcw:  " << mLastFrame.mTcw << endl;
        // Step 4:通过优化3D-2D的重投影误差来获得位姿
        Optimizer::PoseOptimization(&mCurFrame);

        // Discard outliers
        // Step 5：剔除优化后的匹配点中的外点
        //之所以在优化之后才剔除外点，是因为在优化的过程中就有了对这些外点的标记
        int nmatchesMap = 0;
        for (int i = 0; i < mCurFrame.mnKPsLeftNum; i++) {
            //if(i >= mCurFrame.Nleft) break;
            if (mCurFrame.mvpMPs[i]) {
                // 如果对应到的某个特征点是外点
                if (mCurFrame.mvbOutlier[i]) {
                    // 清除它在当前帧中存在过的痕迹
                    MapPoint *pMP = mCurFrame.mvpMPs[i];
                    mCurFrame.mvpMPs[i] = static_cast<MapPoint *>(NULL);
                    mCurFrame.mvbOutlier[i] = false;
                    pMP->mbTrackInLeftView = false;
                    pMP->mnLastFrameSeen = mCurFrame.mnId;
                    nmatches--;
                } else if (mCurFrame.mvpMPs[i]->GetObsTimes() > 0) {
                    nmatchesMap++;
                }
            }
        }
        return true;
    }

/**
 * @brief 更新上一帧位姿，在上一帧中生成临时地图点
 * 单目情况：只计算了上一帧的世界坐标系位姿
 * 双目和rgbd情况：选取有有深度值的并且没有被选为地图点的点生成新的临时地图点，提高跟踪鲁棒性
 */
    void Tracking::UpdateLastFramePose() {
        // Update6DoF pose according to reference keyframe
        // Step 1：利用参考关键帧更新上一帧在世界坐标系下的位姿
        // 上一普通帧的参考关键帧，注意这里用的是参考关键帧（位姿准）而不是上上一帧的普通帧
        KeyFrame *pRef = mLastFrame.mpReferenceKF;
        // ref_keyframe 到 lastframe的位姿变换
        Sophus::SE3f Tlr = mlRelativeFramePoses.back();
        // 将上一帧的世界坐标系下的位姿计算出来
        // l:last, r:reference, mGyr:world
        // Tlw = Tlr*Trw
        mLastFrame.SetPose(Tlr * pRef->GetPose());
    }

/**
 * @brief 根据恒定速度模型用上一帧地图点来对当前帧进行跟踪
 * Step 1：更新上一帧的位姿；对于双目或RGB-D相机，还会根据深度值生成临时地图点
 * Step 2：根据上一帧特征点对应地图点进行投影匹配
 * Step 3：优化当前帧位姿
 * Step 4：剔除地图点中外点
 * @return 如果匹配数大于10，认为跟踪成功，返回true
 */
    bool Tracking::TrackWithMotionModel() {
        // 最小距离 < 0.9*次小距离 匹配成功，检查旋转
        ORBmatcher matcher(0.9, true);

        mCurFrame.SetPose(mVelocity * mLastFrame.GetPose());

        // 清空当前帧的地图点
        fill(mCurFrame.mvpMPs.begin(), mCurFrame.mvpMPs.end(), static_cast<MapPoint *>(NULL));

        // ProjectMono points seen in previous frame
        // 设置特征匹配过程中的搜索半径
        int th = 15;

        // Step 3：用上一帧地图点进行投影匹配，如果匹配点不够，则扩大搜索半径再来一次
        int nmatches = matcher.SearchFrameAndFrameByProject(mCurFrame, mLastFrame, th, false);

        // If few matches, uses a wider window search
        // 如果匹配点太少，则扩大搜索半径再来一次
        if (nmatches < 20) {
            Verbose::PrintMess("Not enough matches, wider window search!!", Verbose::VERBOSITY_NORMAL);
            fill(mCurFrame.mvpMPs.begin(), mCurFrame.mvpMPs.end(), static_cast<MapPoint *>(NULL));

            nmatches = matcher.SearchFrameAndFrameByProject(mCurFrame, mLastFrame, 2 * th, false);
            Verbose::PrintMess("Matches with wider search: " + to_string(nmatches), Verbose::VERBOSITY_NORMAL);
        }

        // 这里不同于ORB-SLAM2的方式
        if (nmatches < 20) {
            Verbose::PrintMess("Not enough matches!!", Verbose::VERBOSITY_NORMAL);
            return false;
        }

        // Optimize frame pose with all matches
        // Step 4：利用3D-2D投影关系，优化当前帧位姿
        Optimizer::PoseOptimization(&mCurFrame);

        // Discard outliers
        // Step 5：剔除地图点中外点
        int nmatchesMap = 0;
        for (int i = 0; i < mCurFrame.mnKPsLeftNum; i++) {
            if (mCurFrame.mvpMPs[i]) {
                if (mCurFrame.mvbOutlier[i]) {
                    // 如果优化后判断某个地图点是外点，清除它的所有关系
                    MapPoint *pMP = mCurFrame.mvpMPs[i];

                    mCurFrame.mvpMPs[i] = static_cast<MapPoint *>(NULL);
                    mCurFrame.mvbOutlier[i] = false;
                    pMP->mnLastFrameSeen = mCurFrame.mnId;
                    nmatches--;
                } else if (mCurFrame.mvpMPs[i]->GetObsTimes() > 0)
                    // 累加成功匹配到的地图点数目
                    nmatchesMap++;
            }
        }
        return true;
    }

/**
 * @brief 用局部地图进行跟踪，进一步优化位姿
 *
 * 1. 更新局部地图，包括局部关键帧和关键点
 * 2. 对局部MapPoints进行投影匹配
 * 3. 根据匹配对估计当前帧的姿态
 * 4. 根据姿态剔除误匹配
 * @return true if success
 *
 * Step 1：更新局部关键帧mvpLocalKeyFrames和局部地图点mvpLocalMapPoints
 * Step 2：在局部地图中查找与当前帧匹配的MapPoints, 其实也就是对局部地图点进行跟踪
 * Step 3：更新局部所有MapPoints后对位姿再次优化
 * Step 4：更新当前帧的MapPoints被观测程度，并统计跟踪局部地图的效果
 * Step 5：决定是否跟踪成功
 */
    bool Tracking::TrackLocalMap() {

        // We have an estimation of the camera pose and some map points tracked in the frame.
        // We retrieve the local map and try to find matches to points in the local map.
        mTrackedFrame++;

        // Step 1：更新局部关键帧 mvpLocalKeyFrames 和局部地图点 mvpLocalMapPoints
        UpdateKFsAndMPsInLocal();
        // Step 2：筛选局部地图中新增的在视野范围内的地图点，投影到当前帧搜索匹配，得到更多的匹配关系
        MatchLocalMPsToCurFrame();


        // 在这个函数之前，在 Relocalization、TrackReferenceKeyFrame、TrackWithMotionModel 中都有位姿优化
        // Step 3：前面新增了更多的匹配关系，BA优化得到更准确的位姿
        int inliers;
        // IMU未初始化，仅优化位姿||初始化，重定位，重新开启一个地图都会使mnLastRelocFrameId变化
        if (!mpAtlas->GetImuInitialized() || mCurFrame.mnId <= mnFrameIdLastReloc + mnFrameNumDurRefLoc) {
//            Verbose::PrintMess("TLM: PoseOptimization ", Verbose::VERBOSITY_DEBUG);
            Optimizer::PoseOptimization(&mCurFrame);
        } else {
            if (!mbMapUpdated) {//mbMapUpdated变化见Tracking::TrackWithIMU()
//                Verbose::PrintMess("TLM: PoseInertialOptimizationLastFrame ", Verbose::VERBOSITY_DEBUG);
                // 使用上一普通帧以及当前帧的视觉信息和IMU信息联合优化当前帧位姿、速度和IMU零偏
                inliers = Optimizer::PoseInertialOptimizationLastFrame(&mCurFrame);
            } else {
//                Verbose::PrintMess("TLM: PoseInertialOptimizationLastKeyFrame ", Verbose::VERBOSITY_DEBUG);
                // 使用上一关键帧以及当前帧的视觉信息和IMU信息联合优化当前帧位姿、速度和IMU零偏
                inliers = Optimizer::PoseInertialOptimizationLastKeyFrame(&mCurFrame);
            }
        }

        mnLMInFMatchNum = 0;

        // Update6DoF MapPoints Statistics
        // Step 4：更新当前帧的地图点被观测程度，并统计跟踪局部地图后匹配数目
        for (int i = 0; i < mCurFrame.mnKPsLeftNum; i++) {
            if (mCurFrame.mvpMPs[i]) {
                // 由于当前帧的地图点可以被当前帧观测到，其被观测统计量加1
                if (!mCurFrame.mvbOutlier[i]) {
                    // 找到该点的帧数mnFound 加 1
                    mCurFrame.mvpMPs[i]->IncreaseFound();
                    // 如果该地图点被相机观测数目nObs大于0，匹配内点计数+1
                    // nTimesObs： 被观测到的相机数目，单目+1，双目或RGB-D则+2
                    if (mCurFrame.mvpMPs[i]->GetObsTimes() > 0)
                        mnLMInFMatchNum++;
                }
            }
        }

        // Decide if the tracking was succesful
        // More restrictive if there was a relocalization recently
        // Step 5：根据跟踪匹配数目及重定位情况决定是否跟踪成功
        // 如果最近刚刚发生了重定位,那么至少成功匹配50个点才认为是成功跟踪
        if (mCurFrame.mnId < mnFrameIdLastReloc + mnFrameNumDurRefLoc && mnLMInFMatchNum < 50)
            return false;

        // RECENTLY_LOST状态下，至少成功跟踪10个才算成功
        if ((mnLMInFMatchNum > 10) && (mState == RECENTLY_LOST))
            return true;

        // 单目IMU模式下做完初始化至少成功跟踪15个才算成功，没做初始化需要50个
        if (mnLMInFMatchNum < 15) {
            return false;
        } else {
            return true;
        }
    }

/**
 * @brief 判断当前帧是否需要插入关键帧
 *
 * Step 1：纯VO模式下不插入关键帧，如果局部地图被闭环检测使用，则不插入关键帧
 * Step 2：如果距离上一次重定位比较近，或者关键帧数目超出最大限制，不插入关键帧
 * Step 3：得到参考关键帧跟踪到的地图点数量
 * Step 4：查询局部地图管理器是否繁忙,也就是当前能否接受新的关键帧
 * Step 5：对于双目或RGBD摄像头，统计可以添加的有效地图点总数 和 跟踪到的地图点数量
 * Step 6：决策是否需要插入关键帧
 * @return true         需要
 * @return false        不需要
 */
    bool Tracking::NeedNewKeyFrame() {
        // 如果是IMU模式并且当前地图中未完成IMU初始化
        if (!mpAtlas->GetCurrentMap()->GetImuInitialized()) {
            // 如果是IMU模式，当前帧距离上一关键帧时间戳超过0.25s，则说明需要插入关键帧，不再进行后续判断
            if ((mCurFrame.mdTimestamp - mpLastKeyFrame->mdTimestamp) >= 0.5)
                return true;
            else
                return false;
        }

        // If Local Mapping is freezed by a Loop Closure do not insert keyframes
        // Step 2：have stop or 如果局部地图线程被闭环检测使用，则不插入关键帧
        if (mpLocalMapping->CheckPaused() || mpLocalMapping->CheckRequestPause()) {
            return false;
        }

        // 获取当前地图中的关键帧数目
        const int nKFs = mpAtlas->KeyFramesInMap();

        // Do not insert keyframes if not enough frames have passed from last relocalisation
        // mCurFrame.mnId是当前帧的ID
        // mnLastRelocFrameId是最近一次重定位帧的ID
        // mMaxFrames等于图像输入的帧率
        //  Step 3：如果距离上一次重定位比较近，并且关键帧数目超出最大限制，不插入关键帧
        if (mCurFrame.mnId < mnFrameIdLastReloc + mnFrameNumDurRefLoc && nKFs > mnMaxFrames) {
            return false;
        }

        // Tracked MapPoints in the reference keyframe
        // Step 4：得到参考关键帧跟踪到的地图点数量
        // UpdateLocalKeyFrames 函数中会将与当前关键帧共视程度最高的关键帧设定为当前帧的参考关键帧

        // 地图点的最小观测次数
        int nMinObs = 3;
        if (nKFs <= 2)
            nMinObs = 2;
        // 参考关键帧地图点中观测的数目>= nMinObs的地图点数目
        int nRefMatches = mpReferenceKF->TrackedMapPoints(nMinObs);

        // Local Mapping accept keyframes?
        // Step 5：查询局部地图线程是否繁忙，当前能否接受新的关键帧
        bool bLocalMappingIdle = mpLocalMapping->AcceptKeyFrames();

        // Check how many "close" points are being tracked and how many could be potentially created.
        // Step 6：对于双目或RGBD摄像头，统计成功跟踪的近点的数量，如果跟踪到的近点太少，没有跟踪到的近点较多，可以插入关键帧
        int nNonTrackedClose = 0;  // 双目或RGB-D中没有跟踪到的近点
        int nTrackedClose = 0;  // 双目或RGB-D中成功跟踪的近点（三维点）
        int N = mCurFrame.mnKPsLeftNum;
        for (int i = 0; i < N; i++) {
            // 深度值在有效范围内
            if (mCurFrame.mvfMPDepth[i] > 0 && mCurFrame.mvfMPDepth[i] < mfThDepth) {
                if (mCurFrame.mvpMPs[i] && !mCurFrame.mvbOutlier[i])
                    nTrackedClose++;
                else
                    nNonTrackedClose++;

            }
        }
        //Verbose::PrintMess("[NEEDNEWKF]-> closed points: " + to_string(nTrackedClose) + "; non tracked closed points: " + to_string(nNonTrackedClose), Verbose::VERBOSITY_NORMAL);// Verbose::VERBOSITY_DEBUG);

        // 双目或RGBD情况下：跟踪到的地图点中近点太少 同时 没有跟踪到的三维点太多，可以插入关键帧了
        // 单目时，为false
        bool bNeedToInsertClose;
        bNeedToInsertClose = (nTrackedClose < 100) && (nNonTrackedClose > 70);

        // Step 7：决策是否需要插入关键帧
        // Thresholds
        // Step 7.1：设定比例阈值，当前帧和参考关键帧跟踪到点的比例，比例越大，越倾向于增加关键帧
        float thRefRatio = 0.75f;
        // 关键帧只有一帧，那么插入关键帧的阈值设置的低一点，插入频率较低
        if (nKFs < 2)
            thRefRatio = 0.4f;

        // Condition 1a: More than "MaxFrames" have passed from last keyframe insertion
        // Step 7.2：很长时间没有插入关键帧，可以插入
        const bool c1a = mCurFrame.mnId >= mnLastKeyFrameId + mnMaxFrames;
        // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
        // Step 7.3：满足插入关键帧的最小间隔并且localMapper处于空闲状态，可以插入
        const bool c1b = ((mCurFrame.mnId >= mnLastKeyFrameId + mMinFrames) &&
                          bLocalMappingIdle); //mpLocalMapping->KeyframesInQueue() < 2);
        //Condition 1c: tracking is weak
        // Step 7.4：在双目，RGB-D的情况下当前帧跟踪到的点比参考关键帧的0.25倍还少，或者满足bNeedToInsertClose
        // Condition 2: Few tracked points compared to reference keyframe. Lots of visual odometry compared to map matches.
        // Step 7.5：和参考帧相比当前跟踪到的点太少 或者满足bNeedToInsertClose；同时跟踪到的内点还不能太少
        const bool c2 = (((mnLMInFMatchNum < nRefMatches * thRefRatio || bNeedToInsertClose)) &&
                         mnLMInFMatchNum > 15);

        //std::cout << "NeedNewKF: c1a=" << c1a << "; c1b=" << c1b << "; c1c=" << c1c << "; c2=" << c2 << std::endl;
        // Temporal condition for Inertial cases
        // 新增的条件c3：单目/双目+IMU模式下，并且IMU完成了初始化（隐藏条件），当前帧和上一关键帧之间时间超过0.5秒，则c3=true
        bool c3 = false;
        if (mpLastKeyFrame) {
            if ((mCurFrame.mdTimestamp - mpLastKeyFrame->mdTimestamp) >= 0.5)
                c3 = true;
        }
        if (((c1a || c1b) && c2) || c3) {
            if (bLocalMappingIdle || mpLocalMapping->IsInitializing()) {
                // 可以插入关键帧
                return true;
            } else {
                mpLocalMapping->InterruptBA();
                if (mpLocalMapping->KeyframesInQueue() < 3)
                    return true;
                else
                    return false;
            }
        } else
            return false;
    }

/**
 * @brief 创建新的关键帧
 * 对于非单目的情况，同时创建新的MapPoints
 *
 * Step 1：将当前帧构造成关键帧
 * Step 2：将当前关键帧设置为当前帧的参考关键帧
 * Step 3：对于双目或rgbd摄像头，为当前帧生成新的MapPoints
 */
    void Tracking::CreateNewKeyFrame() {
        // 如果局部建图线程正在初始化且没做完或关闭了,就无法插入关键帧
        if (mpLocalMapping->IsInitializing() && !mpAtlas->GetImuInitialized()) {
            return;
        }

        if (!mpLocalMapping->RequestNotPauseOrFinish(true)) {
            return;
        }

        // Step 1：将当前帧构造成关键帧
        KeyFrame *pKF = new KeyFrame(mCurFrame, mpAtlas->GetCurrentMap(), mpKeyFrameDB);

        if (mpAtlas->GetImuInitialized()) //  || mpLocalMapping->IsInitializing())
            pKF->bImu = true;

        pKF->SetNewBias(mCurFrame.mImuBias);
        // Step 2：将当前关键帧设置为当前帧的参考关键帧
        // 在UpdateLocalKeyFrames函数中会将与当前关键帧共视程度最高的关键帧设定为当前帧的参考关键帧
        mpReferenceKF = pKF;
        mCurFrame.mpReferenceKF = pKF;

        if (mpLastKeyFrame) {
            pKF->mPrevKF = mpLastKeyFrame;
            mpLastKeyFrame->mNextKF = pKF;
        } else
            Verbose::PrintMess("No last KF in KF creation!!", Verbose::VERBOSITY_NORMAL);

        // CheckRequestReset preintegration from last KF (Create new object)
        mpImuPreintegratedFromLastKF = new IMU::Preintegrated(pKF->GetImuBias(), pKF->mImuCalib);

        // 这段代码和 Tracking::UpdateLastFramePose 中的那一部分代码功能相同
        // Step 3：对于双目或rgbd摄像头，为当前帧生成新的地图点；单目无操作
        // 根据Tcw计算mRcw、mtcw和mRwc、mOw
        mCurFrame.UpdatePoseMatrices();
        // cout << "create new MPs" << endl;
        // We sort points by the measured depth by the stereo/RGBD sensor.
        // We create all those MapPoints whose depth < mfThCloseFar.
        // If there are less than 100 close points we create the 100 closest.
        int maxPoint = 100;

        // Step 3.1：得到当前帧有深度值的特征点（不一定是地图点）
        vector<pair<float, int> > vDepthIdx;
        int N = mCurFrame.mnKPsLeftNum;
        vDepthIdx.reserve(mCurFrame.mnKPsLeftNum);
        for (int i = 0; i < N; i++) {
            float z = mCurFrame.mvfMPDepth[i];
            if (z > 0) {
                // 第一个元素是深度,第二个元素是对应的特征点的id
                vDepthIdx.emplace_back(make_pair(z, i));
            }
        }

        if (!vDepthIdx.empty()) {
            // Step 3.2：按照深度从小到大排序
            sort(vDepthIdx.begin(), vDepthIdx.end());

            // Step 3.3：从中找出不是地图点的生成临时地图点
            // 处理的近点的个数
            int nPoints = 0;
            for (size_t j = 0; j < vDepthIdx.size(); j++) {
                int i = vDepthIdx[j].second;

                bool bCreateNew = false;

                // 如果这个点对应在上一帧中的地图点没有,或者创建后就没有被观测到,那么就生成一个临时的地图点
                MapPoint *pMP = mCurFrame.mvpMPs[i];
                if (!pMP) {
                    bCreateNew = true;
                } else if (pMP->GetObsTimes() < 1) {
                    bCreateNew = true;
                    mCurFrame.mvpMPs[i] = static_cast<MapPoint *>(NULL);
                }

                // 如果需要就新建地图点，这里的地图点不是临时的，是全局地图中新建地图点，用于跟踪
                if (bCreateNew) {
                    Eigen::Vector3f x3D;
                    mCurFrame.UnprojectStereo(i, x3D);

                    MapPoint *pNewMP = new MapPoint(x3D, pKF, mpAtlas->GetCurrentMap());
                    // 这些添加属性的操作是每次创建MapPoint后都要做的
                    pNewMP->AddObsKFAndLRIdx(pKF, i);
                    pKF->AddMapPoint(pNewMP, i);
                    pNewMP->ComputeDistinctiveDescriptors();
                    pNewMP->UpdateNormalAndDepth();
                    mpAtlas->AddMapPoint(pNewMP);

                    mCurFrame.mvpMPs[i] = pNewMP;
                    nPoints++;
                } else {
                    // 因为从近到远排序，记录其中不需要创建地图点的个数
                    nPoints++;
                }

                // Step 3.4：停止新建地图点必须同时满足以下条件：
                // 1、当前的点的深度已经超过了设定的深度阈值（35倍基线）
                // 2、nPoints已经超过100个点，说明距离比较远了，可能不准确，停掉退出
                if (vDepthIdx[j].first > mfThDepth && nPoints > maxPoint) {
                    break;
                }
            }
            //Verbose::PrintMess("new mps for stereo KF: " + to_string(nPoints), Verbose::VERBOSITY_NORMAL);
        }


        // Step 4：插入关键帧
        // 关键帧插入到列表 mlNewKeyFrames中，等待local mapping线程临幸
        mpLocalMapping->InsertKeyFrame(pKF);

        // 插入好了，允许局部建图停止
        mpLocalMapping->RequestNotPauseOrFinish(false);

        // 当前帧成为新的关键帧，更新
        mnLastKeyFrameId = mCurFrame.mnId;
        mpLastKeyFrame = pKF;
    }

/**
 * @brief 用局部地图点进行投影匹配，得到更多的匹配关系
 * 注意：局部地图点中已经是当前帧地图点的不需要再投影，只需要将此外的并且在视野范围内的点和当前帧进行投影匹配
 */
    int Tracking::MatchLocalMPsToCurFrame() {
        // Do not search map points already matched, all happened in initial Frame.
        // Step 1：遍历当前帧的地图点，标记这些地图点不参与之后的投影搜索匹配
        for (vector<MapPoint *>::iterator vit = mCurFrame.mvpMPs.begin(), vend = mCurFrame.mvpMPs.end();
             vit != vend; vit++) {
            MapPoint *pMP = *vit;
            if (pMP) {
                if (pMP->isBad()) {
                    *vit = static_cast<MapPoint *>(NULL);
                } else {
                    // 更新能观测到该点的帧数加1(被当前帧观测了)
                    pMP->IncreaseVisible();
                    // 标记该点被当前帧观测到
                    pMP->mnLastFrameSeen = mCurFrame.mnId;
                    // 标记该点在后面搜索匹配时不被投影，因为已经有匹配了
                    pMP->mbTrackInLeftView = false;
                }
            }
        }

        // 准备进行投影匹配的点的数目
        int nToMatch = 0;

        // ProjectMono points in frame and check its visibility
        // Step 2：判断所有局部地图点中除当前帧地图点外的点，是否在当前帧视野范围内
        for (vector<MapPoint *>::iterator vit = mvpLocalMapPoints.begin(), vend = mvpLocalMapPoints.end();
             vit != vend; vit++) {
            MapPoint *pMP = *vit;
            if (!pMP) {
                continue;
            }
            if (pMP->isBad()) {
                continue;
            }
            // 已经被当前帧观测到的地图点肯定在视野范围内，跳过
            if (pMP->mnLastFrameSeen == mCurFrame.mnId) {
                continue;
            }
            // ProjectMono (this fills MapPoint variables for matching)
            // 判断地图点是否在在当前帧视野内
            if (mCurFrame.isInFrustum(pMP, 0.5)) {
                // 观测到该点的帧数加1
                pMP->IncreaseVisible();
                // 只有在视野范围内的地图点才参与之后的投影匹配
                nToMatch++;
            }
        }

        // Step 3：如果需要进行投影匹配的点的数目大于0，就进行投影匹配，增加更多的匹配关系
        if (nToMatch < 5) {
            return nToMatch;
        }
        ORBmatcher matcher(0.8);
        int nThProjRad = 1;
        if (mpAtlas->GetImuInitialized()) {
            if (mpAtlas->GetCurrentMap()->GetImuIniertialBA2()) {
                nThProjRad = 2;
            } else {
                nThProjRad = 6;  // 0.4版本这里是3
            }
        } else if (!mpAtlas->GetImuInitialized()) {
            nThProjRad = 10;
        }
        // If the camera has been relocalised recently, perform a coarser search
        // 如果不久前进行过重定位，那么进行一个更加宽泛的搜索，阈值需要增大
        if (mCurFrame.mnId < mnFrameIdLastReloc + mnFrameNumDurRefLoc) {
            nThProjRad = 5;
        }
        if (mState == LOST || mState == RECENTLY_LOST) {
            nThProjRad = 15;
        }
        // 投影匹配得到更多的匹配关系
        int matches = matcher.SearchReplaceFrameAndMPsByProject(mCurFrame, mvpLocalMapPoints, nThProjRad,
                                                                mpLocalMapping->mbFarPoints,
                                                                mpLocalMapping->mfThFarPoints);
        return matches;
    }

/**
 * @brief 更新LocalMap
 *
 * 局部地图包括：
 * 1、K1个关键帧、K2个临近关键帧和参考关键帧
 * 2、由这些关键帧观测到的MapPoints
 */
    void Tracking::UpdateKFsAndMPsInLocal() {
        // Update6DoF
        // 用共视图来更新局部关键帧和局部地图点
        UpdateLocalKeyFrames();
        UpdateLocalMapPoints();

        // This is for visualization
        // 设置参考地图点用于绘图显示局部地图点（红色）
        mpAtlas->SetReferenceMapPoints(mvpLocalMapPoints);
        mpAtlas->SetReferenceKeyFrames(mvpLocalKeyFrames);
    }

/*
 * @brief 更新局部关键点。先把局部地图清空，然后将局部关键帧的有效地图点添加到局部地图中
 */
    void Tracking::UpdateLocalMapPoints() {
        // Step 1：清空局部地图点
        mvpLocalMapPoints.clear();
        mvpLocalMapPoints.reserve(mvpLocalKeyFrames.size() * 300);
        int count_pts = 0;
        // Step 2：遍历局部关键帧 mvpLocalKeyFrames
        for (vector<KeyFrame *>::const_reverse_iterator itKF = mvpLocalKeyFrames.rbegin(), itEndKF = mvpLocalKeyFrames.rend();
             itKF != itEndKF; ++itKF) {
            KeyFrame *pKF = *itKF;
            const vector<MapPoint *> vpMPs = pKF->GetVectorMapPointsInKF();

            // step 2：将局部关键帧的地图点添加到mvpLocalMapPoints
            for (vector<MapPoint *>::const_iterator itMP = vpMPs.begin(), itEndMP = vpMPs.end();
                 itMP != itEndMP; itMP++) {
                MapPoint *pMP = *itMP;
                if (!pMP) {
                    continue;
                }
                if (pMP->isBad()) {
                    continue;
                }
                if (pMP->mnTrackReferenceForFrame == mCurFrame.mnId) {
                    continue;
                }
                count_pts++;
                mvpLocalMapPoints.emplace_back(pMP);
                pMP->mnTrackReferenceForFrame = mCurFrame.mnId;
            }
        }
    }

/**
 * @brief 跟踪局部地图函数里，更新局部关键帧
 * 方法是遍历当前帧的地图点，将观测到这些地图点的关键帧和相邻的关键帧及其父子关键帧，作为mvpLocalKeyFrames
 * Step 1：遍历当前帧的地图点，记录所有能观测到当前帧地图点的关键帧
 * Step 2：更新局部关键帧（mvpLocalKeyFrames），添加局部关键帧包括以下3种类型
 *      类型1：能观测到当前帧地图点的关键帧，也称一级共视关键帧
 *      类型2：一级共视关键帧的共视关键帧，称为二级共视关键帧
 *      类型3：一级共视关键帧的子关键帧、父关键帧
 * Step 3：更新当前帧的参考关键帧，与自己共视程度最高的关键帧作为参考关键帧
 */
    void Tracking::UpdateLocalKeyFrames() {
        // Each map point vote for the keyframes in which it has been observed
        // Step 1：遍历当前帧的地图点，记录所有能观测到当前帧地图点的关键帧
        map<KeyFrame *, int> keyframeCounter;
        // 如果IMU未初始化 或者 刚刚完成重定位
        if (!mpAtlas->GetImuInitialized() || (mCurFrame.mnId < mnFrameIdLastReloc + mnFrameNumDurRefLoc)) {
            for (int i = 0; i < mCurFrame.mnKPsLeftNum; i++) {
                MapPoint *pMP = mCurFrame.mvpMPs[i];
                if (pMP) {
                    if (!pMP->isBad()) {
                        // 得到观测到该地图点的关键帧和该地图点在关键帧中的索引
                        const map<KeyFrame *, tuple<int, int>> observations = pMP->GetObsKFAndLRIdx();
                        // 由于一个地图点可以被多个关键帧观测到,因此对于每一次观测,都对观测到这个地图点的关键帧进行累计投票
                        for (map<KeyFrame *, tuple<int, int>>::const_iterator it = observations.begin(), itend = observations.end();
                             it != itend; it++)
                            // map[key] = value，当要插入的键存在时，会覆盖键对应的原来的值。如果键不存在，则添加一组键值对
                            // it->first 是地图点看到的关键帧，同一个关键帧看到的地图点会累加到该关键帧计数
                            // 所以最后keyframeCounter 第一个参数表示某个关键帧，第2个参数表示该关键帧看到了多少当前帧(mCurFrame)的地图点，也就是共视程度
                            keyframeCounter[it->first]++;
                    } else {
                        mCurFrame.mvpMPs[i] = static_cast<MapPoint *>(NULL);
                    }
                }
            }
        } else {
            // ?为什么IMU初始化后用mLastFrame？mLastFrame存储的是上一帧跟踪成功后帧数据。
            for (int i = 0; i < mLastFrame.mnKPsLeftNum; i++) {
                // Using lastframe since current frame has not matches yet
                if (mLastFrame.mvpMPs[i]) {
                    MapPoint *pMP = mLastFrame.mvpMPs[i];
                    if (!pMP)
                        continue;
                    if (!pMP->isBad()) {
                        const map<KeyFrame *, tuple<int, int>> observations = pMP->GetObsKFAndLRIdx();
                        for (map<KeyFrame *, tuple<int, int>>::const_iterator it = observations.begin(), itend = observations.end();
                             it != itend; it++)
                            keyframeCounter[it->first]++;
                    } else {
                        // MODIFICATION
                        mLastFrame.mvpMPs[i] = static_cast<MapPoint *>(NULL);
                    }
                }
            }
        }

        // 存储具有最多观测次数（max）的关键帧
        int max = 0;
        KeyFrame *pKFmax = static_cast<KeyFrame *>(NULL);

        // Step 2：更新局部关键帧（mvpLocalKeyFrames），添加局部关键帧有3种类型
        // 先清空局部关键帧
        mvpLocalKeyFrames.clear();
        mvpLocalKeyFrames.reserve(3 * keyframeCounter.size());

        // All keyframes that observe a map point are included in the local map. Also check which keyframe shares most points
        // Step 2.1 类型1：能观测到当前帧地图点的关键帧作为局部关键帧 （将邻居拉拢入伙）（一级共视关键帧）
        for (map<KeyFrame *, int>::const_iterator it = keyframeCounter.begin(), itEnd = keyframeCounter.end();
             it != itEnd; it++) {
            KeyFrame *pKF = it->first;

            // 如果设定为要删除的，跳过
            if (pKF->isBad())
                continue;

            // 寻找具有最大观测数目的关键帧
            if (it->second > max) {
                max = it->second;
                pKFmax = pKF;
            }

            // 添加到局部关键帧的列表里
            mvpLocalKeyFrames.emplace_back(pKF);
            // 用该关键帧的成员变量mnTrackReferenceForFrame 记录当前帧的id
            // 表示它已经是当前帧的局部关键帧了，可以防止重复添加局部关键帧
            pKF->mnTrackReferenceForFrame = mCurFrame.mnId;
        }

        // Include also some not-already-included keyframes that are neighbors to already-included keyframes
        // Step 2.2 遍历一级共视关键帧，寻找更多的局部关键帧
        for (vector<KeyFrame *>::const_iterator itKF = mvpLocalKeyFrames.begin(), itEndKF = mvpLocalKeyFrames.end();
             itKF != itEndKF; itKF++) {
            // Limit the number of keyframes
            // 处理的局部关键帧不超过80帧
            if (mvpLocalKeyFrames.size() > mnMaxFrames * 2) // 80
                break;

            KeyFrame *pKF = *itKF;

            // 类型2:一级共视关键帧的共视（前10个）关键帧，称为二级共视关键帧（将邻居的邻居拉拢入伙）
            // 如果共视帧不足10帧,那么就返回所有具有共视关系的关键帧
            const vector<KeyFrame *> vNeighs = pKF->GetBestCovisibilityKeyFrames(5);

            // vNeighs 是按照共视程度从大到小排列
            for (vector<KeyFrame *>::const_iterator itNeighKF = vNeighs.begin(), itEndNeighKF = vNeighs.end();
                 itNeighKF != itEndNeighKF; itNeighKF++) {
                KeyFrame *pNeighKF = *itNeighKF;
                if (!pNeighKF->isBad()) {
                    // mnTrackReferenceForFrame防止重复添加局部关键帧
                    if (pNeighKF->mnTrackReferenceForFrame != mCurFrame.mnId) {
                        mvpLocalKeyFrames.emplace_back(pNeighKF);
                        pNeighKF->mnTrackReferenceForFrame = mCurFrame.mnId;
                    }
                }
            }

            // 类型3:将一级共视关键帧的子关键帧作为局部关键帧（将邻居的孩子们拉拢入伙）
            const set<KeyFrame *> spChilds = pKF->GetChilds();
            for (set<KeyFrame *>::const_iterator sit = spChilds.begin(), send = spChilds.end(); sit != send; sit++) {
                KeyFrame *pChildKF = *sit;
                if (!pChildKF->isBad()) {
                    if (pChildKF->mnTrackReferenceForFrame != mCurFrame.mnId) {
                        mvpLocalKeyFrames.emplace_back(pChildKF);
                        pChildKF->mnTrackReferenceForFrame = mCurFrame.mnId;
                    }
                }
            }

            // 类型3:将一级共视关键帧的父关键帧（将邻居的父母们拉拢入伙）
            KeyFrame *pParent = pKF->GetParent();
            if (pParent) {
                // mnTrackReferenceForFrame防止重复添加局部关键帧
                if (pParent->mnTrackReferenceForFrame != mCurFrame.mnId) {
                    mvpLocalKeyFrames.emplace_back(pParent);
                    pParent->mnTrackReferenceForFrame = mCurFrame.mnId;
                }
            }
        }

        // Add 10 last temporal KFs (mainly for IMU)
        // IMU模式下增加了临时的关键帧
        if (mvpLocalKeyFrames.size() < mnMaxFrames * 2) {
            KeyFrame *tempKeyFrame = mCurFrame.mpPrevKeyFrame;
            const int Nd = 20;
            for (int i = 0; i < Nd; i++) {
                if (!tempKeyFrame)
                    break;
                if (tempKeyFrame->mnTrackReferenceForFrame != mCurFrame.mnId) {
                    mvpLocalKeyFrames.emplace_back(tempKeyFrame);
                    tempKeyFrame->mnTrackReferenceForFrame = mCurFrame.mnId;
                    tempKeyFrame = tempKeyFrame->mPrevKF;
                }
            }
        }

        // Step 3：更新当前帧的参考关键帧，与自己共视程度最高的关键帧作为参考关键帧
        if (pKFmax) {
            mpReferenceKF = pKFmax;
            mCurFrame.mpReferenceKF = mpReferenceKF;
        }
    }

/**
 * @details 重定位过程
 * @return true
 * @return false
 *
 * Step 1：计算当前帧特征点的词袋向量
 * Step 2：找到与当前帧相似的候选关键帧
 * Step 3：通过BoW进行匹配
 * Step 4：通过EPnP算法估计姿态
 * Step 5：通过PoseOptimization对姿态进行优化求解
 * Step 6：如果内点较少，则通过投影的方式对之前未匹配的点进行匹配，再进行优化求解
 */
    bool Tracking::Relocalization() {
        Verbose::PrintMess("Starting relocalization", Verbose::VERBOSITY_NORMAL);
        // Compute Bag of Words Vector
        // Step 1: 计算当前帧特征点的Bow映射
        mCurFrame.ComputeBoW();

        // Relocalization is performed when tracking is lost
        // Track Lost: Query KeyFrame Database for keyframe candidates for relocalisation
        // Step 2：找到与当前帧相似的候选关键帧组
        vector<KeyFrame *> vpCandidateKFs = mpKeyFrameDB->DetectRelocalizationCandidates(&mCurFrame,
                                                                                         mpAtlas->GetCurrentMap());

        if (vpCandidateKFs.empty()) {
            Verbose::PrintMess("There are not candidates", Verbose::VERBOSITY_NORMAL);
            return false;
        }

        const int nKFs = vpCandidateKFs.size();

        // We perform first an ORB matching with each candidate
        // If enough matches are found we setup a PnP solver
        ORBmatcher matcher(0.75, true);

        // 每个关键帧的解算器
        vector<MLPnPsolver *> vpMLPnPsolvers;
        vpMLPnPsolvers.resize(nKFs);

        // 每个关键帧和当前帧中特征点的匹配关系
        vector<vector<MapPoint *> > vvpMapPointMatches;
        vvpMapPointMatches.resize(nKFs);

        // 放弃某个关键帧的标记
        vector<bool> vbDiscarded;
        vbDiscarded.resize(nKFs);

        // 有效的候选关键帧数目
        int nCandidates = 0;

        // Step 3：遍历所有的候选关键帧，通过BoW进行快速匹配，用匹配结果初始化PnP Solver
        for (int i = 0; i < nKFs; i++) {
            KeyFrame *pKF = vpCandidateKFs[i];
            if (pKF->isBad())
                vbDiscarded[i] = true;
            else {
                // 当前帧和候选关键帧用BoW进行快速匹配，匹配结果记录在vvpMapPointMatches，nmatches表示匹配的数目
                int nmatches = matcher.SearchMatchFrameAndKFByBoW(pKF, mCurFrame, vvpMapPointMatches[i]);
                // 如果和当前帧的匹配数小于15,那么只能放弃这个关键帧
                if (nmatches < 15) {
                    vbDiscarded[i] = true;
                    continue;
                } else {
                    // 如果匹配数目够用，用匹配结果初始化MLPnPsolver
                    // ? 为什么用MLPnP? 因为考虑了鱼眼相机模型，解耦某些关系？
                    // 参考论文《MLPNP-A REAL-TIME MAXIMUM LIKELIHOOD SOLUTION TO THE PERSPECTIVE-mnCurKPsLeft-POINT PROBLEM》
                    MLPnPsolver *pSolver = new MLPnPsolver(mCurFrame, vvpMapPointMatches[i]);
                    // 构造函数调用了一遍，这里重新设置参数
                    pSolver->SetRansacParameters(
                            0.99,                    // 模型最大概率值，默认0.9
                            10,                      // 内点的最小阈值，默认8
                            300,                     // 最大迭代次数，默认300
                            6,                       // 最小集，每次采样六个点，即最小集应该设置为6，论文里面写着I > 5
                            0.5,                     // 理论最少内点个数，这里是按照总数的比例计算，所以epsilon是比例，默认是0.4
                            5.991);                  // 卡方检验阈值 //This solver needs at least 6 points
                    vpMLPnPsolvers[i] = pSolver;
                    nCandidates++;  // 1.0版本新加的
                }
            }
        }

        // Alternatively perform some iterations of P4P RANSAC
        // Until we found a camera pose supported by enough inliers
        // 足够的内点才能匹配使用PNP算法，MLPnP需要至少6个点
        // 是否已经找到相匹配的关键帧的标志
        bool bMatch = false;
        ORBmatcher matcher2(0.9, true);

        // Step 4: 通过一系列操作,直到找到能够匹配上的关键帧
        // 为什么搞这么复杂？答：是担心误闭环
        while (nCandidates > 0 && !bMatch) {
            // 遍历当前所有的候选关键帧
            for (int i = 0; i < nKFs; i++) {
                // 忽略放弃的
                if (vbDiscarded[i])
                    continue;

                // Perform 5 Ransac Iterations
                // 内点标记
                vector<bool> vbInliers;
                // 内点数
                int nInliers;
                // 表示RANSAC已经没有更多的迭代次数可用 -- 也就是说数据不够好，RANSAC也已经尽力了。。。
                bool bNoMore;

                // Step 4.1：通过MLPnP算法估计姿态，迭代5次
                MLPnPsolver *pSolver = vpMLPnPsolvers[i];
                Eigen::Matrix4f eigTcw;
                // PnP算法的入口函数
                bool bTcw = pSolver->iterate(5, bNoMore, vbInliers, nInliers, eigTcw);

                // If Ransac reachs max. iterations discard keyframe
                // bNoMore 为true 表示已经超过了RANSAC最大迭代次数，就放弃当前关键帧
                if (bNoMore) {
                    vbDiscarded[i] = true;
                    nCandidates--;
                }

                // If a Camera Pose is computed, optimize
                if (bTcw) {
                    // Step 4.2：如果MLPnP 计算出了位姿，对内点进行BA优化
                    Sophus::SE3f Tcw(eigTcw);
                    mCurFrame.SetPose(Tcw);
                    // Tcw.copyTo(mCurFrame.mTcw);
                    // MLPnP 里RANSAC后的内点的集合
                    set<MapPoint *> sFound;

                    const int np = vbInliers.size();

                    // 遍历所有内点
                    for (int j = 0; j < np; j++) {
                        if (vbInliers[j]) {
                            mCurFrame.mvpMPs[j] = vvpMapPointMatches[i][j];
                            sFound.insert(vvpMapPointMatches[i][j]);
                        } else
                            mCurFrame.mvpMPs[j] = static_cast<MapPoint *>(NULL);
                    }

                    // 只优化位姿,不优化地图点的坐标，返回的是内点的数量
                    int nGood = Optimizer::PoseOptimization(&mCurFrame);

                    // 如果优化之后的内点数目不多，跳过了当前候选关键帧,但是却没有放弃当前帧的重定位
                    if (nGood < 10)
                        continue;

                    // 删除外点对应的地图点,这里直接设为空指针
                    for (int io = 0; io < mCurFrame.mnKPsLeftNum; io++)
                        if (mCurFrame.mvbOutlier[io])
                            mCurFrame.mvpMPs[io] = static_cast<MapPoint *>(NULL);

                    // If few inliers, search by projection in a coarse window and optimize again
                    // Step 4.3：如果内点较少，则通过投影的方式对之前未匹配的点进行匹配，再进行优化求解
                    // 前面的匹配关系是用词袋匹配过程得到的
                    if (nGood < 50) {
                        // 通过投影的方式将关键帧中未匹配的地图点投影到当前帧中, 生成新的匹配
                        int nadditional = matcher2.SearchFrameAndKFByProject(mCurFrame, vpCandidateKFs[i], sFound, 10,
                                                                             100);

                        // 如果通过投影过程新增了比较多的匹配特征点对
                        if (nadditional + nGood >= 50) {
                            // 根据投影匹配的结果，再次采用3D-2D pnp BA优化位姿
                            nGood = Optimizer::PoseOptimization(&mCurFrame);

                            // If many inliers but still not enough, search by projection again in a narrower window
                            // the camera has been already optimized with many points
                            // Step 4.4：如果BA后内点数还是比较少(<50)但是还不至于太少(>30)，可以挽救一下, 最后垂死挣扎
                            // 重新执行上一步 4.3的过程，只不过使用更小的搜索窗口
                            // 这里的位姿已经使用了更多的点进行了优化,应该更准，所以使用更小的窗口搜索
                            if (nGood > 30 && nGood < 50) {
                                // 用更小窗口、更严格的描述子阈值，重新进行投影搜索匹配
                                sFound.clear();
                                for (int ip = 0; ip < mCurFrame.mnKPsLeftNum; ip++)
                                    if (mCurFrame.mvpMPs[ip])
                                        sFound.insert(mCurFrame.mvpMPs[ip]);
                                nadditional = matcher2.SearchFrameAndKFByProject(mCurFrame, vpCandidateKFs[i], sFound,
                                                                                 3,
                                                                                 64);

                                // Final optimization
                                // 如果成功挽救回来，匹配数目达到要求，最后BA优化一下
                                if (nGood + nadditional >= 50) {
                                    nGood = Optimizer::PoseOptimization(&mCurFrame);
                                    // 更新地图点
                                    for (int io = 0; io < mCurFrame.mnKPsLeftNum; io++)
                                        if (mCurFrame.mvbOutlier[io])
                                            mCurFrame.mvpMPs[io] = static_cast<MapPoint *>(NULL);
                                }
                                // 如果还是不能够满足就放弃了
                            }
                        }
                    }


                    // If the pose is supported by enough inliers stop ransacs and continue
                    // 如果对于当前的候选关键帧已经有足够的内点(50个)了,那么就认为重定位成功
                    if (nGood >= 50) {
                        bMatch = true;
                        // 只要有一个候选关键帧重定位成功，就退出循环，不考虑其他候选关键帧了
                        break;
                    }
                }
            }// 一直运行,知道已经没有足够的关键帧,或者是已经有成功匹配上的关键帧
        }

        // 折腾了这么久还是没有匹配上，重定位失败
        if (!bMatch) {
            return false;
        } else {
            // 如果匹配上了,说明当前帧重定位成功了(当前帧已经有了自己的位姿)
            // 记录成功重定位帧的id，防止短时间多次重定位
            mnFrameIdLastReloc = mCurFrame.mnId;
            cout << "Relocalized!!" << endl;
            return true;
        }

    }

/**
 * @brief 整个追踪线程执行复位操作
 */
    void Tracking::ResetThread(bool bLocMap) {
        Verbose::PrintMess("System Reseting", Verbose::VERBOSITY_NORMAL);
        // 基本上是挨个请求各个线程终止
        if (mpViewer) {
            mpViewer->RequestReset();
            while (!mpViewer->CheckReseted())
                usleep(5000);
        }

        // CheckRequestReset Local Mapping
        if (!bLocMap) {
            Verbose::PrintMess("Reseting Local Mapper...", Verbose::VERBOSITY_NORMAL);
            mpLocalMapping->RequestReset();
            Verbose::PrintMess("done", Verbose::VERBOSITY_NORMAL);
        }


        // CheckRequestReset Loop Closing
        Verbose::PrintMess("Reseting Loop Closing...", Verbose::VERBOSITY_NORMAL);
        mpLoopClosing->RequestReset();
        Verbose::PrintMess("done", Verbose::VERBOSITY_NORMAL);

        // Clear BoW Database
        Verbose::PrintMess("Reseting Database...", Verbose::VERBOSITY_NORMAL);
        mpKeyFrameDB->clear();
        Verbose::PrintMess("done", Verbose::VERBOSITY_NORMAL);

        // Clear Map (this erase MapPoints and KeyFrames)
        mpAtlas->clearAtlas();
        mpAtlas->CreateNewMap();
        mnInitialFrameId = 0;

        // 然后复位各种变量
        KeyFrame::nNextId = 0;
        Frame::mnNextId = 0;
        mState = NO_IMAGES_YET;

        mlRelativeFramePoses.clear();
        mlpRefKFs.clear();
        mlTimestamp.clear();
        mlbLost.clear();
        mCurFrame = Frame();
        mnFrameIdLastReloc = 0;
        mLastFrame = Frame();
        mpReferenceKF = static_cast<KeyFrame *>(NULL);
        mpLastKeyFrame = static_cast<KeyFrame *>(NULL);
        mvIniMatches.clear();

        if (mpViewer)
            mpViewer->CancelReseted();

        Verbose::PrintMess("   End reseting! ", Verbose::VERBOSITY_NORMAL);
    }

/**
 * @brief 重置当前活动地图
 */
    void Tracking::ResetActiveMap(bool bLocMap) {
        Verbose::PrintMess("Active map Reseting", Verbose::VERBOSITY_NORMAL);
        if (mpViewer) {
            mpViewer->RequestReset();
            while (!mpViewer->CheckReseted())
                usleep(5000);
        }

        Map *pMap = mpAtlas->GetCurrentMap();

        if (!bLocMap) {
            Verbose::PrintMess("Send Reset LM", Verbose::VERBOSITY_VERY_VERBOSE);
            mpLocalMapping->RequestResetActiveMap(pMap);
            Verbose::PrintMess("Reset LM Done", Verbose::VERBOSITY_VERY_VERBOSE);
        }

        // CheckRequestReset Loop Closing
        Verbose::PrintMess("Send Reset LC", Verbose::VERBOSITY_NORMAL);
        mpLoopClosing->RequestResetActiveMap(pMap);
        Verbose::PrintMess("Reset LC Done", Verbose::VERBOSITY_NORMAL);

        // Clear BoW Database
        Verbose::PrintMess("Send Reset Database", Verbose::VERBOSITY_NORMAL);
        mpKeyFrameDB->clearMap(pMap); // Only clear the active map references
        Verbose::PrintMess("Reset Database Done", Verbose::VERBOSITY_NORMAL);

        // Clear Map (this erase MapPoints and KeyFrames)
        mpAtlas->clearMap();

        //KeyFrame::mnNextId = mpAtlas->GetLastInitKFid();
        //Frame::mnNextId = mnLastInitFrameId;
        mnLastInitFrameId = Frame::mnNextId;
        //mnFrameIdLastReloc = mnLastInitFrameId;
        mState = NO_IMAGES_YET; //NOT_INITIALIZED;

        list<bool> lbLost;
        // lbLost.reserve(mlbLost.ParameterSize());
        unsigned int index = mnFirstFrameId;
        cout << "mnFirstFrameId = " << mnFirstFrameId << endl;
        for (Map *pMap: mpAtlas->GetAllMaps()) {
            if (pMap->GetKeyFramesNumInMap() > 0) {
                if (index > pMap->GetLowerKFID())
                    index = pMap->GetLowerKFID();
            }
        }

        //cout << "First Frame id: " << index << endl;
        int num_lost = 0;
        cout << "mnInitialFrameId = " << mnInitialFrameId << endl;
        for (list<bool>::iterator ilbL = mlbLost.begin(); ilbL != mlbLost.end(); ilbL++) {
            if (index < mnInitialFrameId)
                lbLost.emplace_back(*ilbL);
            else {
                lbLost.emplace_back(true);
                num_lost += 1;
            }

            index++;
        }

        cout << num_lost << " Frames set to lost" << endl;
        mlbLost = lbLost;
        mnInitialFrameId = mCurFrame.mnId;
        mnFrameIdLastReloc = mCurFrame.mnId;
        mCurFrame = Frame();
        mLastFrame = Frame();
        mpReferenceKF = static_cast<KeyFrame *>(NULL);
        mpLastKeyFrame = static_cast<KeyFrame *>(NULL);
        mvIniMatches.clear();
        mbVelocity = false;
        if (mpViewer)
            mpViewer->CancelReseted();

        Verbose::PrintMess("   End reseting! ", Verbose::VERBOSITY_NORMAL);
    }

/**
 * @brief 显示用的
 */
    vector<MapPoint *> Tracking::GetLocalMapMPs() {
        return mvpLocalMapPoints;
    }


    void Tracking::InformOnlyTracking(const bool &flag) {
        mbOnlyTracking = flag;
    }

/**
 * @brief 更新了关键帧的位姿，但需要修改普通帧的位姿，因为正常跟踪需要普通帧
 * localmapping中初始化imu中使用，速度的走向（仅在imu模式使用），最开始速度定义于imu初始化时，每个关键帧都根据位移除以时间得到，经过非线性优化保存于KF中.
 * 之后使用本函数，让上一帧与当前帧分别与他们对应的上一关键帧做速度叠加得到，后面新的frame速度由上一个帧速度决定，如果使用匀速模型（大多数情况下），通过imu积分更新速度。
 * 新的关键帧继承于对应帧
 * @param  s 尺度
 * @param  b 初始化后第一帧的偏置
 * @param  pCurrentKeyFrame 当前关键帧
 */
    void Tracking::UpdateFrameIMU(const float s, const IMU::Bias &b, KeyFrame *pCurrentKeyFrame) {
        Map *pMap = pCurrentKeyFrame->GetMap();

        // 每一帧的参考关键帧
        list<ORB_SLAM3::KeyFrame *>::iterator lRit = mlpRefKFs.begin();
        list<bool>::iterator lbL = mlbLost.begin();  // 对应帧是否跟踪丢失

        // mlRelativeFramePoses 存放的是Tcr
        // 三个变量一一对应
        // 这部分操作貌似没用
        for (auto lit = mlRelativeFramePoses.begin(), lend = mlRelativeFramePoses.end();
             lit != lend; lit++, lRit++, lbL++) {
            if (*lbL)
                continue;
            KeyFrame *pKF = *lRit;
            while (pKF->isBad()) {
                pKF = pKF->GetParent();
            }
            if (pKF->GetMap() == pMap) {
                (*lit).translation() *= s;
            }
        }
        // 设置偏置
        mLastBias = b;
        // 设置上一关键帧，如果说mpLastKeyFrame已经是经过添加的新的kf，而pCurrentKeyFrame还是上一个kf，mpLastKeyFrame直接指向之前的kf
        mpLastKeyFrame = pCurrentKeyFrame;
        // 更新偏置
        mLastFrame.SetNewBias(mLastBias);
        mCurFrame.SetNewBias(mLastBias);

        while (!mCurFrame.imuIsPreintegrated()) {
            // 当前帧需要预积分完毕，这段函数实在localmapping里调用的
            usleep(5000);
        }

        // TODO 如果上一帧正好是上一帧的上一关键帧（mLastFrame.mpLastKeyFrame与mLastFrame不可能是一个，可以验证一下）
        if (mLastFrame.mnId == mLastFrame.mpPrevKeyFrame->mnFrameId) {
            mLastFrame.SetImuPoseVelocity(mLastFrame.mpPrevKeyFrame->GetImuRotation(),
                                          mLastFrame.mpPrevKeyFrame->GetImuPosition(),
                                          mLastFrame.mpPrevKeyFrame->GetVelocity());
        } else {
            const Eigen::Vector3f Gz(0, 0, -IMU::GRAVITY_VALUE);
            const Eigen::Vector3f twb1 = mLastFrame.mpPrevKeyFrame->GetImuPosition();
            const Eigen::Matrix3f Rwb1 = mLastFrame.mpPrevKeyFrame->GetImuRotation();
            const Eigen::Vector3f Vwb1 = mLastFrame.mpPrevKeyFrame->GetVelocity();
            float t12 = mLastFrame.mpImuFromPrevKF->mfTs;
            // 根据mLastFrame的上一个关键帧的信息（此时已经经过imu初始化了，所以关键帧的信息都是校正后的）以及imu的预积分重新计算上一帧的位姿
            mLastFrame.SetImuPoseVelocity(
                    IMU::NormalizeRotation(Rwb1 * mLastFrame.mpImuFromPrevKF->GetUpdatedDeltaRotation()),
                    twb1 + Vwb1 * t12 + 0.5f * t12 * t12 * Gz +
                    Rwb1 * mLastFrame.mpImuFromPrevKF->GetUpdatedDeltaPosition(),
                    Vwb1 + Gz * t12 + Rwb1 * mLastFrame.mpImuFromPrevKF->GetUpdatedDeltaVelocity());
        }

        // 当前帧是否做了预积分
        if (mCurFrame.mpImuFromPrevKF) {
            const Eigen::Vector3f Gz(0, 0, -IMU::GRAVITY_VALUE);

            const Eigen::Vector3f twb1 = mCurFrame.mpPrevKeyFrame->GetImuPosition();
            const Eigen::Matrix3f Rwb1 = mCurFrame.mpPrevKeyFrame->GetImuRotation();
            const Eigen::Vector3f Vwb1 = mCurFrame.mpPrevKeyFrame->GetVelocity();
            float t12 = mCurFrame.mpImuFromPrevKF->mfTs;

            mCurFrame.SetImuPoseVelocity(
                    IMU::NormalizeRotation(Rwb1 * mCurFrame.mpImuFromPrevKF->GetUpdatedDeltaRotation()),
                    twb1 + Vwb1 * t12 + 0.5f * t12 * t12 * Gz +
                    Rwb1 * mCurFrame.mpImuFromPrevKF->GetUpdatedDeltaPosition(),
                    Vwb1 + Gz * t12 + Rwb1 * mCurFrame.mpImuFromPrevKF->GetUpdatedDeltaVelocity());
        }
    }


    int Tracking::GetMatchNumInLM() {
        return mnLMInFMatchNum;
    }

    float Tracking::GetImageScale() {
        return mImageScale;
    }


} //namespace ORB_SLAM
