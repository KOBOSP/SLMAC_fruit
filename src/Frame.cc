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

#include "Frame.h"

#include "G2oTypes.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include "ORBextractor.h"
#include "Converter.h"
#include "ORBmatcher.h"
#include "GeometricCamera.h"

#include <thread>
#include <include/CameraModels/Pinhole.h>
#include <include/CameraModels/KannalaBrandt8.h>

namespace ORB_SLAM3 {

// 下一个生成的帧的ID,这里是初始化类的静态成员变量
    long unsigned int Frame::nNextId = 0;

// 是否要进行初始化操作的标志
// 这里给这个标志置位的操作是在最初系统开始加载到内存的时候进行的，下一帧就是整个系统的第一帧，所以这个标志要置位
    bool Frame::mbInitialComputations = true;
    float Frame::cx, Frame::cy, Frame::fx, Frame::fy, Frame::invfx, Frame::invfy;
    float Frame::mfMinX, Frame::mfMinY, Frame::mfMaxX, Frame::mfMaxY;
    float Frame::mfGridElementWidthInv, Frame::mfGridElementHeightInv;

//For stereo fisheye matching
    cv::BFMatcher Frame::BFmatcher = cv::BFMatcher(cv::NORM_HAMMING);

    Frame::Frame() : mpcpi(NULL), mpImuPreintegrated(NULL), mpPrevFrame(NULL), mpImuPreintegratedFrame(NULL),
                     mpReferenceKF(static_cast<KeyFrame *>(NULL)), mbIsSet(false), mbImuPreintegrated(false),
                     mbHasPose(false), mbHasVelocity(false) {

    }


//Copy Constructor
    Frame::Frame(const Frame &frame)
            : mpcpi(frame.mpcpi), mpORBvocabulary(frame.mpORBvocabulary), mpORBextractorLeft(frame.mpORBextractorLeft),
              mpORBextractorRight(frame.mpORBextractorRight),
              mdTimestamp(frame.mdTimestamp), mcvK(frame.mcvK.clone()), mEigenK(Converter::toMatrix3f(frame.mcvK)),
              mDistCoef(frame.mDistCoef.clone()),
              mfBaselineFocal(frame.mfBaselineFocal), mfBaseline(frame.mfBaseline), mfThDepth(frame.mfThDepth), mnKPsLeftNum(frame.mnKPsLeftNum), mvKPsLeft(frame.mvKPsLeft),
              mvKPsRight(frame.mvKPsRight), mvKPsUn(frame.mvKPsUn), mvfXInRight(frame.mvfXInRight),
              mvfMPDepth(frame.mvfMPDepth), mBowVec(frame.mBowVec), mFeatVec(frame.mFeatVec),
              mDescriptorsLeft(frame.mDescriptorsLeft.clone()), mDescriptorsRight(frame.mDescriptorsRight.clone()),
              mvpMPs(frame.mvpMPs), mvbOutlier(frame.mvbOutlier), mImuCalib(frame.mImuCalib),
              mnCloseMPs(frame.mnCloseMPs),
              mpImuPreintegrated(frame.mpImuPreintegrated), mpImuPreintegratedFrame(frame.mpImuPreintegratedFrame),
              mImuBias(frame.mImuBias),
              mnId(frame.mnId), mpReferenceKF(frame.mpReferenceKF), mnScaleLevels(frame.mnScaleLevels),
              mfScaleFactor(frame.mfScaleFactor), mfLogScaleFactor(frame.mfLogScaleFactor),
              mvfScaleFactors(frame.mvfScaleFactors), mvfInvScaleFactors(frame.mvfInvScaleFactors), mnDataset(frame.mnDataset),
              mvfLevelSigma2(frame.mvfLevelSigma2), mvfInvLevelSigma2(frame.mvfInvLevelSigma2),
              mpPrevFrame(frame.mpPrevFrame), mpLastKeyFrame(frame.mpLastKeyFrame),
              mbIsSet(frame.mbIsSet), mbImuPreintegrated(frame.mbImuPreintegrated), mpMutexImu(frame.mpMutexImu),
              mpCamera(frame.mpCamera), mpCamera2(frame.mpCamera2),
              monoLeft(frame.monoLeft), monoRight(frame.monoRight), mvLeftToRightMatch(frame.mvLeftToRightMatch),
              mvRightToLeftMatch(frame.mvRightToLeftMatch), mvStereo3Dpoints(frame.mvStereo3Dpoints),
              mTlr(frame.mTlr), mRlr(frame.mRlr), mtlr(frame.mtlr), mTrl(frame.mTrl),
              mTcw(frame.mTcw), mbHasPose(false), mbHasVelocity(false) {
        for (int i = 0; i < FRAME_GRID_COLS; i++)
            for (int j = 0; j < FRAME_GRID_ROWS; j++) {
                mGrid[i][j] = frame.mGrid[i][j];
            }

        if (frame.mbHasPose)
            SetPose(frame.GetPose());

        if (frame.HasVelocity()) {
            SetVelocity(frame.GetVelocity());
        }

        mmProjectPoints = frame.mmProjectPoints;
        mmMatchedInImage = frame.mmMatchedInImage;

    }

// 立体匹配模式下的双目
    Frame::Frame(const cv::Mat &ImgLeft, const cv::Mat &ImgRight, const double &dTimestamp, ORBextractor *ExtractorLeft,
                 ORBextractor *ExtractorRight, ORBVocabulary *Voc, cv::Mat &cvK, cv::Mat &DistCoef, const float &fBaselineFocal,
                 const float &fThDepth, GeometricCamera *pCamera, Frame *pPrevF, const IMU::Calib &ImuCalib)
            : mpcpi(NULL), mpORBvocabulary(Voc), mpORBextractorLeft(ExtractorLeft), mpORBextractorRight(ExtractorRight),
              mdTimestamp(dTimestamp), mcvK(cvK.clone()), mEigenK(Converter::toMatrix3f(cvK)), mDistCoef(DistCoef.clone()), mfBaselineFocal(fBaselineFocal),
              mfThDepth(fThDepth),
              mImuCalib(ImuCalib), mpImuPreintegrated(NULL), mpPrevFrame(pPrevF), mpImuPreintegratedFrame(NULL),
              mpReferenceKF(static_cast<KeyFrame *>(NULL)), mbIsSet(false), mbImuPreintegrated(false),
              mpCamera(pCamera), mpCamera2(nullptr), mbHasPose(false), mbHasVelocity(false) {
        // Frame ID
        // Step 1 帧的ID 自增
        mnId = nNextId++;

        // Scale Level Info
        // Step 2 计算图像金字塔的参数
        // 获取图像金字塔的层数
        mnScaleLevels = mpORBextractorLeft->GetLevels();
        // 获得层与层之间的缩放比
        mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
        // 计算上面缩放比的对数
        mfLogScaleFactor = log(mfScaleFactor);
        // 获取每层图像的缩放因子
        mvfScaleFactors = mpORBextractorLeft->GetScaleFactors();
        // 同样获取每层图像缩放因子的倒数
        mvfInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
        // 高斯模糊的时候，使用的方差
        mvfLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
        // 获取sigma^2的倒数
        mvfInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();


        // Step 3 对左目右目图像提取ORB特征点, 第一个参数0-左图， 1-右图。为加速计算，同时开了两个线程计算
        thread threadLeft(&Frame::ExtractORB, this, 0, ImgLeft, 0, 0);
        // 对右目图像提取orb特征
        thread threadRight(&Frame::ExtractORB, this, 1, ImgRight, 0, 0);
        // 等待两张图像特征点提取过程完成
        threadLeft.join();
        threadRight.join();


        // mvKeys中保存的是左图像中的特征点，这里是获取左侧图像中特征点的个数
        mnKPsLeftNum = mvKPsLeft.size();

        // 如果左图像中没有成功提取到特征点那么就返回，也意味这这一帧的图像无法使用
        if (mvKPsLeft.empty())
            return;

        // Step 4 用OpenCV的矫正函数、内参对提取到的特征点进行矫正
        UndistortKeyPoints();


        // Step 5 计算双目间特征点的匹配，只有匹配成功的特征点会计算其深度,深度存放在 mvfMPDepth
        // mvuRight中存储的应该是左图像中的点所匹配的在右图像中的点的横坐标（纵坐标相同）
        ComputeStereoMatches();


        // 初始化本帧的地图点
        mvpMPs = vector<MapPoint *>(mnKPsLeftNum, static_cast<MapPoint *>(NULL));
        mvbOutlier = vector<bool>(mnKPsLeftNum, false);
        mmProjectPoints.clear();
        mmMatchedInImage.clear();


        // This is done only for the first Frame (or after a change in the calibration)
        //  Step 5 计算去畸变后图像边界，将特征点分配到网格中。这个过程一般是在第一帧或者是相机标定参数发生变化之后进行
        if (mbInitialComputations) {
            // 计算去畸变后图像的边界
            ComputeImageBounds(ImgLeft);

            // 表示一个图像像素相当于多少个图像网格列（宽）
            mfGridElementWidthInv = static_cast<float>(FRAME_GRID_COLS) / (mfMaxX - mfMinX);
            // 表示一个图像像素相当于多少个图像网格行（高）
            mfGridElementHeightInv = static_cast<float>(FRAME_GRID_ROWS) / (mfMaxY - mfMinY);


            fx = cvK.at<float>(0, 0);
            fy = cvK.at<float>(1, 1);
            cx = cvK.at<float>(0, 2);
            cy = cvK.at<float>(1, 2);
            // 猜测是因为这种除法计算需要的时间略长，所以这里直接存储了这个中间计算结果
            invfx = 1.0f / fx;
            invfy = 1.0f / fy;

            // 特殊的初始化过程完成，标志复位
            mbInitialComputations = false;
        }

        // 双目相机基线长度
        mfBaseline = mfBaselineFocal / fx;

        if (pPrevF) {
            if (pPrevF->HasVelocity())
                SetVelocity(pPrevF->GetVelocity());
        } else {
            mVw.setZero();
        }

        mpMutexImu = new std::mutex();

        //Set no stereo fisheye information
        mvLeftToRightMatch = vector<int>(0);
        mvRightToLeftMatch = vector<int>(0);
        mvStereo3Dpoints = vector<Eigen::Vector3f>(0);
        monoLeft = -1;
        monoRight = -1;

        // Step 6 将特征点分配到图像网格中
        // 上个版本这句话放在了new 锁那个上面，放在目前这个位置更合理，因为要把一些当前模式不用的参数赋值，函数里面要用
        AssignFeaturesToGrid();
    }


/** 
 * @brief 特征分网格
 */
    void Frame::AssignFeaturesToGrid() {
        // Fill matrix with points
        // Step 1  给存储特征点的网格数组 Frame::mGrid 预分配空间
        const int nCells = FRAME_GRID_COLS * FRAME_GRID_ROWS;

        int nReserve = 0.5f * mnKPsLeftNum / (nCells);

        // 开始对mGrid这个二维数组中的每一个vector元素遍历并预分配空间
        for (unsigned int i = 0; i < FRAME_GRID_COLS; i++){
            for (unsigned int j = 0; j < FRAME_GRID_ROWS; j++) {
                mGrid[i][j].reserve(nReserve);
            }
        }
        // Step 2 遍历每个特征点，将每个特征点在mvKeysUn中的索引值放到对应的网格mGrid中
        for (int i = 0; i < mnKPsLeftNum; i++) {
            const cv::KeyPoint &kp = mvKPsUn[i];
            // 存储某个特征点所在网格的网格坐标，nGridPosX范围：[0,FRAME_GRID_COLS], nGridPosY范围：[0,FRAME_GRID_ROWS]
            int nGridPosX, nGridPosY;
            // 计算某个特征点所在网格的网格坐标，如果找到特征点所在的网格坐标，记录在nGridPosX,nGridPosY里，返回true，没找到返回false
            if (PosInGrid(kp, nGridPosX, nGridPosY)) {
                // 如果找到特征点所在网格坐标，将这个特征点的索引添加到对应网格的数组mGrid中
                mGrid[nGridPosX][nGridPosY].push_back(i);
            }
        }
    }

/** 
 * @brief 赋值新的偏置
 * @param flag 左右标志位
 * @param im 图片
 * @param x0 界限
 * @param x1 界限
 */
    void Frame::ExtractORB(int flag, const cv::Mat &im, const int x0, const int x1) {
        vector<int> vLapping = {x0, x1};
        // 判断是左图还是右图
        if (flag == 0)
            // 左图的话就套使用左图指定的特征点提取器，并将提取结果保存到对应的变量中
            monoLeft = (*mpORBextractorLeft)(im, cv::Mat(), mvKPsLeft, mDescriptorsLeft, vLapping);
        else
            // 右图的话就需要使用右图指定的特征点提取器，并将提取结果保存到对应的变量中
            monoRight = (*mpORBextractorRight)(im, cv::Mat(), mvKPsRight, mDescriptorsRight, vLapping);
    }

    bool Frame::isSet() const {
        return mbIsSet;
    }

/** 
 * @brief 赋值位姿
 * @param Tcw 位姿
 */
    void Frame::SetPose(const Sophus::SE3<float> &Tcw) {
        mTcw = Tcw;

        UpdatePoseMatrices();
        mbIsSet = true;
        mbHasPose = true;
    }

/** 
 * @brief 赋值新的偏置
 * @param b 偏置
 */
    void Frame::SetNewBias(const IMU::Bias &b) {
        mImuBias = b;
        if (mpImuPreintegrated)
            mpImuPreintegrated->SetNewBias(b);
    }

/** 
 * @brief 赋值新的速度
 * @param Vwb 速度
 */
    void Frame::SetVelocity(Eigen::Vector3f Vwb) {
        mVw = Vwb;
        mbHasVelocity = true;
    }

/** 
 * @brief 获取速度
 */
    Eigen::Vector3f Frame::GetVelocity() const {
        return mVw;
    }

/** 
 * @brief 赋值位姿与速度
 */
    void Frame::SetImuPoseVelocity(const Eigen::Matrix3f &Rwb, const Eigen::Vector3f &twb, const Eigen::Vector3f &Vwb) {
        mVw = Vwb;
        mbHasVelocity = true;

        Sophus::SE3f Twb(Rwb, twb);
        Sophus::SE3f Tbw = Twb.inverse();

        mTcw = mImuCalib.mTcb * Tbw;

        UpdatePoseMatrices();
        mbIsSet = true;
        mbHasPose = true;
    }

/** 
 * @brief 通过mTcw更新其他关于位姿的变量
 */
    void Frame::UpdatePoseMatrices() {
        Sophus::SE3<float> Twc = mTcw.inverse();
        mRwc = Twc.rotationMatrix();
        mOw = Twc.translation();
        mRcw = mTcw.rotationMatrix();
        mtcw = mTcw.translation();
    }

/** 
 * @brief 获得imu的平移
 */
    Eigen::Matrix<float, 3, 1> Frame::GetImuPosition() const {
        return mRwc * mImuCalib.mTcb.translation() + mOw;
    }

/** 
 * @brief 获得imu的旋转
 */
    Eigen::Matrix<float, 3, 3> Frame::GetImuRotation() {
        return mRwc * mImuCalib.mTcb.rotationMatrix();
    }

/** 
 * @brief 获得imu的位姿
 */
    Sophus::SE3<float> Frame::GetImuPose() {
        return mTcw.inverse() * mImuCalib.mTcb;
    }

/** 
 * @brief 获得左右目的相对位姿
 */
    Sophus::SE3f Frame::GetRelativePoseTrl() {
        return mTrl;
    }

/** 
 * @brief 获得左右目的相对位姿
 */
    Sophus::SE3f Frame::GetRelativePoseTlr() {
        return mTlr;
    }

/** 
 * @brief 获得左右目的相对旋转
 */
    Eigen::Matrix3f Frame::GetRelativePoseTlr_rotation() {
        return mTlr.rotationMatrix();
    }

/** 
 * @brief 获得左右目的相对平移
 */
    Eigen::Vector3f Frame::GetRelativePoseTlr_translation() {
        return mTlr.translation();
    }

/**
 * @brief 判断路标点是否在视野中
 * 步骤
 * Step 1 获得这个地图点的世界坐标
 * Step 2 关卡一：检查这个地图点在当前帧的相机坐标系下，是否有正的深度.如果是负的，表示出错，返回false
 * Step 3 关卡二：将MapPoint投影到当前帧的像素坐标(u,v), 并判断是否在图像有效范围内
 * Step 4 关卡三：计算MapPoint到相机中心的距离, 并判断是否在尺度变化的距离内
 * Step 5 关卡四：计算当前相机指向地图点向量和地图点的平均观测方向夹角的余弦值, 若小于设定阈值，返回false
 * Step 6 根据地图点到光心的距离来预测一个尺度（仿照特征点金字塔层级）
 * Step 7 记录计算得到的一些参数
 * @param[in] pMP                       当前地图点
 * @param[in] viewingCosLimit           夹角余弦，用于限制地图点和光心连线和法线的夹角
 * @return true                         地图点合格，且在视野内
 * @return false                        地图点不合格，抛弃
 */
    bool Frame::isInFrustum(MapPoint *pMP, float viewingCosLimit) {
        // 单目，立体匹配双目，rgbd
        // cout << "\na";
        // mbTrackInView是决定一个地图点是否进行重投影的标志
        // 这个标志的确定要经过多个函数的确定，isInFrustum()只是其中的一个验证关卡。这里默认设置为否
        pMP->mbTrackInLeftView = false;
        pMP->mTrackProjX = -1;
        pMP->mTrackProjY = -1;

        // 3D in absolute coordinates
        // Step 1 获得这个地图点的世界坐标
        Eigen::Matrix<float, 3, 1> P = pMP->GetWorldPos();

        // 3D in camera coordinates
        // 根据当前帧(粗糙)位姿转化到当前相机坐标系下的三维点Pc
        const Eigen::Matrix<float, 3, 1> Pc = mRcw * P + mtcw;
        const float Pc_dist = Pc.norm();

        // Check positive depth
        const float &PcZ = Pc(2);
        const float invz = 1.0f / PcZ;
        // Step 2 关卡一：检查这个地图点在当前帧的相机坐标系下，是否有正的深度.如果是负的，表示出错，直接返回false
        if (PcZ < 0.0f)
            return false;

        const Eigen::Vector2f uv = mpCamera->project(Pc);

        // Step 3 关卡二：将MapPoint投影到当前帧的像素坐标(u,v), 并判断是否在图像有效范围内
        // 判断是否在图像边界中，只要不在那么就说明无法在当前帧下进行重投影
        if (uv(0) < mfMinX || uv(0) > mfMaxX)
            return false;
        if (uv(1) < mfMinY || uv(1) > mfMaxY)
            return false;

        pMP->mTrackProjX = uv(0);
        pMP->mTrackProjY = uv(1);

        // Check distance is in the scale invariance region of the MapPoint
        // Step 4 关卡三：计算MapPoint到相机中心的距离, 并判断是否在尺度变化的距离内
        // 得到认为的可靠距离范围:[0.8f*mfMinDistance, 1.2f*mfMaxDistance]
        const float maxDistance = pMP->GetMaxDistanceInvariance();
        const float minDistance = pMP->GetMinDistanceInvariance();
        // 得到当前地图点距离当前帧相机光心的距离,注意P，mOw都是在同一坐标系下才可以
        //  mOw：当前相机光心在世界坐标系下坐标
        const Eigen::Vector3f PO = P - mOw;
        // 取模就得到了距离
        const float dist = PO.norm();

        // 如果不在允许的尺度变化范围内，认为重投影不可靠
        if (dist < minDistance || dist > maxDistance)
            return false;

        // Check viewing angle
        // Step 5 关卡四：
        // 计算当前相机指向地图点向量和地图点的平均观测方向夹角的余弦值,
        // 若小于cos(viewingCosLimit), 即夹角大于viewingCosLimit弧度则返回
        Eigen::Vector3f Pn = pMP->GetNormal();

        // 计算当前相机指向地图点向量和地图点的平均观测方向夹角的余弦值，注意平均观测方向为单位向量
        const float viewCos = PO.dot(Pn) / dist;

        // 如果大于给定的阈值 cos(60°)=0.5，认为这个点方向太偏了，重投影不可靠，返回false
        if (viewCos < viewingCosLimit)
            return false;

        // Predict scale in the image
        // Step 6 根据地图点到光心的距离来预测一个尺度（仿照特征点金字塔层级）
        const int nPredictedLevel = pMP->PredictScale(dist, this);

        // Step 7 记录计算得到的一些参数
        // Data used by the tracking
        // 通过置位标记 MapPoint::mbTrackInLeftView 来表示这个地图点要被投影
        pMP->mbTrackInLeftView = true;
        // 该地图点投影在当前图像（一般是左图）的像素横坐标
        pMP->mTrackProjX = uv(0);
        // bf/z其实是视差，相减得到右图（如有）中对应点的横坐标
        pMP->mTrackProjXR = uv(0) - mfBaselineFocal * invz;

        pMP->mTrackDepth = Pc_dist;

        // 该地图点投影在当前图像（一般是左图）的像素纵坐标
        pMP->mTrackProjY = uv(1);
        // 根据地图点到光心距离，预测的该地图点的尺度层级
        pMP->mnTrackScaleLevel = nPredictedLevel;
        // 保存当前视角和法线夹角的余弦值
        pMP->mTrackViewCos = viewCos;

        // 执行到这里说明这个地图点在相机的视野中并且进行重投影是可靠的，返回true
        return true;
    }

/** 
 * @brief 暂没用到
 */
    bool Frame::ProjectPointDistort(MapPoint *pMP, cv::Point2f &kp, float &u, float &v) {

        // 3D in absolute coordinates
        Eigen::Vector3f P = pMP->GetWorldPos();

        // 3D in camera coordinates
        const Eigen::Vector3f Pc = mRcw * P + mtcw;
        const float &PcX = Pc(0);
        const float &PcY = Pc(1);
        const float &PcZ = Pc(2);

        // Check positive depth
        if (PcZ < 0.0f) {
            cout << "Negative depth: " << PcZ << endl;
            return false;
        }

        // Project in image and check it is not outside
        const float invz = 1.0f / PcZ;
        u = fx * PcX * invz + cx;
        v = fy * PcY * invz + cy;

        if (u < mfMinX || u > mfMaxX)
            return false;
        if (v < mfMinY || v > mfMaxY)
            return false;

        float u_distort, v_distort;

        float x = (u - cx) * invfx;
        float y = (v - cy) * invfy;
        float r2 = x * x + y * y;
        float k1 = mDistCoef.at<float>(0);
        float k2 = mDistCoef.at<float>(1);
        float p1 = mDistCoef.at<float>(2);
        float p2 = mDistCoef.at<float>(3);
        float k3 = 0;
        if (mDistCoef.total() == 5) {
            k3 = mDistCoef.at<float>(4);
        }

        // Radial distorsion
        float x_distort = x * (1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2);
        float y_distort = y * (1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2);

        // Tangential distorsion
        x_distort = x_distort + (2 * p1 * x * y + p2 * (r2 + 2 * x * x));
        y_distort = y_distort + (p1 * (r2 + 2 * y * y) + 2 * p2 * x * y);

        u_distort = x_distort * fx + cx;
        v_distort = y_distort * fy + cy;


        u = u_distort;
        v = v_distort;

        kp = cv::Point2f(u, v);

        return true;
    }

/** 
 * @brief 暂没用到
 */
    Eigen::Vector3f Frame::inRefCoordinates(Eigen::Vector3f pCw) {
        return mRcw * pCw + mtcw;
    }

/** 
 * @brief 给定一个坐标，返回区域内所有特征点
 * @param x 给定点的x
 * @param y 给定点的y
 * @param r 搜索半径
 * @param minLevel 金字塔下边界
 * @param maxLevel 金字塔上边界
 * @param bRight 是否是右相机
 */
    vector<size_t> Frame::GetFeaturesInArea(
            const float &x, const float &y, const float &r,
            const int minLevel, const int maxLevel, const bool bRight) const {
        // 存储搜索结果的vector
        vector<size_t> vIndices;
        vIndices.reserve(mnKPsLeftNum);

        float factorX = r;
        float factorY = r;

        // Step 1 计算半径为r圆左右上下边界所在的网格列和行的id
        // 查找半径为r的圆左侧边界所在网格列坐标。这个地方有点绕，慢慢理解下：
        // (mfMaxX-mfMinX)/FRAME_GRID_COLS：表示列方向每个网格可以平均分得几个像素（肯定大于1）
        // mfGridElementWidthInv=FRAME_GRID_COLS/(mfMaxX-mfMinX) 是上面倒数，表示每个像素可以均分几个网格列（肯定小于1）
        // (x-mfMinX-r)，可以看做是从图像的左边界mnMinX到半径r的圆的左边界区域占的像素列数
        // 两者相乘，就是求出那个半径为r的圆的左侧边界在哪个网格列中
        // 保证nMinCellX 结果大于等于0
        const int nMinCellX = max(0, (int) floor((x - mfMinX - factorX) * mfGridElementWidthInv));
        // 如果最终求得的圆的左边界所在的网格列超过了设定了上限，那么就说明计算出错，找不到符合要求的特征点，返回空vector
        if (nMinCellX >= FRAME_GRID_COLS) {
            return vIndices;
        }

        // 计算圆所在的右边界网格列索引
        const int nMaxCellX = min((int) FRAME_GRID_COLS - 1,
                                  (int) ceil((x - mfMinX + factorX) * mfGridElementWidthInv));
        // 如果计算出的圆右边界所在的网格不合法，说明该特征点不好，直接返回空vector
        if (nMaxCellX < 0) {
            return vIndices;
        }

        // 后面的操作也都是类似的，计算出这个圆上下边界所在的网格行的id
        const int nMinCellY = max(0, (int) floor((y - mfMinY - factorY) * mfGridElementHeightInv));
        if (nMinCellY >= FRAME_GRID_ROWS) {
            return vIndices;
        }

        const int nMaxCellY = min((int) FRAME_GRID_ROWS - 1,
                                  (int) ceil((y - mfMinY + factorY) * mfGridElementHeightInv));
        if (nMaxCellY < 0) {
            return vIndices;
        }

        // 检查需要搜索的图像金字塔层数范围是否符合要求
        //? 疑似bug。(minLevel>0) 后面条件 (maxLevel>=0)肯定成立
        //? 改为 const bool bCheckLevels = (minLevel>=0) || (maxLevel>=0);
        const bool bCheckLevels = (minLevel > 0) || (maxLevel >= 0);

        // Step 2 遍历圆形区域内的所有网格，寻找满足条件的候选特征点，并将其index放到输出里
        for (int ix = nMinCellX; ix <= nMaxCellX; ix++) {
            for (int iy = nMinCellY; iy <= nMaxCellY; iy++) {
                // 获取这个网格内的所有特征点在 Frame::mvKPsUn 中的索引
                const vector<size_t> vCell = (!bRight) ? mGrid[ix][iy] : mGridRight[ix][iy];
                // 如果这个网格中没有特征点，那么跳过这个网格继续下一个
                if (vCell.empty())
                    continue;

                // 如果这个网格中有特征点，那么遍历这个图像网格中所有的特征点
                for (size_t j = 0, jend = vCell.size(); j < jend; j++) {
                    // 根据索引先读取这个特征点
                    const cv::KeyPoint &kpUn = mvKPsUn[vCell[j]];
                    if (bCheckLevels) {
                        // cv::KeyPoint::octave中表示的是从金字塔的哪一层提取的数据
                        // 保证特征点是在金字塔层级minLevel和maxLevel之间，不是的话跳过
                        if (kpUn.octave < minLevel)
                            continue;
                        if (maxLevel >= 0)
                            if (kpUn.octave > maxLevel)
                                continue;
                    }

                    // 通过检查，计算候选特征点到圆中心的距离，查看是否是在这个圆形区域之内
                    const float distx = kpUn.pt.x - x;
                    const float disty = kpUn.pt.y - y;

                    // 如果x方向和y方向的距离都在指定的半径之内，存储其index为候选特征点
                    if (fabs(distx) < factorX && fabs(disty) < factorY)
                        vIndices.push_back(vCell[j]);
                }
            }
        }

        return vIndices;
    }

/**
 * @brief 计算某个特征点所在网格的网格坐标，如果找到特征点所在的网格坐标，记录在nGridPosX,nGridPosY里，返回true，没找到返回false
 * 
 * @param[in] kp                    给定的特征点
 * @param[in & out] posX            特征点所在网格坐标的横坐标
 * @param[in & out] posY            特征点所在网格坐标的纵坐标
 * @return true                     如果找到特征点所在的网格坐标，返回true
 * @return false                    没找到返回false
 */
    bool Frame::PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY) {
        // 计算特征点x,y坐标落在哪个网格内，网格坐标为posX，posY
        // mfGridElementWidthInv=(FRAME_GRID_COLS)/(mfMaxX-mfMinX);
        // mfGridElementHeightInv=(FRAME_GRID_ROWS)/(mfMaxY-mfMinY);
        posX = round((kp.pt.x - mfMinX) * mfGridElementWidthInv);
        posY = round((kp.pt.y - mfMinY) * mfGridElementHeightInv);

        //Keypoint's coordinates are undistorted, which could cause to go out of the image
        // 因为特征点进行了去畸变，而且前面计算是round取整，所以有可能得到的点落在图像网格坐标外面
        // 如果网格坐标posX，posY超出了[0,FRAME_GRID_COLS] 和[0,FRAME_GRID_ROWS]，表示该特征点没有对应网格坐标，返回false
        if (posX < 0 || posX >= FRAME_GRID_COLS || posY < 0 || posY >= FRAME_GRID_ROWS)
            return false;

        // 计算成功返回true
        return true;
    }

/**
 * @brief 计算当前帧特征点对应的词袋Bow，主要是mBowVec 和 mFeatVec
 * 
 */
    void Frame::ComputeBoW() {
        // 判断是否以前已经计算过了，计算过了就跳过
        if (mBowVec.empty()) {
            // 将描述子mDescriptors转换为DBOW要求的输入格式
            vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(mDescriptorsLeft);
            // 将特征点的描述子转换成词袋向量mBowVec以及特征向量mFeatVec
            mpORBvocabulary->transform(vCurrentDesc,    //当前的描述子vector
                                       mBowVec,         //输出，词袋向量，记录的是单词的id及其对应权重TF-IDF值
                                       mFeatVec,        //输出，记录node id及其对应的图像 feature对应的索引
                                       4);              //4表示从叶节点向前数的层数
        }
    }

/**
 * @brief 用内参对特征点去畸变，结果报存在mvKeysUn中
 * 
 */
    void Frame::UndistortKeyPoints() {
        // Step 1 如果第一个畸变参数为0，不需要矫正。第一个畸变参数k1是最重要的，一般不为0，为0的话，说明畸变参数都是0
        // 变量mDistCoef中存储了opencv指定格式的去畸变参数，格式为：(k1,k2,p1,p2,k3)
        if (mDistCoef.at<float>(0) == 0.0) {
            mvKPsUn = mvKPsLeft;
            return;
        }

        // Fill matrix with points
        // Step 2 如果畸变参数不为0，用OpenCV函数进行畸变矫正
        // Fill matrix with points
        // N为提取的特征点数量，为满足OpenCV函数输入要求，将N个特征点保存在N*2的矩阵中
        cv::Mat mat(mnKPsLeftNum, 2, CV_32F);

        // 遍历每个特征点，并将它们的坐标保存到矩阵中
        for (int i = 0; i < mnKPsLeftNum; i++) {
            // 然后将这个特征点的横纵坐标分别保存
            mat.at<float>(i, 0) = mvKPsLeft[i].pt.x;
            mat.at<float>(i, 1) = mvKPsLeft[i].pt.y;
        }

        // Undistort points
        // 函数reshape(int cn,int rows=0) 其中cn为更改后的通道数，rows=0表示这个行将保持原来的参数不变
        // 为了能够直接调用opencv的函数来去畸变，需要先将矩阵调整为2通道（对应坐标x,y）
        // cv::undistortPoints最后一个矩阵为空矩阵时，得到的点为归一化坐标点
        mat = mat.reshape(2);
        cv::undistortPoints(mat, mat, static_cast<Pinhole *>(mpCamera)->toK(), mDistCoef, cv::Mat(), mcvK);
        // 调整回只有一个通道，回归我们正常的处理方式
        mat = mat.reshape(1);


        // Fill undistorted keypoint vector
        // Step 3 存储校正后的特征点
        mvKPsUn.resize(mnKPsLeftNum);
        for (int i = 0; i < mnKPsLeftNum; i++) {
            // 根据索引获取这个特征点
            // 注意之所以这样做而不是直接重新声明一个特征点对象的目的是，能够得到源特征点对象的其他属性
            cv::KeyPoint kp = mvKPsLeft[i];
            // 读取校正后的坐标并覆盖老坐标
            kp.pt.x = mat.at<float>(i, 0);
            kp.pt.y = mat.at<float>(i, 1);
            mvKPsUn[i] = kp;
        }
    }

/**
 * @brief 计算去畸变图像的边界
 * 
 * @param[in] imLeft            需要计算边界的图像
 */
    void Frame::ComputeImageBounds(const cv::Mat &imLeft) {
        // 如果畸变参数不为0，用OpenCV函数进行畸变矫正
        if (mDistCoef.at<float>(0) != 0.0) {
            // 保存矫正前的图像四个边界点坐标： (0,0) (cols,0) (0,rows) (cols,rows)
            cv::Mat mat(4, 2, CV_32F);
            mat.at<float>(0, 0) = 0.0;
            mat.at<float>(0, 1) = 0.0;
            mat.at<float>(1, 0) = imLeft.cols;
            mat.at<float>(1, 1) = 0.0;
            mat.at<float>(2, 0) = 0.0;
            mat.at<float>(2, 1) = imLeft.rows;
            mat.at<float>(3, 0) = imLeft.cols;
            mat.at<float>(3, 1) = imLeft.rows;

            // 和前面校正特征点一样的操作，将这几个边界点作为输入进行校正
            mat = mat.reshape(2);
            cv::undistortPoints(mat, mat, static_cast<Pinhole *>(mpCamera)->toK(), mDistCoef, cv::Mat(), mcvK);
            mat = mat.reshape(1);

            // Undistort corners
            // 校正后的四个边界点已经不能够围成一个严格的矩形，因此在这个四边形的外侧加边框作为坐标的边界
            mfMinX = min(mat.at<float>(0, 0), mat.at<float>(2, 0));  // 左上和左下横坐标最小的
            mfMaxX = max(mat.at<float>(1, 0), mat.at<float>(3, 0));  // 右上和右下横坐标最大的
            mfMinY = min(mat.at<float>(0, 1), mat.at<float>(1, 1));  // 左上和右上纵坐标最小的
            mfMaxY = max(mat.at<float>(2, 1), mat.at<float>(3, 1));  // 左下和右下纵坐标最小的
        } else {
            // 如果畸变参数为0，就直接获得图像边界
            mfMinX = 0.0f;
            mfMaxX = imLeft.cols;
            mfMinY = 0.0f;
            mfMaxY = imLeft.rows;
        }
    }

/*
 * 双目匹配函数
 *
 * 为左图的每一个特征点在右图中找到匹配点 \n
 * 根据基线(有冗余范围)上描述子距离找到匹配, 再进行SAD精确定位 \n ‘
 * 这里所说的SAD是一种双目立体视觉匹配算法，可参考[https://blog.csdn.net/u012507022/article/details/51446891]
 * 最后对所有SAD的值进行排序, 剔除SAD值较大的匹配对，然后利用抛物线拟合得到亚像素精度的匹配 \n 
 * 这里所谓的亚像素精度，就是使用这个拟合得到一个小于一个单位像素的修正量，这样可以取得更好的估计结果，计算出来的点的深度也就越准确
 * 匹配成功后会更新 mvfXInRight(ur) 和 mvfMPDepth(Z)
 */
    void Frame::ComputeStereoMatches() {
        /*两帧图像稀疏立体匹配（即：ORB特征点匹配，非逐像素的密集匹配，但依然满足行对齐）
         * 输入：两帧立体矫正后的图像img_left 和 img_right 对应的orb特征点集
         * 过程：
              1. 行特征点统计. 统计img_right每一行上的ORB特征点集，便于使用立体匹配思路(行搜索/极线搜索）进行同名点搜索, 避免逐像素的判断.
              2. 粗匹配. 根据步骤1的结果，对img_left第i行的orb特征点pi，在img_right的第i行上的orb特征点集中搜索相似orb特征点, 得到qi
              3. 精确匹配. 以点qi为中心，半径为r的范围内，进行块匹配（归一化SAD），进一步优化匹配结果
              4. 亚像素精度优化. 步骤3得到的视差为uchar/int类型精度，并不一定是真实视差，通过亚像素差值（抛物线插值)获取float精度的真实视差
              5. 最优视差值/深度选择. 通过胜者为王算法（WTA）获取最佳匹配点。
              6. 删除离缺点(outliers). 块匹配相似度阈值判断，归一化sad最小，并不代表就一定是正确匹配，比如光照变化、弱纹理等会造成误匹配
         * 输出：稀疏特征点视差图/深度图（亚像素精度）mvfMPDepth 匹配结果 mvfXInRight
         */

        // 为匹配结果预先分配内存，数据类型为float型
        // mvuRight存储右图匹配点索引
        // mvDepth存储特征点的深度信息
        mvfXInRight = vector<float>(mnKPsLeftNum, -1.0f);
        mvfMPDepth = vector<float>(mnKPsLeftNum, -1.0f);

        // orb特征相似度阈值  -> mean ～= (max  + min) / 2
        const int nThOrbDist = (ORBmatcher::TH_HIGH + ORBmatcher::TH_LOW) / 2;

        // 金字塔顶层（0层）图像高 nLeftRows
        const int nLeftRows = mpORBextractorLeft->mvImagePyramid[0].rows;

        // Assign keypoints to row table
        // 二维vector存储每一行的orb特征点的列坐标的索引，为什么是vector，因为每一行的特征点有可能不一样，例如
        // vvnRightKPsInEachLeftRows[0] = [1，2，5，8, 11]   第1行有5个特征点,他们的列号（即x坐标）分别是1,2,5,8,11
        // vvnRightKPsInEachLeftRows[1] = [2，6，7，9, 13, 17, 20]  第2行有7个特征点.etc
        vector<vector<size_t> > vvnRightKPsInEachLeftRows(nLeftRows, vector<size_t>());

        for (int i = 0; i < nLeftRows; i++)
            vvnRightKPsInEachLeftRows[i].reserve(200);

        // 右图特征点数量，N表示数量 r表示右图，且不能被修改
        const int nKPsRightNum = mvKPsRight.size();

        // Step 1. 行特征点统计. 考虑到尺度金字塔特征，一个特征点可能存在于多行，而非唯一的一行
        for (int iR = 0; iR < nKPsRightNum; iR++) {
            // 获取特征点ir的y坐标，即行号
            const cv::KeyPoint &kp = mvKPsRight[iR];
            const float &kpY = kp.pt.y;
            // 计算特征点ir在行方向上，可能的偏移范围r，即可能的行号为[kpY + r, kpY -r]
            // 2 表示在全尺寸(scale = 1)的情况下，假设有2个像素的偏移，随着尺度变化，r也跟着变化
            const float r = 2.0f * mvfScaleFactors[mvKPsRight[iR].octave];
            const int maxr = ceil(kpY + r);
            const int minr = floor(kpY - r);

            // 将特征点ir保证在可能的行号中
            for (int yi = minr; yi <= maxr; yi++)
                vvnRightKPsInEachLeftRows[yi].push_back(iR);
        }

        // Step 2 -> 3. 粗匹配 + 精匹配
        // 对于立体矫正后的两张图，在列方向(x)存在最大视差maxd和最小视差mind
        // 也即是左图中任何一点p，在右图上的匹配点的范围为应该是[p - maxd, p - mind], 而不需要遍历每一行所有的像素
        // maxd = baseline * length_focal / minZ
        // mind = baseline * length_focal / maxZ
        // Set limits for search
        const float minZ = mfBaseline;
        const float minD = 0;
        const float maxD = mfBaselineFocal / minZ;

        // For each left keypoint search a match in the right image
        // 保存sad块匹配相似度和左图特征点索引
        vector<pair<int, int> > vDistIdx;
        vDistIdx.reserve(mnKPsLeftNum);

        // 为左图每一个特征点il，在右图搜索最相似的特征点ir
        for (int iL = 0; iL < mnKPsLeftNum; iL++) {
            const cv::KeyPoint &kpL = mvKPsLeft[iL];
            const int &LL = kpL.octave;
            const float &YL = kpL.pt.y;
            const float &XL = kpL.pt.x;

            // 获取左图特征点il所在行，以及在右图对应行中可能的匹配点
            const vector<size_t> &vCandidates = vvnRightKPsInEachLeftRows[YL];

            if (vCandidates.empty())
                continue;

            // 计算理论上的最佳搜索范围
            const float minX = XL - maxD;
            const float maxX = XL - minD;

            // 最大搜索范围小于0，说明无匹配点
            if (maxX < 0)
                continue;

            // 初始化最佳相似度，用最大相似度，以及最佳匹配点索引
            int bestDist = ORBmatcher::TH_HIGH;
            size_t bestIdxR = 0;

            const cv::Mat &dL = mDescriptorsLeft.row(iL);

            // Compare descriptor to right keypoints
            // Step2. 粗配准. 左图特征点il与右图中的可能的匹配点进行逐个比较,得到最相似匹配点的相似度和索引
            for (size_t iC = 0; iC < vCandidates.size(); iC++) {
                const size_t iR = vCandidates[iC];
                const cv::KeyPoint &kpR = mvKPsRight[iR];

                // 左图特征点il与带匹配点ic的空间尺度差超过2，放弃
                if (kpR.octave < LL - 1 || kpR.octave > LL + 1)
                    continue;

                // 使用列坐标(x)进行匹配，和stereomatch一样
                const float &XR = kpR.pt.x;

                // 超出理论搜索范围[minX, maxX]，可能是误匹配，放弃
                if (XR >= minX && XR <= maxX) {
                    // 计算匹配点il和待匹配点ic的相似度dist
                    const cv::Mat &dR = mDescriptorsRight.row(iR);
                    const int dist = ORBmatcher::DescriptorDistance(dL, dR);

                    // 统计最小相似度及其对应的列坐标(x)
                    if (dist < bestDist) {
                        bestDist = dist;
                        bestIdxR = iR;
                    }
                }
            }

            // Subpixel match by correlation
            // 如果刚才匹配过程中的最佳描述子距离小于给定的阈值
            // Step 3. 精确匹配.
            if (bestDist < nThOrbDist) {
                // coordinates in image pyramid at keypoint scale
                // 计算右图特征点x坐标和对应的金字塔尺度
                const float XR0 = mvKPsRight[bestIdxR].pt.x;
                const float ScaleFactor = mvfInvScaleFactors[kpL.octave];
                // 尺度缩放后的左右图特征点坐标
                const float XLScaled = round(kpL.pt.x * ScaleFactor);
                const float YLScaled = round(kpL.pt.y * ScaleFactor);
                const float XR0Scaled = round(XR0 * ScaleFactor);

                // sliding window search
                // 滑动窗口搜索, 类似模版卷积或滤波
                // w表示sad相似度的窗口半径
                const int w = 5;
                // 提取左图中，以特征点(XLScaled,YLScaled)为中心, 半径为w的图像快patch
                cv::Mat PatchL = mpORBextractorLeft->mvImagePyramid[kpL.octave].rowRange(YLScaled - w,
                                                                                     YLScaled + w + 1).colRange(
                        XLScaled - w, XLScaled + w + 1);

                // 初始化最佳相似度
                int nBestDist = INT_MAX;
                // 通过滑动窗口搜索优化，得到的列坐标偏移量
                int nBestIncXR = 0;
                // 滑动窗口的滑动范围为（-L, L）
                const int L = 5;
                // 初始化存储图像块相似度
                vector<float> vfDists;
                vfDists.resize(2 * L + 1);

                // 计算滑动窗口滑动范围的边界，因为是块匹配，还要算上图像块的尺寸
                // 列方向起点 IniX = r0 + 最大窗口滑动范围 - 图像块尺寸
                // 列方向终点 eniu = r0 + 最大窗口滑动范围 + 图像块尺寸 + 1
                // 此次 + 1 和下面的提取图像块是列坐标+1是一样的，保证提取的图像块的宽是2 * w + 1
                const float IniX = XR0Scaled + L - w;
                const float EndX = XR0Scaled + L + w + 1;
                // 判断搜索是否越界
                if (IniX < 0 || EndX >= mpORBextractorRight->mvImagePyramid[kpL.octave].cols)
                    continue;

                // 在搜索范围内从左到右滑动，并计算图像块相似度
                for (int IncXR = -L; IncXR <= +L; IncXR++) {
                    // 提取左图中，以特征点(XLScaled,YLScaled)为中心, 半径为w的图像快patch
                    cv::Mat PatchRTmp = mpORBextractorRight->mvImagePyramid[kpL.octave].rowRange(YLScaled - w,
                                                                                          YLScaled + w + 1).colRange(
                            XR0Scaled + IncXR - w, XR0Scaled + IncXR + w + 1);

                    // sad 计算
                    float fDist = cv::norm(PatchL, PatchRTmp, cv::NORM_L1);
                    // 统计最小sad和偏移量
                    if (fDist < nBestDist) {
                        nBestDist = fDist;
                        nBestIncXR = IncXR;
                    }
                    // L+IncXR 为refine后的匹配点列坐标(x)
                    vfDists[L + IncXR] = fDist;
                }

                // 搜索窗口越界判断ß
                if (nBestIncXR == -L || nBestIncXR == L)
                    continue;

                // Step 4. 亚像素插值, 使用最佳匹配点及其左右相邻点构成抛物线
                // 使用3点拟合抛物线的方式，用极小值代替之前计算的最优是差值
                //    \                 / <- 由视差为14，15，16的相似度拟合的抛物线
                //      .             .(16)
                //         .14     .(15) <- int/uchar最佳视差值
                //              .
                //           （14.5）<- 真实的视差值
                //   deltaR = 15.5 - 16 = -0.5
                // 公式参考opencv sgbm源码中的亚像素插值公式
                // 或论文<<On Building an Accurate Stereo Matching System on Graphics Hardware>> 公式7
                // Sub-pixel match (Parabola fitting)
                const float dist1 = vfDists[L + nBestIncXR - 1];
                const float dist2 = vfDists[L + nBestIncXR];
                const float dist3 = vfDists[L + nBestIncXR + 1];

                const float deltaR = (dist1 - dist3) / (2.0f * (dist1 + dist3 - 2.0f * dist2));

                // 亚像素精度的修正量应该是在[-1,1]之间，否则就是误匹配
                if (deltaR < -1 || deltaR > 1)
                    continue;

                // Re-scaled coordinate
                // 根据亚像素精度偏移量delta调整最佳匹配索引
                float fBestXR = mvfScaleFactors[kpL.octave] * ((float) XR0Scaled + (float) nBestIncXR + deltaR);

                float fVisualDisparity = (XL - fBestXR);

                if (fVisualDisparity >= minD && fVisualDisparity < maxD) {
                    // 如果存在负视差，则约束为0.01
                    if (fVisualDisparity <= 0) {
                        fVisualDisparity = 0.01;
                        fBestXR = XL - 0.01;
                    }
                    // 根据视差值计算深度信息
                    // 保存最相似点的列坐标(x)信息
                    // 保存归一化sad最小相似度
                    // Step 5. 最优视差值/深度选择.
                    mvfMPDepth[iL] = mfBaselineFocal / fVisualDisparity;
                    mvfXInRight[iL] = fBestXR;
                    vDistIdx.push_back(pair<int, int>(nBestDist, iL));
                }
            }
        }

        // Step 6. 删除离缺点(outliers)
        // 块匹配相似度阈值判断，归一化sad最小，并不代表就一定是匹配的，比如光照变化、弱纹理、无纹理等同样会造成误匹配
        // 误匹配判断条件  norm_sad > 1.5 * 1.4 * median
        sort(vDistIdx.begin(), vDistIdx.end());
        const float median = vDistIdx[vDistIdx.size() / 2].first;
        const float thDist = 1.5f * 1.4f * median;

        for (int i = vDistIdx.size() - 1; i >= 0; i--) {
            if (vDistIdx[i].first < thDist)
                break;
            else {
                // 误匹配点置为-1，和初始化时保持一直，作为error code
                mvfXInRight[vDistIdx[i].second] = -1;
                mvfMPDepth[vDistIdx[i].second] = -1;
            }
        }
    }


/** 
 * @brief 当某个特征点的深度信息或者双目信息有效时,将它反投影到三维世界坐标系中
 */
    bool Frame::UnprojectStereo(const int &i, Eigen::Vector3f &x3D) {
        const float z = mvfMPDepth[i];
        if (z > 0) {
            const float u = mvKPsUn[i].pt.x;
            const float v = mvKPsUn[i].pt.y;
            const float x = (u - cx) * z * invfx;
            const float y = (v - cy) * z * invfy;
            Eigen::Vector3f x3Dc(x, y, z);
            x3D = mRwc * x3Dc + mOw;
            return true;
        } else
            return false;
    }

/** 
 * @brief 是否做完预积分
 */
    bool Frame::imuIsPreintegrated() {
        unique_lock<std::mutex> lock(*mpMutexImu);
        return mbImuPreintegrated;
    }

/** 
 * @brief 设置为做完预积分
 */
    void Frame::setIntegrated() {
        unique_lock<std::mutex> lock(*mpMutexImu);
        mbImuPreintegrated = true;
    }

/** 
 * @brief 两个相机模式下单相机的验证，判断路标点是否在视野中
 */
    bool Frame::isInFrustumChecks(MapPoint *pMP, float viewingCosLimit, bool bRight) {
        // 3D in absolute coordinates
        // Step 1 获得这个地图点的世界坐标
        Eigen::Vector3f P = pMP->GetWorldPos();

        Eigen::Matrix3f mR;
        Eigen::Vector3f mt, twc;
        mR = mRcw;
        mt = mtcw;
        twc = mOw;


        // 3D in camera coordinates
        // 根据当前帧(粗糙)位姿转化到当前相机坐标系下的三维点Pc
        Eigen::Vector3f Pc = mR * P + mt;
        const float Pc_dist = Pc.norm();
        const float &PcZ = Pc(2);

        // Check positive depth
        // Step 2 关卡一：检查这个地图点在当前帧的相机坐标系下，是否有正的深度.如果是负的，表示出错，直接返回false
        if (PcZ < 0.0f)
            return false;

        // Project in image and check it is not outside
        Eigen::Vector2f uv;
        uv = mpCamera->project(Pc);

        // Step 3 关卡二：将MapPoint投影到当前帧的像素坐标(u,v), 并判断是否在图像有效范围内
        // 判断是否在图像边界中，只要不在那么就说明无法在当前帧下进行重投影
        if (uv(0) < mfMinX || uv(0) > mfMaxX)
            return false;
        if (uv(1) < mfMinY || uv(1) > mfMaxY)
            return false;

        // Check distance is in the scale invariance region of the MapPoint
        // Step 4 关卡三：计算MapPoint到相机中心的距离, 并判断是否在尺度变化的距离内
        // 得到认为的可靠距离范围:[0.8f*mfMinDistance, 1.2f*mfMaxDistance]
        const float maxDistance = pMP->GetMaxDistanceInvariance();
        const float minDistance = pMP->GetMinDistanceInvariance();
        // 得到当前地图点距离当前帧相机光心的距离,注意P，mOw都是在同一坐标系下才可以
        // mOw：当前相机光心在世界坐标系下坐标
        const Eigen::Vector3f PO = P - twc;
        // 取模就得到了距离
        const float dist = PO.norm();

        // 如果不在允许的尺度变化范围内，认为重投影不可靠
        if (dist < minDistance || dist > maxDistance)
            return false;

        // Check viewing angle
        // Step 5 关卡四：计算当前相机指向地图点向量和地图点的平均观测方向夹角的余弦值, 若小于cos(viewingCosLimit), 即夹角大于viewingCosLimit弧度则返回
        Eigen::Vector3f Pn = pMP->GetNormal();

        // 计算当前相机指向地图点向量和地图点的平均观测方向夹角的余弦值，注意平均观测方向为单位向量
        const float viewCos = PO.dot(Pn) / dist;

        // 如果大于给定的阈值 cos(60°)=0.5，认为这个点方向太偏了，重投影不可靠，返回false
        if (viewCos < viewingCosLimit)
            return false;

        // Predict scale in the image
        // Step 6 根据地图点到光心的距离来预测一个尺度（仿照特征点金字塔层级）
        const int nPredictedLevel = pMP->PredictScale(dist, this);

        // Step 7 记录计算得到的一些参数
        pMP->mTrackProjX = uv(0);
        pMP->mTrackProjY = uv(1);
        pMP->mnTrackScaleLevel = nPredictedLevel;
        pMP->mTrackViewCos = viewCos;
        pMP->mTrackDepth = Pc_dist;

        return true;
    }

/** 
 * @brief 根据位姿将第i个点投到世界坐标系下
 */
    Eigen::Vector3f Frame::UnprojectStereoFishEye(const int &i) {
        return mRwc * mvStereo3Dpoints[i] + mOw;
    }

} //namespace ORB_SLAM
