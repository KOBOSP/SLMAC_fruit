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


#include "LocalMapping.h"
#include "LoopClosing.h"
#include "ORBmatcher.h"
#include "Optimizer.h"
#include "Converter.h"
#include "GeometricTools.h"

#include<mutex>
#include<chrono>

namespace ORB_SLAM3 {

/**
 * @brief 局部地图线程构造函数
 * @param pSys 系统类指针
 * @param pAtlas atlas
 * @param bMonocular 是否是单目 (bug)用float赋值了
 * @param bHaveIm 是否是惯性模式
 * @param sSeqName 序列名字，没用到
 */
    LocalMapping::LocalMapping(System *pSys, Atlas *pAtlas, const float bHaveMono, bool bHaveImu, Settings *settings)
            :
            mpSystem(pSys), mbHaveMono(bHaveMono), mbHaveImu(bHaveImu), mbResetRequested(false),
            mbResetRequestedActiveMap(false), mbRequestFinish(false), mbFinished(true), mpAtlas(pAtlas),
            bInitializing(false),
            mbAbortBA(false), mbPaused(false), mbRequestPause(false), mbNotPauseOrFinish(false),
            mbAcceptKeyFrames(true),
            mfScale(1.0), nImuInfo(Eigen::MatrixXd::Zero(9, 9)) {

        /*
         * mbRequestReset:    外部线程调用，为true，表示外部线程请求停止 local mapping
         * mbReseted:          为true表示可以并终止localmapping 线程
         * mbNotStopFinal:          true，表示不要停止 localmapping 线程，因为要插入关键帧了。需要和 mbReseted 结合使用
         * mbAcceptKeyFrames:  true，允许接受关键帧。tracking 和local mapping 之间的关键帧调度
         * mbAbortBA:          是否流产BA优化的标志位
         * mbRequestFinish:  请求终止当前线程的标志。注意只是请求，不一定终止。终止要看 mbFinished
         * mbRequestReset:   请求当前线程复位的标志。true，表示一直请求复位，但复位还未完成；表示复位完成为false
         * mbFinished:         判断最终LocalMapping::Run() 是否完成的标志。
         */
        mbBadImu = false;
        mTimeFirstToCur = 0.f;
        mfThFarPoints = settings->mfThFarPoints;
        mbFarPoints = false;
        if (mfThFarPoints > 0 && mfThFarPoints < 1000) {
            mbFarPoints = true;
        }
        mfCullKFRedundantTh = settings->mfCullKFRedundantTh;
        mnWeakCovisTh = settings->mnWeakCovisTh;
        mnStrongCovisTh = settings->mnStrongCovisTh;
        mnSingleMaxCullKFsNum = settings->mnSingleMaxCullKFsNum;
        cout << "Discard points further than " << mfThFarPoints << " m from current camera" << endl;
    }

/**
 * @brief 设置回环类指针
 * @param pLoopCloser 回环类指针
 */
    void LocalMapping::SetLoopCloser(LoopClosing *pLoopCloser) {
        mpLoopCloser = pLoopCloser;
    }

/**
 * @brief 设置跟踪类指针
 * @param pLoopCloser 回环类指针
 */
    void LocalMapping::SetTracker(Tracking *pTracker) {
        mpTracker = pTracker;
    }

/**
 * @brief 局部地图线程主函数
 */
    void LocalMapping::Run() {
        // 标记状态，表示当前run函数正在运行，尚未结束
        mbFinished = false;

        // 主循环
        while (1) {
            // Tracking will see that Local Mapping is busy
            // Step 1 告诉Tracking，LocalMapping正处于繁忙状态，请不要给我发送关键帧打扰我
            // LocalMapping线程处理的关键帧都是Tracking线程发过来的
            SetAcceptKeyFrames(false);
            mpCurrentMap = mpAtlas->GetCurrentMap();
            // Check if there are keyframes in the queue
            // 等待处理的关键帧列表不为空 并且imu正常
            if (HaveNewKeyFrames() && !mbBadImu) {
                // BoW conversion and insertion in Map
                // Step 2 处理列表中的关键帧，包括计算BoW、更新观测、描述子、共视图，插入到地图等
                ProcessNewKeyFrame();

                // Check recent MapPoints
                // Step 3 根据地图点的观测情况剔除质量不好的地图点
                MapPointCulling();

                // Triangulate new MapPoints
                // Step 4 当前关键帧与相邻关键帧通过三角化产生新的地图点，使得跟踪更稳
                CreateNewMapPoints();

                // 注意orbslam2中放在了函数SearchInNeighbors（用到了mbAbortBA）后面，应该放这里更合适
                mbAbortBA = false;

                // 已经处理完队列中的最后的一个关键帧
                if (!HaveNewKeyFrames()) {
                    // Find more matches in neighbor keyframes and fuse point duplications
                    //  Step 5 检查并融合当前关键帧与相邻关键帧帧（两级相邻）中重复的地图点
                    // 先完成相邻关键帧与当前关键帧的地图点的融合（在相邻关键帧中查找当前关键帧的地图点），
                    // 再完成当前关键帧与相邻关键帧的地图点的融合（在当前关键帧中查找当前相邻关键帧的地图点）
                    FuseMapPointsInNeighbors();
                }

                int num_FixedKF_BA = 0;
                int num_OptKF_BA = 0;
                int num_MPs_BA = 0;
                int num_edges_BA = 0;

                // 已经处理完队列中的最后的一个关键帧，并且闭环检测没有请求停止LocalMapping
                if (!HaveNewKeyFrames() && !CheckRequestPause()) {
                    if (mpCurrentMap->GetKeyFramesNumInMap() < 3) {
                        continue;
                    }
                    // Step 6.1 处于IMU模式并且当前关键帧所在的地图已经完成IMU初始化
                    if (mbHaveImu && mpCurKF->GetMap()->GetImuInitialized()) {
                        // 计算上一关键帧到当前关键帧相机光心的距离 + 上上关键帧到上一关键帧相机光心的距离
                        float dist = (mpCurKF->mPrevKF->GetCameraCenter() -
                                      mpCurKF->GetCameraCenter()).norm() +
                                     (mpCurKF->mPrevKF->mPrevKF->GetCameraCenter() -
                                      mpCurKF->mPrevKF->GetCameraCenter()).norm();
                        // 如果距离大于5厘米，记录当前KF和上一KF时间戳的差，累加到mTinit
                        if (dist > 0.05)
                            mTimeFirstToCur += mpCurKF->mdTimestamp - mpCurKF->mPrevKF->mdTimestamp;
                        // 当前关键帧所在的地图尚未完成IMU BA2（IMU第三阶段初始化）
                        if (!mpCurKF->GetMap()->GetImuIniertialBA2()) {
                            // 如果累计时间差小于10s 并且 距离小于2厘米，认为运动幅度太小，不足以初始化IMU，将mbBadImu设置为true
                            if ((mTimeFirstToCur < 10.f) && (dist < 0.02)) {
                                cout << "Not enough motion for initializing ImuBA2. Reseting..." << endl;
                                unique_lock<mutex> lock(mMutexReset);
                                mbResetRequestedActiveMap = true;
                                mbBadImu = true;  // 在跟踪线程里会重置当前活跃地图
                            }
                        }
                        bool bLarge = (mpTracker->GetMatchNumInLM() > 100);
                        // 局部地图+IMU一起优化，优化关键帧位姿、地图点、IMU参数
                        Optimizer::LocalBAWithImu(mpCurKF, &mbAbortBA, mpCurKF->GetMap(),
                                                  num_FixedKF_BA, num_OptKF_BA, num_MPs_BA, num_edges_BA, bLarge,
                                                  !mpCurKF->GetMap()->GetImuIniertialBA2());
                    } else {
                        Optimizer::LocalBAWithoutImu(mpCurKF, &mbAbortBA, mpCurKF->GetMap(),
                                                     num_FixedKF_BA, num_OptKF_BA, num_MPs_BA, num_edges_BA);
                    }
                    // Initialize IMU here
                    // Step 7 当前关键帧所在地图未完成IMU初始化（第一阶段）
                    if (!mpCurKF->GetMap()->GetImuInitialized() && mbHaveImu) {
                        // 在函数InitializeIMU里设置IMU成功初始化标志 SetImuInitialized
                        // IMU第一阶段初始化
                        InitializeImu(1e2, 1e5, true);
                    }


                    // Check redundant local Keyframes
                    // 跟踪中关键帧插入条件比较松，交给LocalMapping线程的关键帧会比较密，这里再删除冗余
                    // Step 8 检测并剔除当前帧相邻的关键帧中冗余的关键帧
                    // 冗余的判定：该关键帧的90%的地图点可以被其它关键帧观测到
                    KeyFrameCulling();
                    // Step 9 如果距离IMU第一阶段初始化成功累计时间差小于100s，进行VIBA
                    if ((mTimeFirstToCur < 50.0f) && mbHaveImu) {
                        // Step 9.1 根据条件判断是否进行VIBA1（IMU第二阶段初始化）
                        // 条件：1、当前关键帧所在的地图还未完成IMU初始化---并且--------2、正常跟踪状态----------
                        if (mpCurKF->GetMap()->GetImuInitialized() && mpTracker->mState == Tracking::OK) {
                            // 当前关键帧所在的地图还未完成VIBA 1
                            if (!mpCurKF->GetMap()->GetImuIniertialBA1()) {// 如果累计时间差大于5s，开始VIBA1（IMU第二阶段初始化）
                                if (mTimeFirstToCur > 5.0f) {
                                    cout << "Start VIBA 1" << endl;
                                    mpCurKF->GetMap()->SetImuIniertialBA1();
                                    InitializeImu(1.f, 1e5, true);
                                    cout << "End VIBA 1" << endl;
                                }
                            } else if (!mpCurKF->GetMap()->GetImuIniertialBA2()) {// Step 9.2 根据条件判断是否进行VIBA2（IMU第三阶段初始化）
                                if (mTimeFirstToCur > 15.0f) {
                                    cout << "Start VIBA 2" << endl;
                                    mpCurKF->GetMap()->SetImuIniertialBA2();
                                    InitializeImu(0.f, 0.f, true);
                                    cout << "End VIBA 2" << endl;
                                }
                            }
                        }
                    }
                }
                // Step 10 将当前帧加入到闭环检测队列中
                mpLoopCloser->InsertKeyFrame(mpCurKF);
            } else if (CheckRequestPause() && !mbBadImu) {
                while (CheckPaused() && !CheckFinishRequest()) {
                    // 如果还没有结束利索,那么等等它
                    usleep(5000);
                }
            }
            // 查看是否有复位线程的请求
            ResetIfRequested();
            // Tracking will see that Local Mapping is busy
            // 开始接收关键帧
            SetAcceptKeyFrames(true);
            if (CheckFinishRequest() && !CheckNotPauseOrFinish()) {
                break;
            }
            usleep(5000);
        }
        // 设置线程已经终止
        SetFinished();
    }

/**
 * @brief 插入关键帧,由外部（Tracking）线程调用;这里只是插入到列表中,等待线程主函数对其进行处理
 * @param pKF 新的关键帧
 */
    void LocalMapping::InsertKeyFrame(KeyFrame *pKF) {
        unique_lock<mutex> lock(mMutexNewKFs);
        // 将关键帧插入到列表中
        mlNewKeyFrames.emplace_back(pKF);
        mbAbortBA = true;
    }

/**
 * @brief 查看列表中是否有等待被插入的关键帧
 */
    bool LocalMapping::HaveNewKeyFrames() {
        unique_lock<mutex> lock(mMutexNewKFs);
        return (!mlNewKeyFrames.empty());
    }

/**
 * @brief 处理列表中的关键帧，包括计算BoW、更新观测、描述子、共视图，插入到地图等
 */
    void LocalMapping::ProcessNewKeyFrame() {
        // Step 1：从缓冲队列中取出一帧关键帧
        // 该关键帧队列是Tracking线程向LocalMapping中插入的关键帧组成
        {
            unique_lock<mutex> lock(mMutexNewKFs);
            // 取出列表中最前面的关键帧，作为当前要处理的关键帧
            mpCurKF = mlNewKeyFrames.front();
            // 取出最前面的关键帧后，在原来的列表里删掉该关键帧
            mlNewKeyFrames.pop_front();
        }

        // Compute Bags of Words structures
        // Step 2：计算该关键帧特征点的Bow信息
        mpCurKF->ComputeBoW();

        // Associate MapPoints to the new keyframe and update normal and descriptor
        // Step 3：当前处理关键帧中有效的地图点，更新normal，描述子等信息
        // TrackLocalMap中和当前帧新匹配上的地图点和当前关键帧进行关联绑定
        const vector<MapPoint *> vpMapPointsInKF = mpCurKF->GetVectorMapPointsInKF();
        // 对当前处理的这个关键帧中的所有的地图点展开遍历
        for (size_t i = 0; i < vpMapPointsInKF.size(); i++) {
            MapPoint *pMP = vpMapPointsInKF[i];
            if (!pMP) {
                continue;
            }
            if (pMP->isBad()) {
                continue;
            }
            if (!pMP->IsInKeyFrame(mpCurKF)) {
                // 如果地图点不是来自当前帧的观测，为当前地图点添加观测
                pMP->AddObsKFAndLRIdx(mpCurKF, i);
                // 获得该点的平均观测方向和观测距离范围
                pMP->UpdateNormalAndDepth();
                // 更新地图点的最佳描述子
                pMP->ComputeDistinctiveDescriptors();
            } else // this can only happen for new stereo points inserted by the Tracking
            {
                // 如果当前帧中已经包含了这个地图点,但是这个地图点中却没有包含这个关键帧的信息
                // 这些地图点可能来自双目或RGBD跟踪过程中新生成的地图点，或者是CreateNewMapPoints 中通过三角化产生
                // 将上述地图点放入mlpRecentAddedMapPoints，等待后续MapPointCulling函数的检验
                mlpRecentAddedMapPoints.emplace_back(pMP);
            }
        }

        // Update6DoF links in the Covisibility Graph
        // Step 4：更新关键帧间的连接关系（共视图）
        mpCurKF->UpdateCovisGraph();

        // Insert Keyframe in Map
        // Step 5：将该关键帧插入到地图中
        mpAtlas->AddKeyFrame(mpCurKF);
    }

/**
 * @brief 处理新的关键帧，使队列为空，注意这里只是处理了关键帧，并没有生成MP
 */
    void LocalMapping::EmptyQueue() {
        while (HaveNewKeyFrames())
            ProcessNewKeyFrame();
    }

/**
 * @brief 检查新增地图点，根据地图点的观测情况剔除质量不好的新增的地图点
 * mlpRecentAddedMapPoints: 存储新增的地图点，这里是要删除其中不靠谱的
 */
    void LocalMapping::MapPointCulling() {
        // Check Recent Added MapPoints
        list<MapPoint *>::iterator lit = mlpRecentAddedMapPoints.begin();
        const unsigned long int nCurrentKFid = mpCurKF->mnId;
        const int cnThObs = 3;
        while (lit != mlpRecentAddedMapPoints.end()) {
            MapPoint *pMP = *lit;
            if (pMP->isBad()) {
                lit = mlpRecentAddedMapPoints.erase(lit);
            } else if (pMP->GetFoundRatio() < 0.25f) {
                // Step 2.2：跟踪到该MapPoint的Frame数相比预计可观测到该MapPoint的Frame数的比例小于25%，删除
                // (mnFound/mnVisible） < 25%
                // mnFound ：地图点被多少帧（包括普通帧）看到，次数越多越好
                // mnVisible：地图点应该被看到的次数
                // (mnFound/mnVisible）：对于大FOV镜头这个比例会高，对于窄FOV镜头这个比例会低
                pMP->SetBadFlag();
                lit = mlpRecentAddedMapPoints.erase(lit);
            } else if (((int) nCurrentKFid - (int) pMP->mnFirstKFid) >= 2 && pMP->GetObsTimes() <= cnThObs) {
                // Step 2.3：从该点建立开始，到现在已经过了不小于2个关键帧
                // 但是观测到该点的关键帧数却不超过cnThObs帧，那么删除该点
                pMP->SetBadFlag();
                lit = mlpRecentAddedMapPoints.erase(lit);
            } else if (((int) nCurrentKFid - (int) pMP->mnFirstKFid) >= 3) {
                lit = mlpRecentAddedMapPoints.erase(lit);
            } else {
                lit++;
            }
        }
    }

/**
 * @brief 用当前关键帧与相邻关键帧通过三角化产生新的地图点，使得跟踪更稳
 */
    void LocalMapping::CreateNewMapPoints() {
        // Retrieve neighbor keyframes in covisibility graph
        // nn表示搜索最佳共视关键帧的数目
        // 不同传感器下要求不一样,单目的时候需要有更多的具有较好共视关系的关键帧来建立地图
        int nn = 10;
        // Step 1：在当前关键帧的共视关键帧中找到共视程度最高的nn帧相邻关键帧
        vector<KeyFrame *> vpNeighKFs = mpCurKF->GetBestCovisibilityKeyFrames(nn);

        // imu模式下在附近添加更多的帧进来
        if (mbHaveImu) {
            KeyFrame *pKF = mpCurKF;
            int count = 0;
            // 在总数不够且上一关键帧存在，且添加的帧没有超过总数时
            while ((vpNeighKFs.size() <= nn) && (pKF->mPrevKF) && (count++ < nn)) {
                vector<KeyFrame *>::iterator it = std::find(vpNeighKFs.begin(), vpNeighKFs.end(), pKF->mPrevKF);
                if (it == vpNeighKFs.end())
                    vpNeighKFs.emplace_back(pKF->mPrevKF);
                pKF = pKF->mPrevKF;
            }
        }

        float th = 0.6f;
        // 特征点匹配配置 最小距离 < 0.6*次小距离，比较苛刻了。不检查旋转
        ORBmatcher matcher(th, false);

        // 取出当前帧从世界坐标系到相机坐标系的变换矩阵
        Sophus::SE3<float> sophTcw1 = mpCurKF->GetPose();
        Eigen::Matrix<float, 3, 4> eigTcw1 = sophTcw1.matrix3x4();
        Eigen::Matrix<float, 3, 3> Rcw1 = eigTcw1.block<3, 3>(0, 0);
        Eigen::Matrix<float, 3, 3> Rwc1 = Rcw1.transpose();
        Eigen::Vector3f tcw1 = sophTcw1.translation();
        // 得到当前关键帧（左目）光心在世界坐标系中的坐标、内参
        Eigen::Vector3f Ow1 = mpCurKF->GetCameraCenter();

        const float &fx1 = mpCurKF->fx;
        const float &fy1 = mpCurKF->fy;
        const float &cx1 = mpCurKF->cx;
        const float &cy1 = mpCurKF->cy;
        // 用于后面的点深度的验证;这里的1.5是经验值
        const float ratioFactor = 1.5f * mpCurKF->mfScaleFactor;
        // Search matches with epipolar restriction and triangulate
        for (size_t i = 0; i < vpNeighKFs.size(); i++) {
            // 下面的过程会比较耗费时间,因此如果有新的关键帧需要处理的话,就先去处理新的关键帧吧
            if (i > 0 && HaveNewKeyFrames())
                return;
            KeyFrame *pKF2 = vpNeighKFs[i];
            GeometricCamera *pCamera1 = mpCurKF->mpCamera, *pCamera2 = pKF2->mpCamera;

            // Check first that baseline is not too short
            // 邻接的关键帧光心在世界坐标系中的坐标
            Eigen::Vector3f Ow2 = pKF2->GetCameraCenter();
            // 基线向量，两个关键帧间的相机位移
            Eigen::Vector3f vBaseline = Ow2 - Ow1;
            // 基线长度
            const float baseline = vBaseline.norm();

            // Step 3：判断相机运动的基线是不是足够长
            // 如果是双目相机，关键帧间距小于本身的基线时不生成3D点. 因为太短的基线下能够恢复的地图点不稳定
            if (baseline < pKF2->mfBaseline)
                continue;

            // Search matches that fullfil epipolar constraint
            // Step 4：通过BoW对两关键帧的未匹配的特征点快速匹配，用极线约束抑制离群点，生成新的匹配点对
            vector<pair<size_t, size_t>> vMatchIdxPair;
            // 当惯性模式下，并且经过三次初始化，且为刚丢失状态
            bool bCoarse = mbHaveImu && mpTracker->mState == Tracking::RECENTLY_LOST &&
                           mpCurKF->GetMap()->GetImuIniertialBA2();

            // 通过极线约束的方式找到匹配点（且该点还没有成为MP，注意非单目已经生成的MP这里做匹配，所以最后会覆盖掉特征点对应的MP）
            matcher.SearchMatchKFAndKFByTriangulation(mpCurKF, pKF2, vMatchIdxPair, false, bCoarse);

            // 取出与mpCurrentKeyFrame共视关键帧的内外参信息
            Sophus::SE3<float> sophTcw2 = pKF2->GetPose();
            Eigen::Matrix<float, 3, 4> eigTcw2 = sophTcw2.matrix3x4();
            Eigen::Matrix<float, 3, 3> Rcw2 = eigTcw2.block<3, 3>(0, 0);
            Eigen::Matrix<float, 3, 3> Rwc2 = Rcw2.transpose();
            Eigen::Vector3f tcw2 = sophTcw2.translation();

            const float &fx2 = pKF2->fx;
            const float &fy2 = pKF2->fy;
            const float &cx2 = pKF2->cx;
            const float &cy2 = pKF2->cy;

            // Triangulate each match
            // Step 5：对每对匹配通过三角化生成3D点,和 Triangulate函数差不多
            const int nMatches = vMatchIdxPair.size();
            for (int ikp = 0; ikp < nMatches; ikp++) {
                const int &idx1 = vMatchIdxPair[ikp].first;
                const int &idx2 = vMatchIdxPair[ikp].second;
                float kp1_ur = mpCurKF->mvfXInRight[idx1];
                float kp2_ur = pKF2->mvfXInRight[idx2];
                const cv::KeyPoint &kp1 = mpCurKF->mvKPsUn[idx1];
                const cv::KeyPoint &kp2 = pKF2->mvKPsUn[idx2];
                bool bHaveStereoMatch1 = (kp1_ur >= 0);
                bool bHaveStereoMatch2 = (kp2_ur >= 0);


                // Check parallax between rays
                // Step 5.4：利用匹配点反投影得到视差角
                // 特征点反投影,其实得到的是在各自相机坐标系下的一个非归一化的方向向量,和这个点的反投影射线重合
                Eigen::Vector3f xn1 = pCamera1->UnprojectEig(kp1.pt);
                Eigen::Vector3f xn2 = pCamera2->UnprojectEig(kp2.pt);
                // 由相机坐标系转到世界坐标系(得到的是那条反投影射线的一个同向向量在世界坐标系下的表示,还是只能够表示方向)，得到视差角余弦值
                Eigen::Vector3f ray1 = Rwc1 * xn1;
                Eigen::Vector3f ray2 = Rwc2 * xn2;
                // 这个就是求向量之间角度公式
                const float cosParallaxRays = ray1.dot(ray2) / (ray1.norm() * ray2.norm());

                // 加1是为了让cosParallaxStereo随便初始化为一个很大的值
                float cosParallaxStereo = cosParallaxRays + 1;
                float cosParallaxStereo1 = cosParallaxStereo;
                float cosParallaxStereo2 = cosParallaxStereo;

                // Step 5.5：对于双目，利用双目得到视差角；单目相机没有特殊操作
                // 传感器是双目相机,并且当前的关键帧的这个点有对应的深度
                // 假设是平行的双目相机，计算出两个相机观察这个点的时候的视差角;
                // ? 感觉直接使用向量夹角的方式计算会准确一些啊（双目的时候），那么为什么不直接使用那个呢？
                // 回答：因为双目深度值、基线是更可靠的，比特征匹配再三角化出来的稳
                if (bHaveStereoMatch1) {
                    cosParallaxStereo1 = cos(2 * atan2(mpCurKF->mfBaseline / 2, mpCurKF->mvfMPDepth[idx1]));
                }
                if (bHaveStereoMatch2) {
                    cosParallaxStereo2 = cos(2 * atan2(pKF2->mfBaseline / 2, pKF2->mvfMPDepth[idx2]));
                }
                // 得到双目观测的视差角
                cosParallaxStereo = min(cosParallaxStereo1, cosParallaxStereo2);
                Eigen::Vector3f x3D;
                bool goodProj = false;
                // cosParallaxRays>0 && (bHaveStereoMatch1 || bHaveStereoMatch2 || cosParallaxRays<0.9998)表明视线角正常
                // cosParallaxRays<cosParallaxStereo表明前后帧视线角比双目视线角大，所以用前后帧三角化而来，反之使用双目的，如果没有双目则跳过
                // 视差角度小时用三角法恢复3D点，视差角大时（离相机近）用双目恢复3D点（双目以及深度有效）
                if (cosParallaxRays < cosParallaxStereo && cosParallaxRays > 0 &&
                    (bHaveStereoMatch1 || bHaveStereoMatch2 ||
                     (cosParallaxRays < 0.9996 &&
                      mbHaveImu) ||
                     (cosParallaxRays < 0.9998 &&
                      !mbHaveImu))) {
                    goodProj = GeometricTools::Triangulate(xn1, xn2, eigTcw1, eigTcw2, x3D);
                } else if (bHaveStereoMatch1 && cosParallaxStereo1 < cosParallaxStereo2) {
                    goodProj = mpCurKF->UnprojectStereo(idx1, x3D);
                } else if (bHaveStereoMatch2 && cosParallaxStereo2 < cosParallaxStereo1) {
                    goodProj = pKF2->UnprojectStereo(idx2, x3D);
                } else {
                    continue; //No stereo and very low parallax
                }

                if (!goodProj)
                    continue;

                //Check triangulation in front of cameras
                float z1 = Rcw1.row(2).dot(x3D) + tcw1(2);
                if (z1 <= 0)
                    continue;
                float z2 = Rcw2.row(2).dot(x3D) + tcw2(2);
                if (z2 <= 0)
                    continue;

                //Check reprojection error in first keyframe
                const float &sigmaSquare1 = mpCurKF->mvfLevelSigma2[kp1.octave];
                const float x1 = Rcw1.row(0).dot(x3D) + tcw1(0);
                const float y1 = Rcw1.row(1).dot(x3D) + tcw1(1);
                const float invz1 = 1.0 / z1;
                if (!bHaveStereoMatch1) {
                    cv::Point2f uv1 = pCamera1->ProjectMPToKP(cv::Point3f(x1, y1, z1));
                    float errX1 = uv1.x - kp1.pt.x;
                    float errY1 = uv1.y - kp1.pt.y;
                    // 假设测量有一个像素的偏差，2自由度卡方检验阈值是5.991
                    if ((errX1 * errX1 + errY1 * errY1) > 5.991 * sigmaSquare1)
                        continue;
                } else {
                    float u1 = fx1 * x1 * invz1 + cx1;
                    // 根据视差公式计算假想的右目坐标
                    float u1_r = u1 - mpCurKF->mfBaselineFocal * invz1;
                    float v1 = fy1 * y1 * invz1 + cy1;
                    float errX1 = u1 - kp1.pt.x;
                    float errY1 = v1 - kp1.pt.y;
                    float errX1_r = u1_r - kp1_ur;
                    // 自由度为3，卡方检验阈值是7.8
                    if ((errX1 * errX1 + errY1 * errY1 + errX1_r * errX1_r) > 7.8 * sigmaSquare1)
                        continue;
                }

                const float sigmaSquare2 = pKF2->mvfLevelSigma2[kp2.octave];
                const float x2 = Rcw2.row(0).dot(x3D) + tcw2(0);
                const float y2 = Rcw2.row(1).dot(x3D) + tcw2(1);
                const float invz2 = 1.0 / z2;
                if (!bHaveStereoMatch2) {
                    cv::Point2f uv2 = pCamera2->ProjectMPToKP(cv::Point3f(x2, y2, z2));
                    float errX2 = uv2.x - kp2.pt.x;
                    float errY2 = uv2.y - kp2.pt.y;
                    if ((errX2 * errX2 + errY2 * errY2) > 5.991 * sigmaSquare2)
                        continue;
                } else {
                    float u2 = fx2 * x2 * invz2 + cx2;
                    float u2_r = u2 - mpCurKF->mfBaselineFocal * invz2;
                    float v2 = fy2 * y2 * invz2 + cy2;
                    float errX2 = u2 - kp2.pt.x;
                    float errY2 = v2 - kp2.pt.y;
                    float errX2_r = u2_r - kp2_ur;
                    if ((errX2 * errX2 + errY2 * errY2 + errX2_r * errX2_r) > 7.8 * sigmaSquare2)
                        continue;
                }

                Eigen::Vector3f normal1 = x3D - Ow1;
                float dist1 = normal1.norm();
                Eigen::Vector3f normal2 = x3D - Ow2;
                float dist2 = normal2.norm();
                if (dist1 <= 0 || dist2 <= 0)
                    continue;
                if (mbFarPoints && (dist1 >= mfThFarPoints || dist2 >= mfThFarPoints)) // MODIFICATION
                    continue;
                // ratioDist是不考虑金字塔尺度下的距离比例
                const float ratioDist = dist2 / dist1;
                const float ratioOctave =
                        mpCurKF->mvScaleFactors[kp1.octave] / pKF2->mvScaleFactors[kp2.octave];
                // 距离的比例和图像金字塔的比例不应该差太多，否则就跳过
                if (ratioDist * ratioFactor < ratioOctave || ratioDist > ratioOctave * ratioFactor)
                    continue;


                MapPoint *pMP = new MapPoint(x3D, mpCurKF, mpCurrentMap);
                pMP->AddObsKFAndLRIdx(mpCurKF, idx1);
                pMP->AddObsKFAndLRIdx(pKF2, idx2);
                mpCurKF->AddMapPoint(pMP, idx1);
                pKF2->AddMapPoint(pMP, idx2);
                pMP->ComputeDistinctiveDescriptors();
                pMP->UpdateNormalAndDepth();
                mpAtlas->AddMapPoint(pMP);
                mlpRecentAddedMapPoints.emplace_back(pMP);
            }
        }
    }

/**
 * @brief 检查并融合当前关键帧与相邻帧（两级相邻）重复的MapPoints
 */
    void LocalMapping::FuseMapPointsInNeighbors() {
        // Retrieve neighbor keyframes
        // Step 1：获得当前关键帧在共视图中权重排名前nn的邻接关键帧
        // 开始之前先定义几个概念
        // 当前关键帧的邻接关键帧，称为一级相邻关键帧，也就是邻居
        // 与一级相邻关键帧相邻的关键帧，称为二级相邻关键帧，也就是邻居的邻居

        int nn = 10;

        // 和当前关键帧相邻的关键帧，也就是一级相邻关键帧
        const vector<KeyFrame *> vpFirstNeighKFs = mpCurKF->GetBestCovisibilityKeyFrames(nn);
        // Step 2：存储一级相邻关键帧及其二级相邻关键帧
        vector<KeyFrame *> vpProjectKFs;
        // 开始对所有候选的一级关键帧展开遍历：
        for (vector<KeyFrame *>::const_iterator vit = vpFirstNeighKFs.begin(), vend = vpFirstNeighKFs.end();
             vit != vend; vit++) {
            KeyFrame *pKFi = *vit;
            // 没有和当前帧进行过融合的操作
            if (pKFi->isBad() || pKFi->mnFuseFlagInLocalMapping == mpCurKF->mnId)
                continue;
            // 加入一级相邻关键帧
            vpProjectKFs.emplace_back(pKFi);
            // 标记已经加入
            pKFi->mnFuseFlagInLocalMapping = mpCurKF->mnId;
        }

        // Add some covisible of covisible
        // Extend to some second neighbors if abort is not requested
        // 以一级相邻关键帧的共视关系最好的5个相邻关键帧 作为二级相邻关键帧
        for (int i = 0, imax = vpProjectKFs.size(); i < imax; i++) {
            const vector<KeyFrame *> vpSecondNeighKFs = vpProjectKFs[i]->GetBestCovisibilityKeyFrames(nn);
            // 遍历得到的二级相邻关键帧
            for (vector<KeyFrame *>::const_iterator vit2 = vpSecondNeighKFs.begin(), vend2 = vpSecondNeighKFs.end();
                 vit2 != vend2; vit2++) {
                KeyFrame *pKFi2 = *vit2;
                if (pKFi2->isBad() || pKFi2->mnFuseFlagInLocalMapping == mpCurKF->mnId ||
                    pKFi2->mnId == mpCurKF->mnId)
                    continue;
                // 存入二级相邻关键帧
                vpProjectKFs.emplace_back(pKFi2);
                pKFi2->mnFuseFlagInLocalMapping = mpCurKF->mnId;
            }
            if (mbAbortBA)
                break;
        }

        // Extend to temporal neighbors
        // imu模式下加了一些prevKF
        if (mbHaveImu) {
            KeyFrame *pKFi = mpCurKF->mPrevKF;
            while (vpProjectKFs.size() < 2 * nn && pKFi) {
                if (pKFi->isBad() || pKFi->mnFuseFlagInLocalMapping == mpCurKF->mnId) {
                    pKFi = pKFi->mPrevKF;
                    continue;
                }
                vpProjectKFs.emplace_back(pKFi);
                pKFi->mnFuseFlagInLocalMapping = mpCurKF->mnId;
                pKFi = pKFi->mPrevKF;
            }
        }

        // Search matches by projection from current KF in target KFs
        // 使用默认参数, 最优和次优比例0.6,匹配时检查特征点的旋转
        ORBmatcher matcher;

        // Step 3：将当前帧的地图点分别与一级二级相邻关键帧地图点进行融合 -- 正向
        vector<MapPoint *> vpMPsInCurKF = mpCurKF->GetVectorMapPointsInKF();
        for (vector<KeyFrame *>::iterator vKFit = vpProjectKFs.begin(), vend = vpProjectKFs.end();
             vKFit != vend; vKFit++) {
            KeyFrame *pKFi = *vKFit;

            // 将地图点投影到关键帧中进行匹配和融合；融合策略如下
            // 1.如果地图点能匹配关键帧的特征点，并且该点有对应的地图点，那么选择观测数目多的替换两个地图点
            // 2.如果地图点能匹配关键帧的特征点，并且该点没有对应的地图点，那么为该点添加该投影地图点
            // 注意这个时候对地图点融合的操作是立即生效的
            matcher.SearchReplaceKFAndMPsByProjectInLocalMap(pKFi, vpMPsInCurKF);
        }


        if (mbAbortBA)
            return;

        // Search matches by projection from target KFs in current KF
        // Step 4：将一级二级相邻关键帧地图点分别与当前关键帧地图点进行融合 -- 反向
        // 用于进行存储要融合的一级邻接和二级邻接关键帧所有MapPoints的集合
        vector<MapPoint *> vpMPsInNeighKFs;
        vpMPsInNeighKFs.reserve(vpProjectKFs.size() * vpMPsInCurKF.size());

        //  Step 4.1：遍历每一个一级邻接和二级邻接关键帧，收集他们的地图点存储到 vpMPsInNeighKFs
        for (vector<KeyFrame *>::iterator vitKF = vpProjectKFs.begin(), vendKF = vpProjectKFs.end();
             vitKF != vendKF; vitKF++) {
            KeyFrame *pKFi = *vitKF;
            vector<MapPoint *> vpMPsInKFi = pKFi->GetVectorMapPointsInKF();
            // 遍历当前一级邻接和二级邻接关键帧中所有的MapPoints,找出需要进行融合的并且加入到集合中
            for (vector<MapPoint *>::iterator vMPit = vpMPsInKFi.begin(), vendMP = vpMPsInKFi.end();
                 vMPit != vendMP; vMPit++) {
                MapPoint *pMP = *vMPit;
                if (!pMP)
                    continue;
                // 如果地图点是坏点，或者已经加进集合vpFuseCandidates，跳过
                if (pMP->isBad() || pMP->mnFuseFlagInLocalMapping == mpCurKF->mnId)
                    continue;
                // 加入集合，并标记已经加入
                pMP->mnFuseFlagInLocalMapping = mpCurKF->mnId;
                vpMPsInNeighKFs.emplace_back(pMP);
            }
        }

        // Step 4.2：进行地图点投影融合,和正向融合操作是完全相同的
        // 不同的是正向操作是"每个关键帧和当前关键帧的地图点进行融合",而这里的是"当前关键帧和所有邻接关键帧的地图点进行融合"
        matcher.SearchReplaceKFAndMPsByProjectInLocalMap(mpCurKF, vpMPsInNeighKFs);


        // Update6DoF points
        // Step 5：更新当前帧地图点的描述子、深度、观测主方向等属性
        vpMPsInCurKF = mpCurKF->GetVectorMapPointsInKF();
        for (size_t i = 0, iend = vpMPsInCurKF.size(); i < iend; i++) {
            MapPoint *pMP = vpMPsInCurKF[i];
            if (pMP) {
                if (!pMP->isBad()) {
                    // 在所有找到pMP的关键帧中，获得最佳的描述子
                    pMP->ComputeDistinctiveDescriptors();
                    // 更新平均观测方向和观测距离
                    pMP->UpdateNormalAndDepth();
                }
            }
        }

        // Update6DoF connections in covisibility graph
        // Step 6：更新当前帧的MapPoints后更新与其它帧的连接关系
        // 更新covisibility图
        mpCurKF->UpdateCovisGraph();
    }


/**
 * @brief 检查是否要把当前的局部建图线程停止工作,运行的时候要检查是否有终止请求,如果有就执行. 由run函数调用
 */
    bool LocalMapping::CheckNotPauseOrFinish() {
        unique_lock<mutex> lock(mMutexStop);
        return mbNotPauseOrFinish;
    }

/**
 * @brief 检查mbStopped是否为true，为true表示可以并终止localmapping 线程
 */
    bool LocalMapping::CheckPaused() {
        unique_lock<mutex> lock(mMutexStop);
        return mbPaused;
    }

/**
 * @brief 求外部线程调用，为true，表示外部线程请求停止 local mapping
 */
    bool LocalMapping::CheckRequestPause() {
        unique_lock<mutex> lock(mMutexStop);
        if (mbRequestPause && !mbNotPauseOrFinish) {
            mbPaused = true;
            cout << "LM: LM Paused" << endl;
        }
        return mbRequestPause;
    }

    /**
 * @brief 外部线程调用,请求停止当前线程的工作; 其实是回环检测线程调用,来避免在进行全局优化的过程中局部建图线程添加新的关键帧
 */
    void LocalMapping::RequestPause() {
        unique_lock<mutex> lock(mMutexStop);
        mbRequestPause = true;
        unique_lock<mutex> lock2(mMutexNewKFs);
        mbAbortBA = true;
    }

/**
 * @brief 释放当前还在缓冲区中的关键帧指针
 */
    void LocalMapping::CancelPause() {
        unique_lock<mutex> lock(mMutexStop);
        unique_lock<mutex> lock2(mMutexFinish);
        if (mbFinished)
            return;
        mbPaused = false;
        mbRequestPause = false;
        for (list<KeyFrame *>::iterator lit = mlNewKeyFrames.begin(), lend = mlNewKeyFrames.end(); lit != lend; lit++)
            delete *lit;
        mlNewKeyFrames.clear();
        cout << "LM: LM Begin" << endl;
    }


/**
 * @brief 查看是否接收关键帧，也就是当前线程是否在处理数据，当然tracking线程也不会全看这个值，他会根据队列阻塞情况
 */
    bool LocalMapping::AcceptKeyFrames() {
        unique_lock<mutex> lock(mMutexAccept);
        return mbAcceptKeyFrames;
    }

/**
 * @brief 设置"允许接受关键帧"的状态标志
 */
    void LocalMapping::SetAcceptKeyFrames(bool flag) {
        unique_lock<mutex> lock(mMutexAccept);
        mbAcceptKeyFrames = flag;
    }

/**
 * @brief 如果不让它暂停，即使发出了暂停信号也不暂停
 * true: dont stop
 * false: can stop
 * bug: during insert new keyframe, a pause or finish occur, insert will down
 */
    bool LocalMapping::RequestNotPauseOrFinish(bool flag) {
        unique_lock<mutex> lock(mMutexStop);
        if (flag && (mbPaused || mbFinished))
            return false;
        mbNotPauseOrFinish = flag;
        return true;
    }


/**
 * @brief 放弃这次操作，虽然叫BA但并不是只断优化
 */
    void LocalMapping::InterruptBA() {
        mbAbortBA = true;
    }

/**
 * @brief 检测当前关键帧在共视图中的关键帧，根据地图点在共视图中的冗余程度剔除该共视关键帧
 * 冗余关键帧的判定：90%以上的地图点能被其他关键帧（至少3个）观测到
 */
    void LocalMapping::KeyFrameCulling() {
        // Check redundant keyframes (only local keyframes)
        // A keyframe is considered redundant if the 90% of the MapPoints it sees, are seen
        // in at least other 3 keyframes (in the same or finer scale)
        // We only consider close stereo points

        // 该函数里变量层层深入，这里列一下：
        // mpCurKF：当前关键帧，本程序就是判断它是否需要删除
        // pKF： mpCurrentKeyFrame的某一个共视关键帧
        // vpMapPoints：pKF对应的所有地图点
        // pMP：vpMapPoints中的某个地图点
        // observations：所有能观测到pMP的关键帧
        // pKFi：observations中的某个关键帧
        // scaleLeveli：pKFi的金字塔尺度
        // scaleLevel：pKF的金字塔尺度
        // 更新共视关系
        mpCurKF->UpdateBestCovisibles();
        // 1. 根据Covisibility Graph提取当前帧的共视关键帧
        vector<KeyFrame *> vpLocalKeyFrames = mpCurKF->GetVectorCovisibleKeyFrames();
        const bool bInitImu = mpCurrentMap->GetImuInitialized();

        // Compoute last KF from optimizable window:
        unsigned int nBeginKFId;
        if (mbHaveImu) {
            int nPre = 0;
            KeyFrame *aux_KF = mpCurKF;
            // 找到第前21个关键帧的关键帧id
            while (nPre < mnSingleMaxCullKFsNum && aux_KF->mPrevKF) {
                aux_KF = aux_KF->mPrevKF;
                nPre++;
            }
            nBeginKFId = aux_KF->mnId;
        }

        int nProcessedKFNum = 0;
        for (vector<KeyFrame *>::iterator vit = vpLocalKeyFrames.begin(),
                     vend = vpLocalKeyFrames.end(); vit != vend; vit++) {
            nProcessedKFNum++;
            KeyFrame *pKF = *vit;
            // 如果是地图里第1个关键帧或者是标记为坏帧，则跳过
            if ((pKF->mnId == pKF->GetMap()->GetInitKFId()) || pKF->isBad())
                continue;
            // Step 2：提取每个共视关键帧的地图点
            const vector<MapPoint *> vpMapPoints = pKF->GetVectorMapPointsInKF();

            // 记录冗余观测点的数目
            int nRedMPs = 0;
            int nTotMPs = 0;

            // Step 3：遍历该共视关键帧的所有地图点，判断是否90%以上的地图点能被其它至少3个关键帧（同样或者更低层级）观测到
            for (size_t i = 0, iend = vpMapPoints.size(); i < iend; i++) {
                MapPoint *pMP = vpMapPoints[i];
                if (!pMP) {
                    continue;
                }
                if (pMP->isBad()) {
                    continue;
                }
                if (!mbHaveMono) {
                    // 对于双目，仅考虑近处（不超过基线的40倍 ）的地图点
                    if (pKF->mvfMPDepth[i] > pKF->mfThDepth || pKF->mvfMPDepth[i] < 0)
                        continue;
                }
                nTotMPs++;
                // pMP->GetObsTimes() 是观测到该地图点的相机总数目（单目1，双目2）
                if (pMP->GetObsTimes() > mnWeakCovisTh) {
                    const int &CurScaleLevel = pKF->mvKPsUn[i].octave;
                    const map<KeyFrame *, tuple<int, int>> ObsKFAndLRIdx = pMP->GetObsKFAndLRIdx();
                    int nCurObs = 0;
                    // 遍历观测到该地图点的关键帧
                    for (map<KeyFrame *, tuple<int, int>>::const_iterator mit = ObsKFAndLRIdx.begin(), mend = ObsKFAndLRIdx.end();
                         mit != mend; mit++) {
                        KeyFrame *pKFi = mit->first;
                        if (pKFi == pKF)
                            continue;
                        tuple<int, int> indexes = mit->second;
                        int leftIndex = get<0>(indexes), rightIndex = get<1>(indexes);
                        int ScaleLeveli = -1;
                        ScaleLeveli = pKFi->mvKPsUn[leftIndex].octave;
                        // 同样或更低金字塔层级的地图点更准确
                        if (ScaleLeveli <= CurScaleLevel + 1) {
                            nCurObs++;
                            // 已经找到3个满足条件的关键帧，就停止不找了
                            if (nCurObs > mnWeakCovisTh)
                                break;
                        }
                    }
                    // 地图点至少被3个关键帧观测到，就记录为冗余点，更新冗余点计数数目
                    if (nCurObs > mnWeakCovisTh) {
                        nRedMPs++;
                    }
                }
            }
            // Step 4：该关键帧90%以上的有效地图点被判断为冗余的，则删除该关键帧
            if (nRedMPs > mfCullKFRedundantTh * nTotMPs) {
                // imu模式下需要更改前后关键帧的连续性，且预积分要叠加起来
                // 关键帧少于Nd个，跳过不删
                if (mpCurrentMap->GetKeyFramesNumInMap() <= mnSingleMaxCullKFsNum)
                    continue;

                // 关键帧与当前关键帧id差一个，跳过不删
                if (pKF->mnId > (mpCurKF->mnId - 2))
                    continue;

                // 关键帧具有前后关键帧
                if (pKF->mPrevKF && pKF->mNextKF) {
                    const double dTPN = pKF->mNextKF->mdTimestamp - pKF->mPrevKF->mdTimestamp;
                    // 下面两个括号里的内容一模一样
                    // imu初始化了，且距当前帧的ID超过21，且前后两个关键帧时间间隔小于3s
                    // 或者时间间隔小于0.5s
                    if ((bInitImu && (pKF->mnId < nBeginKFId) && dTPN < 3.0) || (dTPN < 0.5)) {
                        pKF->mNextKF->mpImuPreintegrated->MergePrevious(pKF->mpImuPreintegrated);
                        pKF->mNextKF->mPrevKF = pKF->mPrevKF;
                        pKF->mPrevKF->mNextKF = pKF->mNextKF;
                        pKF->mNextKF = static_cast<KeyFrame *>(NULL);
                        pKF->mPrevKF = static_cast<KeyFrame *>(NULL);
                        pKF->SetBadFlag();
                    }// 没经过imu初始化的第三阶段，且关键帧与其前一个关键帧的距离小于0.02m，且前后两个关键帧时间间隔小于3s
                    else if (!mpCurKF->GetMap()->GetImuIniertialBA2() &&
                             ((pKF->GetImuPosition() - pKF->mPrevKF->GetImuPosition()).norm() < 0.02) &&
                             (dTPN < 3)) {
                        pKF->mNextKF->mpImuPreintegrated->MergePrevious(pKF->mpImuPreintegrated);
                        pKF->mNextKF->mPrevKF = pKF->mPrevKF;
                        pKF->mPrevKF->mNextKF = pKF->mNextKF;
                        pKF->mNextKF = static_cast<KeyFrame *>(NULL);
                        pKF->mPrevKF = static_cast<KeyFrame *>(NULL);
                        pKF->SetBadFlag();
                    }
                }
            }
            // 遍历共视关键帧个数超过一定，就不弄了
            if ((nProcessedKFNum > mnSingleMaxCullKFsNum && mbAbortBA) || nProcessedKFNum > 3 * mnSingleMaxCullKFsNum) {
                break;
            }
        }
    }

/**
 * @brief 请求reset
 */
    void LocalMapping::RequestReset() {
        {
            unique_lock<mutex> lock(mMutexReset);
            cout << "LM: Map Reset Recieved" << endl;
            mbResetRequested = true;
        }

        while (1) {
            {
                unique_lock<mutex> lock2(mMutexReset);
                if (!mbResetRequested)
                    break;
            }
            usleep(5000);
        }
        cout << "LM: Map Reset, Done" << endl;
    }

/**
 * @brief 接收重置当前地图的信号
 */
    void LocalMapping::RequestResetActiveMap(Map *pMap) {
        {
            unique_lock<mutex> lock(mMutexReset);
            cout << "LM: ActiveMap Reset Recieved" << endl;
            mbResetRequestedActiveMap = true;
        }

        while (1) {
            {
                unique_lock<mutex> lock2(mMutexReset);
                if (!mbResetRequestedActiveMap)
                    break;
            }
            usleep(5000);
        }
        cout << "LM: ActiveMap Reset Done" << endl;
    }

/**
 * @brief 检查是否有复位线程的请求
 */
    void LocalMapping::ResetIfRequested() {
        bool executed_reset = false;
        {
            unique_lock<mutex> lock(mMutexReset);
            // 执行复位操作:清空关键帧缓冲区,清空待cull的地图点缓冲
            if (mbResetRequested || mbResetRequestedActiveMap) {
                executed_reset = true;
                cout << "LM: ActiveMap Reset Doing" << endl;
                mlNewKeyFrames.clear();
                mlpRecentAddedMapPoints.clear();
                mTimeFirstToCur = 0.f;
                mbBadImu = false;
                mbResetRequested = false;
                mbResetRequestedActiveMap = false;
            }
        }
        if (executed_reset) {
            cout << "LM: Reset Free Mutex" << endl;
        }
    }

/**
 * @brief 请求终止当前线程
 */
    void LocalMapping::RequestFinish() {
        unique_lock<mutex> lock(mMutexFinish);
        mbRequestFinish = true;
    }

/**
 * @brief 查看完成信号，跳出while循环
 */
    bool LocalMapping::CheckFinishRequest() {
        unique_lock<mutex> lock(mMutexFinish);
        return mbRequestFinish;
    }

/**
 * @brief 设置当前线程已经真正地结束了
 */
    void LocalMapping::SetFinished() {
        unique_lock<mutex> lock(mMutexFinish);
        mbFinished = true;
        unique_lock<mutex> lock2(mMutexStop);
        mbPaused = true;
        cout << "LocalMap Finished" << endl;
    }

/**
 * @brief 当前线程的run函数是否已经终止
 */
    bool LocalMapping::CheckFinished() {
        unique_lock<mutex> lock(mMutexFinish);
        return mbFinished;
    }

/** 
 * @brief imu初始化
 * @param GInfo 陀螺仪偏置的信息矩阵系数，主动设置时一般bInit为true，也就是只优化最后一帧的偏置，这个数会作为计算信息矩阵时使用
 * @param AInfo 加速度计偏置的信息矩阵系数
 * @param bNeedGBA 是否做BA优化，目前都为true
 */
    void LocalMapping::InitializeImu(float GInfo, float AInfo, bool bNeedGBA) {
        // 1. 将所有关键帧放入列表及向量里，且查看是否满足初始化条件
        if (mbResetRequested)
            return;

        float fMinTime = 1.0;
        int nMinKF = 10;

        // 当前地图大于10帧才进行初始化
        if (mpCurrentMap->GetKeyFramesNumInMap() < nMinKF)
            return;

        // Retrieve all keyframe in temporal order
        // 按照顺序存放目前地图里的关键帧，顺序按照前后顺序来，包括当前关键帧
        list<KeyFrame *> lpNearKFs;
        KeyFrame *pKF = mpCurKF;
        while (pKF->mPrevKF) {
            lpNearKFs.push_front(pKF);
            pKF = pKF->mPrevKF;
        }
        lpNearKFs.push_front(pKF);
        // 同样内容再构建一个和lpKF一样的容器vpKF
        vector<KeyFrame *> vpNearKFs(lpNearKFs.begin(), lpNearKFs.end());
        if (vpNearKFs.size() < nMinKF)
            return;

        mFirstTs = vpNearKFs.front()->mdTimestamp;
        if (mpCurKF->mdTimestamp - mFirstTs < fMinTime) {
            return;
        }

        // 正在做IMU的初始化，在tracking里面使用，如果为true，暂不添加关键帧
        bInitializing = true;

        // 先处理新关键帧，防止堆积且保证数据量充足
        while (HaveNewKeyFrames()) {
            ProcessNewKeyFrame();
            vpNearKFs.emplace_back(mpCurKF);
        }

        // 2. 正式IMU初始化
        const int nNearKFsNum = vpNearKFs.size();
        IMU::Bias BiasInit(0, 0, 0, 0, 0, 0);

        // Compute and KF velocities mRwg estimation
        // 在IMU连一次初始化都没有做的情况下
        if (!mpCurKF->GetMap()->GetImuInitialized()) {
            Eigen::Matrix3f Rwg;
            Eigen::Vector3f GNow;
            GNow.setZero();
            int nImuNum = 0;
            for (vector<KeyFrame *>::iterator itKF = vpNearKFs.begin(); itKF != vpNearKFs.end(); itKF++) {
                if (!(*itKF)->mpImuPreintegrated)
                    continue;
                if (!(*itKF)->mPrevKF)
                    continue;
                nImuNum++;
                // 初始化时关于速度的预积分定义Ri.mTs()*(s*Vj - s*Vi - Rwg*g*tij)
                GNow -= (*itKF)->mPrevKF->GetImuRotation() * (*itKF)->mpImuPreintegrated->GetUpdatedDeltaVelocity();
                // 求取实际的速度，位移/时间
                Eigen::Vector3f _vel = ((*itKF)->GetImuPosition() - (*itKF)->mPrevKF->GetImuPosition()) /
                                       (*itKF)->mpImuPreintegrated->mfTs;
                (*itKF)->SetVelocity(_vel);
                (*itKF)->mPrevKF->SetVelocity(_vel);
            }

            if (nImuNum < 6) {
                cout << "Not Enough to Initialize IMU" << endl;
                bInitializing = false;
                mbBadImu = true;
                return;
            }

            // GNow = sV1 - sVn + n*Rwg*g*mTs
            // 归一化
            GNow = GNow / GNow.norm();
            // 原本的重力方向
            Eigen::Vector3f GInit(0.0f, 0.0f, -1.0f);
            // 求速度方向与重力方向的角轴
            Eigen::Vector3f v = GInit.cross(
                    GNow);//the magnitude of the product equals the area of a parallelogram with the vectors for sides
            // 求角轴模长
            const float nv = v.norm();
            // 求转角大小
            const float cosg = GInit.dot(
                    GNow);// Geometrically, it is the product of the Euclidean magnitudes of the two vectors and the cosine of the angle between them
            const float ang = acos(cosg);
            // 先计算旋转向量，在除去角轴大小
            Eigen::Vector3f Qzg = v * ang / nv;
            // 获得重力方向到当前速度方向的旋转向量
            Rwg = Sophus::SO3f::exp(Qzg).matrix();
            mRwg = Rwg.cast<double>();
            mTimeFirstToCur = mpCurKF->mdTimestamp - mFirstTs;
        } else {
            mRwg = Eigen::Matrix3d::Identity();
            mbg = mpCurKF->GetGyroBias().cast<double>();
            mba = mpCurKF->GetAccBias().cast<double>();
        }

        mfScale = 1.0;
        // 3. 计算残差及偏置差，优化尺度重力方向及速度偏置，偏置先验为0，双目时不优化尺度
        Optimizer::InertialOptimization(mpCurrentMap, mRwg, mfScale, mbg, mba, mbHaveMono, nImuInfo,
                                        false, false, GInfo, AInfo);

        // 尺度太小的话初始化认为失败
        if (mfScale < 1e-1) {
            cout << "Scale Too Small" << endl;
            bInitializing = false;
            return;
        }

        // Before this line we are not changing the map
        {
            unique_lock<mutex> lock(mpCurrentMap->mMutexMapUpdate);
            // 尺度变化超过设定值，或者非单目时（无论带不带imu，但这个函数只在带imu时才执行，所以这个可以理解为双目imu）
            if ((fabs(mfScale - 1.f) > 0.00001) || !mbHaveMono) {
                Sophus::SE3f Twg(mRwg.cast<float>().transpose(), Eigen::Vector3f::Zero());
                mpCurrentMap->ApplyScaledRotation(Twg, mfScale, true);
            }

            // Check if initialization OK
            // 即使初始化成功后面还会执行这个函数重新初始化
            // 在之前没有初始化成功情况下（此时刚刚初始化成功）对每一帧都标记，后面的kf全部都在tracking里面标记为true
            // 也就是初始化之前的那些关键帧即使有imu信息也不算
            if (!mpCurrentMap->GetImuInitialized()) {
                for (int i = 0; i < nNearKFsNum; i++) {
                    KeyFrame *pKF2 = vpNearKFs[i];
                    pKF2->bImu = true;
                }
            }
        }

        // 设置经过初始化了
        if (!mpCurrentMap->GetImuInitialized()) {
            mpCurrentMap->SetImuInitialized();
            mpCurKF->bImu = true;
        }

        unsigned long GBAId = mpCurKF->mnId;
        if (bNeedGBA) {
            // 5. 承接上一步纯imu优化，按照之前的结果更新了尺度信息及适应重力方向，所以要结合地图进行一次视觉加imu的全局优化，这次带了MP等信息
            // 1.0版本里面不直接赋值了，而是将所有优化后的信息保存到变量里面
            if (AInfo != 0.f) {
                Optimizer::GlobalBundleAdjustemetWithImu(mpCurrentMap, 100, false, GBAId,
                                                         NULL, true,
                                                         GInfo, AInfo);
            } else {
                Optimizer::GlobalBundleAdjustemetWithImu(mpCurrentMap, 100, false, GBAId,
                                                         NULL, false);
            }
        }

        while (HaveNewKeyFrames()) {
            ProcessNewKeyFrame();
            vpNearKFs.emplace_back(mpCurKF);
        }

        Verbose::PrintMess("Global Bundle Adjustment finished, Updating map ...", Verbose::VERBOSITY_NORMAL);
        unique_lock<mutex> lock(mpCurrentMap->mMutexMapUpdate);
        list<KeyFrame *> lpKFstoUpdate(mpCurrentMap->mvpInitKeyFrames.begin(),
                                       mpCurrentMap->mvpInitKeyFrames.end());
        vector<MapPoint *> vpMPsToUpdate = mpCurrentMap->GetAllMapPoints();
        mpLoopCloser->UpdateKFsAndMPsAfterBA(lpKFstoUpdate, vpMPsToUpdate, GBAId);

        Verbose::PrintMess("Map updated!", Verbose::VERBOSITY_NORMAL);
        mpCurKF->GetMap()->IncreaseChangeIdx();
        mpTracker->UpdateLastAndCurFrameIMU(mfScale, vpNearKFs[0]->GetImuBias(), mpCurKF);

        mpTracker->mState = Tracking::OK;
        bInitializing = false;
        return;
    }

/**
 * @brief 返回是否正在做IMU的初始化，在tracking里面使用，如果为true，暂不添加关键帧
 */
    bool LocalMapping::IsInitializing() {
        return bInitializing;
    }

/**
 * @brief 获取当前关键帧的时间戳，System::GetTimeFromIMUInit()中调用
 */
    double LocalMapping::GetCurrKFTime() {

        if (mpCurKF) {
            return mpCurKF->mdTimestamp;
        } else
            return 0.0;
    }

} //namespace ORB_SLAM
