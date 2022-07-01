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
            mScale(1.0), infoInertial(Eigen::MatrixXd::Zero(9, 9)) {

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
        mbFarPoints = true;
        mfThFarPoints = settings->mfThFarPoints;
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

                bool b_doneLBA = false;
                int num_FixedKF_BA = 0;
                int num_OptKF_BA = 0;
                int num_MPs_BA = 0;
                int num_edges_BA = 0;

                // 已经处理完队列中的最后的一个关键帧，并且闭环检测没有请求停止LocalMapping
                if (!HaveNewKeyFrames() && !CheckRequestPause()) {
                    // 当前地图中关键帧数目大于2个
                    if (mpAtlas->KeyFramesInMap() > 2) {
                        // Step 6.1 处于IMU模式并且当前关键帧所在的地图已经完成IMU初始化
                        if (mbHaveImu && mpCurrentKeyFrame->GetMap()->GetImuInitialized()) {
                            // 计算上一关键帧到当前关键帧相机光心的距离 + 上上关键帧到上一关键帧相机光心的距离
                            float dist = (mpCurrentKeyFrame->mPrevKF->GetCameraCenter() -
                                          mpCurrentKeyFrame->GetCameraCenter()).norm() +
                                         (mpCurrentKeyFrame->mPrevKF->mPrevKF->GetCameraCenter() -
                                          mpCurrentKeyFrame->mPrevKF->GetCameraCenter()).norm();
                            // 如果距离大于5厘米，记录当前KF和上一KF时间戳的差，累加到mTinit
                            if (dist > 0.05)
                                mTimeFirstToCur += mpCurrentKeyFrame->mdTimestamp - mpCurrentKeyFrame->mPrevKF->mdTimestamp;
                            // 当前关键帧所在的地图尚未完成IMU BA2（IMU第三阶段初始化）
                            if (!mpCurrentKeyFrame->GetMap()->GetImuIniertialBA2()) {
                                // 如果累计时间差小于10s 并且 距离小于2厘米，认为运动幅度太小，不足以初始化IMU，将mbBadImu设置为true
                                if ((mTimeFirstToCur < 10.f) && (dist < 0.02)) {
                                    cout << "Not enough motion for initializing ImuBA2. Reseting..." << endl;
                                    unique_lock<mutex> lock(mMutexReset);
                                    mbResetRequestedActiveMap = true;
                                    mbBadImu = true;  // 在跟踪线程里会重置当前活跃地图
                                }
                            }
                            // 判断成功跟踪匹配的点数是否足够多
                            // 条件---------1.1、跟踪成功的内点数目大于75-----1.2、并且是单目--或--2.1、跟踪成功的内点数目大于100-----2.2、并且不是单目
                            bool bLarge = (mpTracker->GetMatchNumInLM() > 100);
                            // 局部地图+IMU一起优化，优化关键帧位姿、地图点、IMU参数
                            Optimizer::LocalBAWithImu(mpCurrentKeyFrame, &mbAbortBA, mpCurrentKeyFrame->GetMap(),
                                                      num_FixedKF_BA, num_OptKF_BA, num_MPs_BA, num_edges_BA, bLarge,
                                                      !mpCurrentKeyFrame->GetMap()->GetImuIniertialBA2());
                            b_doneLBA = true;
                        } else {
                            // Step 6.2 不是IMU模式或者当前关键帧所在的地图还未完成IMU初始化
                            // 局部地图BA，不包括IMU数据
                            // 注意这里的第二个参数是按地址传递的,当这里的 mbAbortBA 状态发生变化时，能够及时执行/停止BA
                            // 局部地图优化，不包括IMU信息。优化关键帧位姿、地图点
                            Optimizer::LocalBAWithoutImu(mpCurrentKeyFrame, &mbAbortBA, mpCurrentKeyFrame->GetMap(),
                                                         num_FixedKF_BA, num_OptKF_BA, num_MPs_BA, num_edges_BA);
                            b_doneLBA = true;
                        }
                    }
                    // Initialize IMU here
                    // Step 7 当前关键帧所在地图未完成IMU初始化（第一阶段）
                    if (!mpCurrentKeyFrame->GetMap()->GetImuInitialized() && mbHaveImu) {
                        // 在函数InitializeIMU里设置IMU成功初始化标志 SetImuInitialized
                        // IMU第一阶段初始化
                        InitializeIMU(1e2, 1e5, true);
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
                        if (mpCurrentKeyFrame->GetMap()->GetImuInitialized() && mpTracker->mState == Tracking::OK) {
                            // 当前关键帧所在的地图还未完成VIBA 1
                            if (!mpCurrentKeyFrame->GetMap()->GetImuIniertialBA1()) {// 如果累计时间差大于5s，开始VIBA1（IMU第二阶段初始化）
                                if (mTimeFirstToCur > 5.0f) {
                                    cout << "Start VIBA 1" << endl;
                                    mpCurrentKeyFrame->GetMap()->SetImuIniertialBA1();
                                    InitializeIMU(1.f, 1e5, true);
                                    cout << "End VIBA 1" << endl;
                                }
                            } else if (!mpCurrentKeyFrame->GetMap()->GetImuIniertialBA2()) {// Step 9.2 根据条件判断是否进行VIBA2（IMU第三阶段初始化）
                                if (mTimeFirstToCur > 15.0f) {
                                    cout << "Start VIBA 2" << endl;
                                    mpCurrentKeyFrame->GetMap()->SetImuIniertialBA2();
                                    InitializeIMU(0.f, 0.f, true);
                                    cout << "End VIBA 2" << endl;
                                }
                            }
                        }
                    }
                }
                // Step 10 将当前帧加入到闭环检测队列中
                mpLoopCloser->InsertKeyFrame(mpCurrentKeyFrame);
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
            mpCurrentKeyFrame = mlNewKeyFrames.front();
            // 取出最前面的关键帧后，在原来的列表里删掉该关键帧
            mlNewKeyFrames.pop_front();
        }

        // Compute Bags of Words structures
        // Step 2：计算该关键帧特征点的Bow信息
        mpCurrentKeyFrame->ComputeBoW();

        // Associate MapPoints to the new keyframe and update normal and descriptor
        // Step 3：当前处理关键帧中有效的地图点，更新normal，描述子等信息
        // TrackLocalMap中和当前帧新匹配上的地图点和当前关键帧进行关联绑定
        const vector<MapPoint *> vpMapPointsInKF = mpCurrentKeyFrame->GetMapPointsInKF();
        // 对当前处理的这个关键帧中的所有的地图点展开遍历
        for (size_t i = 0; i < vpMapPointsInKF.size(); i++) {
            MapPoint *pMP = vpMapPointsInKF[i];
            if (pMP) {
                if (!pMP->isBad()) {
                    if (!pMP->IsInKeyFrame(mpCurrentKeyFrame)) {
                        // 如果地图点不是来自当前帧的观测，为当前地图点添加观测
                        pMP->AddObsKFAndLRIdx(mpCurrentKeyFrame, i);
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
            }
        }

        // Update links in the Covisibility Graph
        // Step 4：更新关键帧间的连接关系（共视图）
        mpCurrentKeyFrame->UpdateCovisGraph();

        // Insert Keyframe in Map
        // Step 5：将该关键帧插入到地图中
        mpAtlas->AddKeyFrame(mpCurrentKeyFrame);
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
        const unsigned long int nCurrentKFid = mpCurrentKeyFrame->mnId;

        // Step 1：根据相机类型设置不同的观测阈值
        const int cnThObs = 3;
        // Step 2：遍历检查的新添加的MapPoints
        while (lit != mlpRecentAddedMapPoints.end()) {
            MapPoint *pMP = *lit;

            if (pMP->isBad()) {
                // Step 2.1：已经是坏点的MapPoints直接从检查链表中删除
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
            }
                // Step 2.4：从建立该点开始，已经过了3个关键帧而没有被剔除，则认为是质量高的点
                // 因此没有SetBadFlag()，仅从队列中删除，放弃继续对该MapPoint的检测
            else if (((int) nCurrentKFid - (int) pMP->mnFirstKFid) >= 3) {
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
        vector<KeyFrame *> vpNeighKFs = mpCurrentKeyFrame->GetBestCovisibilityKeyFrames(nn);

        // imu模式下在附近添加更多的帧进来
        if (mbHaveImu) {
            KeyFrame *pKF = mpCurrentKeyFrame;
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
        Sophus::SE3<float> sophTcw1 = mpCurrentKeyFrame->GetPose();
        Eigen::Matrix<float, 3, 4> eigTcw1 = sophTcw1.matrix3x4();
        Eigen::Matrix<float, 3, 3> Rcw1 = eigTcw1.block<3, 3>(0, 0);
        Eigen::Matrix<float, 3, 3> Rwc1 = Rcw1.transpose();
        Eigen::Vector3f tcw1 = sophTcw1.translation();
        // 得到当前关键帧（左目）光心在世界坐标系中的坐标、内参
        Eigen::Vector3f Ow1 = mpCurrentKeyFrame->GetCameraCenter();

        const float &fx1 = mpCurrentKeyFrame->fx;
        const float &fy1 = mpCurrentKeyFrame->fy;
        const float &cx1 = mpCurrentKeyFrame->cx;
        const float &cy1 = mpCurrentKeyFrame->cy;
        const float &invfx1 = mpCurrentKeyFrame->invfx;
        const float &invfy1 = mpCurrentKeyFrame->invfy;

        // 用于后面的点深度的验证;这里的1.5是经验值
        const float ratioFactor = 1.5f * mpCurrentKeyFrame->mfScaleFactor;

        // 以下是统计点数用的，没啥作用
        int countStereo = 0;
        int countStereoGoodProj = 0;
        int countStereoAttempt = 0;
        int totalStereoPts = 0;
        // Search matches with epipolar restriction and triangulate

        // Step 2：遍历相邻关键帧vpNeighKFs
        for (size_t i = 0; i < vpNeighKFs.size(); i++) {
            // 下面的过程会比较耗费时间,因此如果有新的关键帧需要处理的话,就先去处理新的关键帧吧
            if (i > 0 && HaveNewKeyFrames())
                return;

            KeyFrame *pKF2 = vpNeighKFs[i];

            GeometricCamera *pCamera1 = mpCurrentKeyFrame->mpCamera, *pCamera2 = pKF2->mpCamera;

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
            vector<pair<size_t, size_t> > vMatchedIndices;
            // 当惯性模式下，并且经过三次初始化，且为刚丢失状态
            bool bCoarse = mbHaveImu && mpTracker->mState == Tracking::RECENTLY_LOST &&
                           mpCurrentKeyFrame->GetMap()->GetImuIniertialBA2();

            // 通过极线约束的方式找到匹配点（且该点还没有成为MP，注意非单目已经生成的MP这里直接跳过不做匹配，所以最后并不会覆盖掉特征点对应的MP）
            matcher.SearchKFsByTriangulation(mpCurrentKeyFrame, pKF2, vMatchedIndices, false, bCoarse);

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
            const float &invfx2 = pKF2->invfx;
            const float &invfy2 = pKF2->invfy;

            // Triangulate each match
            // Step 5：对每对匹配通过三角化生成3D点,和 Triangulate函数差不多
            const int nmatches = vMatchedIndices.size();
            for (int ikp = 0; ikp < nmatches; ikp++) {
                // 5.0
                // 当前匹配对在当前关键帧中的索引
                const int &idx1 = vMatchedIndices[ikp].first;
                // 当前匹配对在邻接关键帧中的索引
                const int &idx2 = vMatchedIndices[ikp].second;


                // 5.1
                // 当前匹配在当前关键帧中的特征点
                const cv::KeyPoint &kp1 = mpCurrentKeyFrame->mvKPsUn[idx1];
                // mvuRight中存放着极限校准后双目特征点在右目对应的像素横坐标，如果不是基线校准的双目或者没有找到匹配点，其值将为-1（或者rgbd）
                const float kp1_ur = mpCurrentKeyFrame->mvfXInRight[idx1];
                bool bStereo1 = (kp1_ur >= 0);

                // 5.2
                // 当前匹配在邻接关键帧中的特征点
                const cv::KeyPoint &kp2 = pKF2->mvKPsUn[idx2];
                // mvuRight中存放着双目的深度值，如果不是双目，其值将为-1
                // mvuRight中存放着极限校准后双目特征点在右目对应的像素横坐标，如果不是基线校准的双目或者没有找到匹配点，其值将为-1（或者rgbd）
                const float kp2_ur = pKF2->mvfXInRight[idx2];
                bool bStereo2 = (kp2_ur >= 0);

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
                if (bStereo1)
                    // 传感器是双目相机,并且当前的关键帧的这个点有对应的深度
                    // 假设是平行的双目相机，计算出两个相机观察这个点的时候的视差角;
                    // ? 感觉直接使用向量夹角的方式计算会准确一些啊（双目的时候），那么为什么不直接使用那个呢？
                    // 回答：因为双目深度值、基线是更可靠的，比特征匹配再三角化出来的稳
                    cosParallaxStereo1 = cos(
                            2 * atan2(mpCurrentKeyFrame->mfBaseline / 2, mpCurrentKeyFrame->mvfMPDepth[idx1]));
                else if (bStereo2)
                    //传感器是双目相机,并且邻接的关键帧的这个点有对应的深度，和上面一样操作
                    cosParallaxStereo2 = cos(2 * atan2(pKF2->mfBaseline / 2, pKF2->mvfMPDepth[idx2]));

                // 统计用的
                if (bStereo1 || bStereo2) {
                    totalStereoPts++;
                }

                // 得到双目观测的视差角
                cosParallaxStereo = min(cosParallaxStereo1, cosParallaxStereo2);

                // Step 5.6：三角化恢复3D点
                Eigen::Vector3f x3D;

                bool goodProj = false;
                bool bPointStereo = false;
                // cosParallaxRays>0 && (bStereo1 || bStereo2 || cosParallaxRays<0.9998)表明视线角正常
                // cosParallaxRays<cosParallaxStereo表明前后帧视线角比双目视线角大，所以用前后帧三角化而来，反之使用双目的，如果没有双目则跳过
                // 视差角度小时用三角法恢复3D点，视差角大时（离相机近）用双目恢复3D点（双目以及深度有效）
                if (cosParallaxRays < cosParallaxStereo && cosParallaxRays > 0 && (bStereo1 || bStereo2 ||
                                                                                   (cosParallaxRays < 0.9996 &&
                                                                                    mbHaveImu) ||
                                                                                   (cosParallaxRays < 0.9998 &&
                                                                                    !mbHaveImu))) {
                    // 三角化，包装成了函数
                    goodProj = GeometricTools::Triangulate(xn1, xn2, eigTcw1, eigTcw2, x3D);
                    if (!goodProj)
                        continue;
                } else if (bStereo1 && cosParallaxStereo1 < cosParallaxStereo2) {
                    countStereoAttempt++;
                    bPointStereo = true;
                    // 如果是双目，用视差角更大的那个双目信息来恢复，直接用已知3D点反投影了
                    goodProj = mpCurrentKeyFrame->UnprojectStereo(idx1, x3D);
                } else if (bStereo2 && cosParallaxStereo2 < cosParallaxStereo1) {
                    countStereoAttempt++;
                    bPointStereo = true;
                    // 如果是双目，用视差角更大的那个双目信息来恢复，直接用已知3D点反投影了
                    goodProj = pKF2->UnprojectStereo(idx2, x3D);
                } else {
                    continue; //No stereo and very low parallax
                }

                // 成功三角化
                if (goodProj && bPointStereo)
                    countStereoGoodProj++;

                if (!goodProj)
                    continue;

                //Check triangulation in front of cameras
                // Step 5.7：检测生成的3D点是否在相机前方,不在的话就放弃这个点
                float z1 = Rcw1.row(2).dot(x3D) + tcw1(2);
                if (z1 <= 0)
                    continue;

                float z2 = Rcw2.row(2).dot(x3D) + tcw2(2);
                if (z2 <= 0)
                    continue;

                //Check reprojection error in first keyframe
                // Step 5.7：计算3D点在当前关键帧下的重投影误差
                const float &sigmaSquare1 = mpCurrentKeyFrame->mvfLevelSigma2[kp1.octave];
                const float x1 = Rcw1.row(0).dot(x3D) + tcw1(0);
                const float y1 = Rcw1.row(1).dot(x3D) + tcw1(1);
                const float invz1 = 1.0 / z1;

                if (!bStereo1) {
                    // 单目情况下
                    cv::Point2f uv1 = pCamera1->ProjectMPToKP(cv::Point3f(x1, y1, z1));
                    float errX1 = uv1.x - kp1.pt.x;
                    float errY1 = uv1.y - kp1.pt.y;

                    // 假设测量有一个像素的偏差，2自由度卡方检验阈值是5.991
                    if ((errX1 * errX1 + errY1 * errY1) > 5.991 * sigmaSquare1)
                        continue;

                } else {
                    // 双目情况
                    float u1 = fx1 * x1 * invz1 + cx1;
                    // 根据视差公式计算假想的右目坐标
                    float u1_r = u1 - mpCurrentKeyFrame->mfBaselineFocal * invz1;
                    float v1 = fy1 * y1 * invz1 + cy1;
                    float errX1 = u1 - kp1.pt.x;
                    float errY1 = v1 - kp1.pt.y;
                    float errX1_r = u1_r - kp1_ur;
                    // 自由度为3，卡方检验阈值是7.8
                    if ((errX1 * errX1 + errY1 * errY1 + errX1_r * errX1_r) > 7.8 * sigmaSquare1)
                        continue;
                }

                //Check reprojection error in second keyframe
                // 计算3D点在另一个关键帧下的重投影误差，操作同上
                const float sigmaSquare2 = pKF2->mvfLevelSigma2[kp2.octave];
                const float x2 = Rcw2.row(0).dot(x3D) + tcw2(0);
                const float y2 = Rcw2.row(1).dot(x3D) + tcw2(1);
                const float invz2 = 1.0 / z2;
                if (!bStereo2) {
                    cv::Point2f uv2 = pCamera2->ProjectMPToKP(cv::Point3f(x2, y2, z2));
                    float errX2 = uv2.x - kp2.pt.x;
                    float errY2 = uv2.y - kp2.pt.y;
                    if ((errX2 * errX2 + errY2 * errY2) > 5.991 * sigmaSquare2)
                        continue;
                } else {
                    float u2 = fx2 * x2 * invz2 + cx2;
                    float u2_r = u2 - mpCurrentKeyFrame->mfBaselineFocal * invz2;
                    float v2 = fy2 * y2 * invz2 + cy2;
                    float errX2 = u2 - kp2.pt.x;
                    float errY2 = v2 - kp2.pt.y;
                    float errX2_r = u2_r - kp2_ur;
                    if ((errX2 * errX2 + errY2 * errY2 + errX2_r * errX2_r) > 7.8 * sigmaSquare2)
                        continue;
                }

                //Check scale consistency
                // Step 5.8：检查尺度连续性

                // 世界坐标系下，3D点与相机间的向量，方向由相机指向3D点
                Eigen::Vector3f normal1 = x3D - Ow1;
                float dist1 = normal1.norm();

                Eigen::Vector3f normal2 = x3D - Ow2;
                float dist2 = normal2.norm();

                if (dist1 == 0 || dist2 == 0)
                    continue;

                if (mbFarPoints && (dist1 >= mfThFarPoints || dist2 >= mfThFarPoints)) // MODIFICATION
                    continue;
                // ratioDist是不考虑金字塔尺度下的距离比例
                const float ratioDist = dist2 / dist1;
                // 金字塔尺度因子的比例
                const float ratioOctave =
                        mpCurrentKeyFrame->mvScaleFactors[kp1.octave] / pKF2->mvScaleFactors[kp2.octave];

                // 距离的比例和图像金字塔的比例不应该差太多，否则就跳过
                if (ratioDist * ratioFactor < ratioOctave || ratioDist > ratioOctave * ratioFactor)
                    continue;

                // Triangulation is succesfull
                // Step 6：三角化生成3D点成功，构造成MapPoint
                MapPoint *pMP = new MapPoint(x3D, mpCurrentKeyFrame, mpAtlas->GetCurrentMap());
                if (bPointStereo)
                    countStereo++;

                // Step 6.1：为该MapPoint添加属性：
                // a.观测到该MapPoint的关键帧
                pMP->AddObsKFAndLRIdx(mpCurrentKeyFrame, idx1);
                pMP->AddObsKFAndLRIdx(pKF2, idx2);

                mpCurrentKeyFrame->AddMapPoint(pMP, idx1);
                pKF2->AddMapPoint(pMP, idx2);

                // mBiasOri.该MapPoint的描述子
                pMP->ComputeDistinctiveDescriptors();

                // c.该MapPoint的平均观测方向和深度范围
                pMP->UpdateNormalAndDepth();

                mpAtlas->AddMapPoint(pMP);
                // Step 7：将新产生的点放入检测队列
                // 这些MapPoints都会经过MapPointCulling函数的检验
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
        const vector<KeyFrame *> vpNeighKFs = mpCurrentKeyFrame->GetBestCovisibilityKeyFrames(nn);
        // Step 2：存储一级相邻关键帧及其二级相邻关键帧
        vector<KeyFrame *> vpProjectKFs;
        // 开始对所有候选的一级关键帧展开遍历：
        for (vector<KeyFrame *>::const_iterator vit = vpNeighKFs.begin(), vend = vpNeighKFs.end(); vit != vend; vit++) {
            KeyFrame *pKFi = *vit;
            // 没有和当前帧进行过融合的操作
            if (pKFi->isBad() || pKFi->mnFuseFlagInLocalMapping == mpCurrentKeyFrame->mnId)
                continue;
            // 加入一级相邻关键帧
            vpProjectKFs.emplace_back(pKFi);
            // 标记已经加入
            pKFi->mnFuseFlagInLocalMapping = mpCurrentKeyFrame->mnId;
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
                if (pKFi2->isBad() || pKFi2->mnFuseFlagInLocalMapping == mpCurrentKeyFrame->mnId ||
                    pKFi2->mnId == mpCurrentKeyFrame->mnId)
                    continue;
                // 存入二级相邻关键帧
                vpProjectKFs.emplace_back(pKFi2);
                pKFi2->mnFuseFlagInLocalMapping = mpCurrentKeyFrame->mnId;
            }
            if (mbAbortBA)
                break;
        }

        // Extend to temporal neighbors
        // imu模式下加了一些prevKF
        if (mbHaveImu) {
            KeyFrame *pKFi = mpCurrentKeyFrame->mPrevKF;
            while (vpProjectKFs.size() < 2 * nn && pKFi) {
                if (pKFi->isBad() || pKFi->mnFuseFlagInLocalMapping == mpCurrentKeyFrame->mnId) {
                    pKFi = pKFi->mPrevKF;
                    continue;
                }
                vpProjectKFs.emplace_back(pKFi);
                pKFi->mnFuseFlagInLocalMapping = mpCurrentKeyFrame->mnId;
                pKFi = pKFi->mPrevKF;
            }
        }

        // Search matches by projection from current KF in target KFs
        // 使用默认参数, 最优和次优比例0.6,匹配时检查特征点的旋转
        ORBmatcher matcher;

        // Step 3：将当前帧的地图点分别与一级二级相邻关键帧地图点进行融合 -- 正向
        vector<MapPoint *> vpMapPointsInKF = mpCurrentKeyFrame->GetMapPointsInKF();
        for (vector<KeyFrame *>::iterator vKFit = vpProjectKFs.begin(), vend = vpProjectKFs.end();
             vKFit != vend; vKFit++) {
            KeyFrame *pKFi = *vKFit;

            // 将地图点投影到关键帧中进行匹配和融合；融合策略如下
            // 1.如果地图点能匹配关键帧的特征点，并且该点有对应的地图点，那么选择观测数目多的替换两个地图点
            // 2.如果地图点能匹配关键帧的特征点，并且该点没有对应的地图点，那么为该点添加该投影地图点
            // 注意这个时候对地图点融合的操作是立即生效的
            matcher.SearchKFAndMapPointsByProjection(pKFi, vpMapPointsInKF);
        }


        if (mbAbortBA)
            return;

        // Search matches by projection from target KFs in current KF
        // Step 4：将一级二级相邻关键帧地图点分别与当前关键帧地图点进行融合 -- 反向
        // 用于进行存储要融合的一级邻接和二级邻接关键帧所有MapPoints的集合
        vector<MapPoint *> vpProjectMPs;
        vpProjectMPs.reserve(vpProjectKFs.size() * vpMapPointsInKF.size());

        //  Step 4.1：遍历每一个一级邻接和二级邻接关键帧，收集他们的地图点存储到 vpProjectMPs
        for (vector<KeyFrame *>::iterator vitKF = vpProjectKFs.begin(), vendKF = vpProjectKFs.end();
             vitKF != vendKF; vitKF++) {
            KeyFrame *pKFi = *vitKF;

            vector<MapPoint *> vpMapPointsInKFi = pKFi->GetMapPointsInKF();

            // 遍历当前一级邻接和二级邻接关键帧中所有的MapPoints,找出需要进行融合的并且加入到集合中
            for (vector<MapPoint *>::iterator vMPit = vpMapPointsInKFi.begin(), vendMP = vpMapPointsInKFi.end();
                 vMPit != vendMP; vMPit++) {
                MapPoint *pMP = *vMPit;
                if (!pMP)
                    continue;

                // 如果地图点是坏点，或者已经加进集合vpFuseCandidates，跳过
                if (pMP->isBad() || pMP->mnFuseFlagInLocalMapping == mpCurrentKeyFrame->mnId)
                    continue;

                // 加入集合，并标记已经加入
                pMP->mnFuseFlagInLocalMapping = mpCurrentKeyFrame->mnId;
                vpProjectMPs.emplace_back(pMP);
            }
        }

        // Step 4.2：进行地图点投影融合,和正向融合操作是完全相同的
        // 不同的是正向操作是"每个关键帧和当前关键帧的地图点进行融合",而这里的是"当前关键帧和所有邻接关键帧的地图点进行融合"
        matcher.SearchKFAndMapPointsByProjection(mpCurrentKeyFrame, vpProjectMPs);


        // Update points
        // Step 5：更新当前帧地图点的描述子、深度、观测主方向等属性
        vpMapPointsInKF = mpCurrentKeyFrame->GetMapPointsInKF();
        for (size_t i = 0, iend = vpMapPointsInKF.size(); i < iend; i++) {
            MapPoint *pMP = vpMapPointsInKF[i];
            if (pMP) {
                if (!pMP->isBad()) {
                    // 在所有找到pMP的关键帧中，获得最佳的描述子
                    pMP->ComputeDistinctiveDescriptors();
                    // 更新平均观测方向和观测距离
                    pMP->UpdateNormalAndDepth();
                }
            }
        }

        // Update connections in covisibility graph
        // Step 6：更新当前帧的MapPoints后更新与其它帧的连接关系
        // 更新covisibility图
        mpCurrentKeyFrame->UpdateCovisGraph();
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
        // mpCurrentKeyFrame：当前关键帧，本程序就是判断它是否需要删除
        // pKF： mpCurrentKeyFrame的某一个共视关键帧
        // vpMapPoints：pKF对应的所有地图点
        // pMP：vpMapPoints中的某个地图点
        // observations：所有能观测到pMP的关键帧
        // pKFi：observations中的某个关键帧
        // scaleLeveli：pKFi的金字塔尺度
        // scaleLevel：pKF的金字塔尺度
        // 更新共视关系
        mpCurrentKeyFrame->UpdateBestCovisibles();
        // 1. 根据Covisibility Graph提取当前帧的共视关键帧
        vector<KeyFrame *> vpLocalKeyFrames = mpCurrentKeyFrame->GetVectorCovisibleKeyFrames();
        const bool bInitImu = mpAtlas->GetImuInitialized();

        // Compoute last KF from optimizable window:
        unsigned int last_ID;
        if (mbHaveImu) {
            int nPre = 0;
            KeyFrame *aux_KF = mpCurrentKeyFrame;
            // 找到第前21个关键帧的关键帧id
            while (nPre < mnSingleMaxCullKFsNum && aux_KF->mPrevKF) {
                aux_KF = aux_KF->mPrevKF;
                nPre++;
            }
            last_ID = aux_KF->mnId;
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
            const vector<MapPoint *> vpMapPoints = pKF->GetMapPointsInKF();

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
                if (mpAtlas->KeyFramesInMap() <= mnSingleMaxCullKFsNum)
                    continue;

                // 关键帧与当前关键帧id差一个，跳过不删
                if (pKF->mnId > (mpCurrentKeyFrame->mnId - 2))
                    continue;

                // 关键帧具有前后关键帧
                if (pKF->mPrevKF && pKF->mNextKF) {
                    const double dTPN = pKF->mNextKF->mdTimestamp - pKF->mPrevKF->mdTimestamp;
                    // 下面两个括号里的内容一模一样
                    // imu初始化了，且距当前帧的ID超过21，且前后两个关键帧时间间隔小于3s
                    // 或者时间间隔小于0.5s
                    if ((bInitImu && (pKF->mnId < last_ID) && dTPN < 3.0) || (dTPN < 0.5)) {
                        pKF->mNextKF->mpImuPreintegrated->MergePrevious(pKF->mpImuPreintegrated);
                        pKF->mNextKF->mPrevKF = pKF->mPrevKF;
                        pKF->mPrevKF->mNextKF = pKF->mNextKF;
                        pKF->mNextKF = static_cast<KeyFrame *>(NULL);
                        pKF->mPrevKF = static_cast<KeyFrame *>(NULL);
                        pKF->SetBadFlag();
                    }// 没经过imu初始化的第三阶段，且关键帧与其前一个关键帧的距离小于0.02m，且前后两个关键帧时间间隔小于3s
                    else if (!mpCurrentKeyFrame->GetMap()->GetImuIniertialBA2() &&
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
 * @param priorG 陀螺仪偏置的信息矩阵系数，主动设置时一般bInit为true，也就是只优化最后一帧的偏置，这个数会作为计算信息矩阵时使用
 * @param priorA 加速度计偏置的信息矩阵系数
 * @param bNeedGBA 是否做BA优化，目前都为true
 */
    void LocalMapping::InitializeIMU(float priorG, float priorA, bool bNeedGBA) {
        // 1. 将所有关键帧放入列表及向量里，且查看是否满足初始化条件
        if (mbResetRequested)
            return;

        float fMinTime = 1.0;
        int nMinKF = 10;

        // 当前地图大于10帧才进行初始化
        if (mpAtlas->KeyFramesInMap() < nMinKF)
            return;

        // Retrieve all keyframe in temporal order
        // 按照顺序存放目前地图里的关键帧，顺序按照前后顺序来，包括当前关键帧
        list<KeyFrame *> lpNearKFs;
        KeyFrame *pKF = mpCurrentKeyFrame;
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
        if (mpCurrentKeyFrame->mdTimestamp - mFirstTs < fMinTime) {
            return;
        }

        // 正在做IMU的初始化，在tracking里面使用，如果为true，暂不添加关键帧
        bInitializing = true;

        // 先处理新关键帧，防止堆积且保证数据量充足
        while (HaveNewKeyFrames()) {
            ProcessNewKeyFrame();
            vpNearKFs.emplace_back(mpCurrentKeyFrame);
            lpNearKFs.emplace_back(mpCurrentKeyFrame);
        }

        // 2. 正式IMU初始化
        const int nNearKFsNum = vpNearKFs.size();
        IMU::Bias BiasInit(0, 0, 0, 0, 0, 0);

        // Compute and KF velocities mRwg estimation
        // 在IMU连一次初始化都没有做的情况下
        if (!mpCurrentKeyFrame->GetMap()->GetImuInitialized()) {
            Eigen::Matrix3f Rwg;
            Eigen::Vector3f DirG;
            DirG.setZero();
            int nImuNum = 0;
            for (vector<KeyFrame *>::iterator itKF = vpNearKFs.begin(); itKF != vpNearKFs.end(); itKF++) {
                if (!(*itKF)->mpImuPreintegrated)
                    continue;
                if (!(*itKF)->mPrevKF)
                    continue;
                nImuNum++;
                // 初始化时关于速度的预积分定义Ri.mTs()*(s*Vj - s*Vi - Rwg*g*tij)
                DirG -= (*itKF)->mPrevKF->GetImuRotation() * (*itKF)->mpImuPreintegrated->GetUpdatedDeltaVelocity();
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

            // DirG = sV1 - sVn + n*Rwg*g*mTs
            // 归一化
            DirG = DirG / DirG.norm();
            // 原本的重力方向
            Eigen::Vector3f GInit(0.0f, 0.0f, -1.0f);
            // 求速度方向与重力方向的角轴
            Eigen::Vector3f v = GInit.cross(DirG);
            // 求角轴模长
            const float nv = v.norm();
            // 求转角大小
            const float cosg = GInit.dot(DirG);
            const float ang = acos(cosg);
            // 先计算旋转向量，在除去角轴大小
            Eigen::Vector3f vzg = v * ang / nv;
            // 获得重力方向到当前速度方向的旋转向量
            Rwg = Sophus::SO3f::exp(vzg).matrix();
            mRwg = Rwg.cast<double>();
            mTimeFirstToCur = mpCurrentKeyFrame->mdTimestamp - mFirstTs;
        } else {
            mRwg = Eigen::Matrix3d::Identity();
            mbg = mpCurrentKeyFrame->GetGyroBias().cast<double>();
            mba = mpCurrentKeyFrame->GetAccBias().cast<double>();
        }

        mScale = 1.0;
        // 3. 计算残差及偏置差，优化尺度重力方向及速度偏置，偏置先验为0，双目时不优化尺度
        Optimizer::InertialOptimization(mpAtlas->GetCurrentMap(), mRwg, mScale, mbg, mba, mbHaveMono, infoInertial,
                                        false, false, priorG, priorA);

        // 尺度太小的话初始化认为失败
        if (mScale < 1e-1) {
            cout << "Scale Too Small" << endl;
            bInitializing = false;
            return;
        }

        // Before this line we are not changing the map
        {
            unique_lock<mutex> lock(mpAtlas->GetCurrentMap()->mMutexMapUpdate);
            // 尺度变化超过设定值，或者非单目时（无论带不带imu，但这个函数只在带imu时才执行，所以这个可以理解为双目imu）
            if ((fabs(mScale - 1.f) > 0.00001) || !mbHaveMono) {
                // 4.1 恢复重力方向与尺度信息
                Sophus::SE3f Twg(mRwg.cast<float>().transpose(), Eigen::Vector3f::Zero());
                mpAtlas->GetCurrentMap()->ApplyScaledRotation(Twg, mScale, true);
                // 4.2 更新普通帧的位姿，主要是当前帧与上一帧
                mpTracker->UpdateFrameIMU(mScale, vpNearKFs[0]->GetImuBias(), mpCurrentKeyFrame);
            }

            // Check if initialization OK
            // 即使初始化成功后面还会执行这个函数重新初始化
            // 在之前没有初始化成功情况下（此时刚刚初始化成功）对每一帧都标记，后面的kf全部都在tracking里面标记为true
            // 也就是初始化之前的那些关键帧即使有imu信息也不算
            if (!mpAtlas->GetImuInitialized()) {
                for (int i = 0; i < nNearKFsNum; i++) {
                    KeyFrame *pKF2 = vpNearKFs[i];
                    pKF2->bImu = true;
                }
            }
        }

        // TODO 这步更新是否有必要做待研究，0.4版本是放在FullInertialBA下面做的
        // 这个版本FullInertialBA不直接更新位姿及三位点了
        mpTracker->UpdateFrameIMU(1.0, vpNearKFs[0]->GetImuBias(), mpCurrentKeyFrame);

        // 设置经过初始化了
        if (!mpAtlas->GetImuInitialized()) {
            mpAtlas->SetImuInitialized();
            mpCurrentKeyFrame->bImu = true;
        }

        if (bNeedGBA) {
            // 5. 承接上一步纯imu优化，按照之前的结果更新了尺度信息及适应重力方向，所以要结合地图进行一次视觉加imu的全局优化，这次带了MP等信息
            // 1.0版本里面不直接赋值了，而是将所有优化后的信息保存到变量里面
            if (priorA != 0.f) {
                Optimizer::GlobalBundleAdjustemetWithImu(mpAtlas->GetCurrentMap(), 100, false, mpCurrentKeyFrame->mnId,
                                                         NULL, true,
                                                         priorG, priorA);
            } else {
                Optimizer::GlobalBundleAdjustemetWithImu(mpAtlas->GetCurrentMap(), 100, false, mpCurrentKeyFrame->mnId,
                                                         NULL, false);
            }

        }
        Verbose::PrintMess("Global Bundle Adjustment finished\nUpdating map ...", Verbose::VERBOSITY_NORMAL);

        // Get Map Mutex
        unique_lock<mutex> lock(mpAtlas->GetCurrentMap()->mMutexMapUpdate);
        unsigned long GBAid = mpCurrentKeyFrame->mnId;
        // Process keyframes in the queue
        // 6. 处理一下新来的关键帧，这些关键帧没有参与优化
        while (HaveNewKeyFrames()) {
            ProcessNewKeyFrame();
            vpNearKFs.emplace_back(mpCurrentKeyFrame);
            lpNearKFs.emplace_back(mpCurrentKeyFrame);
        }

        // Correct keyframes starting at map first keyframe
        // 7. 更新位姿与三维点
        // 获取地图中初始关键帧
        list<KeyFrame *> lpKFtoCheck(mpAtlas->GetCurrentMap()->mvpInitKeyFrames.begin(),
                                     mpAtlas->GetCurrentMap()->mvpInitKeyFrames.end());

        // 初始就一个关键帧，顺藤摸瓜找到父子相连的所有关键帧
        // 类似于树的广度优先搜索，其实也就是根据父子关系遍历所有的关键帧，有的参与了FullInertialBA有的没参与
        while (!lpKFtoCheck.empty()) {
            // 7.1 获得这个关键帧的子关键帧
            KeyFrame *pKF = lpKFtoCheck.front();
            const set<KeyFrame *> sChilds = pKF->GetChilds();
            Sophus::SE3f Twc = pKF->GetPoseInverse();  // 获得关键帧的优化前的位姿
            // 7.2 遍历这个关键帧所有的子关键帧
            for (set<KeyFrame *>::const_iterator sit = sChilds.begin(); sit != sChilds.end(); sit++) {
                // 确认是否能用
                KeyFrame *pChild = *sit;
                if (!pChild || pChild->isBad())
                    continue;

                // 这个判定为true表示pChild没有参与前面的优化，因此要根据已经优化过的更新，结果全部暂存至变量
                if (pChild->mnBAGlobalForKF != GBAid) {
                    // pChild->GetPose()也是优化前的位姿，Twc也是优化前的位姿
                    // 7.3 因此他们的相对位姿是比较准的，可以用于更新pChild的位姿
                    Sophus::SE3f Tchildc = pChild->GetPose() * Twc;
                    // 使用相对位姿，根据pKF优化后的位姿更新pChild位姿，最后结果都暂时放于mTcwGBA
                    pChild->mTcwGBA = Tchildc * pKF->mTcwGBA;

                    // 7.4 使用相同手段更新速度
                    Sophus::SO3f Rcor = pChild->mTcwGBA.so3().inverse() * pChild->GetPose().so3();
                    if (pChild->isVelocitySet()) {
                        pChild->mVwbGBA = Rcor * pChild->GetVelocity();
                    } else {
                        Verbose::PrintMess("Child velocity empty!! ", Verbose::VERBOSITY_NORMAL);
                    }

                    pChild->mBiasGBA = pChild->GetImuBias();
                    pChild->mnBAGlobalForKF = GBAid;
                }
                // 加入到list中，再去寻找pChild的子关键帧
                lpKFtoCheck.emplace_back(pChild);
            }

            // 7.5 此时pKF的利用价值就没了，但是里面的数值还都是优化前的，优化后的全部放于类似mTcwGBA这样的变量中
            // 所以要更新到正式的状态里，另外mTcwBefGBA要记录更新前的位姿，用于同样的手段更新三维点用
            pKF->mTcwBefGBA = pKF->GetPose();
            pKF->SetPose(pKF->mTcwGBA);

            // 速度偏置同样更新
            if (pKF->bImu) {
                pKF->SetVelocity(pKF->mVwbGBA);
                pKF->SetNewBias(pKF->mBiasGBA);
            } else {
                cout << "KF " << pKF->mnId << " not set to inertial!! \n";
            }

            // pop
            lpKFtoCheck.pop_front();
        }

        // Correct MapPoints
        // 8. 更新三维点，三维点在优化后同样没有正式的更新，而是找了个中间变量保存了优化后的数值
        const vector<MapPoint *> vpMPs = mpAtlas->GetCurrentMap()->GetAllMapPoints();

        for (size_t i = 0; i < vpMPs.size(); i++) {
            MapPoint *pMP = vpMPs[i];

            if (pMP->isBad())
                continue;

            // 8.1 如果这个点参与了全局优化，那么直接使用优化后的值来赋值
            if (pMP->mnBAGlobalForKF == GBAid) {
                // If optimized by Global BA, just update
                pMP->SetWorldPos(pMP->mPosGBA);
            }
                // 如果没有参与，与关键帧的更新方式类似
            else {
                // Update according to the correction of its reference keyframe
                KeyFrame *pRefKF = pMP->GetReferenceKeyFrame();

                if (pRefKF->mnBAGlobalForKF != GBAid)
                    continue;

                // Map to non-corrected camera
                // 8.2 根据优化前的世界坐标系下三维点的坐标以及优化前的关键帧位姿计算这个点在关键帧下的坐标
                Eigen::Vector3f Xc = pRefKF->mTcwBefGBA * pMP->GetWorldPos();

                // Backproject using corrected camera
                // 8.3 根据优化后的位姿转到世界坐标系下作为这个点优化后的三维坐标
                pMP->SetWorldPos(pRefKF->GetPoseInverse() * Xc);
            }
        }

        Verbose::PrintMess("Map updated!", Verbose::VERBOSITY_NORMAL);

        for (list<KeyFrame *>::iterator lit = mlNewKeyFrames.begin(), lend = mlNewKeyFrames.end(); lit != lend; lit++) {
            (*lit)->SetBadFlag();
            delete *lit;
        }
        mlNewKeyFrames.clear();
        mpTracker->mState = Tracking::OK;
        bInitializing = false;
        mpCurrentKeyFrame->GetMap()->IncreaseChangeIdx();
        return;
    }

/**
 * @brief 通过BA优化进行尺度更新，关键帧小于100，使用了所有关键帧的信息，但只优化尺度和重力方向。每10s在这里的时间段内时多次进行尺度更新
 */
    void LocalMapping::ScaleRefinement() {
        // Minimum number of keyframes to compute a solution
        // Minimum time (seconds) between first and last keyframe to compute a solution. Make the difference between monocular and stereo
        // unique_lock<mutex> lock0(mMutexImuInit);
        if (mbResetRequested)
            return;

        // Retrieve all keyframes in temporal order
        // 1. 检索所有的关键帧（当前地图）
        list<KeyFrame *> lpKF;
        KeyFrame *pKF = mpCurrentKeyFrame;
        while (pKF->mPrevKF) {
            lpKF.push_front(pKF);
            pKF = pKF->mPrevKF;
        }
        lpKF.push_front(pKF);
        vector<KeyFrame *> vpKF(lpKF.begin(), lpKF.end());

        // 加入新添加的帧
        while (HaveNewKeyFrames()) {
            ProcessNewKeyFrame();
            vpKF.emplace_back(mpCurrentKeyFrame);
            lpKF.emplace_back(mpCurrentKeyFrame);
        }

        const int N = vpKF.size();

        // 2. 更新旋转与尺度
        // 待优化变量的初值
        mRwg = Eigen::Matrix3d::Identity();
        mScale = 1.0;

        // 优化重力方向与尺度
        Optimizer::InertialOptimization(mpAtlas->GetCurrentMap(), mRwg, mScale);

        if (mScale < 1e-1) // 1e-1
        {
            cout << "scale too small" << endl;
            bInitializing = false;
            return;
        }

        Sophus::SO3d so3wg(mRwg);


        // Before this line we are not changing the map
        // 3. 开始更新地图
        unique_lock<mutex> lock(mpAtlas->GetCurrentMap()->mMutexMapUpdate);
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        // 3.1 如果尺度更新较多，或是在双目imu情况下更新地图
        // 0.4版本这个值为0.00001
        if ((fabs(mScale - 1.f) > 0.002) || !mbHaveMono) {
            Sophus::SE3f Tgw(mRwg.cast<float>().transpose(), Eigen::Vector3f::Zero());
            mpAtlas->GetCurrentMap()->ApplyScaledRotation(Tgw, mScale, true);
            mpTracker->UpdateFrameIMU(mScale, mpCurrentKeyFrame->GetImuBias(), mpCurrentKeyFrame);
        }
        std::chrono::steady_clock::time_point t3 = std::chrono::steady_clock::now();

        // 3.2 优化的这段时间新进来的kf全部清空不要
        for (list<KeyFrame *>::iterator lit = mlNewKeyFrames.begin(), lend = mlNewKeyFrames.end(); lit != lend; lit++) {
            (*lit)->SetBadFlag();
            delete *lit;
        }
        mlNewKeyFrames.clear();
        // To perform pose-inertial opt mGyr.r.mTs. last keyframe
        mpCurrentKeyFrame->GetMap()->IncreaseChangeIdx();

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

        if (mpCurrentKeyFrame) {
            return mpCurrentKeyFrame->mdTimestamp;
        } else
            return 0.0;
    }

} //namespace ORB_SLAM
