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


#include "LoopClosing.h"

#include "Sim3Solver.h"
#include "Converter.h"
#include "Optimizer.h"
#include "ORBmatcher.h"
#include "G2oTypes.h"

#include<mutex>
#include<thread>


namespace ORB_SLAM3 {

/**
 * @brief 回环线程构造函数
 * @param pAtlas atlas
 * @param pKFDB 关键帧词典数据库
 * @param bFixScale 除了单目都为true，包括imu单目
 * @param bActiveLC 开启回环，默认是开启的
 */
    LoopClosing::LoopClosing(Atlas *pAtlas, KeyFrameDatabase *pDB, ORBVocabulary *pVoc, const bool bFixScale,
                             const bool bActiveLC, Settings *settings) :
            mbResetRequested(false), mbResetActiveMapRequested(false), mbRequestFinish(false), mbFinished(true),
            mpAtlas(pAtlas),
            mpKeyFrameDB(pDB), mpORBVocabulary(pVoc), mbRunningGBA(false),
            mbFinishedGBA(true),
            mbStopGBA(false), mpThreadGBA(NULL), mbFixScale(bFixScale), mnFullBAIdx(0), mnLoopNumCoincidences(0),
            mnMergeNumCoincidences(0),
            mbLoopDetected(false), mbMergeDetected(false), mnLoopNumNotFound(0), mnMergeNumNotFound(0),
            mbActiveLC(bActiveLC) {
        // 连续性阈值
        mnCovisibilityConsistencyTh = settings->mnWeakCovisTh;
        mnNumCorrection = 0;
        mnCorrectionGBA = 0;
    }

    void LoopClosing::SetTracker(Tracking *pTracker) {
        mpTracker = pTracker;
    }

    void LoopClosing::SetLocalMapper(LocalMapping *pLocalMapper) {
        mpLocalMapper = pLocalMapper;
    }

/**
 * @brief 回环线程主函数
 */
    void LoopClosing::Run() {
        mbFinished = false;
        // 线程主循环
        while (1) {
            //NEW LOOP AND MERGE DETECTION ALGORITHM
            //----------------------------

            // Loopclosing中的关键帧是LocalMapping发送过来的，LocalMapping是Tracking中发过来的
            // 在LocalMapping中通过 InsertKeyFrame 将关键帧插入闭环检测队列mlpLoopKeyFrameQueue
            // Step 1 查看闭环检测队列mlpLoopKeyFrameQueue中有没有关键帧进来
            if (CheckNewKeyFrames()) {
                // Step 2 检测有没有共视的区域
                bool bFindedRegion = DetectCommonRegionsExist();
                if (bFindedRegion) {
                    // Step 3 如果检测到融合（当前关键帧与其他地图有关联）, 则合并地图
                    if (mbMergeDetected) {
                        // 在imu没有初始化就放弃融合
                        if (!mpCurrentKF->GetMap()->GetImuInitialized()) {
                            cout << "IMU is not initilized, merge is aborted" << endl;
                        } else {
                            // 拿到融合帧在自己地图所在坐标系(w2)下的位姿
                            Sophus::SE3d mTmw = mpMergeMatchedKF->GetPose().cast<double>();
                            g2o::Sim3 gSmw2(mTmw.unit_quaternion(), mTmw.translation(), 1.0);
                            // 拿到当前帧在自己地图所在坐标系(w1)下的位姿
                            Sophus::SE3d mTcw = mpCurrentKF->GetPose().cast<double>();
                            g2o::Sim3 gScw1(mTcw.unit_quaternion(), mTcw.translation(), 1.0);
                            // 根据共同区域检测时的Sim3结果得到当前帧在w2下的位姿
                            // mg2oMergeSlw 里存放的是融合候选关键帧所在的世界坐标系w2到当前帧的Sim3位姿
                            // l = c , w2是融合候选关键帧所在的世界坐标系
                            g2o::Sim3 gSw2c = mg2oMergeSlw.inverse();
                            // 这个没有用到 : 融合帧在w1下的位姿
                            g2o::Sim3 gSw1m = mg2oMergeSlw;

                            // 记录焊接变换(Sim3) T_w2_w1 , 这个量实际是两个地图坐标系的关系 T_w2_w1 = T_w2_c * T_c_w1
                            mSold_new = (gSw2c * gScw1);

                            // 如果是imu模式
                            cout << "Merge check transformation with IMU" << endl;
                            // 如果尺度变换太大, 认为累积误差较大，则放弃融合
                            if (mSold_new.scale() < 0.90 || mSold_new.scale() > 1.1) {
                                mpMergeLastCurrentKF->SetCanErase();
                                mpMergeMatchedKF->SetCanErase();
                                mnMergeNumCoincidences = 0;
                                mvpMergeMatchedMPs.clear();
                                mvpMergeMPs.clear();
                                mnMergeNumNotFound = 0;
                                mbMergeDetected = false;
                                Verbose::PrintMess("scale bad estimated. Abort merging", Verbose::VERBOSITY_NORMAL);
                                continue;
                            }

                            // If inertial, force only yaw
                            // 如果是imu模式并且完成了初始化,强制将焊接变换的 roll 和 pitch 设为0
                            // 通过物理约束来保证两个坐标轴都是水平的
                            if (mpCurrentKF->GetMap()->GetImuIniertialBA1()) // TODO, maybe with GetImuIniertialBA1
                            {
                                Eigen::Vector3d phi = LogSO3(mSold_new.rotation().toRotationMatrix());
                                phi(0) = 0;
                                phi(1) = 0;
                                mSold_new = g2o::Sim3(ExpSO3(phi), mSold_new.translation(), 1.0);
                            }

                            // 这个变量没有用到
                            mg2oMergeSmw = gSmw2 * gSw2c * gScw1;
                            // 更新mg2oMergeScw
                            mg2oMergeScw = mg2oMergeSlw;

                            //mpTracker->SetStepByStep(true);
                            Verbose::PrintMess("*Merge detected", Verbose::VERBOSITY_QUIET);
                            // TODO UNCOMMENT
                            // 如果是imu模式,则开启 Visual-Inertial Map Merging
                            MergeLocalWithImu();
                            Verbose::PrintMess("Merge finished!", Verbose::VERBOSITY_QUIET);
                        }
                        // 记录时间戳
                        vdPR_CurrentTime.emplace_back(mpCurrentKF->mdTimestamp);
                        vdPR_MatchedTime.emplace_back(mpMergeMatchedKF->mdTimestamp);
                        // 标记Place recognition结果为地图融合
                        vnPR_TypeRecogn.emplace_back(1);

                        // CheckRequestReset all variables
                        // 重置所有融合相关变量
                        mpMergeLastCurrentKF->SetCanErase();
                        mpMergeMatchedKF->SetCanErase();
                        mnMergeNumCoincidences = 0;
                        mvpMergeMatchedMPs.clear();
                        mvpMergeMPs.clear();
                        mnMergeNumNotFound = 0;
                        mbMergeDetected = false;

                        // 重置所有回环相关变量, 说明对与当前帧同时有回环和融合的情况只进行融合
                        if (mbLoopDetected) {
                            // CheckRequestReset Loop variables
                            mpLoopLastCurrentKF->SetCanErase();
                            mpLoopMatchedKF->SetCanErase();
                            mnLoopNumCoincidences = 0;
                            mvpLoopMatchedMPs.clear();
                            mvpLoopMPs.clear();
                            mnLoopNumNotFound = 0;
                            mbLoopDetected = false;
                        }
                    }

                    // Step 4 如果(没有检测到融合)检测到回环, 则回环矫正
                    if (mbLoopDetected) {
                        // 标记时间戳
                        bool bGoodLoop = true;
                        vdPR_CurrentTime.emplace_back(mpCurrentKF->mdTimestamp);
                        vdPR_MatchedTime.emplace_back(mpLoopMatchedKF->mdTimestamp);
                        vnPR_TypeRecogn.emplace_back(0);
                        Verbose::PrintMess("*Loop detected", Verbose::VERBOSITY_QUIET);
                        // 更新 mg2oLoopScw
                        mg2oLoopScw = mg2oLoopSlw; //*mvg2oSim3LoopTcw[nCurrentIndex];
                        // 拿到当前关键帧相对于世界坐标系的位姿
                        Sophus::SE3d Twc = mpCurrentKF->GetPoseInverse().cast<double>();
                        g2o::Sim3 g2oTwc(Twc.unit_quaternion(), Twc.translation(), 1.0);
                        // mg2oLoopScw是通过回环检测的Sim3计算出的回环矫正后的当前关键帧的初始位姿, Twc是当前关键帧回环矫正前的位姿.
                        // g2oSww_new 可以理解为correction
                        g2o::Sim3 g2oSww_new = g2oTwc * mg2oLoopScw;

                        // 拿到 roll ,pitch ,yaw
                        Eigen::Vector3d phi = LogSO3(g2oSww_new.rotation().toRotationMatrix());
                        cout << "phi = " << phi.transpose() << endl;
                        // 这里算是通过imu重力方向验证回环结果, 如果pitch或roll角度偏差稍微有一点大,则回环失败. 对yaw容忍比较大(20度)
                        if (fabs(phi(0)) < 0.008f && fabs(phi(1)) < 0.008f && fabs(phi(2)) < 0.349f) {
                            // If inertial, force only yaw
                            // 如果是imu模式,强制将焊接变换的的 roll 和 pitch 设为0
                            if (mpCurrentKF->GetMap()->GetImuIniertialBA2()) {
                                phi(0) = 0;
                                phi(1) = 0;
                                g2oSww_new = g2o::Sim3(ExpSO3(phi), g2oSww_new.translation(), 1.0);
                                mg2oLoopScw = g2oTwc.inverse() * g2oSww_new;
                            }
                        } else {
                            cout << "BAD LOOP!!!" << endl;
                            bGoodLoop = false;
                        }
                        if (bGoodLoop) {
                            mvpLoopMapPoints = mvpLoopMPs;
                            // 开启回环矫正
                            CorrectLoop();
                            mnNumCorrection += 1;
                        }
                        // CheckRequestReset all variables
                        mpLoopLastCurrentKF->SetCanErase();
                        mpLoopMatchedKF->SetCanErase();
                        mnLoopNumCoincidences = 0;
                        mvpLoopMatchedMPs.clear();
                        mvpLoopMPs.clear();
                        mnLoopNumNotFound = 0;
                        mbLoopDetected = false;
                    }
                } else if (mpLastMap) {
                    InitializeRtk();
                }
            }
            // 查看是否有外部线程请求复位当前线程
            ResetIfRequested();
            // 查看外部线程是否有终止当前线程的请求,如果有的话就跳出这个线程的主函数的主循环
            if (CheckRequestFinish()) {
                break;
            }
            usleep(5000);
        }
        SetFinished();
    }

    void LoopClosing::InitializeRtk() {
        if (mpLastMap->GetKeyFramesNumInMap() < 50 || !mpLastMap->GetImuInitialized()) {
            return;
        }
        const vector<KeyFrame *> vpKFs = mpLastMap->GetAllKeyFrames();

        Eigen::Matrix3f R12i, RBestOri12i;
        Eigen::Vector3f t12i, tBestOri12i;
        float s12i, sBestOri12i, fRtkToLocalDist, fBestOriDist = 0;
        Eigen::Matrix4f T12i, TBestOri12i;
        bool bFixScale = false;
        if (mpLastMap->GetRtkInitialized()) {
            mpLastMap->GetSim3FRtkToLocal(TBestOri12i, RBestOri12i, tBestOri12i, sBestOri12i);
            for (size_t i = 0; i < vpKFs.size(); i++) {
                fBestOriDist += (sBestOri12i * RBestOri12i * vpKFs[i]->GetRtkTransF() + tBestOri12i -
                                 vpKFs[i]->GetCameraCenter()).norm();
            }
            fBestOriDist /= vpKFs.size();
        } else {
            fBestOriDist = 1e10;
        }
        if (fBestOriDist > 1) {
            mpLastMap->SetRtkInitialized(false);
            cout << "RtkToLocal need to be update: OriDist: sBestOri12i: " << fBestOriDist << " " << sBestOri12i
                 << endl;
        }

        Eigen::Matrix<float, 3, Eigen::Dynamic> TKFLocal(3, vpKFs.size());
        Eigen::Matrix<float, 3, Eigen::Dynamic> TKFRtk(3, vpKFs.size());
        for (size_t i = 0; i < vpKFs.size(); ++i) {
            TKFLocal.col(i) = vpKFs[i]->GetCameraCenter();
            TKFRtk.col(i) = vpKFs[i]->GetRtkTransF();
        }
        T12i = Eigen::umeyama(TKFRtk, TKFLocal, true);
        Converter::tosRt(T12i, s12i, R12i, t12i);
        if (s12i < 0.9 || s12i > 1.1) {
            return;
        }

        fRtkToLocalDist = 0;
        for (size_t i = 0; i < vpKFs.size(); i++) {
            fRtkToLocalDist += (s12i * R12i * vpKFs[i]->GetRtkTransF() + t12i - vpKFs[i]->GetCameraCenter()).norm();
        }
        fRtkToLocalDist /= vpKFs.size();

        if (fRtkToLocalDist < 1 && fRtkToLocalDist < fBestOriDist) {
            mpLastMap->SetRtkInitialized(true);
            mpLastMap->SetSim3FRtkToLocal(T12i, R12i, t12i, s12i);
            cout << "NowDist: OriDist: sBest: " << fRtkToLocalDist << " " << fBestOriDist << " " << s12i
                 << endl;
        }
    }

/**
 * @brief 插入关键帧
 */
    void LoopClosing::InsertKeyFrame(KeyFrame *pKF) {
        unique_lock<mutex> lock(mMutexLoopQueue);
        if (pKF->mnId != 0)
            mlpLoopKeyFrameQueue.emplace_back(pKF);
    }

/**
 * @brief 查看有没有未处理的关键帧
 */
    bool LoopClosing::CheckNewKeyFrames() {
        unique_lock<mutex> lock(mMutexLoopQueue);
        return (!mlpLoopKeyFrameQueue.empty());
    }


/**
 * @brief 检测有没有共同区域,包括检测回环和融合匹配,sim3计算,验证
 * 对应于ORB-SLAM2里的函数DetectLoop
 * @return true
 * @return false
 */
    bool LoopClosing::DetectCommonRegionsExist() {
        {
            // Step 1 从队列中取出一个关键帧,作为当前检测共同区域的关键帧
            unique_lock<mutex> lock(mMutexLoopQueue);
            mpCurrentKF = mlpLoopKeyFrameQueue.front();
            mlpLoopKeyFrameQueue.pop_front();
            mpLastMap = mpCurrentKF->GetMap();
            if (mbActiveLC) {
                // 设置当前关键帧不要在优化的过程中被删除
                mpCurrentKF->SetNotErase();
            }
        }
        if (!mbActiveLC)
            return false;

        // Step 2 在某些情况下不进行共同区域检测
        if (!mpLastMap->GetImuIniertialBA2() || mpLastMap->GetKeyFramesNumInMap() < 12) {
            mpKeyFrameDB->add(mpCurrentKF);
            mpCurrentKF->SetCanErase();
            return false;
        }

        bool bLoopDetectedInKF = false;

        // Step 3.1 回环的时序几何校验。注意初始化时mnLoopNumCoincidences=0,
        if (mnLoopNumCoincidences > 0) {
            Sophus::SE3d mTcl = (mpCurrentKF->GetPose() * mpLoopLastCurrentKF->GetPoseInverse()).cast<double>();
            g2o::Sim3 gScl(mTcl.unit_quaternion(), mTcl.translation(), 1.0);
            g2o::Sim3 gScw = gScl * mg2oLoopSlw;
            int numProjMatches = 0;
            vector<MapPoint *> vpMatchedMPs;

            // 通过把候选帧局部窗口内的地图点向新进来的关键帧投影来验证回环检测结果,并优化Sim3位姿
            bool bCommonRegion = DetectAndRefineSim3FromLastKF(mpCurrentKF, mpLoopMatchedKF, gScw, numProjMatches,
                                                               mvpLoopMPs, vpMatchedMPs);
            // 如果找到共同区域(时序验证成功一次)
            if (bCommonRegion) {
                //标记时序检验成功一次
                bLoopDetectedInKF = true;
                // 累计正检验的成功次数
                mnLoopNumCoincidences++;
                // 不再参与新的回环检测
                mpLoopLastCurrentKF->SetCanErase();
                // 将当前关键帧作为上次关键帧
                mpLoopLastCurrentKF = mpCurrentKF;
                mg2oLoopSlw = gScw;  // 记录当前优化的结果为{last T_cw}即为 T_lw
                // 记录匹配到的点
                mvpLoopMatchedMPs = vpMatchedMPs;

                // 如果验证数大于等于3则为成功回环
                mbLoopDetected = mnLoopNumCoincidences >= 3;
                // 记录失败的时序校验数为0
                mnLoopNumNotFound = 0;
                if (mbLoopDetected) {
                    cout << "PR: Loop detected with Reffine Sim3" << endl;
                }
            } else {
                // 当前时序验证失败
                bLoopDetectedInKF = false;
                // 递增失败的时序验证次数
                mnLoopNumNotFound++;
                // 若果连续两帧时序验证失败则整个回环检测失败
                if (mnLoopNumNotFound > 2) {
                    // 失败后标记重置一些信息
                    mpLoopLastCurrentKF->SetCanErase();
                    mpLoopMatchedKF->SetCanErase();
                    mnLoopNumCoincidences = 0;
                    mvpLoopMatchedMPs.clear();
                    mvpLoopMPs.clear();
                    mnLoopNumNotFound = 0;
                }
            }
        }

        //Merge candidates
        bool bMergeDetectedInKF = false;
        if (mnMergeNumCoincidences > 0) {
            // Find from the last KF candidates
            // 通过上一关键帧的信息,计算新的当前帧的sim3
            // Tcl = Tcw*Twl
            Sophus::SE3d mTcl = (mpCurrentKF->GetPose() * mpMergeLastCurrentKF->GetPoseInverse()).cast<double>();

            g2o::Sim3 gScl(mTcl.unit_quaternion(), mTcl.translation(), 1.0);
            // mg2oMergeSlw 中的w指的是融合候选关键帧世界坐标系
            g2o::Sim3 gScw = gScl * mg2oMergeSlw;
            int numProjMatches = 0;
            vector<MapPoint *> vpMatchedMPs;

            // 通过把候选帧局部窗口内的地图点向新进来的关键帧投影来验证回环检测结果,并优化Sim3位姿
            bool bCommonRegion = DetectAndRefineSim3FromLastKF(mpCurrentKF, mpMergeMatchedKF, gScw, numProjMatches,
                                                               mvpMergeMPs, vpMatchedMPs);
            // 如果找到共同区域(时序验证成功一次)
            if (bCommonRegion) {
                // 标记时序检验成功一次
                bMergeDetectedInKF = true;
                // 成功验证的总次数+1
                mnMergeNumCoincidences++;
                // 不再参与新的回环检测
                mpMergeLastCurrentKF->SetCanErase();
                mpMergeLastCurrentKF = mpCurrentKF;
                mg2oMergeSlw = gScw;
                mvpMergeMatchedMPs = vpMatchedMPs;
                // 如果验证数大于等于3则为成功
                mbMergeDetected = mnMergeNumCoincidences >= 3;
            } else {
                mbMergeDetected = false;
                bMergeDetectedInKF = false;
                mnMergeNumNotFound++;
                // 若果连续两帧时序验证失败则整个融合检测失败
                if (mnMergeNumNotFound >= 2) {
                    // 失败后标记重置一些信息
                    mpMergeLastCurrentKF->SetCanErase();
                    mpMergeMatchedKF->SetCanErase();
                    mnMergeNumCoincidences = 0;
                    mvpMergeMatchedMPs.clear();
                    mvpMergeMPs.clear();
                    mnMergeNumNotFound = 0;
                }
            }
        }

        if (mbMergeDetected || mbLoopDetected) {
            mpKeyFrameDB->add(mpCurrentKF);
            return true;
        }

        vector<KeyFrame *> vpMergeBowCand, vpLoopBowCand;
        if (!bMergeDetectedInKF || !bLoopDetectedInKF) {
            mpKeyFrameDB->DetectNBestLoopAndMergeKFs(mpCurrentKF, vpLoopBowCand, vpMergeBowCand, 3);
        }

        // Check the BoW candidates if the geometric candidate list is not empty
        if (!bLoopDetectedInKF && !vpLoopBowCand.empty()) {
            mbLoopDetected = DetectCommonRegionsFromBoW(vpLoopBowCand, mpLoopMatchedKF, mpLoopLastCurrentKF,
                                                        mg2oLoopSlw, mnLoopNumCoincidences, mvpLoopMPs,
                                                        mvpLoopMatchedMPs);
        }
        if (!bMergeDetectedInKF && !vpMergeBowCand.empty()) {
            mbMergeDetected = DetectCommonRegionsFromBoW(vpMergeBowCand, mpMergeMatchedKF, mpMergeLastCurrentKF,
                                                         mg2oMergeSlw, mnMergeNumCoincidences, mvpMergeMPs,
                                                         mvpMergeMatchedMPs);
        }
        mpKeyFrameDB->add(mpCurrentKF);
        if (mbMergeDetected || mbLoopDetected) {
            return true;
        }
        mpCurrentKF->SetCanErase();
        return false;
    }

/**
 * @brief 对新进来的关键帧进行时序几何验证,同时继续优化之前估计的 Tcw
 *
 * @param[in] pCurrentKF 当前关键帧
 * @param[in] pCandidKF 候选帧
 * @param[out] gScw 世界坐标系在验证帧下的Sim3
 * @param[out] nNumProjMatches 记录匹配点的数量
 * @param[out] vpMPs 候选帧窗口内所有的地图点
 * @param[out] vpMatchedMPs 候选帧窗口内所有被匹配到的点
 * @return true 时序几何验证成功
 * @return false 时序几何验证失败
 */
    bool LoopClosing::DetectAndRefineSim3FromLastKF(KeyFrame *pCurrentKF, KeyFrame *pCandidKF, g2o::Sim3 &gScw,
                                                    int &nNumProjMatches, std::vector<MapPoint *> &vpMPs,
                                                    std::vector<MapPoint *> &vpMatchedMPs) {
        set<MapPoint *> spAlreadyMatchedMPs;
        nNumProjMatches = FindMatchesByProjection(pCurrentKF, pCandidKF, gScw, vpMPs, vpMatchedMPs);

        int nThProjMatches = 30;
        int nThProjOptMatches = 50;
        int nThProjMatchesRep = 100;

        // 2.点数如果不符合返回false
        if (nNumProjMatches >= nThProjMatches) {
            Verbose::PrintMess("Sim3 reffine: There are " + to_string(nNumProjMatches) + " initial matches ",
                               Verbose::VERBOSITY_DEBUG);
            Sophus::SE3d mTwm = pCandidKF->GetPoseInverse().cast<double>();
            g2o::Sim3 gSwm(mTwm.unit_quaternion(), mTwm.translation(), 1.0);
            g2o::Sim3 gScm = gScw * gSwm;
            Eigen::Matrix<double, 7, 7> mHessian7x7;
            int numOptMatches = Optimizer::OptimizeKFsSim3(mpCurrentKF, pCandidKF, vpMatchedMPs, gScm, 10, mbFixScale,
                                                           mHessian7x7, true);

            Verbose::PrintMess(
                    "Sim3 reffine: There are " + to_string(numOptMatches) + " matches after of the optimization ",
                    Verbose::VERBOSITY_DEBUG);
            if (numOptMatches > nThProjOptMatches) {
                g2o::Sim3 gScw_estimation((gScm * (gSwm.inverse())).rotation(), (gScm * (gSwm.inverse())).translation(),
                                          (gScm * (gSwm.inverse())).scale());
                vector<MapPoint *> vpMatchedMP;
                vpMatchedMP.resize(mpCurrentKF->GetVectorMapPointsInKF().size(), static_cast<MapPoint *>(NULL));
                nNumProjMatches = FindMatchesByProjection(pCurrentKF, pCandidKF, gScw_estimation, vpMPs, vpMatchedMPs);
                if (nNumProjMatches >= nThProjMatchesRep) {
                    gScw = gScw_estimation;
                    return true;
                }
            }
        }
        // 验证失败
        return false;
    }

/**
 * @brief 实现论文第8页的2-5步中的一部分功能(对后面新进来的关键帧的验证没有放在这个函数里进行)
 * 1. 构造局部窗口
 * 2. Ransac 得到 Scm的初始值
 * 3. guided matching refinement
 * 4. 利用地图中的共视关键帧验证(共视几何校验)
 *
 * @param[in] vpBowCandKFs bow 给出的一些候选关键帧
 * @param[out] pMatchedKF2 最后成功匹配的候选关键帧
 * @param[out] pLastCurrentKF 用于记录当前关键帧为上一个关键帧(后续若仍需要时序几何校验需要记录此信息)
 * @param[out] g2oScw 候选关键帧世界坐标系到当前关键帧的Sim3变换
 * @param[out] nNumCoincidences 成功几何验证的帧数，超过3就认为几何验证成功，不超过继续进行时序验证
 * @param[out] vpMPs  所有地图点
 * @param[out] vpMatchedMPs 成功匹配的地图点
 * @return true 检测到一个合格的共同区域
 * @return false 没检测到一个合格的共同区域
 */
    bool LoopClosing::DetectCommonRegionsFromBoW(
            std::vector<KeyFrame *> &vpBowCandKFs, KeyFrame *&pMatchedKF2, KeyFrame *&pLastCurrentKF, g2o::Sim3 &g2oScw,
            int &nNumCoincidences, std::vector<MapPoint *> &vpMPs, std::vector<MapPoint *> &vpMatchedMPs) {
        // 一些后面会使用的阀值
        int nThBoWMatches = 20; // 最低bow匹配特征点数
        int nThBoWInliers = 15; // RANSAC最低的匹配点数
        int nThSim3Inliers = 20; // sim3 最低内点数
        int nThProjMatches = 50; // 通过投影得到的匹配点数量最低阀值
        int nThProjOptMatches = 80; // 通过更小的半径,更严的距离搜索到的匹配点数量

        // 1. 获取当前帧的共视帧(在共同区域检测中应该避免当前关键帧的共视关键帧中)
        set<KeyFrame *> spCurKFConnectedKFs = mpCurrentKF->GetConnectedKeyFrames();

        // change 定义最佳共视关键帧的数量 0.4版本这里为5
        int nNumCovisibles = 10;


        ORBmatcher matcherByBoW(0.9, true);  // 用于search by bow
        ORBmatcher matcherByProjection(0.75, true);  // 用与seach by projection

        // Varibles to select the best numbe
        // 一些用于统计最优数据的变量,我们最后返回的是最佳的一个关键帧(几何校验匹配数最高的)
        KeyFrame *pBestMatchedKF;
        int nBestMatchesReproj = 0;
        int nBestNumCoindicendes = 0;
        g2o::Sim3 g2oBestScw;
        std::vector<MapPoint *> vpBestMapPoints;
        std::vector<MapPoint *> vpBestMatchedMapPoints;

        int nNumCandKFs = vpBowCandKFs.size();
        for (KeyFrame *pKFi: vpBowCandKFs) {
            if (!pKFi || pKFi->isBad()) {
                continue;
            }

            std::vector<KeyFrame *> vpCandCovKFi = pKFi->GetBestCovisibilityKeyFrames(nNumCovisibles);
            if (vpCandCovKFi.empty()) {
                std::cout << "Covisible list empty" << std::endl;
                vpCandCovKFi.emplace_back(pKFi);
            } else {
                vpCandCovKFi.emplace_back(vpCandCovKFi[0]);
                vpCandCovKFi[0] = pKFi;
            }

            bool bAbortByNearCurKF = false;
            for (int j = 0; j < vpCandCovKFi.size(); ++j) {
                if (spCurKFConnectedKFs.find(vpCandCovKFi[j]) != spCurKFConnectedKFs.end()) {
                    bAbortByNearCurKF = true;
                    break;
                }
            }
            if (bAbortByNearCurKF) {
                continue;
            }

            // search by bow 返回的参数, 记录窗口Wm中每个关键帧有哪些点能在当前关键帧Ka中通过bow找到匹配点
            std::vector<std::vector<MapPoint *> > vvpMatchedMPs;
            vvpMatchedMPs.resize(vpCandCovKFi.size());

            // 记录整个窗口中有那些点能在Ka中通过bow找到匹配点(这个set是辅助容器,避免重复添加地图点)
            std::set<MapPoint *> spMatchedMPi;
            int numBoWMatches = 0;

            // 记录窗口中能通过bow在当前关键帧ka中找到最多匹配点的关键帧
            KeyFrame *pMostBoWMatchesKF = pKFi;
            // 记录窗口中能通过bow在当前关键帧ka中找到最多匹配点的数量
            int nMostBoWNumMatches = 0;

            // 下面两个变量是为了sim3 solver准备的            // 标记是否因为窗口内有当前关键帧的共视关键帧
            //记录窗口中的地图点能在当前关键帧中找到的匹配的点(数量的上限是当前关键帧地图点的数量)
            std::vector<MapPoint *> vpMatchedPoints = std::vector<MapPoint *>(
                    mpCurrentKF->GetVectorMapPointsInKF().size(),
                    static_cast<MapPoint *>(NULL));
            // 记录上面的地图点分别对应窗口中的关键帧(数量的上限是当前关键帧地图点的数量)
            std::vector<KeyFrame *> vpKeyFrameMatchedMP = std::vector<KeyFrame *>(
                    mpCurrentKF->GetVectorMapPointsInKF().size(), static_cast<KeyFrame *>(NULL));


            // 2.3 通过Bow寻找候选帧窗口内的关键帧地图点与当前关键帧的匹配点
            for (int j = 0; j < vpCandCovKFi.size(); ++j) {
                if (!vpCandCovKFi[j] || vpCandCovKFi[j]->isBad())
                    continue;
                int num = matcherByBoW.SearchKFsByBoWInLoopClosing(mpCurrentKF, vpCandCovKFi[j], vvpMatchedMPs[j]);
                if (num > nMostBoWNumMatches) {
                    nMostBoWNumMatches = num;
                }
            }

            // 遍历窗口内的每个关键帧
            // 2.4 把窗口内的匹配点转换为Sim3Solver接口定义的格式
            for (int j = 0; j < vpCandCovKFi.size(); ++j) {
                // 遍历窗口内的某一个关键帧与当前关键帧由bow得到的匹配的地图点
                for (int k = 0; k < vvpMatchedMPs[j].size(); ++k) {
                    MapPoint *pMPi_j = vvpMatchedMPs[j][k];
                    if (!pMPi_j || pMPi_j->isBad())
                        continue;

                    if (spMatchedMPi.find(pMPi_j) == spMatchedMPi.end()) {
                        spMatchedMPi.insert(pMPi_j);
                        numBoWMatches++;
                        vpMatchedPoints[k] = pMPi_j;
                        vpKeyFrameMatchedMP[k] = vpCandCovKFi[j];
                    }
                }
            }
            // 1. 前面统计了vpCovKFi中每个帧与当前帧匹配点的位置，可否用点数高的代替
            // 2. 有可能作者认为在DetectNBestCandidates已经找到共视关键帧中分数最多的了，所以这里不做判断直接使用原始的候选关键帧
            //pMostBoWMatchesKF = vpCandCovKFi[pMostBoWMatchesKF];

            // 当窗口内的帧不是当前关键帧的相邻帧且匹配点足够多时
            // 3. 利用RANSAC寻找候选关键帧窗口与当前关键帧的相对位姿T_cm的初始值(可能是Sim3)
            // nThBoWMatches = 20; // 最低bow匹配特征点数
            if (numBoWMatches < nThBoWMatches) {
                continue;
            }
            Sim3Solver solver = Sim3Solver(mpCurrentKF, pMostBoWMatchesKF, vpMatchedPoints, mbFixScale,
                                           vpKeyFrameMatchedMP);
            solver.SetRansacParameters(0.99, nThBoWInliers, 300); // at least 15 inliers

            bool bNoMore = false;
            vector<bool> vbInliers;
            int nInliers;
            bool bConverge = false;
            Eigen::Matrix4f mTcm;
            // 3.2 迭代到收敛
            while (!bConverge && !bNoMore) {
                mTcm = solver.iterate(20, bNoMore, vbInliers, nInliers, bConverge);
            }

            // 3.3 Guide matching refinement: 利用初始的Scm信息,进行双向重投影,并非线性优化得到更精确的Scm
            if (!bConverge) {
                continue;
            }

            Verbose::PrintMess("BoW guess: Convergende with " + to_string(nInliers) + " geometrical inliers among " + to_string(nThBoWInliers) + " BoW matches", Verbose::VERBOSITY_DEBUG);
            // Match by reprojection
            vpCandCovKFi.clear();
            // 拿到窗口内匹配最多的帧的最佳10个共视帧和它自己组成的窗口
            vpCandCovKFi = pMostBoWMatchesKF->GetBestCovisibilityKeyFrames(nNumCovisibles);
            vpCandCovKFi.emplace_back(pMostBoWMatchesKF);
            // 辅助容器,避免重复添加地图点
            set<MapPoint *> spMapPoints;
            // 这两个容器是searchByProjection定义的容器
            // 记录窗口内地图点
            vector<MapPoint *> vpMapPoints;
            // 遍历窗Wm内的所有关键帧
            for (KeyFrame *pCovKFi: vpCandCovKFi) {
                // 遍历窗口内每个关键帧的所有地图点
                for (MapPoint *pCovMPij: pCovKFi->GetVectorMapPointsInKF()) {
                    // 如果指针为空或者改地图点被标记为bad
                    if (!pCovMPij || pCovMPij->isBad())
                        continue;
                    // 避免重复添加
                    if (spMapPoints.find(pCovMPij) == spMapPoints.end()) {
                        spMapPoints.insert(pCovMPij);
                        vpMapPoints.emplace_back(pCovMPij);
                    }
                }
            }


            // 拿到solver 估计的 Scm初始值, 为后续的非线性优化做准备, 在这里 c 表示当前关键帧, m 表示回环/融合候选帧
            g2o::Sim3 gScm(solver.GetEstimatedRotation().cast<double>(),
                           solver.GetEstimatedTranslation().cast<double>(),
                           (double) solver.GetEstimatedScale());
            // 候选关键帧在其世界坐标系下的坐标
            g2o::Sim3 gSmw(pMostBoWMatchesKF->GetRotation().cast<double>(),
                           pMostBoWMatchesKF->GetTranslation().cast<double>(), 1.0);
            // 利用初始的Scm估计确定世界坐标系在当前相机中的位姿
            g2o::Sim3 gScw = gScm * gSmw; // Similarity matrix of current from the world position
            // 准备用来SearchByProjection的位姿信息
            Sophus::Sim3f mScw = Converter::toSophus(gScw);

            // 记录最后searchByProjection的结果
            vector<MapPoint *> vpMatchedMP;
            vpMatchedMP.resize(mpCurrentKF->GetVectorMapPointsInKF().size(), static_cast<MapPoint *>(NULL));
            // 3.3.1 重新利用之前计算的mScw信息, 通过投影寻找更多的匹配点
            int numProjMatches = matcherByProjection.SearchKFAndMPsByProjectionInLC(mpCurrentKF, mScw,
                                                                                     vpMapPoints,
                                                                                     vpMatchedMP,
                                                                                     8, 1.5);
            //cout <<"BoW: " << numProjMatches << " matches between " << vpMapPoints.ParameterSize() << " points with coarse Sim3" << endl;

            // 如果拿到了足够多的匹配点, nThProjMatches = 50
            if (numProjMatches >= nThProjMatches) {
                // Optimize Sim3 transformation with every matches
                Eigen::Matrix<double, 7, 7> mHessian7x7;

                bool bFixedScale = mbFixScale;
                // 3.3.2 利用搜索到的更多的匹配点用Sim3优化投影误差得到的更好的 gScm
                // pKFi是候选关键帧
                int numOptMatches = Optimizer::OptimizeKFsSim3(mpCurrentKF, pKFi, vpMatchedMP, gScm, 10,
                                                               mbFixScale, mHessian7x7, true);

                // 3.3.3 如果内点足够多,用更小的半径搜索匹配点,并且再次进行优化(p.s.这里与论文不符,并没有再次优化)
                if (numOptMatches >= nThSim3Inliers) {
                    // 前面已经声明了这些变量了,无需再次声明
                    g2o::Sim3 gSmw(pMostBoWMatchesKF->GetRotation().cast<double>(),
                                   pMostBoWMatchesKF->GetTranslation().cast<double>(), 1.0);
                    g2o::Sim3 gScw = gScm * gSmw; // Similarity matrix of current from the world position
                    Sophus::Sim3f mScw = Converter::toSophus(gScw);

                    vector<MapPoint *> vpMatchedMP;
                    vpMatchedMP.resize(mpCurrentKF->GetVectorMapPointsInKF().size(),
                                       static_cast<MapPoint *>(NULL));
                    // 3.3.4 重新利用之前计算的mScw信息, 通过更小的半径和更严格的距离的投影寻找匹配点
                    // 5 : 半径的增益系数(对比之前下降了)---> 更小的半径, 1.0 , hamming distance 的阀值增益系数---> 允许更小的距离
                    int numProjOptMatches = matcherByProjection.SearchKFAndMPsByProjectionInLC(mpCurrentKF,
                                                                                               mScw,
                                                                                               vpMapPoints,
                                                                                               vpMatchedMP, 5,
                                                                                               1.0);

                    // 当新的投影得到的内点数量大于nProjOptMatches=80时
                    if (numProjOptMatches >= nThProjOptMatches) {
                        // 4. 用当前关键帧的相邻关键来验证前面得到的Tam(共视几何校验)
                        // 统计验证成功的关键帧数量
                        int nNumKFs = 0;
                        //vpMatchedMPs = vpMatchedMP;
                        //vpMPs = vpMapPoints;
                        // Check the Sim3 transformation with the current KeyFrame covisibles
                        // 4.1 拿到用来验证的关键帧组(后称为验证组): 当前关键帧的共视关键帧，nNumCovisibles = 5;
                        vector<KeyFrame *> vpCurrentCovKFs = mpCurrentKF->GetBestCovisibilityKeyFrames(
                                nNumCovisibles);

                        int j = 0;
                        // 遍历验证组当有三个关键帧验证成功或遍历所有的关键帧后结束循环
                        while (nNumKFs < 3 && j < vpCurrentCovKFs.size()) {
                            // 拿出验证组中的一个关键帧
                            KeyFrame *pKFj = vpCurrentCovKFs[j];
                            // 为 DetectCommonRegionsFromLastKF准备一个初始位姿, 这个用来进行searchByProjection
                            Sophus::SE3d mTjc = (pKFj->GetPose() *
                                                 mpCurrentKF->GetPoseInverse()).cast<double>();
                            g2o::Sim3 gSjc(mTjc.unit_quaternion(), mTjc.translation(), 1.0);
                            g2o::Sim3 gSjw = gSjc * gScw;
                            int numProjMatches_j = 0;
                            vector<MapPoint *> vpMatchedMPs_j;
                            // 4.2 几何校验函数, 这个函数里面其实是个searchByProjection : 通过之前计算的位姿转换地图点并通过投影搜索匹配点, 若大于一定数目的任务成功验证一次
                            bool bValid = VerifyCommonRegionsFromCovisKF(pKFj, pMostBoWMatchesKF, gSjw,
                                                                         numProjMatches_j, vpMapPoints,
                                                                         vpMatchedMPs_j);

                            // 统计valid的帧的数量
                            if (bValid) {
                                Sophus::SE3f Tc_w = mpCurrentKF->GetPose();
                                Sophus::SE3f Tw_cj = pKFj->GetPoseInverse();
                                Sophus::SE3f Tc_cj = Tc_w * Tw_cj;
                                Eigen::Vector3f vector_dist = Tc_cj.translation();
                                nNumKFs++;
                            }
                            j++;
                        }

                        // 记录第二次searchByProjection得到最多匹配点的关键帧的各种信息,最后作为回环帧/融合帧
                        if (nBestMatchesReproj < numProjOptMatches) {
                            nBestMatchesReproj = numProjOptMatches; // 投影匹配的数量
                            nBestNumCoindicendes = nNumKFs; // 成功验证的帧数
                            pBestMatchedKF = pMostBoWMatchesKF; // 记录候选帧窗口内与当前关键帧相似度最高的帧
                            g2oBestScw = gScw; // 记录最优的位姿(这个位姿是由Tam推到出来的 : Taw = Tam * Tmw,这里a表示c)
                            vpBestMapPoints = vpMapPoints; //  记录所有的地图点
                            vpBestMatchedMapPoints = vpMatchedMP; // 记录所有的地图点中被成功匹配的点
                        }
                    }
                }
            }
        }

        // 如果成功找到了共同区域帧把记录的最优值存到输出变量中
        if (nBestMatchesReproj > 0) {
            pLastCurrentKF = mpCurrentKF;
            nNumCoincidences = nBestNumCoindicendes;  // 成功几何验证的帧数
            pMatchedKF2 = pBestMatchedKF;
            pMatchedKF2->SetNotErase();
            g2oScw = g2oBestScw;
            vpMPs = vpBestMapPoints;
            vpMatchedMPs = vpBestMatchedMapPoints;
            // 如果有三个成功验证则return ture
            return nNumCoincidences >= 3;
        }
        // 如果少于3个当前关键帧的共视关键帧验证了这个候选帧,那么返回失败,注意,这里的失败并不代表最终的验证失败,后续会开启时序校验
        return false;
    }

/**
 * @brief 用来验证候选帧的函数, 这个函数的名字取的不好, 函数的本意是想利用候选帧的共视关键帧来验证候选帧,不如改叫做：DetectCommonRegionsFromCoVKF
 *
 * @param[in] pCurrentKF 当前关键帧
 * @param[in] pCandidKF 候选帧
 * @param[in] gScw 世界坐标系在验证帧下的Sim3
 * @param[out] nNumProjMatches 最后匹配的数目
 * @param[out] vpMPs 候选帧的窗口内所有的地图点
 * @param[out] vpMatchedMPs 候选帧的窗口内被匹配到的地图点
 * @return true 验证成功
 * @return false 验证失败
 */
    bool LoopClosing::VerifyCommonRegionsFromCovisKF(
            KeyFrame *pCurrentKF, KeyFrame *pCandidKF, g2o::Sim3 &gScw, int &nNumProjMatches,
            std::vector<MapPoint *> &vpMPs, std::vector<MapPoint *> &vpMatchedMPs) {
        // 所有的匹配点
        set<MapPoint *> spAlreadyMatchedMPs(vpMatchedMPs.begin(), vpMatchedMPs.end());
        // 通过Sim3变换后投影寻找匹配点
        nNumProjMatches = FindMatchesByProjection(pCurrentKF, pCandidKF, gScw, vpMPs, vpMatchedMPs);

        int nProjMatches = 30;
        // 如果匹配点数目大于一定的阀值,则认为验证成功,返回ture
        if (nNumProjMatches >= nProjMatches) {
            return true;
        }

        return false;
    }

/**
 * @brief 包装了一下searchByProjection, 把窗口内的所有地图点往当前关键帧上投影, 寻找匹配点
 *
 * @param[in] pCurrentKF 当前关键帧
 * @param[in] pCandidKFw 候选帧
 * @param[in] g2oScw 世界坐标系在验证帧坐标系下的位姿
 * @param[in] vpCandidMPs 候选帧及其共视关键帧组成的窗口里所有的地图点
 * @param[in] vpMatchedMPs 候选帧及其共视关键帧组成的窗口里所有被匹配上的地图点
 * @return int 匹配点的数量
 */
    int LoopClosing::FindMatchesByProjection(
            KeyFrame *pCurrentKF, KeyFrame *pCandidKFw, g2o::Sim3 &g2oScw,
            vector<MapPoint *> &vpCandidMPs,
            vector<MapPoint *> &vpMatchedMPs) {
        int nCurCovKFs = 10;  // change 上个版本为5
        // 拿出候选帧的10个最好的共视关键帧
        vector<KeyFrame *> vpCandidCovKFs = pCandidKFw->GetBestCovisibilityKeyFrames(nCurCovKFs);
        int nCandidCovKFs = vpCandidCovKFs.size();
        // 把自己也加进去, 组成一个局部窗口
        vpCandidCovKFs.emplace_back(pCandidKFw);

        // 辅助容器,防止重复添加
        set<KeyFrame *> spCandidKFs(vpCandidCovKFs.begin(), vpCandidCovKFs.end());
        set<KeyFrame *> spCurCovKFs = pCurrentKF->GetConnectedKeyFrames();

        // 1. 如果数量不够 扩充窗口
        if (nCandidCovKFs < nCurCovKFs) {
            for (int i = 0; i < nCandidCovKFs; ++i) {
                vector<KeyFrame *> vpKFs = vpCandidCovKFs[i]->GetBestCovisibilityKeyFrames(nCurCovKFs);
                int nInserted = 0;
                int j = 0;
                while (j < vpKFs.size() && nInserted < nCurCovKFs) {
                    // 如果没有被重复添加且不是当前关键帧的共视关键帧
                    if (spCandidKFs.find(vpKFs[j]) == spCandidKFs.end() &&
                        spCurCovKFs.find(vpKFs[j]) == spCurCovKFs.end()) {
                        spCandidKFs.insert(vpKFs[j]);
                        ++nInserted;
                    }
                    ++j;
                }
                // 把每个帧的共视关键帧都加到窗口内
                vpCandidCovKFs.insert(vpCandidCovKFs.end(), spCandidKFs.begin(), spCandidKFs.end());
            }
        }

        set<MapPoint *> spCandidCovMPs;
        vpCandidMPs.clear();
        vpMatchedMPs.clear();

        for (KeyFrame *pKFi: vpCandidCovKFs) {
            for (MapPoint *pMPij: pKFi->GetVectorMapPointsInKF()) {
                if (!pMPij || pMPij->isBad())
                    continue;

                if (spCandidCovMPs.find(pMPij) == spCandidCovMPs.end()) {
                    spCandidCovMPs.insert(pMPij);
                    vpCandidMPs.emplace_back(pMPij);
                }
            }
        }

        Sophus::Sim3f mScw = Converter::toSophus(g2oScw);
        ORBmatcher matcher(0.9, true);
        vpMatchedMPs.resize(pCurrentKF->GetVectorMapPointsInKF().size(), static_cast<MapPoint *>(NULL));
        int nMatchedMPs = matcher.SearchKFAndMPsByProjectionInLC(pCurrentKF, mScw, vpCandidMPs, vpMatchedMPs, 3,
                                                                 1.5);
        return nMatchedMPs;
    }

/**
 * @brief 相同地图检测到共同区域叫回环，不同地图叫融合，这个函数是在检测到回环后进行修正优化位姿
 */
    void LoopClosing::CorrectLoop() {
        //cout << "Loop detected!" << endl;

        // Send a stop signal to Local Mapping
        // Avoid new keyframes are inserted while correcting the loop
        // Step1. 结束局部地图线程、全局BA，为闭环矫正做准备
        // 请求局部地图停止，防止在回环矫正时局部地图线程中InsertKeyFrame函数插入新的关键帧
        mpLocalMapper->RequestPause();
        mpLocalMapper->EmptyQueue(); // Proccess keyframes in the queue

        // 如果正在进行全局BA，丢弃它
        // If a Global Bundle Adjustment is running, abort it
        if (CheckRunningGBA()) {
            cout << "Stoping Global Bundle Adjustment...";
            unique_lock<mutex> lock(mMutexGBA);
            mbStopGBA = true;
            // 记录全局BA次数
            mnFullBAIdx++;
            if (mpThreadGBA) {
                mpThreadGBA->detach();
                delete mpThreadGBA;
            }
            cout << "  Done!!" << endl;
        }

        // Wait until Local Mapping has effectively stopped
        // 一直等到局部地图线程结束再继续
        while (!mpLocalMapper->CheckPaused()) {
            usleep(5000);
        }

        // Ensure current keyframe is updated
        //cout << "Start updating connections" << endl;
        //assert(mpCurrentKF->GetMap()->CheckEssentialGraph());
        // Step 2. 根据共视关系更新当前关键帧与其它关键帧之间的连接关系
        // 因为之前闭环检测、计算Sim3中改变了该关键帧的地图点，所以需要更新
        mpCurrentKF->UpdateCovisGraph();
        //assert(mpCurrentKF->GetMap()->CheckEssentialGraph());

        // Retrive keyframes connected to the current keyframe and compute corrected Sim3 pose by propagation
        // Step 3. 通过位姿传播，得到Sim3优化后，与当前帧相连的关键帧的位姿，以及它们的地图点
        // 当前帧与世界坐标系之间的Sim变换在ComputeSim3函数中已经确定并优化，
        // 通过相对位姿关系，可以确定这些相连的关键帧与世界坐标系之间的Sim3变换
        // 取出当前关键帧及其共视关键帧，称为“当前关键帧组”
        mvpCurrentConnectedKFs = mpCurrentKF->GetVectorCovisibleKeyFrames();
        mvpCurrentConnectedKFs.emplace_back(mpCurrentKF);

        //std::cout << "Loop: number of connected KFs -> " + to_string(mvpCurrentConnectedKFs.ParameterSize()) << std::endl;
        // CorrectedSim3：存放闭环g2o优化后当前关键帧的共视关键帧的世界坐标系下Sim3 变换
        // NonCorrectedSim3：存放没有矫正的当前关键帧的共视关键帧的世界坐标系下Sim3 变换
        KeyFrameAndPose CorrectedSim3, NonCorrectedSim3;
        // 先将mpCurrentKF的Sim3变换存入，认为是准的，所以固定不动
        CorrectedSim3[mpCurrentKF] = mg2oLoopScw;
        // 当前关键帧到世界坐标系下的变换矩阵
        Sophus::SE3f Twc = mpCurrentKF->GetPoseInverse();
        Sophus::SE3f Tcw = mpCurrentKF->GetPose();
        g2o::Sim3 g2oScw(Tcw.unit_quaternion().cast<double>(), Tcw.translation().cast<double>(), 1.0);
        NonCorrectedSim3[mpCurrentKF] = g2oScw;

        // Update6DoF keyframe pose with corrected Sim3. First transform Sim3 to SE3 (scale translation)
        Sophus::SE3d correctedTcw(mg2oLoopScw.rotation(), mg2oLoopScw.translation() / mg2oLoopScw.scale());
        mpCurrentKF->SetPose(correctedTcw.cast<float>());
        Map *pLoopMap = mpCurrentKF->GetMap();

        // 对地图点操作
        {
            // Get Map Mutex
            unique_lock<mutex> lock(pLoopMap->mMutexMapUpdate);

            const bool bImuInit = pLoopMap->GetImuInitialized();
            // 3.1：通过mg2oLoopScw（认为是准的）来进行位姿传播，得到当前关键帧的共视关键帧的世界坐标系下Sim3 位姿（还没有修正）
            // 遍历"当前关键帧组""
            for (vector<KeyFrame *>::iterator vit = mvpCurrentConnectedKFs.begin(), vend = mvpCurrentConnectedKFs.end();
                 vit != vend; vit++) {
                KeyFrame *pKFi = *vit;
                if (pKFi != mpCurrentKF)  // 跳过当前关键帧，因为当前关键帧的位姿已经在前面优化过了，在这里是参考基准
                {
                    // 得到当前关键帧 mpCurrentKF 到其共视关键帧 pKFi 的相对变换
                    Sophus::SE3f Tiw = pKFi->GetPose();
                    Sophus::SE3d Tic = (Tiw * Twc).cast<double>();

                    // g2oSic：当前关键帧 mpCurrentKF 到其共视关键帧 pKFi 的Sim3 相对变换
                    // 这里是non-correct, 所以scale=1.0
                    g2o::Sim3 g2oSic(Tic.unit_quaternion(), Tic.translation(), 1.0);
                    // 当前帧的位姿固定不动，其它的关键帧根据相对关系得到Sim3调整的位姿
                    g2o::Sim3 g2oCorrectedSiw = g2oSic * mg2oLoopScw;
                    //Pose corrected with the Sim3 of the loop closure
                    // 存放闭环g2o优化后当前关键帧的共视关键帧的Sim3 位姿
                    CorrectedSim3[pKFi] = g2oCorrectedSiw;
                    // Update6DoF keyframe pose with corrected Sim3. First transform Sim3 to SE3 (scale translation)
                    Sophus::SE3d correctedTiw(g2oCorrectedSiw.rotation(),
                                              g2oCorrectedSiw.translation() / g2oCorrectedSiw.scale());
                    pKFi->SetPose(correctedTiw.cast<float>());
                    //Pose without correction
                    g2o::Sim3 g2oSiw(Tiw.unit_quaternion().cast<double>(), Tiw.translation().cast<double>(), 1.0);
                    NonCorrectedSim3[pKFi] = g2oSiw;
                }
            }

            // Correct all MapPoints obsrved by current keyframe and neighbors, so that they align with the other side of the loop
            // 3.2：得到矫正的当前关键帧的共视关键帧位姿后，修正这些关键帧的地图点
            // 遍历待矫正的共视关键帧（不包括当前关键帧）
            for (KeyFrameAndPose::iterator mit = CorrectedSim3.begin(), mend = CorrectedSim3.end();
                 mit != mend; mit++) {
                KeyFrame *pKFi = mit->first;
                g2o::Sim3 g2oCorrectedSiw = mit->second;
                g2o::Sim3 g2oCorrectedSwi = g2oCorrectedSiw.inverse();
                g2o::Sim3 g2oSiw = NonCorrectedSim3[pKFi];

                // Update6DoF keyframe pose with corrected Sim3. First transform Sim3 to SE3 (scale translation)
                /*Sophus::SE3d correctedTiw(g2oCorrectedSiw.rotation(),g2oCorrectedSiw.translation() / g2oCorrectedSiw.scale());
            pKFi->SetPose(correctedTiw.cast<float>());*/
                vector<MapPoint *> vpMPsi = pKFi->GetVectorMapPointsInKF();
                // 遍历待矫正共视关键帧中的每一个地图点
                for (size_t iMP = 0, endMPi = vpMPsi.size(); iMP < endMPi; iMP++) {
                    MapPoint *pMPi = vpMPsi[iMP];
                    if (!pMPi)
                        continue;
                    if (pMPi->isBad())
                        continue;
                    if (pMPi->mnCorrectedByKF == mpCurrentKF->mnId)  // 标记，防止重复矫正
                        continue;
                    // ProjectMono with non-corrected pose and ProjectMPToKP back with corrected pose
                    // 矫正过程本质上也是基于当前关键帧的优化后的位姿展开的
                    // 将该未校正的eigP3Dw先从世界坐标系映射到未校正的pKFi相机坐标系，然后再反映射到校正后的世界坐标系下
                    Eigen::Vector3d P3Dw = pMPi->GetWorldPos().cast<double>();
                    // map(P) 内部做了变换 R*P +mTs
                    // 下面变换是：eigP3Dw： world →g2oSiw→ i →g2oCorrectedSwi→ world
                    Eigen::Vector3d eigCorrectedP3Dw = g2oCorrectedSwi.map(g2oSiw.map(P3Dw));

                    pMPi->SetWorldPos(eigCorrectedP3Dw.cast<float>());
                    // 记录矫正该地图点的关键帧id，防止重复
                    pMPi->mnCorrectedByKF = mpCurrentKF->mnId;
                    // 记录该地图点所在的关键帧id
                    pMPi->mnCorrectedReference = pKFi->mnId;
                    // 因为地图点更新了，需要更新其平均观测方向以及观测距离范围
                    pMPi->UpdateNormalAndDepth();
                }
                // 修改速度
                // Correct velocity according to orientation correction
                if (bImuInit) {
                    Eigen::Quaternionf Rcor = (g2oCorrectedSiw.rotation().inverse() * g2oSiw.rotation()).cast<float>();
                    pKFi->SetVelocity(Rcor * pKFi->GetVelocity());
                }

                // Make sure connections are updated
                // 3.3 根据共视关系更新当前帧与其它关键帧之间的连接
                // 地图点的位置改变了,可能会引起共视关系\权值的改变
                pKFi->UpdateCovisGraph();
            }
            // TODO Check this index increasement
            mpAtlas->GetCurrentMap()->IncreaseChangeIdx();

            // Start Loop Fusion
            // Update6DoF matched map points and replace if duplicated
            // Step 4. 检查当前帧的地图点与经过闭环匹配后该帧的地图点是否存在冲突，对冲突的进行替换或填补
            // mvpCurrentMatchedPoints 是当前关键帧和闭环关键帧组的所有地图点进行投影得到的匹配点
            for (size_t i = 0; i < mvpLoopMatchedMPs.size(); i++) {
                if (mvpLoopMatchedMPs[i]) {
                    // 取出同一个索引对应的两种地图点，决定是否要替换
                    // 匹配投影得到的地图点
                    MapPoint *pLoopMP = mvpLoopMatchedMPs[i];
                    // 原来的地图点
                    MapPoint *pCurMP = mpCurrentKF->GetIdxMapPoint(i);
                    // 如果有重复的MapPoint，则用匹配的地图点代替现有的
                    // 因为匹配的地图点是经过一系列操作后比较精确的，现有的地图点很可能有累计误差
                    if (pCurMP)
                        pCurMP->Replace(pLoopMP);
                    else {
                        // 如果当前帧没有该MapPoint，则直接添加
                        mpCurrentKF->AddMapPoint(pLoopMP, i);
                        pLoopMP->AddObsKFAndLRIdx(mpCurrentKF, i);
                        pLoopMP->ComputeDistinctiveDescriptors();
                    }
                }
            }
            //cout << "LC: end replacing duplicated" << endl;
        }

        // ProjectMono MapPoints observed in the neighborhood of the loop keyframe
        // into the current keyframe and neighbors using corrected poses.
        // SearchKFAndMapPointsByProjection duplications.
        // Step 5. 将闭环相连关键帧组mvpLoopMapPoints 投影到当前关键帧组中，进行匹配，融合，新增或替换当前关键帧组中KF的地图点
        // 因为 闭环相连关键帧组mvpLoopMapPoints 在地图中时间比较久经历了多次优化，认为是准确的
        // 而当前关键帧组中的关键帧的地图点是最近新计算的，可能有累积误差
        // CorrectedSim3：存放矫正后当前关键帧的共视关键帧，及其世界坐标系下Sim3 变换
        SearchAndFuse(CorrectedSim3, mvpLoopMapPoints);
        // After the MapPoint fusion, new links in the covisibility graph will appear attaching mbFrameBoth sides of the loop
        // Step 6. 更新当前关键帧之间的共视相连关系，得到因闭环时MapPoints融合而新得到的连接关系
        // LoopConnections：存储因为闭环地图点调整而新生成的连接关系
        map<KeyFrame *, set<KeyFrame *> > LoopConnections;

        // 6.1 遍历当前帧相连关键帧组（一级相连）
        for (vector<KeyFrame *>::iterator vit = mvpCurrentConnectedKFs.begin(), vend = mvpCurrentConnectedKFs.end();
             vit != vend; vit++) {
            KeyFrame *pKFi = *vit;
            // 6.2 得到与当前帧相连关键帧的相连关键帧（二级相连）
            vector<KeyFrame *> vpPreviousNeighbors = pKFi->GetVectorCovisibleKeyFrames();
            // Update6DoF connections. Detect new links.
            // 6.3 更新一级相连关键帧的连接关系(会把当前关键帧添加进去,因为地图点已经更新和替换了)
            pKFi->UpdateCovisGraph();
            // 6.4 取出该帧更新后的连接关系
            LoopConnections[pKFi] = pKFi->GetConnectedKeyFrames();
            // 6.5 从连接关系中去除闭环之前的二级连接关系，剩下的连接就是由闭环得到的连接关系
            for (vector<KeyFrame *>::iterator vit_prev = vpPreviousNeighbors.begin(), vend_prev = vpPreviousNeighbors.end();
                 vit_prev != vend_prev; vit_prev++) {
                LoopConnections[pKFi].erase(*vit_prev);
            }
            // 6.6 从连接关系中去除闭环之前的一级连接关系，剩下的连接就是由闭环得到的连接关系
            for (vector<KeyFrame *>::iterator vit2 = mvpCurrentConnectedKFs.begin(), vend2 = mvpCurrentConnectedKFs.end();
                 vit2 != vend2; vit2++) {
                LoopConnections[pKFi].erase(*vit2);
            }
        }
        // Optimize graph
        bool bFixedScale = mbFixScale;
        // TODO CHECK; Solo para el monocular inertial
        //cout << "Optimize essential graph" << endl;
        if (pLoopMap->GetImuInitialized()) {
            Optimizer::OptimizeEssentialGraph4DoF(pLoopMap, mpLoopMatchedKF, mpCurrentKF, NonCorrectedSim3,
                                                  CorrectedSim3, LoopConnections);
        } else {
            //cout << "Loop -> Scale correction: " << mg2oLoopScw.scale() << endl;
            // Step 7. 进行EssentialGraph优化，LoopConnections是形成闭环后新生成的连接关系，不包括步骤7中当前帧与闭环匹配帧之间的连接关系
            Optimizer::OptimizeEssentialGraph(pLoopMap, mpLoopMatchedKF, mpCurrentKF, NonCorrectedSim3, CorrectedSim3,
                                              LoopConnections, bFixedScale);
        }
        mpAtlas->InformNewBigChange();
        // Add loop edge
        // Step 7：添加当前帧与闭环匹配帧之间的边（这个连接关系不优化）
        // 它在下一次的Essential Graph里面使用
        mpLoopMatchedKF->AddLoopEdge(mpCurrentKF);
        mpCurrentKF->AddLoopEdge(mpLoopMatchedKF);

        // Launch a new thread to perform Global Bundle Adjustment (Only if few keyframes, if not it would take too much time)
        // 闭环地图没有imu初始化或者 仅有一个地图且内部关键帧<200时才执行全局BA，否则太慢
        if (!pLoopMap->GetImuInitialized() || (pLoopMap->GetKeyFramesNumInMap() < 200 && mpAtlas->CountMaps() == 1)) {
            // Step 9. 新建一个线程用于全局BA优化
            // OptimizeEssentialGraph只是优化了一些主要关键帧的位姿，这里进行全局BA可以全局优化所有位姿和MapPoints
            mbRunningGBA = true;
            mbFinishedGBA = false;
            mbStopGBA = false;
            mnCorrectionGBA = mnNumCorrection;
            mpThreadGBA = new thread(&LoopClosing::RunGlobalBundleAdjustment, this, pLoopMap, mpCurrentKF->mnId);
        }
        // Loop closed. CancelPause Local Mapping.
        mpLocalMapper->CancelPause();
    }

/**
 * @brief 纯视觉地图融合。在检测到成功验证的融合帧后进行
 * 1. 焊缝区域局部BA
 * 2. Essential Graph BA
 * 融合两张地图为一张地图
 */
    void LoopClosing::MergeLocalWithoutImu() {
        // 窗口内共视关键帧的数量， 上个0.4版本是15
        int numTemporalKFs = 25; //Temporal KFs in the local window if the map is inertial.

        //Relationship to rebuild the essential graph, it is used two times, first in the local window and later in the rest of the map
        // 建立本质图必须的量
        KeyFrame *pNewChild;
        KeyFrame *pNewParent;

        // 当前关键帧的窗口
        vector<KeyFrame *> vpLocalCurrentWindowKFs;
        // 候选关键帧的窗口
        vector<KeyFrame *> vpMergeConnectedKFs;

        // Flag that is true only when we stopped a running BA, in this case we need relaunch at the end of the merge
        // 记录是否把全局BA停下
        bool bRelaunchBA = false;

        //Verbose::PrintMess("MERGE-VISUAL: Check Full Bundle Adjustment", Verbose::VERBOSITY_DEBUG);
        // If a Global Bundle Adjustment is running, abort it
        if (CheckRunningGBA()) {
            unique_lock<mutex> lock(mMutexGBA);
            mbStopGBA = true;

            mnFullBAIdx++;

            if (mpThreadGBA) {
                mpThreadGBA->detach();
                delete mpThreadGBA;
            }
            bRelaunchBA = true;  // 以后还会重新开启
        }

        //Verbose::PrintMess("MERGE-VISUAL: Request CheckRequestReset Local Mapping", Verbose::VERBOSITY_DEBUG);
        //cout << "Request CheckRequestReset Local Mapping" << endl;
        // 请求局部建图线程停止
        mpLocalMapper->RequestPause();
        // Wait until Local Mapping has effectively stopped
        // 等待局部建图工作停止
        while (!mpLocalMapper->CheckPaused()) {
            usleep(5000);
        }
        //cout << "Local Map stopped" << endl;

        mpLocalMapper->EmptyQueue();

        // Merge map will become in the new active map with the local window of KFs and MPs from the current map.
        // Later, the elements of the current map will be transform to the new active map reference, in order to keep real time tracking
        // 当前关键帧的共视关键帧和他们观测到的地图点会先被融合, 融合后的图会变成新的当前激活地图.
        // 之后, 所有当前地图的其他部分会被转换到当前地图中, 这样是为了保证tracking的实时性.

        // 当前关键帧地图的指针
        Map *pCurrentMap = mpCurrentKF->GetMap();
        // 融合关键帧地图的指针
        Map *pMergeMap = mpMergeMatchedKF->GetMap();

        //std::cout << "Merge local, Active map: " << pCurrentMap->GetId() << std::endl;
        //std::cout << "Merge local, Non-Active map: " << pMergeMap->GetId() << std::endl;

        // Ensure current keyframe is updated
        // 先保证当前关键帧的链接关系是最新的
        mpCurrentKF->UpdateCovisGraph();

        // Step 1 构建当前关键帧和融合关键帧的局部窗口(关键帧+地图点)
        //Get the current KF and its neighbors(visual->covisibles; inertial->temporal+covisibles)
        // 当前关键帧的局部窗口(仅是辅助容器,防止重复添加)
        set<KeyFrame *> spLocalWindowKFs;
        //Get MPs in the welding area from the current map
        // 当前关键帧局部串口能观测到的所有地图点(仅是辅助容器,防止重复添加)
        set<MapPoint *> spLocalWindowMPs;

        // 这段代码只在visual状态下才会被使用，所以只会执行else
        // Step 1.1 构造当前关键帧局部共视帧窗口
        KeyFrame *pKFi = mpCurrentKF;
        int nInserted = 0;
        while (pKFi && nInserted < numTemporalKFs) {
            spLocalWindowKFs.insert(pKFi);
            pKFi = mpCurrentKF->mPrevKF;
            nInserted++;

            set<MapPoint *> spMPi = pKFi->GetMapPoints();
            spLocalWindowMPs.insert(spMPi.begin(), spMPi.end());
        }

        pKFi = mpCurrentKF->mNextKF;
        while (pKFi)  //! 这里会死循环,不过无所谓，这个外面的if永远不会执行
        {
            spLocalWindowKFs.insert(pKFi);

            set<MapPoint *> spMPi = pKFi->GetMapPoints();
            spLocalWindowMPs.insert(spMPi.begin(), spMPi.end());

            pKFi = mpCurrentKF->mNextKF;
        }

        // 拿到当前关键帧的numTemporalKFs(15)个最佳共视关键帧
        vector<KeyFrame *> vpCovisibleKFs = mpCurrentKF->GetBestCovisibilityKeyFrames(numTemporalKFs);
        // 把当前帧的共视帧都加到窗口里
        spLocalWindowKFs.insert(vpCovisibleKFs.begin(), vpCovisibleKFs.end());
        // 1.0版本新加的，将当前关键帧也添加进来
        spLocalWindowKFs.insert(mpCurrentKF);

        // 限制while循环次数 0.4版本是3
        const int nMaxTries = 5;
        int nNumTries = 0;

        // 如果窗口里的关键帧数量不够就再拉一些窗口里的关键帧的共视关键帧(二级共视关键帧)进来
        while (spLocalWindowKFs.size() < numTemporalKFs && nNumTries < nMaxTries) {
            vector<KeyFrame *> vpNewCovKFs;
            vpNewCovKFs.empty();
            // 遍历创口内的每一个关键帧
            for (KeyFrame *pKFi: spLocalWindowKFs) {
                // 拿到一些二级共视关键帧
                vector<KeyFrame *> vpKFiCov = pKFi->GetBestCovisibilityKeyFrames(numTemporalKFs / 2);

                // 对于每个二级共视关键帧
                for (KeyFrame *pKFcov: vpKFiCov) {
                    // 如果指针不为空,且关键帧没有被标记为bad,且没有被添加过则加到窗口内
                    if (pKFcov && !pKFcov->isBad() && spLocalWindowKFs.find(pKFcov) == spLocalWindowKFs.end()) {
                        vpNewCovKFs.emplace_back(pKFcov);
                    }

                }
            }
            // 用set记录,防止重复添加
            spLocalWindowKFs.insert(vpNewCovKFs.begin(), vpNewCovKFs.end());
            // 递增循环次数
            nNumTries++;
        }

        // Step 1.2 把当前关键帧窗口里关键帧观测到的地图点加进来
        for (KeyFrame *pKFi: spLocalWindowKFs) {
            if (!pKFi || pKFi->isBad())
                continue;

            set<MapPoint *> spMPs = pKFi->GetMapPoints();
            spLocalWindowMPs.insert(spMPs.begin(), spMPs.end());
        }

        //std::cout << "[Merge]: Ma = " << to_string(pCurrentMap->GetId()) << "; #KFs = " << to_string(spLocalWindowKFs.size()) << "; #MPs = " << to_string(spLocalWindowMPs.ParameterSize()) << std::endl;

        // Step 1.3 构造融合帧的共视帧窗口
        // 融合关键帧的共视关键帧们
        set<KeyFrame *> spMergeConnectedKFs;
        {
            KeyFrame *pKFi = mpMergeMatchedKF;
            int nInserted = 0;
            while (pKFi && nInserted < numTemporalKFs / 2) {
                spMergeConnectedKFs.insert(pKFi);
                pKFi = mpCurrentKF->mPrevKF;
                nInserted++;
            }
        }

        pKFi = mpMergeMatchedKF->mNextKF;
        while (pKFi && nInserted < numTemporalKFs) {
            spMergeConnectedKFs.insert(pKFi);
            pKFi = mpCurrentKF->mNextKF;
        }
        // 拿到融合关键帧最好的numTemporalKFs(25)个最佳共视关键帧
        vpCovisibleKFs = mpMergeMatchedKF->GetBestCovisibilityKeyFrames(numTemporalKFs);
        spMergeConnectedKFs.insert(vpCovisibleKFs.begin(), vpCovisibleKFs.end());
        spMergeConnectedKFs.insert(mpMergeMatchedKF);
        // 记录循环次数
        nNumTries = 0;
        // 如果融合关键帧窗口里的关键帧不够就再拉一些窗口里的关键帧的共视帧进来(二级共视关键帧)
        while (spMergeConnectedKFs.size() < numTemporalKFs && nNumTries < nMaxTries) {
            vector<KeyFrame *> vpNewCovKFs;
            // 遍历融合关键帧内的所有的一级共视关键帧
            for (KeyFrame *pKFi: spMergeConnectedKFs) {
                // 拿到一些二级共视关键帧
                vector<KeyFrame *> vpKFiCov = pKFi->GetBestCovisibilityKeyFrames(numTemporalKFs / 2);
                // 对于每个二级共视关键帧
                for (KeyFrame *pKFcov: vpKFiCov) {
                    // 如果指针不为空,且关键帧没有被标记为bad,且没有被添加过则加到窗口内
                    if (pKFcov && !pKFcov->isBad() && spMergeConnectedKFs.find(pKFcov) == spMergeConnectedKFs.end()) {
                        vpNewCovKFs.emplace_back(pKFcov);
                    }

                }
            }
            // 用set记录,防止重复添加
            spMergeConnectedKFs.insert(vpNewCovKFs.begin(), vpNewCovKFs.end());
            // 递增循环次数
            nNumTries++;
        }

        // Step 1.4  把融合关键帧窗口里关键帧观测到的地图点加进来
        // 融合关键帧共视窗口内的所有地图点
        set<MapPoint *> spMapPointMerge;
        for (KeyFrame *pKFi: spMergeConnectedKFs) {
            set<MapPoint *> vpMPs = pKFi->GetMapPoints();
            spMapPointMerge.insert(vpMPs.begin(), vpMPs.end());
        }

        // 把融合关键帧窗口内的地图点加到待融合的向量中
        vector<MapPoint *> vpCheckFuseMapPoint;
        vpCheckFuseMapPoint.reserve(spMapPointMerge.size());
        // 把spMapPointMerge拷贝到vpCheckFuseMapPoint里
        std::copy(spMapPointMerge.begin(), spMapPointMerge.end(), std::back_inserter(vpCheckFuseMapPoint));

        //std::cout << "[Merge]: Mm = " << to_string(pMergeMap->GetId()) << "; #KFs = " << to_string(spMergeConnectedKFs.size()) << "; #MPs = " << to_string(spMapPointMerge.ParameterSize()) << std::endl;


        // Step 2 根据之前的Sim3初始值, 记录当前帧窗口内关键帧,地图点的矫正前的值,和矫正后的初始值

        // Step 2.1 先计算当前关键帧矫正前的值,和矫正后的初始值
        Sophus::SE3d Twc = mpCurrentKF->GetPoseInverse().cast<double>();

        // 记录没有融合矫正之前的Swc和Scw
        g2o::Sim3 g2oNonCorrectedSwc(Twc.unit_quaternion(), Twc.translation(), 1.0);
        g2o::Sim3 g2oNonCorrectedScw = g2oNonCorrectedSwc.inverse();

        // 拿到之前通过Sim3(见 NewDetectCommonRegion)计算得到的当前关键帧融合矫正之后的初始位姿
        // mg2oMergeScw存放的是融合关键帧所在的世界坐标系到当前帧的Sim3位姿
        g2o::Sim3 g2oCorrectedScw = mg2oMergeScw; //TODO Check the transformation

        // 记录当前关键帧窗口内所有关键帧融合矫正之前的位姿,和融合矫正之后的初始位姿
        KeyFrameAndPose vCorrectedSim3, vNonCorrectedSim3;
        // g2oNonCorrectedScw 是当前关键帧世界坐标系下的
        // g2oCorrectedScw 是融合关键帧所在的世界坐标系下的
        vCorrectedSim3[mpCurrentKF] = g2oCorrectedScw;
        vNonCorrectedSim3[mpCurrentKF] = g2oNonCorrectedScw;

        // Step 2.2 通过当前关键帧融合矫正前后的位姿,把当前关键帧的共视窗口里面剩余的关键帧矫正前后的位姿都给填写上
        // 对于每个当前关键帧共视窗口里的关键帧
        for (KeyFrame *pKFi: spLocalWindowKFs) {
            // 跳过空的指针,或者坏的关键帧
            if (!pKFi || pKFi->isBad()) {
                Verbose::PrintMess("Bad KF in correction", Verbose::VERBOSITY_DEBUG);
                continue;
            }

            if (pKFi->GetMap() != pCurrentMap)
                Verbose::PrintMess("Other map KF, this should'mTs happen", Verbose::VERBOSITY_DEBUG);

            // 确保这些共视关键帧在当前地图下
            // 保存第i个共视关键帧融合矫正后的初始位姿
            g2o::Sim3 g2oCorrectedSiw;

            if (pKFi != mpCurrentKF) {
                // 先记录下融合矫正前的位姿
                Sophus::SE3d Tiw = (pKFi->GetPose()).cast<double>();
                g2o::Sim3 g2oSiw(Tiw.unit_quaternion(), Tiw.translation(), 1.0);
                //Pose without correction
                vNonCorrectedSim3[pKFi] = g2oSiw;

                // 根据链式法则,利用当前关键帧和第i个共视关键帧的位姿关系,以及当前关键帧矫正后的初始位姿,推得第i个共视关键帧的矫正后的初始位姿
                Sophus::SE3d Tic = Tiw * Twc;
                g2o::Sim3 g2oSic(Tic.unit_quaternion(), Tic.translation(), 1.0);
                g2oCorrectedSiw = g2oSic * mg2oMergeScw;
                vCorrectedSim3[pKFi] = g2oCorrectedSiw;
            } else {
                g2oCorrectedSiw = g2oCorrectedScw;
            }

            // 这一句没有什么作用,下面又被覆盖了
            pKFi->mTcwMerge = pKFi->GetPose();

            // Update6DoF keyframe pose with corrected Sim3. First transform Sim3 to SE3 (scale translation)
            // 根据上面计算得到的融合矫正后的初始位姿(Sim3),更新窗口内每个关键帧的mTcwMerge(矫正后的初始位姿)
            double s = g2oCorrectedSiw.scale();
            pKFi->mfScale = s;

            // s*Riw * Pw + tiw = Pi  此时Pi在i坐标系下的坐标，尺度保留的是原来的
            // Riw * Pw + tiw/s = Pi/s 此时Pi/s在i坐标系下的坐标，尺度是最新的的，所以Rt要这么保留
            Sophus::SE3d correctedTiw(g2oCorrectedSiw.rotation(), g2oCorrectedSiw.translation() / s);  // 修正尺度

            // 赋值得到的矫正后的初始位姿
            pKFi->mTcwMerge = correctedTiw.cast<float>();

            // !纯视觉模式，以下代码不执行
            if (pCurrentMap->GetImuInitialized()) {
                Eigen::Quaternionf Rcor = (g2oCorrectedSiw.rotation().inverse() *
                                           vNonCorrectedSim3[pKFi].rotation()).cast<float>();
                pKFi->mVwbMerge = Rcor * pKFi->GetVelocity();
            }

            //TODO DEBUG to know which are the KFs that had been moved to the other map
        }

        int numPointsWithCorrection = 0;

        //for(MapPoint* pMPi : spLocalWindowMPs)
        // Step 2.3 记录所有地图点矫正前的位置,和矫正后的初始值
        // 对于每个窗口内的地图点，之前用的for循环，改成迭代器方便删点
        set<MapPoint *>::iterator itMP = spLocalWindowMPs.begin();
        while (itMP != spLocalWindowMPs.end()) {
            MapPoint *pMPi = *itMP;
            // 不好的点去掉，1.0新加
            if (!pMPi || pMPi->isBad()) {
                itMP = spLocalWindowMPs.erase(itMP);
                continue;
            }

            // 拿到参考关键帧，删掉不存在的，1.0新加
            KeyFrame *pKFref = pMPi->GetReferenceKeyFrame();
            if (vCorrectedSim3.find(pKFref) == vCorrectedSim3.end()) {
                itMP = spLocalWindowMPs.erase(itMP);
                numPointsWithCorrection++;
                continue;
            }

            // 拿到计算好的矫正后参考关键帧的初始位姿
            g2o::Sim3 g2oCorrectedSwi = vCorrectedSim3[pKFref].inverse();
            // 拿到矫正前的参考关键帧的位姿
            g2o::Sim3 g2oNonCorrectedSiw = vNonCorrectedSim3[pKFref];

            // ProjectMono with non-corrected pose and ProjectMPToKP back with corrected pose
            // 先把3D点转换到参考关键帧矫正前的坐标系中
            Eigen::Vector3d P3Dw = pMPi->GetWorldPos().cast<double>();
            // 再转换到矫正后的初始坐标系中
            Eigen::Vector3d eigCorrectedP3Dw = g2oCorrectedSwi.map(g2oNonCorrectedSiw.map(P3Dw));
            // 计算旋转部分的变化
            Eigen::Quaterniond Rcor = g2oCorrectedSwi.rotation() * g2oNonCorrectedSiw.rotation();

            // 矫正后3d点的初始位置
            pMPi->mPosMerge = eigCorrectedP3Dw.cast<float>();
            // 用旋转部分的变化更新计算3D点normal的新值
            pMPi->mNormalVectorMerge = Rcor.cast<float>() * pMPi->GetNormal();

            itMP++;
        }
        /*if(numPointsWithCorrection>0)
    {
        std::cout << "[Merge]: " << std::to_string(numPointsWithCorrection) << " points removed from Ma due to its reference KF is not in welding area" << std::endl;
        std::cout << "[Merge]: Ma has " << std::to_string(spLocalWindowMPs.ParameterSize()) << " points" << std::endl;
    }*/
        // Step 3 两个地图以新（当前帧所在地图）换旧（融合帧所在地图），包括关键帧及地图点关联地图的以新换旧、地图集的以新换旧
        {
            // 当前地图会被更新，老的地图中的重复地图点会被剔除
            unique_lock<mutex> currentLock(
                    pCurrentMap->mMutexMapUpdate); // We update the current map with the Merge information
            unique_lock<mutex> mergeLock(
                    pMergeMap->mMutexMapUpdate); // We remove the Kfs and MPs in the merged area from the old map

            //std::cout << "Merge local window: " << spLocalWindowKFs.ParameterSize() << std::endl;
            //std::cout << "[Merge]: init merging maps " << std::endl;
            // 对于当前关键帧共视窗口内的每一个关键帧
            // Step 3.1 更新当前关键帧共视窗口内的关键帧信息
            for (KeyFrame *pKFi: spLocalWindowKFs) {
                if (!pKFi || pKFi->isBad()) {
                    //std::cout << "Bad KF in correction" << std::endl;
                    continue;
                }

                //std::cout << "KF id: " << pKFi->mnId << std::endl;
                // 记录融合矫正前的位姿
                pKFi->mTcwBefMerge = pKFi->GetPose();
                pKFi->mTwcBefMerge = pKFi->GetPoseInverse();
                // 把这个关键帧的位姿设置为融合矫正后的初始位姿
                pKFi->SetPose(pKFi->mTcwMerge);

                // Make sure connections are updated
                // 把这个关键帧的地图设置为融合帧所在的地图
                pKFi->UpdateMap(pMergeMap);
                // 把这个关键帧所有权给到融合帧所在地图里
                pMergeMap->AddKeyFrame(pKFi);
                // 把这个关键帧从当前活跃地图中删掉
                pCurrentMap->EraseKeyFrame(pKFi);

                // 下面是没用的代码
                if (pCurrentMap->GetImuInitialized()) {
                    pKFi->SetVelocity(pKFi->mVwbMerge);
                }
            }

            // Step 3.2 更新当前关键帧共视帧窗口所能观测到的地图点的信息
            // 把所有地图点的所有权给到老图里面
            // 对于每个当前关键帧共视窗口内的所有地图点
            for (MapPoint *pMPi: spLocalWindowMPs) {
                if (!pMPi || pMPi->isBad())
                    continue;
                // 把3D点的位置设置成融合矫正之后的位置
                pMPi->SetWorldPos(pMPi->mPosMerge);
                // 把3D点normal设置成融合矫正之后的法向量
                pMPi->SetNormalVector(pMPi->mNormalVectorMerge);
                // 把3D点所在的地图设置成融合帧所在的地图
                pMPi->UpdateMap(pMergeMap);
                // 把3D点添加进融合帧所在地图
                pMergeMap->AddMapPoint(pMPi);
                // 把3D点从当前活跃地图中删掉
                pCurrentMap->EraseMapPoint(pMPi);
            }
            // Step 3.3 更新剩余信息,如Altas和融合帧所在地图的信息
            // 在Altas中把当前地图休眠，老图重新激活
            mpAtlas->ChangeMap(pMergeMap);
            // 当前地图的信息都到融合帧所在地图里去了,可以设置为bad
            mpAtlas->SetMapBad(pCurrentMap);
            // 记录地图变化次数
            pMergeMap->IncreaseChangeIdx();
            //TODO for debug
            pMergeMap->ChangeId(pCurrentMap->GetId());

            //std::cout << "[Merge]: merging maps finished" << std::endl;
        }

        // Step 4 融合新旧地图的生成树
        //Rebuild the essential graph in the local window
        // 重新构造essential graph的相关信息
        // 设置当前地图的第一个关键帧不再是第一次生成树了
        pCurrentMap->GetOriginKF()->SetFirstConnection(false);
        // 从当前关键帧开始反向遍历整个地图
        pNewChild = mpCurrentKF->GetParent(); // Old parent, it will be the new child of this KF
        pNewParent = mpCurrentKF; // Old child, now it will be the parent of its own parent(we need eliminate this KF from children list in its old parent)
        // 把当前帧的父亲设置为融合帧
        mpCurrentKF->ChangeParent(mpMergeMatchedKF);

        // mpMergeMatchedKF表示待融合地图中与当前关键帧对应上的帧
        // 因为整个待融合地图要融入到当前地图里，为了避免有两个父节点，mpMergeMatchedKF的原始父节点变成了它的子节点，而当前关键帧成了mpMergeMatchedKF的父节点
        // 同理，为了避免mpMergeMatchedKF原始父节点（现在已成它的子节点）有两个父节点，需要向上一直改到待融合地图最顶端的父节点
        while (pNewChild) {
            // new parent (old child) 不再是 new child (old parent) 的 child 了
            pNewChild->EraseChild(
                    pNewParent); // We remove the relation between the old parent and the new for avoid loop
            // 记录原先老parent的parent, 用于后续遍历
            KeyFrame *pOldParent = pNewChild->GetParent();

            // 把 new parent 设置为 new child 的 parent (父子关系互换)
            pNewChild->ChangeParent(pNewParent);

            // 赋值指针, 用于遍历下一组父子
            pNewParent = pNewChild;
            pNewChild = pOldParent;

        }

        //Update6DoF the connections between the local window
        // 更新融合帧局部的连接关系
        mpMergeMatchedKF->UpdateCovisGraph();

        // 重新拿到融合帧局部的共视帧窗窗口
        vpMergeConnectedKFs = mpMergeMatchedKF->GetVectorCovisibleKeyFrames();
        vpMergeConnectedKFs.emplace_back(mpMergeMatchedKF);
        //vpCheckFuseMapPoint.reserve(spMapPointMerge.ParameterSize());
        //std::copy(spMapPointMerge.begin(), spMapPointMerge.end(), std::back_inserter(vpCheckFuseMapPoint));

        // ProjectMono MapPoints observed in the neighborhood of the merge keyframe
        // into the current keyframe and neighbors using corrected poses.
        // SearchKFAndMapPointsByProjection duplications.
        //std::cout << "[Merge]: start fuse points" << std::endl;
        // Step 5 把融合关键帧的共视窗口里的地图点投到当前关键帧的共视窗口里,把重复的点融合掉(以旧换新)
        SearchAndFuse(vCorrectedSim3, vpCheckFuseMapPoint);
        //std::cout << "[Merge]: fuse points finished" << std::endl;

        // Update6DoF connectivity
        // Step 6 因为融合导致地图点变化。需要更新关键帧中图的连接关系
        // 更新当前关键帧共视窗口内所有关键帧的连接
        for (KeyFrame *pKFi: spLocalWindowKFs) {
            if (!pKFi || pKFi->isBad())
                continue;

            pKFi->UpdateCovisGraph();
        }
        // 更新融合关键帧共视窗口内所有关键帧的连接
        for (KeyFrame *pKFi: spMergeConnectedKFs) {
            if (!pKFi || pKFi->isBad())
                continue;

            pKFi->UpdateCovisGraph();
        }

        //std::cout << "[Merge]: Start welding bundle adjustment" << std::endl;

        bool bStop = false;
        // 为Local BA的接口, 把set转为vector
        // Step 7 在缝合(Welding)区域进行local BA
        vpLocalCurrentWindowKFs.clear();    //当前关键帧的窗口
        vpMergeConnectedKFs.clear();        //融合关键帧的窗口
        std::copy(spLocalWindowKFs.begin(), spLocalWindowKFs.end(), std::back_inserter(vpLocalCurrentWindowKFs));
        std::copy(spMergeConnectedKFs.begin(), spMergeConnectedKFs.end(), std::back_inserter(vpMergeConnectedKFs));
        Optimizer::MergeInertialBA(mpCurrentKF, mpMergeMatchedKF, &bStop, pCurrentMap, vCorrectedSim3);

        //std::cout << "[Merge]: Welding bundle adjustment finished" << std::endl;

        // Loop closed. CancelPause Local Mapping.
        mpLocalMapper->CancelPause();

        // Step 8 在当前帧整个剩下的地图中（局部窗口外，认为没那么紧急处理）对位姿和地图点进行矫正传播

        //Update6DoF the non critical area from the current map to the merged map
        // 把前面优位姿优化的结果传递到地图中其他的关键帧
        vector<KeyFrame *> vpCurrentMapKFs = pCurrentMap->GetAllKeyFrames();
        vector<MapPoint *> vpCurrentMapMPs = pCurrentMap->GetAllMapPoints();


        mpLocalMapper->RequestPause();
        // Wait until Local Mapping has effectively stopped
        while (!mpLocalMapper->CheckPaused()) {
            usleep(5000);
        }

        // Optimize graph (and update the loop position for each element form the begining to the end)
        // Step 8.2 本质图优化
        // 固定 : 所有融合帧共视窗口内的关键帧 + 所有当前关键帧共视窗口内的关键帧
        // 优化:  当前关键帧所在地图里的所有关键帧(除了当前关键帧共视窗口内的关键帧) + 当前地图里的所有地图点
        Optimizer::OptimizeEssentialGraph(mpCurrentKF, vpMergeConnectedKFs, vpLocalCurrentWindowKFs,
                                          vpCurrentMapKFs, vpCurrentMapMPs);


        {
            // Get Merge Map Mutex
            unique_lock<mutex> currentLock(
                    pCurrentMap->mMutexMapUpdate); // We update the current map with the Merge information
            unique_lock<mutex> mergeLock(
                    pMergeMap->mMutexMapUpdate); // We remove the Kfs and MPs in the merged area from the old map

            //std::cout << "Merge outside KFs: " << vpCurrentMapKFs.ParameterSize() << std::endl;
            // 确保融合后信息被更新
            for (KeyFrame *pKFi: vpCurrentMapKFs) {
                if (!pKFi || pKFi->isBad() || pKFi->GetMap() != pCurrentMap) {
                    continue;
                }
                //std::cout << "KF id: " << pKFi->mnId << std::endl;

                // Make sure connections are updated
                // 当前关键帧所在地图更新为融合地图，并建立关联。并从当前地图中删掉
                pKFi->UpdateMap(pMergeMap);
                pMergeMap->AddKeyFrame(pKFi);
                pCurrentMap->EraseKeyFrame(pKFi);
            }

            for (MapPoint *pMPi: vpCurrentMapMPs) {
                if (!pMPi || pMPi->isBad())
                    continue;

                pMPi->UpdateMap(pMergeMap);
                pMergeMap->AddMapPoint(pMPi);
                pCurrentMap->EraseMapPoint(pMPi);
            }
        }



        // Essential graph 优化后可以重新开始局部建图了
        mpLocalMapper->CancelPause();

        // 全局的BA（永远不会执行）
        // 这里没有imu, 所以isImuInitialized一定是false, 此时地图融合Atlas至少2个地图，所以第二个条件也一定是false
        // Step 9 全局BA
        if (bRelaunchBA &&
            (!pCurrentMap->GetImuInitialized() ||
             (pCurrentMap->GetKeyFramesNumInMap() < 200 && mpAtlas->CountMaps() == 1))) {
            // Launch a new thread to perform Global Bundle Adjustment
            mbRunningGBA = true;
            mbFinishedGBA = false;
            mbStopGBA = false;
            // 执行全局BA
            mpThreadGBA = new thread(&LoopClosing::RunGlobalBundleAdjustment, this, pMergeMap, mpCurrentKF->mnId);
        }

        // 添加融合边(这里不是参与优化的边,只是记录)
        mpMergeMatchedKF->AddMergeEdge(mpCurrentKF);
        mpCurrentKF->AddMergeEdge(mpMergeMatchedKF);

        pCurrentMap->IncreaseChangeIdx();
        pMergeMap->IncreaseChangeIdx();

        // altas移除不好的地图
        mpAtlas->RemoveBadMaps();

    }

/**
 * @brief 惯性模式下的地图融合
 */
    void LoopClosing::MergeLocalWithImu() {
        //cout << "Merge detected!!!!" << endl;
        // 没用上
        int numTemporalKFs = 11; //TODO (set by parameter): Temporal KFs in the local window if the map is inertial.

        //Relationship to rebuild the essential graph, it is used two times, first in the local window and later in the rest of the map
        // 用来重新构造Essential Graph
        KeyFrame *pNewChild;
        KeyFrame *pNewParent;

        // 没用上
        vector<KeyFrame *> vpLocalCurrentWindowKFs;
        vector<KeyFrame *> vpMergeConnectedKFs;

        // 记录用初始Sim3 计算出来的当前关键帧局部共视帧窗口内的所有关键帧矫正前的值和矫正后的初始值
        KeyFrameAndPose CorrectedSim3, NonCorrectedSim3;
        // NonCorrectedSim3[mpCurrentKF]=mg2oLoopScw;

        // Flag that is true only when we stopped a running BA, in this case we need relaunch at the end of the merge
        // 记录要不要重新进行全局ba
        bool bRelaunchBA = false;

        //cout << "Check Full Bundle Adjustment" << endl;
        // If a Global Bundle Adjustment is running, abort it
        // Step 1 如果正在进行全局BA，停掉它
        if (CheckRunningGBA()) {
            unique_lock<mutex> lock(mMutexGBA);
            mbStopGBA = true;

            mnFullBAIdx++;

            if (mpThreadGBA) {
                mpThreadGBA->detach();
                delete mpThreadGBA;
            }
            bRelaunchBA = true;
        }


        //cout << "Request CheckRequestReset Local Mapping" << endl;
        // Step 2 暂停局部建图线程
        mpLocalMapper->RequestPause();
        // Wait until Local Mapping has effectively stopped
        // 等待直到完全停掉
        while (!mpLocalMapper->CheckPaused()) {
            usleep(5000);
        }
        //cout << "Local Map stopped" << endl;

        // 当前关键帧地图的指针
        Map *pCurrentMap = mpCurrentKF->GetMap();
        // 融合关键帧地图的指针
        Map *pMergeMap = mpMergeMatchedKF->GetMap();

        // Step 3 利用前面计算的坐标系变换位姿，把整个当前地图（关键帧及地图点）变换到融合帧所在地图
        {
            // 把当前关键帧所在的地图位姿带到融合关键帧所在的地图
            // mSold_new = gSw2w1 记录的是当前关键帧世界坐标系到融合关键帧世界坐标系的变换
            float s_on = mSold_new.scale();
            Sophus::SE3f T_on(mSold_new.rotation().cast<float>(), mSold_new.translation().cast<float>());

            // 锁住altas更新地图
            unique_lock<mutex> lock(mpAtlas->GetCurrentMap()->mMutexMapUpdate);

            //cout << "KFs before empty: " << mpAtlas->GetCurrentMap()->GetKeyFramesNumInMap() << endl;
            // 队列里还没来得及处理的关键帧清空
            mpLocalMapper->EmptyQueue();
            //cout << "KFs after empty: " << mpAtlas->GetCurrentMap()->GetKeyFramesNumInMap() << endl;

            std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
            //cout << "updating active map to merge reference" << endl;
            //cout << "curr merge KF id: " << mpCurrentKF->mnId << endl;
            //cout << "curr tracking KF id: " << mpTracker->GetLastKeyFrame()->mnId << endl;
            // 是否将尺度更新到速度
            bool bScaleVel = false;
            if (s_on != 1)  // ?判断浮点数和1严格相等是不是不合适？
                bScaleVel = true;
            // 利用mSold_new位姿把整个当前地图中的关键帧和地图点变换到融合帧所在地图的坐标系下
            mpAtlas->GetCurrentMap()->ApplyScaledRotation(T_on, s_on, bScaleVel);
            // 尺度更新到普通帧位姿
            mpTracker->UpdateFrameIMU(s_on, mpCurrentKF->GetImuBias(), mpTracker->GetLastKeyFrame());

            std::chrono::steady_clock::time_point t3 = std::chrono::steady_clock::now();
        }

        // Step 4 如果当前地图IMU没有完全初始化，帮助IMU快速优化；
        // 反正都要融合了，这里就拔苗助长完成IMU优化，回头直接全部放到融合地图里就好了
        const int numKFnew = pCurrentMap->GetKeyFramesNumInMap();

        if (!pCurrentMap->GetImuIniertialBA2()) {
            // Map is not completly initialized
            Eigen::Vector3d bg, ba;
            bg << 0., 0., 0.;
            ba << 0., 0., 0.;
            // 优化当前地图中参数bg,ba
            Optimizer::InertialOptimization(pCurrentMap, bg, ba);
            IMU::Bias b(ba[0], ba[1], ba[2], bg[0], bg[1], bg[2]);
            unique_lock<mutex> lock(mpAtlas->GetCurrentMap()->mMutexMapUpdate);
            // 用优化得到的 bias 更新普通帧位姿
            mpTracker->UpdateFrameIMU(1.0f, b, mpTracker->GetLastKeyFrame());

            // Set map initialized
            // 设置IMU已经完成初始化
            pCurrentMap->SetImuIniertialBA2();
            pCurrentMap->SetImuIniertialBA1();
            pCurrentMap->SetImuInitialized();

        }


        //cout << "MergeMap init ID: " << pMergeMap->GetInitKFid() << "       CurrMap init ID: " << pCurrentMap->GetInitKFId() << endl;

        // Load KFs and MPs from merge map
        //cout << "updating current map" << endl;
        // Step 5 地图以旧换新。把融合帧所在地图里的关键帧和地图点从原地图里删掉，变更为当前关键帧所在地图。
        {
            // Get Merge Map Mutex (This section stops tracking!!)
            unique_lock<mutex> currentLock(
                    pCurrentMap->mMutexMapUpdate); // We update the current map with the Merge information
            unique_lock<mutex> mergeLock(
                    pMergeMap->mMutexMapUpdate); // We remove the Kfs and MPs in the merged area from the old map

            // 融合帧所在地图的所有关键帧和地图点
            vector<KeyFrame *> vpMergeMapKFs = pMergeMap->GetAllKeyFrames();
            vector<MapPoint *> vpMergeMapMPs = pMergeMap->GetAllMapPoints();

            // 遍历每个融合帧所在地图的关键帧
            for (KeyFrame *pKFi: vpMergeMapKFs) {
                if (!pKFi || pKFi->isBad() || pKFi->GetMap() != pMergeMap) {
                    continue;
                }

                // Make sure connections are updated
                // 把该关键帧从融合帧所在地图删掉,加入到当前的地图中
                pKFi->UpdateMap(pCurrentMap);
                pCurrentMap->AddKeyFrame(pKFi);
                pMergeMap->EraseKeyFrame(pKFi);
            }

            // 遍历每个融合帧所在地图的地图点
            for (MapPoint *pMPi: vpMergeMapMPs) {
                if (!pMPi || pMPi->isBad() || pMPi->GetMap() != pMergeMap)
                    continue;

                // 把地图点添加到当前帧所在地图,从融合帧所在地图删掉
                pMPi->UpdateMap(pCurrentMap);
                pCurrentMap->AddMapPoint(pMPi);
                pMergeMap->EraseMapPoint(pMPi);
            }
            // ? BUG! pMergeMap没有设置为BAD
            // ? 应该加入如下代码吧？
            mpAtlas->SetMapBad(pMergeMap);

            // Save non corrected poses (already merged maps)
            // 存下所有关键帧在融合矫正之前的位姿
            vector<KeyFrame *> vpKFs = pCurrentMap->GetAllKeyFrames();
            for (KeyFrame *pKFi: vpKFs) {
                Sophus::SE3d Tiw = (pKFi->GetPose()).cast<double>();
                g2o::Sim3 g2oSiw(Tiw.unit_quaternion(), Tiw.translation(), 1.0);
                NonCorrectedSim3[pKFi] = g2oSiw;
            }
        }

        //cout << "MergeMap init ID: " << pMergeMap->GetInitKFid() << "       CurrMap init ID: " << pCurrentMap->GetInitKFId() << endl;

        //cout << "end updating current map" << endl;

        // Critical zone
        //bool good = pCurrentMap->CheckEssentialGraph();
        /*if(!good)
        cout << "BAD ESSENTIAL GRAPH!!" << endl;*/

        //cout << "Update6DoF essential graph" << endl;
        // mpCurrentKF->UpdateCovisGraph(); // to put at false mbFirstConnection
        // Step 6 融合新旧地图的生成树
        pMergeMap->GetOriginKF()->SetFirstConnection(false);
        pNewChild = mpMergeMatchedKF->GetParent(); // Old parent, it will be the new child of this KF
        pNewParent = mpMergeMatchedKF; // Old child, now it will be the parent of its own parent(we need eliminate this KF from children list in its old parent)
        mpMergeMatchedKF->ChangeParent(mpCurrentKF);
        while (pNewChild) {
            pNewChild->EraseChild(
                    pNewParent); // We remove the relation between the old parent and the new for avoid loop
            KeyFrame *pOldParent = pNewChild->GetParent();
            pNewChild->ChangeParent(pNewParent);
            pNewParent = pNewChild;
            pNewChild = pOldParent;

        }


        //cout << "MergeMap init ID: " << pMergeMap->GetInitKFid() << "       CurrMap init ID: " << pCurrentMap->GetInitKFId() << endl;

        //cout << "end update essential graph" << endl;

        /*good = pCurrentMap->CheckEssentialGraph();
    if(!good)
        cout << "BAD ESSENTIAL GRAPH 1!!" << endl;*/

        //cout << "Update6DoF relationship between KFs" << endl;
        vector<MapPoint *> vpCheckFuseMapPoint; // MapPoint vector from current map to allow to fuse duplicated points with the old map (merge)
        vector<KeyFrame *> vpCurrentConnectedKFs;

        // 为后续SearchAndFuse准备数据
        // 拿出融合帧的局部窗口, 确保最后是(1+5), 1: 融合帧自己 2: 5个共视关键帧
        mvpMergeConnectedKFs.emplace_back(mpMergeMatchedKF);
        vector<KeyFrame *> aux = mpMergeMatchedKF->GetVectorCovisibleKeyFrames();
        mvpMergeConnectedKFs.insert(mvpMergeConnectedKFs.end(), aux.begin(), aux.end());
        if (mvpMergeConnectedKFs.size() > 6)
            mvpMergeConnectedKFs.erase(mvpMergeConnectedKFs.begin() + 6, mvpMergeConnectedKFs.end());
        /*mvpMergeConnectedKFs = mpMergeMatchedKF->GetVectorCovisibleKeyFrames();
    mvpMergeConnectedKFs.emplace_back(mpMergeMatchedKF);*/

        // 拿出当前关键帧的局部窗口, 确保最后是(1+5), 1: 融合帧自己 2: 5个共视关键帧
        mpCurrentKF->UpdateCovisGraph();
        vpCurrentConnectedKFs.emplace_back(mpCurrentKF);
        /*vpCurrentConnectedKFs = mpCurrentKF->GetVectorCovisibleKeyFrames();
    vpCurrentConnectedKFs.emplace_back(mpCurrentKF);*/
        aux = mpCurrentKF->GetVectorCovisibleKeyFrames();
        vpCurrentConnectedKFs.insert(vpCurrentConnectedKFs.end(), aux.begin(), aux.end());
        if (vpCurrentConnectedKFs.size() > 6)
            vpCurrentConnectedKFs.erase(vpCurrentConnectedKFs.begin() + 6, vpCurrentConnectedKFs.end());

        // 所有融合帧局部窗口的地图点
        set<MapPoint *> spMapPointMerge;
        for (KeyFrame *pKFi: mvpMergeConnectedKFs) {
            set<MapPoint *> vpMPs = pKFi->GetMapPoints();
            spMapPointMerge.insert(vpMPs.begin(), vpMPs.end());
            if (spMapPointMerge.size() > 1000)
                break;
        }

        /*cout << "vpCurrentConnectedKFs.size() " << vpCurrentConnectedKFs.size() << endl;
    cout << "mvpMergeConnectedKFs.size() " << mvpMergeConnectedKFs.size() << endl;
    cout << "spMapPointMerge.size() " << spMapPointMerge.ParameterSize() << endl;*/


        vpCheckFuseMapPoint.reserve(spMapPointMerge.size());
        std::copy(spMapPointMerge.begin(), spMapPointMerge.end(), std::back_inserter(vpCheckFuseMapPoint));
        //cout << "Finished to update relationship between KFs" << endl;

        //cout << "MergeMap init ID: " << pMergeMap->GetInitKFid() << "       CurrMap init ID: " << pCurrentMap->GetInitKFId() << endl;

        /*good = pCurrentMap->CheckEssentialGraph();
    if(!good)
        cout << "BAD ESSENTIAL GRAPH 2!!" << endl;*/

        //cout << "start SearchAndFuse" << endl;
        // Step 7 把融合关键帧的共视窗口里的地图点投到当前关键帧的共视窗口里，把重复的点融合掉（以旧换新）
        SearchAndFuse(vpCurrentConnectedKFs, vpCheckFuseMapPoint);
        //cout << "end SearchAndFuse" << endl;

        //cout << "MergeMap init ID: " << pMergeMap->GetInitKFid() << "       CurrMap init ID: " << pCurrentMap->GetInitKFId() << endl;

        /*good = pCurrentMap->CheckEssentialGraph();
    if(!good)
        cout << "BAD ESSENTIAL GRAPH 3!!" << endl;

    cout << "Init to update connections" << endl;*/

        // 更新当前关键帧共视窗口内所有关键帧的连接
        for (KeyFrame *pKFi: vpCurrentConnectedKFs) {
            if (!pKFi || pKFi->isBad())
                continue;

            pKFi->UpdateCovisGraph();
        }

        // 更新融合关键帧共视窗口内所有关键帧的连接
        for (KeyFrame *pKFi: mvpMergeConnectedKFs) {
            if (!pKFi || pKFi->isBad())
                continue;

            pKFi->UpdateCovisGraph();
        }
        //cout << "end update connections" << endl;

        //cout << "MergeMap init ID: " << pMergeMap->GetInitKFId() << "       CurrMap init ID: " << pCurrentMap->GetInitKFid() << endl;

        /*good = pCurrentMap->CheckEssentialGraph();
    if(!good)
        cout << "BAD ESSENTIAL GRAPH 4!!" << endl;*/

        // TODO Check: If new map is too small, we suppose that not informaiton can be propagated from new to old map
        if (numKFnew < 10) {
            mpLocalMapper->CancelPause();
            return;
        }

        /*good = pCurrentMap->CheckEssentialGraph();
    if(!good)
        cout << "BAD ESSENTIAL GRAPH 5!!" << endl;*/

        // Perform BA
        bool bStopFlag = false;
        KeyFrame *pCurrKF = mpTracker->GetLastKeyFrame();
        //cout << "start MergeInertialBA" << endl;
        // Step 8 针对缝合区域的窗口内进行进行welding BA
        Optimizer::MergeInertialBA(pCurrKF, mpMergeMatchedKF, &bStopFlag, pCurrentMap, CorrectedSim3);
        //cout << "end MergeInertialBA" << endl;

        /*good = pCurrentMap->CheckEssentialGraph();
    if(!good)
        cout << "BAD ESSENTIAL GRAPH 6!!" << endl;*/

        // CancelPause Local Mapping.
        mpLocalMapper->CancelPause();


        return;
    }

/**
 * @brief 1.0版本新的调试函数，暂时跳过
 */
    void LoopClosing::CheckObservations(set<KeyFrame *> &spKFsMap1, set<KeyFrame *> &spKFsMap2) {
        cout << "----------------------" << endl;
        for (KeyFrame *pKFi1: spKFsMap1) {
            map<KeyFrame *, int> mMatchedMP;
            set<MapPoint *> spMPs = pKFi1->GetMapPoints();

            for (MapPoint *pMPij: spMPs) {
                if (!pMPij || pMPij->isBad()) {
                    continue;
                }

                map<KeyFrame *, tuple<int, int >> mMPijObs = pMPij->GetObsKFAndLRIdx();
                for (KeyFrame *pKFi2: spKFsMap2) {
                    if (mMPijObs.find(pKFi2) != mMPijObs.end()) {
                        if (mMatchedMP.find(pKFi2) != mMatchedMP.end()) {
                            mMatchedMP[pKFi2] = mMatchedMP[pKFi2] + 1;
                        } else {
                            mMatchedMP[pKFi2] = 1;
                        }
                    }
                }

            }

            if (mMatchedMP.size() == 0) {
                cout << "CHECK-OBS: KF " << pKFi1->mnId << " has not any matched MP with the other map" << endl;
            } else {
                cout << "CHECK-OBS: KF " << pKFi1->mnId << " has matched MP with " << mMatchedMP.size()
                     << " KF from the other map" << endl;
                for (pair<KeyFrame *, int> matchedKF: mMatchedMP) {
                    cout << "   -KF: " << matchedKF.first->mnId << ", Number of matches: " << matchedKF.second << endl;
                }
            }
        }
        cout << "----------------------" << endl;
    }

/**
 * @brief 查找对应MP与融合
 * @param CorrectedPosesMap 关键帧及对应的pose
 * @param vpMapPoints 待融合地图的融合帧及其5个共视关键帧对应的mp（1000个以内）（注意此时所有kf与mp全部移至当前地图，这里的待融合地图的说法只为区分，因为还没有融合）
 */
    void LoopClosing::SearchAndFuse(const KeyFrameAndPose &CorrectedPosesMap, vector<MapPoint *> &vpMapPoints) {
        ORBmatcher matcher(0.8);

        int total_replaces = 0;

        // 遍历每个关键帧
        //cout << "[FUSE]: Initially there are " << vpMapPoints.ParameterSize() << " MPs" << endl;
        //cout << "FUSE: Intially there are " << CorrectedPosesMap.ParameterSize() << " KFs" << endl;
        for (KeyFrameAndPose::const_iterator mit = CorrectedPosesMap.begin(), mend = CorrectedPosesMap.end();
             mit != mend; mit++) {
            int num_replaces = 0;
            KeyFrame *pKFi = mit->first;
            Map *pMap = pKFi->GetMap();

            g2o::Sim3 g2oScw = mit->second;
            Sophus::Sim3f Scw = Converter::toSophus(g2oScw);

            vector<MapPoint *> vpReplacePoints(vpMapPoints.size(), static_cast<MapPoint *>(NULL));

            // 新点表示pKFi对应的点，老点表示pKFi对应的回环点
            // 将vpMapPoints投到pKF里面看看有没有匹配的MP，如果没有直接添加，如果有，暂时将老点放入至vpReplacePoints
            // vpReplacePoints下标表示第n个vpMapPoints，存放着新点，可以直接找到对应信息
            int numFused = matcher.Fuse(pKFi, Scw, vpMapPoints, 4, vpReplacePoints);

            // Get Map Mutex
            unique_lock<mutex> lock(pMap->mMutexMapUpdate);
            // 更新点
            const int nLP = vpMapPoints.size();
            for (int i = 0; i < nLP; i++) {
                // vpReplacePoints如果存在新点，则替换成老点，这里注意如果老点已经在新点对应的kf中
                // 也就是之前某次matcher.SearchKFAndMapPointsByProjection 把老点放入到新的关键帧中，下次遍历时，如果老点已经在被代替点的对应的某一个关键帧内
                MapPoint *pRep = vpReplacePoints[i];
                if (pRep) {


                    num_replaces += 1;
                    // 替换掉较新的
                    pRep->Replace(vpMapPoints[i]);

                }
            }

            total_replaces += num_replaces;
        }
        //cout << "[FUSE]: " << total_replaces << " MPs had been fused" << endl;
    }

/**
 * @brief 查找对应MP与融合，与上个函数类似
 * @param vConectedKFs 当前地图的当前关键帧及5个共视关键帧
 * @param vpMapPoints 待融合地图的融合帧及其5个共视关键帧对应的mp（1000个以内）（注意此时所有kf与mp全部移至当前地图，这里的待融合地图的说法只为区分，因为还没有融合）
 */
    void LoopClosing::SearchAndFuse(const vector<KeyFrame *> &vConectedKFs, vector<MapPoint *> &vpMapPoints) {
        ORBmatcher matcher(0.8);

        int total_replaces = 0;

        //cout << "FUSE-POSE: Initially there are " << vpMapPoints.ParameterSize() << " MPs" << endl;
        //cout << "FUSE-POSE: Intially there are " << vConectedKFs.ParameterSize() << " KFs" << endl;
        for (auto mit = vConectedKFs.begin(), mend = vConectedKFs.end(); mit != mend; mit++) {
            int num_replaces = 0;
            KeyFrame *pKF = (*mit);
            Map *pMap = pKF->GetMap();
            Sophus::SE3f Tcw = pKF->GetPose();
            Sophus::Sim3f Scw(Tcw.unit_quaternion(), Tcw.translation());
            Scw.setScale(1.f);
            /*std::cout << "These should be zeros: " <<
            Scw.rotationMatrix() - Tcw.rotationMatrix() << std::endl <<
            Scw.translation() - Tcw.translation() << std::endl <<
            Scw.scale() - 1.f << std::endl;*/
            vector<MapPoint *> vpReplacePoints(vpMapPoints.size(), static_cast<MapPoint *>(NULL));
            matcher.Fuse(pKF, Scw, vpMapPoints, 4, vpReplacePoints);

            // Get Map Mutex
            unique_lock<mutex> lock(pMap->mMutexMapUpdate);
            const int nLP = vpMapPoints.size();
            for (int i = 0; i < nLP; i++) {
                MapPoint *pRep = vpReplacePoints[i];
                if (pRep) {
                    num_replaces += 1;
                    pRep->Replace(vpMapPoints[i]);
                }
            }
            /*cout << "FUSE-POSE: KF " << pKF->mnId << " ->" << num_replaces << " MPs fused" << endl;
        total_replaces += num_replaces;*/
        }
        //cout << "FUSE-POSE: " << total_replaces << " MPs had been fused" << endl;
    }


/**
 * @brief 由外部线程调用,请求复位当前线程
 */
    void LoopClosing::RequestReset() {
        {
            unique_lock<mutex> lock(mMutexReset);
            cout << "LC: LoopCloser Reset Recieved" << endl;
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
        cout << "LC: LoopCloser Reset Done" << endl;
    }

    void LoopClosing::RequestResetActiveMap(Map *pMap) {
        {
            unique_lock<mutex> lock(mMutexReset);
            cout << "LC: ActiveMap Reset Recieved" << endl;
            mbResetActiveMapRequested = true;
            mpMapToReset = pMap;
        }
        while (1) {
            {
                unique_lock<mutex> lock2(mMutexReset);
                if (!mbResetActiveMapRequested)
                    break;
            }
            usleep(5000);
        }
        cout << "LC: ActiveMap Reset Done" << endl;

    }

/**
 * @brief 当前线程调用,检查是否有外部线程请求复位当前线程,如果有的话就复位回环检测线程
 */
    void LoopClosing::ResetIfRequested() {
        unique_lock<mutex> lock(mMutexReset);
        // 如果有来自于外部的线程的复位请求,那么就复位当前线程
        if (mbResetRequested) {
            cout << "LC: LoopCloser Reset Doing" << endl;
            // 清空参与和进行回环检测的关键帧队列
            mlpLoopKeyFrameQueue.clear();
            // 复位请求标志复位
            mbResetRequested = false;
            mbResetActiveMapRequested = false;
            cout << "LC: Reset Free Mutex" << endl;
        } else if (mbResetActiveMapRequested) {
            cout << "LC: ActiveMap Reset Doing" << endl;
            for (list<KeyFrame *>::const_iterator it = mlpLoopKeyFrameQueue.begin();
                 it != mlpLoopKeyFrameQueue.end();) {
                KeyFrame *pKFi = *it;
                if (pKFi->GetMap() == mpMapToReset) {
                    it = mlpLoopKeyFrameQueue.erase(it);
                } else {
                    ++it;
                }
            }
            mbResetActiveMapRequested = false;
            cout << "LC: Reset Free Mutex" << endl;
        }
    }

/**
 * @brief MergeLocalWithoutImu CorrectLoop 中调用
 * @param pActiveMap 当前地图
 * @param nLoopKF 检测到回环成功的关键帧，不是与之匹配的老关键帧
 */
    void LoopClosing::RunGlobalBundleAdjustment(Map *pActiveMap, unsigned long nLoopKF) {
        Verbose::PrintMess("Starting Global Bundle Adjustment", Verbose::VERBOSITY_NORMAL);
        const bool bImuInit = pActiveMap->GetImuInitialized();
        if (!bImuInit)
            Optimizer::GlobalBundleAdjustemntWithoutImu(pActiveMap, 10, &mbStopGBA, nLoopKF, false);
        else {
            Optimizer::GlobalBundleAdjustemetWithImu(pActiveMap, 7, false, nLoopKF, &mbStopGBA);
        }

        // 记录GBA已经迭代次数,用来检查全局BA过程是否是因为意外结束的
        int idx = mnFullBAIdx;
        {
            unique_lock<mutex> lock(mMutexGBA);

            if (!mbStopGBA) {
                Verbose::PrintMess("Global Bundle Adjustment finished", Verbose::VERBOSITY_NORMAL);
                Verbose::PrintMess("Updating map ...", Verbose::VERBOSITY_NORMAL);

                mpLocalMapper->RequestPause();
                // Wait until Local Mapping has effectively stopped

                while (!mpLocalMapper->CheckPaused() && !mpLocalMapper->CheckFinished()) {
                    usleep(5000);
                }

                // Get Map Mutex
                unique_lock<mutex> lock(pActiveMap->mMutexMapUpdate);
                // cout << "LC: Update6DoF Map Mutex adquired" << endl;

                //pActiveMap->PrintEssentialGraph();
                // Correct keyframes starting at map first keyframe
                list<KeyFrame *> lpKFtoCheck(pActiveMap->mvpInitKeyFrames.begin(),
                                             pActiveMap->mvpInitKeyFrames.end());

                // 通过树的方式更新未参与全局优化的关键帧，一个关键帧与其父节点的共视点数最多，所以选其作为参考帧
                while (!lpKFtoCheck.empty()) {
                    KeyFrame *pKF = lpKFtoCheck.front();
                    const set<KeyFrame *> sChilds = pKF->GetChilds();
                    Sophus::SE3f Twc = pKF->GetPoseInverse();
                    // 广度优先搜索
                    for (set<KeyFrame *>::const_iterator sit = sChilds.begin(); sit != sChilds.end(); sit++) {
                        KeyFrame *pChild = *sit;
                        if (!pChild || pChild->isBad())
                            continue;

                        // 专门处理没有参与优化的新关键帧
                        if (pChild->mnBAGlobalForKF != nLoopKF) {
                            //cout << "++++New child with flag " << pChild->mnBAGlobalForKF << "; LoopKF: " << nLoopKF << endl;
                            //cout << " child id: " << pChild->mnId << endl;
                            Sophus::SE3f Tchildc = pChild->GetPose() * Twc;
                            //cout << "Child pose: " << Tchildc << endl;
                            //cout << "pKF->mTcwGBA: " << pKF->mTcwGBA << endl;
                            pChild->mTcwGBA = Tchildc * pKF->mTcwGBA;//*Tcorc*pKF->mTcwGBA;

                            Sophus::SO3f Rcor = pChild->mTcwGBA.so3().inverse() * pChild->GetPose().so3();
                            if (pChild->isVelocitySet()) {
                                pChild->mVwbGBA = Rcor * pChild->GetVelocity();
                            } else
                                Verbose::PrintMess("Child velocity empty!! ", Verbose::VERBOSITY_NORMAL);


                            //cout << "Child bias: " << pChild->GetImuBias() << endl;
                            pChild->mBiasGBA = pChild->GetImuBias();


                            pChild->mnBAGlobalForKF = nLoopKF;  // 标记成更新过的

                        }
                        lpKFtoCheck.emplace_back(pChild);
                    }

                    //cout << "-------Update6DoF pose" << endl;
                    pKF->mTcwBefGBA = pKF->GetPose();
                    //cout << "pKF->mTcwBefGBA: " << pKF->mTcwBefGBA << endl;
                    pKF->SetPose(pKF->mTcwGBA);

                    if (pKF->bImu) {
                        //cout << "-------Update6DoF inertial values" << endl;
                        //if (pKF->mVwbGBA.empty())
                        //    Verbose::PrintMess("pKF->mVwbGBA is empty", Verbose::VERBOSITY_NORMAL);

                        //assert(!pKF->mVwbGBA.empty());
                        pKF->SetVelocity(pKF->mVwbGBA);
                        pKF->SetNewBias(pKF->mBiasGBA);
                    }

                    lpKFtoCheck.pop_front();
                }

                //cout << "GBA: Correct MapPoints" << endl;
                // Correct MapPoints
                // 更新mp点
                const vector<MapPoint *> vpMPs = pActiveMap->GetAllMapPoints();

                for (size_t i = 0; i < vpMPs.size(); i++) {
                    MapPoint *pMP = vpMPs[i];

                    if (pMP->isBad())
                        continue;

                    // NOTICE 并不是所有的地图点都会直接参与到全局BA优化中,但是大部分的地图点需要根据全局BA优化后的结果来重新纠正自己的位姿
                    // 如果这个地图点直接参与到了全局BA优化的过程,那么就直接重新设置器位姿即可
                    if (pMP->mnBAGlobalForKF == nLoopKF) {
                        // If optimized by Global BA, just update
                        pMP->SetWorldPos(pMP->mPosGBA);
                    } else  // 如故这个地图点并没有直接参与到全局BA优化的过程中,那么就使用器参考关键帧的新位姿来优化自己的位姿
                    {
                        // Update6DoF according to the correction of its reference keyframe
                        // 说明这个关键帧，在前面的过程中也没有因为“当前关键帧”得到全局BA优化
                        //? 可是,为什么会出现这种情况呢? 难道是因为这个地图点的参考关键帧设置成为了bad?
                        KeyFrame *pRefKF = pMP->GetReferenceKeyFrame();

                        if (pRefKF->mnBAGlobalForKF != nLoopKF)
                            continue;

                        /*if(pRefKF->mTcwBefGBA.empty())
                        continue;*/

                        // Map to non-corrected camera
                        // cv::Mat Rcw = pRefKF->mTcwBefGBA.rowRange(0,3).colRange(0,3);
                        // cv::Mat tcw = pRefKF->mTcwBefGBA.rowRange(0,3).col(3);
                        // 转换到其参考关键帧相机坐标系下的坐标
                        Eigen::Vector3f Xc = pRefKF->mTcwBefGBA * pMP->GetWorldPos();

                        // Backproject using corrected camera
                        // 然后使用已经纠正过的参考关键帧的位姿,再将该地图点变换到世界坐标系下
                        pMP->SetWorldPos(pRefKF->GetPoseInverse() * Xc);
                    }
                }

                pActiveMap->InformNewBigChange();
                pActiveMap->IncreaseChangeIdx();

                // TODO Check this update
                // mpTracker->UpdateFrameIMU(1.0f, mpTracker->GetLastKeyFrame()->GetImuBias(), mpTracker->GetLastKeyFrame());
                mpLocalMapper->CancelPause();
                Verbose::PrintMess("Map updated!", Verbose::VERBOSITY_NORMAL);
            }
            mbFinishedGBA = true;
            mbRunningGBA = false;
        }
    }

// 由外部线程调用,请求终止当前线程
    void LoopClosing::RequestFinish() {
        unique_lock<mutex> lock(mMutexFinish);
        // cout << "LC: Finish requested" << endl;
        mbRequestFinish = true;
    }

// 当前线程调用,查看是否有外部线程请求当前线程
    bool LoopClosing::CheckRequestFinish() {
        unique_lock<mutex> lock(mMutexFinish);
        return mbRequestFinish;
    }

// 有当前线程调用,执行完成该函数之后线程主函数退出,线程销毁
    void LoopClosing::SetFinished() {
        unique_lock<mutex> lock(mMutexFinish);
        mbFinished = true;
        cout << "LoopClose Finished" << endl;
    }

// 由外部线程调用,判断当前回环检测线程是否已经正确终止了
    bool LoopClosing::CheckFinished() {
        unique_lock<mutex> lock(mMutexFinish);
        return mbFinished;
    }


} //namespace ORB_SLAM
