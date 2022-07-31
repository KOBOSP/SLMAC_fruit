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
            mbStopGBA(false), mpThreadGBA(NULL), mbFixScale(bFixScale), mnLoopContiCoinDetect(0),
            mnMergeContiCoinDetect(0),
            mbLoopCoinSuccess(false), mbMergeCoinSuccess(false), mnLoopContiCoinFail(0), mnMergeContiCoinFail(0),
            mbActiveLC(bActiveLC) {
        // 连续性阈值
        mnThContiCoinSuccess = settings->mnThContiCoinSuccess;
        mnThContiCoinGiveup = settings->mnThContiCoinGiveup;
        mnThOriProjMatches = settings->mnThOriProjMatches;
        mnThBoWMatches = settings->mnThBoWMatches;
        mnThIterInliers = settings->mnThIterInliers;
        mnThOptInliers = settings->mnThOptInliers;
        mnThIterProjMatches = settings->mnThIterProjMatches;
        mnThOptProjMatches = settings->mnThOptProjMatches;
    }

    void LoopClosing::SetTracker(Tracking *pTracker) {
        mpTracker = pTracker;
    }

    void LoopClosing::SetLocalMapper(LocalMapping *pLocalMapper) {
        mpLocalMapper = pLocalMapper;
    }

    void LoopClosing::SetViewer(Viewer *pViewer) {
        mpViewer = pViewer;
    }

/**
 * @brief 回环线程主函数
 */
    void LoopClosing::Run() {
        mbFinished = false;
        // 线程主循环
        while (1) {
            //NEW LOOP AND MERGE DETECTION ALGORITHM
            // Loopclosing中的关键帧是LocalMapping发送过来的，LocalMapping是Tracking中发过来的
            // 在LocalMapping中通过 InsertKeyFrame 将关键帧插入闭环检测队列mlpLoopKeyFrameQueue
            // Step 1 查看闭环检测队列mlpLoopKeyFrameQueue中有没有关键帧进来
            if (CheckNewKeyFrames()) {
                // Step 2 检测有没有共视的区域
                bool bFindedRegion = DetectCommonRegionsExist();
                if (bFindedRegion) {
                    // Step 3 如果检测到融合（当前关键帧与其他地图有关联）, 则合并地图
                    if (mbMergeCoinSuccess) {
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
                                mnMergeContiCoinDetect = 0;
                                mvpMergeMatchedMPs.clear();
                                mvpMergeCandidMPs.clear();
                                mnMergeContiCoinFail = 0;
                                mbMergeCoinSuccess = false;
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
                            // 更新mg2oMergeScw
                            mg2oMergeScw = mg2oMergeSlw;

                            //mpTracker->SetStepByStep(true);
                            Verbose::PrintMess("*Merge detected", Verbose::VERBOSITY_QUIET);
                            // TODO UNCOMMENT
                            // 如果是imu模式,则开启 Visual-Inertial Map Merging
                            MergeLocalWithImu();
                            Verbose::PrintMess("Merge finished!", Verbose::VERBOSITY_QUIET);
                        }

                        // CheckRequestReset all variables
                        // 重置所有融合相关变量
                        mpMergeLastCurrentKF->SetCanErase();
                        mpMergeMatchedKF->SetCanErase();
                        mnMergeContiCoinDetect = 0;
                        mvpMergeMatchedMPs.clear();
                        mvpMergeCandidMPs.clear();
                        mnMergeContiCoinFail = 0;
                        mbMergeCoinSuccess = false;

                        // 重置所有回环相关变量, 说明对与当前帧同时有回环和融合的情况只进行融合
                        if (mbLoopCoinSuccess) {
                            // CheckRequestReset Loop variables
                            mpLoopLastCoinKF->SetCanErase();
                            mpLoopMatchedKF->SetCanErase();
                            mnLoopContiCoinDetect = 0;
                            mvpLoopMatchedMPs.clear();
                            mvpLoopCandidMPs.clear();
                            mnLoopContiCoinFail = 0;
                            mbLoopCoinSuccess = false;
                        }
                    }

                    // Step 4 如果(没有检测到融合)检测到回环, 则回环矫正
                    if (mbLoopCoinSuccess) {
                        // 标记时间戳
                        bool bGoodLoop = true;
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
                        if (fabs(phi(0)) < 0.087f && fabs(phi(1)) < 0.087f && fabs(phi(2)) < 0.349f) {
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
                            CorrectLoop();
                        }
                        // CheckRequestReset all variables
                        mpLoopLastCoinKF->SetCanErase();
                        mpLoopMatchedKF->SetCanErase();
                        mnLoopContiCoinDetect = 0;
                        mvpLoopMatchedMPs.clear();
                        mvpLoopCandidMPs.clear();
                        mnLoopContiCoinFail = 0;
                        mbLoopCoinSuccess = false;
                    }
                } else if (mpMapInCurKF) {
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
        if (mpMapInCurKF->GetKeyFramesNumInMap() < 50 || !mpMapInCurKF->GetImuIniertialBA2()) {
            return;
        }
        const vector<KeyFrame *> vpKFs = mpMapInCurKF->GetAllKeyFrames();

        Eigen::Matrix3f R12i, RBestOri12i;
        Eigen::Vector3f t12i, tBestOri12i;
        float s12i, sBestOri12i, fRtkToLocalDist, fBestOriDist = 0;
        Eigen::Matrix4f T12i, TBestOri12i;
        bool bFixScale = false;
        if (mpMapInCurKF->GetRtkInitialized()) {
            mpMapInCurKF->GetSim3FRtkToLocal(TBestOri12i, RBestOri12i, tBestOri12i, sBestOri12i);
            for (size_t i = 0; i < vpKFs.size(); i++) {
                fBestOriDist += (sBestOri12i * RBestOri12i * vpKFs[i]->GetRtkTransF() + tBestOri12i -
                                 vpKFs[i]->GetCameraCenter()).norm();
            }
            fBestOriDist /= vpKFs.size();
            if (fBestOriDist > 1) {
                mpMapInCurKF->SetRtkInitialized(false);
                cout << "RtkToLocal need to be update: OriDist: sBestOri12i: " << fBestOriDist << " " << sBestOri12i
                     << endl;
            }
        } else {
            fBestOriDist = 1e10;
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
            mpMapInCurKF->SetRtkInitialized(true);
            mpMapInCurKF->SetSim3FRtkToLocal(T12i, R12i, t12i, s12i);
            cout << "NowDist: OriDist: sBest: " << fRtkToLocalDist << " " << fBestOriDist << " " << s12i
                 << endl;
        }
    }

/**
 * @brief 插入关键帧
 */
    void LoopClosing::InsertKeyFrame(KeyFrame *pKF) {
        unique_lock<mutex> lock(mMutexNewKFQueue);
        if (pKF->mnId != 0)
            mlpKFQueueInLC.emplace_back(pKF);
    }

/**
 * @brief 查看有没有未处理的关键帧
 */
    bool LoopClosing::CheckNewKeyFrames() {
        unique_lock<mutex> lock(mMutexNewKFQueue);
        return (!mlpKFQueueInLC.empty());
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
            unique_lock<mutex> lock(mMutexNewKFQueue);
            mpCurrentKF = mlpKFQueueInLC.front();
            mlpKFQueueInLC.pop_front();
            mpMapInCurKF = mpCurrentKF->GetMap();
            if (mbActiveLC) {
                // 设置当前关键帧不要在优化的过程中被删除
                mpCurrentKF->SetNotErase();
            }
        }
        if (!mbActiveLC)
            return false;

        // Step 2 在某些情况下不进行共同区域检测
        if (!mpMapInCurKF->GetImuIniertialBA2() || mpMapInCurKF->GetKeyFramesNumInMap() < 12) {
            mpKeyFrameDB->add(mpCurrentKF);
            mpCurrentKF->SetCanErase();
            return false;
        }

        bool bLoopDetectedInKF = false;

        // Step 3.1 回环的时序几何校验。注意初始化时mnLoopNumCoincidences=0,
        if (mnLoopContiCoinDetect > 0) {
            Sophus::SE3d mTcl = (mpCurrentKF->GetPose() * mpLoopLastCoinKF->GetPoseInverse()).cast<double>();
            g2o::Sim3 gScl(mTcl.unit_quaternion(), mTcl.translation(), 1.0);
            g2o::Sim3 gScw = gScl * mg2oLoopSlw;
            int numProjMatches = 0;
            vector<MapPoint *> vpMatchedMPs;

            // 通过把候选帧局部窗口内的地图点向新进来的关键帧投影来验证回环检测结果,并优化Sim3位姿
            bool bCommonRegion = MatchKFsByProAndOptSim3AndPro(mpCurrentKF, mpLoopMatchedKF, gScw,
                                                               numProjMatches,
                                                               mvpLoopCandidMPs, vpMatchedMPs);
            if (bCommonRegion) {
                bLoopDetectedInKF = true;
                mnLoopContiCoinDetect++;
                mpLoopLastCoinKF->SetCanErase();
                mpLoopLastCoinKF = mpCurrentKF;
                mg2oLoopSlw = gScw;  // 记录当前优化的结果为{last T_cw}即为 T_lw
                mvpLoopMatchedMPs = vpMatchedMPs;
                mbLoopCoinSuccess = mnLoopContiCoinDetect >= mnThContiCoinSuccess;
                mnLoopContiCoinFail = 0;
            } else {
                bLoopDetectedInKF = false;
                mnLoopContiCoinFail++;
                if (mnLoopContiCoinFail > mnThContiCoinGiveup) {
                    mpLoopLastCoinKF->SetCanErase();
                    mpLoopMatchedKF->SetCanErase();
                    mnLoopContiCoinDetect = 0;
                    mvpLoopMatchedMPs.clear();
                    mvpLoopCandidMPs.clear();
                    mnLoopContiCoinFail = 0;
                }
            }
        }

        //Merge candidates
        bool bMergeDetectedInKF = false;
        if (mnMergeContiCoinDetect > 0) {
            Sophus::SE3d mTcl = (mpCurrentKF->GetPose() * mpMergeLastCurrentKF->GetPoseInverse()).cast<double>();
            g2o::Sim3 gScl(mTcl.unit_quaternion(), mTcl.translation(), 1.0);
            g2o::Sim3 gScw = gScl * mg2oMergeSlw;            // mg2oMergeSlw 中的w指的是融合候选关键帧世界坐标系
            int numProjMatches = 0;
            vector<MapPoint *> vpMatchedMPs;
            bool bCommonRegion = MatchKFsByProAndOptSim3AndPro(mpCurrentKF, mpMergeMatchedKF, gScw,
                                                               numProjMatches,
                                                               mvpMergeCandidMPs, vpMatchedMPs);
            if (bCommonRegion) {
                bMergeDetectedInKF = true;
                mnMergeContiCoinDetect++;
                mpMergeLastCurrentKF->SetCanErase();
                mpMergeLastCurrentKF = mpCurrentKF;
                mg2oMergeSlw = gScw;
                mvpMergeMatchedMPs = vpMatchedMPs;
                mbMergeCoinSuccess = mnMergeContiCoinDetect >= mnThContiCoinSuccess;
                mnMergeContiCoinFail = 0;
            } else {
                bMergeDetectedInKF = false;
                mnMergeContiCoinFail++;
                if (mnMergeContiCoinFail >= mnThContiCoinGiveup) {
                    mpMergeLastCurrentKF->SetCanErase();
                    mpMergeMatchedKF->SetCanErase();
                    mnMergeContiCoinDetect = 0;
                    mvpMergeMatchedMPs.clear();
                    mvpMergeCandidMPs.clear();
                    mnMergeContiCoinFail = 0;
                }
            }
        }

        if (mbMergeCoinSuccess || mbLoopCoinSuccess) {
            mpKeyFrameDB->add(mpCurrentKF);
            cout << "PR: Merge/Loop detected in times: " << mnMergeContiCoinDetect << " "
                 << mnLoopContiCoinDetect << " " << mnThContiCoinSuccess << endl;
            return true;
        }

        vector<KeyFrame *> vpMergeBowCand, vpLoopBowCand;
        if (!bMergeDetectedInKF || !bLoopDetectedInKF) {
            mpKeyFrameDB->DetectNBestLoopAndMergeKFs(mpCurrentKF, vpLoopBowCand, vpMergeBowCand, 3);
        }

        // Check the BoW candidates if the geometric candidate list is not empty
        if (!bLoopDetectedInKF && !vpLoopBowCand.empty()) {
            mbLoopCoinSuccess = MatchKFsByBowAndIterSim3AndMatchKFAndMPsByProAndOptSIm3AndPro(vpLoopBowCand,
                                                                                              mpLoopMatchedKF,
                                                                                              mpLoopLastCoinKF,
                                                                                              mg2oLoopSlw,
                                                                                              mnLoopContiCoinDetect,
                                                                                              mvpLoopCandidMPs,
                                                                                              mvpLoopMatchedMPs);
        }
        if (!bMergeDetectedInKF && !vpMergeBowCand.empty()) {
            mbMergeCoinSuccess = MatchKFsByBowAndIterSim3AndMatchKFAndMPsByProAndOptSIm3AndPro(vpMergeBowCand,
                                                                                               mpMergeMatchedKF,
                                                                                               mpMergeLastCurrentKF,
                                                                                               mg2oMergeSlw,
                                                                                               mnMergeContiCoinDetect,
                                                                                               mvpMergeCandidMPs,
                                                                                               mvpMergeMatchedMPs);
        }
        mpKeyFrameDB->add(mpCurrentKF);
        if (mbMergeCoinSuccess || mbLoopCoinSuccess) {
            cout << "PR: Merge/Loop detected in Bow: " << mnMergeContiCoinDetect << " "
                 << mnLoopContiCoinDetect << " " << mnThContiCoinSuccess << endl;
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
 * @param[out] nNumMatches 记录匹配点的数量
 * @param[out] vpCanCovMPs 候选帧窗口内所有的地图点
 * @param[out] vpMatchedMPs 候选帧窗口内所有被匹配到的点
 * @return true 时序几何验证成功
 * @return false 时序几何验证失败
 */
    bool LoopClosing::MatchKFsByProAndOptSim3AndPro(KeyFrame *pCurrentKF, KeyFrame *pCandidKF, g2o::Sim3 &gScw,
                                                    int &nNumMatches, std::vector<MapPoint *> &vpCanCovMPs,
                                                    std::vector<MapPoint *> &vpMatchedMPs) {
        set<MapPoint *> spAlreadyMatchedMPs;
        nNumMatches = FindMatchesByProjection(pCurrentKF, pCandidKF, gScw, vpCanCovMPs, vpMatchedMPs);

        // 2.点数如果不符合返回false
        if (nNumMatches >= mnThOriProjMatches) {
            Verbose::PrintMess("Sim3 reffine: There are " + to_string(nNumMatches) + " initial matches ",
                               Verbose::VERBOSITY_DEBUG);
            Sophus::SE3d mTwm = pCandidKF->GetPoseInverse().cast<double>();
            g2o::Sim3 gSwm(mTwm.unit_quaternion(), mTwm.translation(), 1.0);
            g2o::Sim3 gScm = gScw * gSwm;
            Eigen::Matrix<double, 7, 7> mHessian7x7;
            int numOptInliers = Optimizer::OptimizeKFsSim3(mpCurrentKF, pCandidKF, vpMatchedMPs, gScm, 10, mbFixScale,
                                                           mHessian7x7, true);
            Verbose::PrintMess("Sim3 reffine: There are " + to_string(numOptInliers)
                               + " matches after of the optimization ", Verbose::VERBOSITY_DEBUG);
            if (numOptInliers > mnThOptInliers) {
                g2o::Sim3 gScw_estimation((gScm * (gSwm.inverse())).rotation(), (gScm * (gSwm.inverse())).translation(),
                                          (gScm * (gSwm.inverse())).scale());
                vector<MapPoint *> vpMatchedMP;
                vpMatchedMP.resize(mpCurrentKF->GetVectorMapPointsInKF().size(), static_cast<MapPoint *>(NULL));
                nNumMatches = FindMatchesByProjection(pCurrentKF, pCandidKF, gScw_estimation, vpCanCovMPs,
                                                      vpMatchedMPs);
                if (nNumMatches >= mnThOptProjMatches) {
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
 * @param[in] vpCanKFs bow 给出的一些候选关键帧
 * @param[out] pMatchedKF 最后成功匹配的候选关键帧
 * @param[out] pLastCurrentKF 用于记录当前关键帧为上一个关键帧(后续若仍需要时序几何校验需要记录此信息)
 * @param[out] g2oScw 候选关键帧世界坐标系到当前关键帧的Sim3变换
 * @param[out] nNumCoincidences 成功几何验证的帧数，超过3就认为几何验证成功，不超过继续进行时序验证
 * @param[out] vpCanMPs  所有地图点
 * @param[out] vpMatchedMPs 成功匹配的地图点
 * @return true 检测到一个合格的共同区域
 * @return false 没检测到一个合格的共同区域
 */
    bool LoopClosing::MatchKFsByBowAndIterSim3AndMatchKFAndMPsByProAndOptSIm3AndPro(
            std::vector<KeyFrame *> &vpCanKFs, KeyFrame *&pMatchedKF, KeyFrame *&pLastCurrentKF, g2o::Sim3 &g2oScw,
            int &nNumCoincidences, std::vector<MapPoint *> &vpCanMPs, std::vector<MapPoint *> &vpMatchedMPs) {

        // 1. 获取当前帧的共视帧(在共同区域检测中应该避免当前关键帧的共视关键帧中)
        set<KeyFrame *> spCurCovKFs = mpCurrentKF->GetConnectedKeyFrames();

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

        int nNumCandKFs = vpCanKFs.size();
        for (KeyFrame *pKFi: vpCanKFs) {
            if (!pKFi || pKFi->isBad()) {
                continue;
            }

            std::vector<KeyFrame *> vpCanCovKFi = pKFi->GetBestCovisibilityKeyFrames(nNumCovisibles);
            if (vpCanCovKFi.empty()) {
                std::cout << "Covisible list empty" << std::endl;
                vpCanCovKFi.emplace_back(pKFi);
            } else {
                vpCanCovKFi.emplace_back(vpCanCovKFi[0]);
                vpCanCovKFi[0] = pKFi;
            }

            bool bAbortByNearCurKF = false;
            for (int j = 0; j < vpCanCovKFi.size(); ++j) {
                if (spCurCovKFs.find(vpCanCovKFi[j]) != spCurCovKFs.end()) {
                    bAbortByNearCurKF = true;
                    break;
                }
            }
            if (bAbortByNearCurKF) {
                cout << "bAbortByNearCurKF" << endl;
                continue;
            }

            // search by bow 返回的参数, 记录窗口Wm中每个关键帧有哪些点能在当前关键帧Ka中通过bow找到匹配点
            std::vector<std::vector<MapPoint *> > vvpBowMatchedMPsCurToCanCov;
            vvpBowMatchedMPsCurToCanCov.resize(vpCanCovKFi.size());

            // 记录整个窗口中有那些点能在Ka中通过bow找到匹配点(这个set是辅助容器,避免重复添加地图点)
            std::set<MapPoint *> spBowMatchedMPs;
            int nBoWMatchedMPsNum = 0;

            // 记录窗口中能通过bow在当前关键帧ka中找到最多匹配点的关键帧
            KeyFrame *pMostBoWMatchesKF;
            // 记录窗口中能通过bow在当前关键帧ka中找到最多匹配点的数量
            int nMostMatchNumByBow = 0;

            // 下面两个变量是为了sim3 solver准备的            // 标记是否因为窗口内有当前关键帧的共视关键帧
            //记录窗口中的地图点能在当前关键帧中找到的匹配的点(数量的上限是当前关键帧地图点的数量)
            std::vector<MapPoint *> vpBowMatchedMPs = std::vector<MapPoint *>(
                    mpCurrentKF->GetVectorMapPointsInKF().size(),
                    static_cast<MapPoint *>(NULL));
            // 记录上面的地图点分别对应窗口中的关键帧(数量的上限是当前关键帧地图点的数量)
            std::vector<KeyFrame *> vpBowMatchedMPsInKF = std::vector<KeyFrame *>(
                    mpCurrentKF->GetVectorMapPointsInKF().size(), static_cast<KeyFrame *>(NULL));


            // 2.3 通过Bow寻找候选帧窗口内的关键帧地图点与当前关键帧的匹配点
            for (int j = 0; j < vpCanCovKFi.size(); ++j) {
                if (!vpCanCovKFi[j] || vpCanCovKFi[j]->isBad())
                    continue;
                int nMatchNum = matcherByBoW.SearchMatchKFAndKFByBoW(mpCurrentKF, vpCanCovKFi[j],
                                                                     vvpBowMatchedMPsCurToCanCov[j]);
                if (nMatchNum > nMostMatchNumByBow) {
                    nMostMatchNumByBow = nMatchNum;
                    pMostBoWMatchesKF = vpCanCovKFi[j];
                }
            }

            // 遍历窗口内的每个关键帧
            // 2.4 把窗口内的匹配点转换为Sim3Solver接口定义的格式
            for (int j = 0; j < vpCanCovKFi.size(); ++j) {
                if (!vpCanCovKFi[j] || vpCanCovKFi[j]->isBad())
                    continue;
                // 遍历窗口内的某一个关键帧与当前关键帧由bow得到的匹配的地图点
                for (int k = 0; k < vvpBowMatchedMPsCurToCanCov[j].size(); ++k) {
                    MapPoint *pMPi_j = vvpBowMatchedMPsCurToCanCov[j][k];
                    if (!pMPi_j || pMPi_j->isBad())
                        continue;
                    if (spBowMatchedMPs.find(pMPi_j) == spBowMatchedMPs.end()) {
                        spBowMatchedMPs.insert(pMPi_j);
                        nBoWMatchedMPsNum++;
                        vpBowMatchedMPs[k] = pMPi_j;
                        vpBowMatchedMPsInKF[k] = vpCanCovKFi[j];
                    }
                }
            }

            // 当窗口内的帧不是当前关键帧的相邻帧且匹配点足够多时
            // 3. 利用RANSAC寻找候选关键帧窗口与当前关键帧的相对位姿T_cm的初始值(可能是Sim3)
            // mnThBoWMatches = 20; // 最低bow匹配特征点数
            if (nBoWMatchedMPsNum < mnThBoWMatches) {
                cout << "nBoWMatchedMPsNum < mnThBoWMatches: " << nBoWMatchedMPsNum << " " << mnThBoWMatches << endl;
                continue;
            }
            Sim3Solver solver = Sim3Solver(mpCurrentKF, pMostBoWMatchesKF, vpBowMatchedMPs, mbFixScale,
                                           vpBowMatchedMPsInKF);
            solver.SetRansacParameters(0.99, mnThIterInliers, 300); // at least 15 inliers
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

            Verbose::PrintMess("BoW Iter: Convergende with " + to_string(nInliers) + " geometrical inliers among " +
                               to_string(mnThIterInliers) + " BoW matches", Verbose::VERBOSITY_DEBUG);


            // Match by reprojection
            vpCanCovKFi.clear();
            vpCanCovKFi = pMostBoWMatchesKF->GetBestCovisibilityKeyFrames(nNumCovisibles);
            vpCanCovKFi.emplace_back(pMostBoWMatchesKF);
            set<MapPoint *> spMPsInMostBowCovKFs;
            vector<MapPoint *> vpMPsInMostBowCovKFs;
            for (KeyFrame *pCovKFi: vpCanCovKFi) {
                for (MapPoint *pCovMPij: pCovKFi->GetVectorMapPointsInKF()) {
                    if (!pCovMPij || pCovMPij->isBad())
                        continue;
                    // 避免重复添加
                    if (spMPsInMostBowCovKFs.find(pCovMPij) == spMPsInMostBowCovKFs.end()) {
                        spMPsInMostBowCovKFs.insert(pCovMPij);
                        vpMPsInMostBowCovKFs.emplace_back(pCovMPij);
                    }
                }
            }

            // 拿到solver 估计的 Scm初始值, 为后续的非线性优化做准备, 在这里 c 表示当前关键帧, m 表示回环/融合候选帧
            g2o::Sim3 gScmIter(solver.GetEstimatedRotation().cast<double>(),
                               solver.GetEstimatedTranslation().cast<double>(),
                               (double) solver.GetEstimatedScale());
            // 候选关键帧在其世界坐标系下的坐标
            g2o::Sim3 gSmwIter(pMostBoWMatchesKF->GetRotation().cast<double>(),
                               pMostBoWMatchesKF->GetTranslation().cast<double>(), 1.0);
            // 利用初始的Scm估计确定世界坐标系在当前相机中的位姿
            g2o::Sim3 gScwIter = gScmIter * gSmwIter; // Similarity matrix of current from the world position
            // 准备用来SearchByProjection的位姿信息
            Sophus::Sim3f mScwIter = Converter::toSophus(gScwIter);
            vector<MapPoint *> vpMatchedMPByProject;
            vpMatchedMPByProject.resize(mpCurrentKF->GetVectorMapPointsInKF().size(), static_cast<MapPoint *>(NULL));
            // 3.3.1 重新利用之前计算的mScw信息, 通过投影寻找更多的匹配点
            int numIterProjMatches = matcherByProjection.SearchMatchKFAndMPsByProject(mpCurrentKF, mScwIter,
                                                                                      vpMPsInMostBowCovKFs,
                                                                                      vpMatchedMPByProject,
                                                                                      8, 1.5);
            if (numIterProjMatches < mnThIterProjMatches) {
                cout << "numIterProjMatches < mnThIterProjMatches: " << numIterProjMatches << " " << mnThIterProjMatches << endl;
                continue;
            }


            // Optimize Sim3 transformation with every matches
            Eigen::Matrix<double, 7, 7> mHessian7x7;
            bool bFixedScale = mbFixScale;
            // 3.3.2 利用搜索到的更多的匹配点用Sim3优化投影误差得到的更好的 gScmIter
            // pKFi是候选关键帧
            int numOptInliers = Optimizer::OptimizeKFsSim3(mpCurrentKF, pKFi, vpMatchedMPByProject, gScmIter, 10,
                                                           mbFixScale, mHessian7x7, true);
            if (numOptInliers < mnThOptInliers) {
                cout << "numOptInliers < mnThOptInliers: " << numOptInliers << " " << mnThOptInliers << endl;
                continue;
            }


            // 前面已经声明了这些变量了,无需再次声明
            g2o::Sim3 gScwOpt = gScmIter * gSmwIter; // Similarity matrix of current from the world position
            Sophus::Sim3f mScwOpt = Converter::toSophus(gScwOpt);
            vpMatchedMPByProject.resize(mpCurrentKF->GetVectorMapPointsInKF().size(),
                                        static_cast<MapPoint *>(NULL));
            // 3.3.4 重新利用之前计算的mScw信息, 通过更小的半径和更严格的距离的投影寻找匹配点
            // 5 : 半径的增益系数(对比之前下降了)---> 更小的半径, 1.0 , hamming distance 的阀值增益系数---> 允许更小的距离
            int numOptProjMatches = matcherByProjection.SearchMatchKFAndMPsByProject(mpCurrentKF,
                                                                                     mScwOpt,
                                                                                     vpMPsInMostBowCovKFs,
                                                                                     vpMatchedMPByProject, 5,
                                                                                     1.0);
            if (numOptProjMatches < mnThOptProjMatches) {
                cout << "numOptProjMatches < mnThOptProjMatches: " << numOptProjMatches << " " << mnThOptProjMatches << endl;
                continue;
            }

            // 4. 用当前关键帧的相邻关键来验证前面得到的Tam(共视几何校验)
            // 统计验证成功的关键帧数量
            int nNumKFs = 0;
            vector<KeyFrame *> vpCurCovKFs = mpCurrentKF->GetBestCovisibilityKeyFrames(nNumCovisibles);
            int j = 0;
            // 遍历验证组当有三个关键帧验证成功或遍历所有的关键帧后结束循环
            while (nNumKFs < 3 && j < vpCurCovKFs.size()) {
                // 拿出验证组中的一个关键帧
                KeyFrame *pKFj = vpCurCovKFs[j];
                // 为 DetectCommonRegionsFromLastKF准备一个初始位姿, 这个用来进行searchByProjection
                Sophus::SE3d mTjc = (pKFj->GetPose() * mpCurrentKF->GetPoseInverse()).cast<double>();
                g2o::Sim3 gSjc(mTjc.unit_quaternion(), mTjc.translation(), 1.0);
                g2o::Sim3 gSjw = gSjc * gScwOpt;
                int numProjMatches_j = 0;
                vector<MapPoint *> vpMatchedMPs_j;
                // 4.2 几何校验函数, 这个函数里面其实是个searchByProjection : 通过之前计算的位姿转换地图点并通过投影搜索匹配点, 若大于一定数目的任务成功验证一次
                bool bValid = VerifyKFsByPro(pKFj, pMostBoWMatchesKF, gSjw,
                                             numProjMatches_j, vpMPsInMostBowCovKFs,
                                             vpMatchedMPs_j);
                if (bValid) {
                    nNumKFs++;
                }
                j++;
            }

            // 记录第二次searchByProjection得到最多匹配点的关键帧的各种信息,最后作为回环帧/融合帧
            if (nBestMatchesReproj < numOptProjMatches) {
                nBestMatchesReproj = numOptProjMatches; // 投影匹配的数量
                nBestNumCoindicendes = nNumKFs; // 成功验证的帧数
                pBestMatchedKF = pMostBoWMatchesKF; // 记录候选帧窗口内与当前关键帧相似度最高的帧
                g2oBestScw = gScwOpt; // 记录最优的位姿(这个位姿是由Tam推到出来的 : Taw = Tam * Tmw,这里a表示c)
                vpBestMapPoints = vpMPsInMostBowCovKFs; //  记录所有的地图点
                vpBestMatchedMapPoints = vpMatchedMPByProject; // 记录所有的地图点中被成功匹配的点
            }
        }

        // 如果成功找到了共同区域帧把记录的最优值存到输出变量中
        if (nBestMatchesReproj > 0) {
            pLastCurrentKF = mpCurrentKF;
            nNumCoincidences = nBestNumCoindicendes;  // 成功几何验证的帧数
            pMatchedKF = pBestMatchedKF;
            pMatchedKF->SetNotErase();
            g2oScw = g2oBestScw;
            vpCanMPs = vpBestMapPoints;
            vpMatchedMPs = vpBestMatchedMapPoints;
            cout << "nNumCoincidences >= mnThContiCoinSuccess: " << nNumCoincidences << " " << mnThContiCoinSuccess << endl;
            return nNumCoincidences >= mnThContiCoinSuccess;
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
    bool LoopClosing::VerifyKFsByPro(
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
 * @param[in] vpCanCovMPs 候选帧及其共视关键帧组成的窗口里所有的地图点
 * @param[in] vpMatchedMPs 候选帧及其共视关键帧组成的窗口里所有被匹配上的地图点
 * @return int 匹配点的数量
 */
    int LoopClosing::FindMatchesByProjection(
            KeyFrame *pCurrentKF, KeyFrame *pCandidKFw, g2o::Sim3 &g2oScw,
            vector<MapPoint *> &vpCanCovMPs,
            vector<MapPoint *> &vpMatchedMPs) {
        int nCurCovKFs = 10;  // change 上个版本为5
        // 拿出候选帧的10个最好的共视关键帧
        vector<KeyFrame *> vpCanCovKFs = pCandidKFw->GetBestCovisibilityKeyFrames(nCurCovKFs);
        int nCandidCovKFs = vpCanCovKFs.size();
        // 把自己也加进去, 组成一个局部窗口
        vpCanCovKFs.emplace_back(pCandidKFw);

        // 辅助容器,防止重复添加
        set<KeyFrame *> spCanCovKFs(vpCanCovKFs.begin(), vpCanCovKFs.end());
        set<KeyFrame *> spCurCovKFs = pCurrentKF->GetConnectedKeyFrames();

        // 1. 如果数量不够 扩充窗口
        if (nCandidCovKFs < nCurCovKFs) {
            for (int i = 0; i < nCandidCovKFs; ++i) {
                vector<KeyFrame *> vpKFs = vpCanCovKFs[i]->GetBestCovisibilityKeyFrames(nCurCovKFs);
                int nInserted = 0;
                int j = 0;
                while (j < vpKFs.size() && nInserted < nCurCovKFs) {
                    // 如果没有被重复添加且不是当前关键帧的共视关键帧
                    if (spCanCovKFs.find(vpKFs[j]) == spCanCovKFs.end() &&
                        spCurCovKFs.find(vpKFs[j]) == spCurCovKFs.end()) {
                        spCanCovKFs.insert(vpKFs[j]);
                        ++nInserted;
                    }
                    ++j;
                }
                // 把每个帧的共视关键帧都加到窗口内
                vpCanCovKFs.insert(vpCanCovKFs.end(), spCanCovKFs.begin(), spCanCovKFs.end());
            }
        }

        set<MapPoint *> spCanCovMPs;
        vpCanCovMPs.clear();
        vpMatchedMPs.clear();

        for (KeyFrame *pKFi: vpCanCovKFs) {
            for (MapPoint *pMPij: pKFi->GetVectorMapPointsInKF()) {
                if (!pMPij || pMPij->isBad())
                    continue;
                if (spCanCovMPs.find(pMPij) == spCanCovMPs.end()) {
                    spCanCovMPs.insert(pMPij);
                    vpCanCovMPs.emplace_back(pMPij);
                }
            }
        }

        Sophus::Sim3f mScw = Converter::toSophus(g2oScw);
        ORBmatcher matcher(0.9, true);
        vpMatchedMPs.resize(pCurrentKF->GetVectorMapPointsInKF().size(), static_cast<MapPoint *>(NULL));
        int nMatchedMPs = matcher.SearchMatchKFAndMPsByProject(pCurrentKF, mScw, vpCanCovMPs, vpMatchedMPs, 3,
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
        mpLocalMapper->RequestPause();
        mpLocalMapper->EmptyQueue(); // Proccess keyframes in the queue
        if (CheckRunningGBA()) {
            cout << "Stoping Global Bundle Adjustment...";
            unique_lock<mutex> lock(mMutexGBA);
            mbStopGBA = true;
            // 记录全局BA次数
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
        std::vector<KeyFrame *> vpCurCovKFs = mpCurrentKF->GetVectorCovisibleKeyFrames();
        vpCurCovKFs.emplace_back(mpCurrentKF);

        //std::cout << "Loop: number of connected KFs -> " + to_string(vpCurCovKFs.ParameterSize()) << std::endl;
        // CorrectedSim3：存放闭环g2o优化后当前关键帧的共视关键帧的世界坐标系下Sim3 变换
        // NonCorrectedSim3：存放没有矫正的当前关键帧的共视关键帧的世界坐标系下Sim3 变换
        KFAndPose CorrectedSim3, NonCorrectedSim3;
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
            for (vector<KeyFrame *>::iterator vit = vpCurCovKFs.begin(), vend = vpCurCovKFs.end();
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
            for (KFAndPose::iterator mit = CorrectedSim3.begin(), mend = CorrectedSim3.end();
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
        // SearchReplaceKFAndMPsByProjectInLocalMap duplications.
        // Step 5. 将闭环相连关键帧组mvpLoopCandidMPs 投影到当前关键帧组中，进行匹配，融合，新增或替换当前关键帧组中KF的地图点
        // 因为 闭环相连关键帧组mvpLoopCandidMPs 在地图中时间比较久经历了多次优化，认为是准确的
        // 而当前关键帧组中的关键帧的地图点是最近新计算的，可能有累积误差
        // CorrectedSim3：存放矫正后当前关键帧的共视关键帧，及其世界坐标系下Sim3 变换
        FuseBetweenKFsAndMPsWithPose(CorrectedSim3, mvpLoopCandidMPs);
        // After the MapPoint fusion, new links in the covisibility graph will appear attaching mbFrameBoth sides of the loop
        // Step 6. 更新当前关键帧之间的共视相连关系，得到因闭环时MapPoints融合而新得到的连接关系
        // LoopConnections：存储因为闭环地图点调整而新生成的连接关系
        map<KeyFrame *, set<KeyFrame *> > LoopConnections;

        // 6.1 遍历当前帧相连关键帧组（一级相连）
        for (vector<KeyFrame *>::iterator vit = vpCurCovKFs.begin(), vend = vpCurCovKFs.end();
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
            for (vector<KeyFrame *>::iterator vit2 = vpCurCovKFs.begin(), vend2 = vpCurCovKFs.end();
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
        mpAtlas->GetCurrentMap()->IncreaseChangeIdx();
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
            mpThreadGBA = new thread(&LoopClosing::RunGlobalBundleAdjustment, this, pLoopMap, mpCurrentKF->mnId);
        }
        // Loop closed. CancelPause Local Mapping.
        mpLocalMapper->CancelPause();
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
        KFAndPose CorrectedSim3, NonCorrectedSim3;
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
            mpTracker->UpdateLastAndCurFrameIMU(s_on, mpCurrentKF->GetImuBias(), mpTracker->GetLastKeyFrame());

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
            mpTracker->UpdateLastAndCurFrameIMU(1.0f, b, mpTracker->GetLastKeyFrame());

            // Set map initialized
            // 设置IMU已经完成初始化
            pCurrentMap->SetImuIniertialBA2();
            pCurrentMap->SetImuIniertialBA1();
            pCurrentMap->SetImuInitialized();

        }


        // Load KFs and MPs from merge map
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

        // 拿出当前关键帧的局部窗口, 确保最后是(1+5), 1: 融合帧自己 2: 5个共视关键帧
        mpCurrentKF->UpdateCovisGraph();
        vpCurrentConnectedKFs.emplace_back(mpCurrentKF);
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


        vpCheckFuseMapPoint.reserve(spMapPointMerge.size());
        std::copy(spMapPointMerge.begin(), spMapPointMerge.end(), std::back_inserter(vpCheckFuseMapPoint));

        // Step 7 把融合关键帧的共视窗口里的地图点投到当前关键帧的共视窗口里，把重复的点融合掉（以旧换新）
        FuseBetweenKFsAndMPsWithoutPose(vpCurrentConnectedKFs, vpCheckFuseMapPoint);

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


        // TODO Check: If new map is too small, we suppose that not informaiton can be propagated from new to old map
        if (numKFnew < 10) {
            mpLocalMapper->CancelPause();
            return;
        }

        // Step 8 针对缝合区域的窗口内进行进行welding BA
        bool bStopFlag = false;
        KeyFrame *pCurrKF = mpTracker->GetLastKeyFrame();
        cout << "start MergeInertialBA" << endl;
        Optimizer::MergeInertialBA(pCurrKF, mpMergeMatchedKF, &bStopFlag, pCurrentMap, CorrectedSim3);
        cout << "end MergeInertialBA" << endl;

        // CancelPause Local Mapping.
        mpLocalMapper->CancelPause();


        return;
    }

/**
 * @brief 查找对应MP与融合
 * @param CorrectedKFAndPose 关键帧及对应的pose
 * @param vpMapPoints 待融合地图的融合帧及其5个共视关键帧对应的mp（1000个以内）（注意此时所有kf与mp全部移至当前地图，这里的待融合地图的说法只为区分，因为还没有融合）
 */
    void LoopClosing::FuseBetweenKFsAndMPsWithPose(const KFAndPose &CorrectedKFAndPose,
                                                   vector<MapPoint *> &vpMapPoints) {
        ORBmatcher matcher(0.8);

        int nTotalReplace = 0;

        // 遍历每个关键帧
        //cout << "[FUSE]: Initially there are " << vpMapPoints.ParameterSize() << " MPs" << endl;
        //cout << "FUSE: Intially there are " << CorrectedKFAndPose.ParameterSize() << " KFs" << endl;
        for (KFAndPose::const_iterator mit = CorrectedKFAndPose.begin(), mend = CorrectedKFAndPose.end();
             mit != mend; mit++) {
            int nEachKFReplace = 0;
            KeyFrame *pKFi = mit->first;
            Map *pMap = pKFi->GetMap();

            g2o::Sim3 g2oScw = mit->second;
            Sophus::Sim3f Scw = Converter::toSophus(g2oScw);
            Sophus::SE3f Tcw = Sophus::SE3f(Scw.rotationMatrix(), Scw.translation() / Scw.scale());
            vector<MapPoint *> vpReplacePoints(vpMapPoints.size(), static_cast<MapPoint *>(NULL));

            // 新点表示pKFi对应的点，老点表示pKFi对应的回环点
            // 将vpMapPoints投到pKF里面看看有没有匹配的MP，如果没有直接添加，如果有，暂时将老点放入至vpReplacePoints
            // vpReplacePoints下标表示第n个vpMapPoints，存放着新点，可以直接找到对应信息
            int numFused = matcher.SearchReplaceKFAndMPsByProjectInGlobalMap(pKFi, Tcw, vpMapPoints, 4,
                                                                             vpReplacePoints);

            // Get Map Mutex
            unique_lock<mutex> lock(pMap->mMutexMapUpdate);
            // 更新点
            const int nLP = vpMapPoints.size();
            for (int i = 0; i < nLP; i++) {
                // vpReplacePoints如果存在新点，则替换成老点，这里注意如果老点已经在新点对应的kf中
                // 也就是之前某次matcher.SearchReplaceKFAndMPsByProjectInLocalMap 把老点放入到新的关键帧中，下次遍历时，如果老点已经在被代替点的对应的某一个关键帧内
                MapPoint *pRep = vpReplacePoints[i];
                if (pRep) {
                    nEachKFReplace += 1;
                    pRep->Replace(vpMapPoints[i]);
                }
            }
            nTotalReplace += nEachKFReplace;
        }
        cout << "[FUSE]: " << nTotalReplace << " MPs had been fused" << endl;
    }

/**
 * @brief 查找对应MP与融合，与上个函数类似
 * @param vConectedKFs 当前地图的当前关键帧及5个共视关键帧
 * @param vpMapPoints 待融合地图的融合帧及其5个共视关键帧对应的mp（1000个以内）（注意此时所有kf与mp全部移至当前地图，这里的待融合地图的说法只为区分，因为还没有融合）
 */
    void LoopClosing::FuseBetweenKFsAndMPsWithoutPose(const vector<KeyFrame *> &vConectedKFs,
                                                      vector<MapPoint *> &vpMapPoints) {
        ORBmatcher matcher(0.8);

        int nTotalReplace = 0;
        for (auto mit = vConectedKFs.begin(), mend = vConectedKFs.end(); mit != mend; mit++) {
            int nEachKFReplace = 0;
            KeyFrame *pKF = (*mit);
            Map *pMap = pKF->GetMap();
            Sophus::SE3f Tcw = pKF->GetPose();

            vector<MapPoint *> vpReplacePoints(vpMapPoints.size(), static_cast<MapPoint *>(NULL));
            matcher.SearchReplaceKFAndMPsByProjectInGlobalMap(pKF, Tcw, vpMapPoints, 4, vpReplacePoints);

            // Get Map Mutex
            unique_lock<mutex> lock(pMap->mMutexMapUpdate);
            const int nLP = vpMapPoints.size();
            for (int i = 0; i < nLP; i++) {
                MapPoint *pRep = vpReplacePoints[i];
                if (pRep) {
                    pRep->Replace(vpMapPoints[i]);
                    nEachKFReplace += 1;
                }
            }
            cout << "FUSE-POSE: KF " << pKF->mnId << " ->" << nEachKFReplace << " MPs fused" << endl;
            nTotalReplace += nEachKFReplace;
        }
        cout << "FUSE-POSE: " << nTotalReplace << " MPs had been fused" << endl;
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
            mlpKFQueueInLC.clear();
            // 复位请求标志复位
            mbResetRequested = false;
            mbResetActiveMapRequested = false;
            cout << "LC: Reset Free Mutex" << endl;
        } else if (mbResetActiveMapRequested) {
            cout << "LC: ActiveMap Reset Doing" << endl;
            for (list<KeyFrame *>::const_iterator it = mlpKFQueueInLC.begin();
                 it != mlpKFQueueInLC.end();) {
                KeyFrame *pKFi = *it;
                if (pKFi->GetMap() == mpMapToReset) {
                    it = mlpKFQueueInLC.erase(it);
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
 * @param nGBAId 检测到回环成功的关键帧，不是与之匹配的老关键帧
 */
    void LoopClosing::RunGlobalBundleAdjustment(Map *pActiveMap, unsigned long nGBAId) {
        Verbose::PrintMess("Starting Global Bundle Adjustment", Verbose::VERBOSITY_NORMAL);
        const bool bImuInit = pActiveMap->GetImuInitialized();
        if (!bImuInit)
            Optimizer::GlobalBundleAdjustemntWithoutImu(pActiveMap, 10, &mbStopGBA, nGBAId, false);
        else {
            Optimizer::GlobalBundleAdjustemetWithImu(pActiveMap, 7, false, nGBAId, &mbStopGBA);
        }

        {
            unique_lock<mutex> lock(mMutexGBA);

            if (!mbStopGBA) {
                mpLocalMapper->RequestPause();
                while (!mpLocalMapper->CheckPaused() && !mpLocalMapper->CheckFinished()) {
                    usleep(5000);
                }

                Verbose::PrintMess("Global Bundle Adjustment finished, Updating map ...", Verbose::VERBOSITY_NORMAL);
                unique_lock<mutex> lock(pActiveMap->mMutexMapUpdate);
                list<KeyFrame *> lpKFstoUpdate(pActiveMap->mvpInitKeyFrames.begin(),
                                               pActiveMap->mvpInitKeyFrames.end());
                vector<MapPoint *> vpMPsToUpdate = pActiveMap->GetAllMapPoints();
                UpdateKFsAndMPsAfterBA(lpKFstoUpdate, vpMPsToUpdate, nGBAId);

                Verbose::PrintMess("Map updated!", Verbose::VERBOSITY_NORMAL);
                pActiveMap->IncreaseChangeIdx();

                mpTracker->UpdateLastAndCurFrameIMU(1.0f, mpTracker->GetLastKeyFrame()->GetImuBias(),
                                                    mpTracker->GetLastKeyFrame());
                mpLocalMapper->CancelPause();
            }
            mbFinishedGBA = true;
            mbRunningGBA = false;
        }
    }

    void LoopClosing::UpdateKFsAndMPsAfterBA(list<KeyFrame *> &lpKFstoUpdate, vector<MapPoint *> &vpMPsToUpdate,
                                             unsigned long nGBAId) {
        // 通过树的方式更新未参与全局优化的关键帧，一个关键帧与其父节点的共视点数最多，所以选其作为参考帧
        while (!lpKFstoUpdate.empty()) {
            KeyFrame *pKF = lpKFstoUpdate.front();
            const set<KeyFrame *> sChilds = pKF->GetChilds();
            Sophus::SE3f Twc = pKF->GetPoseInverse();
            for (set<KeyFrame *>::const_iterator sit = sChilds.begin(); sit != sChilds.end(); sit++) {
                KeyFrame *pChild = *sit;
                if (!pChild || pChild->isBad())
                    continue;
                if (pChild->mnLMGBAFlag != nGBAId) {
                    Sophus::SE3f Tchildwwc = pChild->GetPose() * Twc;
                    pChild->mLMGBATcw = Tchildwwc * pKF->mLMGBATcw;//*Tcorc*pKF->mLMGBATcw;
                    Sophus::SO3f Rcor = pChild->mLMGBATcw.so3().inverse() * pChild->GetPose().so3();
                    if (pChild->isVelocitySet()) {
                        pChild->mLMGBAVwb = Rcor * pChild->GetVelocity();
                    } else {
                        Verbose::PrintMess("Child velocity empty!! ", Verbose::VERBOSITY_NORMAL);
                    }
//                            pChild->mLMGBABias = pChild->GetImuBias();
                    pChild->mLMGBABias = pKF->mLMGBABias;
                    pChild->mnLMGBAFlag = nGBAId;  // 标记成更新过的
                }
                lpKFstoUpdate.emplace_back(pChild);
            }
            pKF->mBefGBATcw = pKF->GetPose();
            pKF->SetPose(pKF->mLMGBATcw);
            if (pKF->bImu) {
                pKF->SetVelocity(pKF->mLMGBAVwb);
                pKF->SetNewBias(pKF->mLMGBABias);
            }
            lpKFstoUpdate.pop_front();
        }
        for (size_t i = 0; i < vpMPsToUpdate.size(); i++) {
            MapPoint *pMP = vpMPsToUpdate[i];

            if (!pMP || pMP->isBad())
                continue;

            // NOTICE 并不是所有的地图点都会直接参与到全局BA优化中,但是大部分的地图点需要根据全局BA优化后的结果来重新纠正自己的位姿
            // 如果这个地图点直接参与到了全局BA优化的过程,那么就直接重新设置器位姿即可
            if (pMP->mnLMGBAFlag == nGBAId) {
                // If optimized by Global BA, just update
                pMP->SetWorldPos(pMP->mLMGBAPos);
            } else  // 如故这个地图点并没有直接参与到全局BA优化的过程中,那么就使用器参考关键帧的新位姿来优化自己的位姿
            {
                // Update6DoF according to the correction of its reference keyframe
                KeyFrame *pRefKF = pMP->GetReferenceKeyFrame();
                if (pRefKF->mnLMGBAFlag != nGBAId)
                    continue;
                Eigen::Vector3f Xc = pRefKF->mBefGBATcw * pMP->GetWorldPos();
                // Backproject using corrected camera
                pMP->SetWorldPos(pRefKF->GetPoseInverse() * Xc);
            }
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
