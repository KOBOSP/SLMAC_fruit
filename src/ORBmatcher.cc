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


#include "ORBmatcher.h"

#include<limits.h>

#include<opencv2/core/core.hpp>

#include "DBoW2/FeatureVector.h"

#include<stdint-gcc.h>

using namespace std;

namespace ORB_SLAM3 {

    const int ORBmatcher::TH_HIGH = 100;
    const int ORBmatcher::TH_LOW = 50;
    const int ORBmatcher::HISTO_LENGTH = 30;

    // 构造函数,参数默认值为0.6,true
    ORBmatcher::ORBmatcher(float nnratio, bool checkOri) : mfNNratio(nnratio), mbCheckOrientation(checkOri) {
    }

    int ORBmatcher::SearchReplaceFrameAndMPsByProject(Frame &F, const vector<MapPoint *> &vpMapPoints,
                                                      const float nThProjRad,
                                                      const bool bFarPoints, const float thFarPoints) {
        int nReplaced = 0;
        for (size_t iMP = 0; iMP < vpMapPoints.size(); iMP++) {
            MapPoint *pMP = vpMapPoints[iMP];
            if (!pMP) {
                continue;
            }
            if (pMP->isBad()) {
                continue;
            }
            if (!pMP->mbTrackInLeftView) {
                continue;
            }
            if (bFarPoints && pMP->mTrackDepth > thFarPoints) {
                continue;
            }
            const int &nPredictedLevel = pMP->mnTrackScaleLevel;
            float fProjRad = RadiusByViewingCos(pMP->mTrackViewCos) * nThProjRad;
            const vector<size_t> vIndices =
                    F.GetFeaturesInArea(pMP->mTrackProjX, pMP->mTrackProjY,
                                        fProjRad * F.mvfScaleFactors[nPredictedLevel],
                                        nPredictedLevel - 1, nPredictedLevel + 1);
            if (vIndices.empty()) {
                continue;
            }
            const cv::Mat dMP = pMP->GetDescriptor();
            int bestDist = INT_MAX;
            int bestLevel = -1;
            int bestIdx = -1;
            int secondDist = INT_MAX;
            int secondLevel = -1;
            for (vector<size_t>::const_iterator vit = vIndices.begin(), vend = vIndices.end(); vit != vend; vit++) {
                const size_t idx = *vit;
                // 如果Frame中的该兴趣点已经有对应的MapPoint了,则退出该次循环
                if (F.mvpMPs[idx])
                    if (F.mvpMPs[idx]->GetObsTimes() > 0)
                        continue;

                //如果是双目数据
                if (F.mvfXInRight[idx] > 0) {
                    //计算在X轴上的投影误差
                    const float er = fabs(pMP->mTrackProjXR - F.mvfXInRight[idx]);
                    //超过阈值,说明这个点不行,丢掉.
                    //这里的阈值定义是以给定的搜索范围r为参考,然后考虑到越近的点(nPredictedLevel越大), 相机运动时对其产生的影响也就越大,
                    //因此需要扩大其搜索空间.
                    //当给定缩放倍率为1.2的时候, mvfScaleFactors 中的数据是: 1 1.2 1.2^2 1.2^3 ...
                    if (er > fProjRad * F.mvfScaleFactors[nPredictedLevel])
                        continue;
                }

                const cv::Mat &dF = F.mDescriptorsLeft.row(idx);
                const int nDist = GetDescriptorDistance(dMP, dF);
                if (nDist < bestDist) {
                    secondDist = bestDist;
                    bestDist = nDist;
                    secondLevel = bestLevel;
                    bestLevel = F.mvKPsUn[idx].octave;
                    bestIdx = idx;
                } else if (nDist < secondDist) {
                    secondLevel = F.mvKPsUn[idx].octave;
                    secondDist = nDist;
                }
            }

            // Apply ratio to second match (only if best and second are in the same scale level)
            if (bestDist <= TH_HIGH) {
                // 条件1：bestLevel==secondLevel 表示 最佳和次佳在同一金字塔层级
                // 条件2：bestDist>mfNNratio*secondDist 表示最佳和次佳距离不满足阈值比例。理论来说 bestDist/secondDist 越小越好
                if ((bestDist > mfNNratio * secondDist) ||
                    (bestDist > 0.3 * mfNNratio * secondDist && bestLevel == secondLevel)) {
                    F.mvpMPs[bestIdx] = pMP;
                    nReplaced++;
                }
            }
        }
        return nReplaced;
    }

    // 根据观察的视角来计算匹配的时的搜索窗口大小
    float ORBmatcher::RadiusByViewingCos(const float &viewCos) {
        // 当视角相差小于3.6°，对应cos(3.6°)=0.998，搜索范围是2.5，否则是4
        if (viewCos > 0.998)
            return 2.5;
        else
            return 4.0;
    }

    /**
     * @brief 通过词袋，对关键帧的特征点进行跟踪
     * 
     * @param[in] pKF               关键帧
     * @param[in] F                 当前普通帧
     * @param[in] vpMPMatches F中地图点对应的匹配，NULL表示未匹配
     * @return int                  成功匹配的数量
     */
    int ORBmatcher::SearchMatchFrameAndKFByBoW(KeyFrame *pKF, Frame &F, vector<MapPoint *> &vpMPMatches) {
        const vector<MapPoint *> vpMapPointsKF = pKF->GetVectorMapPointsInKF();
        vpMPMatches = vector<MapPoint *>(F.mnKPsLeftNum, static_cast<MapPoint *>(NULL));
        const DBoW2::FeatureVector &vFeatVecKF = pKF->mFeatVec;
        int nmatches = 0;
        vector<int> rotHist[HISTO_LENGTH];
        for (int i = 0; i < HISTO_LENGTH; i++)
            rotHist[i].reserve(500);
        const float factor = HISTO_LENGTH / 360.0f;

        // We perform the matching over ORB that belong to the same vocabulary node (at a certain level)
        // 将属于同一节点(特定层)的ORB特征进行匹配
        DBoW2::FeatureVector::const_iterator KFit = vFeatVecKF.begin();
        DBoW2::FeatureVector::const_iterator KFend = vFeatVecKF.end();
        DBoW2::FeatureVector::const_iterator Fit = F.mFeatVec.begin();
        DBoW2::FeatureVector::const_iterator Fend = F.mFeatVec.end();

        while (KFit != KFend && Fit != Fend) {
            // Step 1：分别取出属于同一node的ORB特征点(只有属于同一node，才有可能是匹配点)
            // first 元素就是node id，遍历
            if (KFit->first == Fit->first) {
                // second 是该node内存储的feature index
                const vector<unsigned int> vKPsIdxInKF = KFit->second;
                const vector<unsigned int> vKPsIdxInF = Fit->second;

                // Step 2：遍历KF中属于该node的特征点
                for (size_t iKF = 0; iKF < vKPsIdxInKF.size(); iKF++) {
                    const unsigned int KPIdxInKF = vKPsIdxInKF[iKF];
                    MapPoint *pMP = vpMapPointsKF[KPIdxInKF];
                    if (!pMP)
                        continue;
                    if (pMP->isBad())
                        continue;
                    // 取出关键帧KF中该特征对应的描述子
                    const cv::Mat &dKF = pKF->mDescriptors.row(KPIdxInKF);

                    int BestDist = INT_MAX;
                    int BestIdxInF = -1;
                    int SecondDist = INT_MAX;
                    // Step 3：遍历F中属于该node的特征点，寻找最佳匹配点
                    for (size_t iF = 0; iF < vKPsIdxInF.size(); iF++) {
                        // 这里的realIdxF是指普通帧该节点中特征点的索引
                        const unsigned int KPIdxInF = vKPsIdxInF[iF];
                        // 如果地图点存在，说明这个点已经被匹配过了，不再匹配，加快速度
                        if (vpMPMatches[KPIdxInF])
                            continue;
                        // 取出普通帧F中该特征对应的描述子
                        const cv::Mat &dF = F.mDescriptorsLeft.row(KPIdxInF);
                        // 计算描述子的距离
                        const int dist = GetDescriptorDistance(dKF, dF);
                        // 遍历，记录最佳距离、最佳距离对应的索引、次佳距离等
                        if (dist < BestDist) {
                            SecondDist = BestDist;
                            BestDist = dist;
                            BestIdxInF = KPIdxInF;
                        } else if (dist < SecondDist) {
                            SecondDist = dist;
                        }
                    }
                    // Step 4：根据阈值 和 角度投票剔除误匹配
                    // Step 4.1：第一关筛选：匹配距离必须小于设定阈值
                    if (BestDist <= TH_LOW) {
                        // Step 4.2：第二关筛选：最佳匹配比次佳匹配明显要好，那么最佳匹配才真正靠谱
                        if (static_cast<float>(BestDist) < mfNNratio * static_cast<float>(SecondDist)) {
                            // Step 4.3：记录成功匹配特征点的对应的地图点(来自关键帧)
                            vpMPMatches[BestIdxInF] = pMP;
                            // 这里的realIdxKF是当前遍历到的关键帧的特征点id
                            const cv::KeyPoint &KPInKf = pKF->mvKPsUn[KPIdxInKF];
                            // Step 4.4：计算匹配点旋转角度差所在的直方图
                            if (mbCheckOrientation) {
                                cv::KeyPoint &KPInF = F.mvKPsLeft[BestIdxInF];
                                // 所有的特征点的角度变化应该是一致的，通过直方图统计得到最准确的角度变化值
                                float rot = KPInKf.angle - KPInF.angle;
                                if (rot < 0.0)
                                    rot += 360.0f;
                                int bin = round(rot * factor);// 将rot分配到bin组, 四舍五入, 其实就是离散到对应的直方图组中
                                if (bin == HISTO_LENGTH)
                                    bin = 0;
                                assert(bin >= 0 && bin < HISTO_LENGTH);
                                rotHist[bin].emplace_back(BestIdxInF);
                            }
                            nmatches++;
                        }
                    }
                }
                KFit++;
                Fit++;
            } else if (KFit->first < Fit->first) {
                KFit = vFeatVecKF.lower_bound(Fit->first);
            } else {
                Fit = F.mFeatVec.lower_bound(KFit->first);
            }
        }
        // Step 5 根据方向剔除误匹配的点
        if (mbCheckOrientation) {
            int ind1 = -1;
            int ind2 = -1;
            int ind3 = -1;
            // 筛选出在旋转角度差落在在直方图区间内数量最多的前三个bin的索引
            ComputeThreeMaxima(rotHist, HISTO_LENGTH, ind1, ind2, ind3);
            for (int i = 0; i < HISTO_LENGTH; i++) {
                // 如果特征点的旋转角度变化量属于这三个组，则保留
                if (i == ind1 || i == ind2 || i == ind3)
                    continue;
                // 剔除掉不在前三的匹配对，因为他们不符合“主流旋转方向”  
                for (size_t j = 0, jend = rotHist[i].size(); j < jend; j++) {
                    vpMPMatches[rotHist[i][j]] = static_cast<MapPoint *>(NULL);
                    nmatches--;
                }
            }
        }
        return nmatches;
    }

    int
    ORBmatcher::SearchMatchKFAndMPsByProject(KeyFrame *pKF, Sophus::Sim3f &Scw, const vector<MapPoint *> &vpCandidMPs,
                                             vector<MapPoint *> &vpMatchedMPs, int nThProjRad, float ratioHamming) {
        Sophus::SE3f Tcw = Sophus::SE3f(Scw.rotationMatrix(), Scw.translation() / Scw.scale());
        Eigen::Vector3f Ow = Tcw.inverse().translation();

        // Set of MapPoints already found in the KeyFrame
        set<MapPoint *> spAlreadyFound(vpMatchedMPs.begin(), vpMatchedMPs.end());
        spAlreadyFound.erase(static_cast<MapPoint *>(NULL));

        int nmatches = 0, bestDist, bestIdx;

        // For each Candidate MapPoint ProjectMono and Match
        for (int iMP = 0, iendMP = vpCandidMPs.size(); iMP < iendMP; iMP++) {
            MapPoint *pMP = vpCandidMPs[iMP];
            if (!pMP) {
                continue;
            }
            if (pMP->isBad()) {
                continue;
            }
            if (spAlreadyFound.count(pMP)) {
                continue;
            }
            if (!GetBestKPIdxInKFToMP(pKF, Tcw, pMP, nThProjRad, bestDist, bestIdx, false)) {
                continue;
            }
//            if (vpMatchedMPs[bestIdx]){
//                continue;
//            }
            if (bestDist <= TH_LOW * ratioHamming) {
                vpMatchedMPs[bestIdx] = pMP;
                nmatches++;
            }
        }
        //  Step 3 返回新的成功匹配的点对的数目
        return nmatches;
    }

    /*
    * @brief 通过词袋，对关键帧的特征点进行跟踪，该函数用于闭环检测时两个关键帧间的特征点匹配
    * @details 通过bow对pKF和F中的特征点进行快速匹配（不属于同一node的特征点直接跳过匹配） 
    * 对属于同一node的特征点通过描述子距离进行匹配 
    * 通过距离阈值、比例阈值和角度投票进行剔除误匹配
    * @param  pKF1               KeyFrame1
    * @param  pKF2               KeyFrame2
    * @param  vpMatches12        pKF2中与pKF1匹配的MapPoint，vpMatches12[i]表示匹配的地图点，null表示没有匹配，i表示匹配的pKF1 特征点索引
    * @return                    成功匹配的数量
    */
    int ORBmatcher::SearchMatchKFAndKFByBoW(KeyFrame *pKF1, KeyFrame *pKF2, vector<MapPoint *> &vpMatches12) {
        // Step 1 分别取出两个关键帧的特征点、BoW 向量、地图点、描述子
        const vector<cv::KeyPoint> &vKeysUn1 = pKF1->mvKPsUn;
        const DBoW2::FeatureVector &vFeatVec1 = pKF1->mFeatVec;
        const vector<MapPoint *> vpMapPoints1 = pKF1->GetVectorMapPointsInKF();
        const cv::Mat &Descriptors1 = pKF1->mDescriptors;

        const vector<cv::KeyPoint> &vKeysUn2 = pKF2->mvKPsUn;
        const DBoW2::FeatureVector &vFeatVec2 = pKF2->mFeatVec;
        const vector<MapPoint *> vpMapPoints2 = pKF2->GetVectorMapPointsInKF();
        const cv::Mat &Descriptors2 = pKF2->mDescriptors;

        // 保存匹配结果
        vpMatches12 = vector<MapPoint *>(vpMapPoints1.size(), static_cast<MapPoint *>(NULL));
        vector<bool> vbMatched2(vpMapPoints2.size(), false);

        // Step 2 构建旋转直方图，HISTO_LENGTH = 30
        vector<int> rotHist[HISTO_LENGTH];
        for (int i = 0; i < HISTO_LENGTH; i++)
            rotHist[i].reserve(500);

        const float factor = HISTO_LENGTH / 360.0f;
        int nmatches = 0;

        DBoW2::FeatureVector::const_iterator f1it = vFeatVec1.begin();
        DBoW2::FeatureVector::const_iterator f2it = vFeatVec2.begin();
        DBoW2::FeatureVector::const_iterator f1end = vFeatVec1.end();
        DBoW2::FeatureVector::const_iterator f2end = vFeatVec2.end();

        while (f1it != f1end && f2it != f2end) {
            // Step 3 开始遍历，分别取出属于同一node的特征点(只有属于同一node，才有可能是匹配点)
            if (f1it->first == f2it->first) {
                // 遍历KF中属于该node的特征点
                for (size_t i1 = 0, iend1 = f1it->second.size(); i1 < iend1; i1++) {
                    const size_t idx1 = f1it->second[i1];

                    MapPoint *pMP1 = vpMapPoints1[idx1];
                    if (!pMP1)
                        continue;
                    if (pMP1->isBad())
                        continue;

                    const cv::Mat &d1 = Descriptors1.row(idx1);

                    int bestDist1 = INT_MAX;
                    int bestIdx2 = -1;
                    int bestDist2 = INT_MAX;

                    // Step 4 遍历KF2中属于该node的特征点，找到了最优及次优匹配点
                    for (size_t i2 = 0, iend2 = f2it->second.size(); i2 < iend2; i2++) {
                        const size_t idx2 = f2it->second[i2];

                        MapPoint *pMP2 = vpMapPoints2[idx2];

                        // 如果已经有匹配的点，或者遍历到的特征点对应的地图点无效
                        if (vbMatched2[idx2] || !pMP2)
                            continue;

                        if (pMP2->isBad())
                            continue;

                        const cv::Mat &d2 = Descriptors2.row(idx2);

                        int dist = GetDescriptorDistance(d1, d2);

                        if (dist < bestDist1) {
                            bestDist2 = bestDist1;
                            bestDist1 = dist;
                            bestIdx2 = idx2;
                        } else if (dist < bestDist2) {
                            bestDist2 = dist;
                        }
                    }

                    // Step 5 对匹配结果进行检查，满足阈值、最优/次优比例，记录旋转直方图信息
                    if (bestDist1 < TH_LOW) {
                        if (static_cast<float>(bestDist1) < mfNNratio * static_cast<float>(bestDist2)) {
                            vpMatches12[idx1] = vpMapPoints2[bestIdx2];
                            vbMatched2[bestIdx2] = true;

                            if (mbCheckOrientation) {
                                float rot = vKeysUn1[idx1].angle - vKeysUn2[bestIdx2].angle;
                                if (rot < 0.0)
                                    rot += 360.0f;
                                int bin = round(rot * factor);
                                if (bin == HISTO_LENGTH)
                                    bin = 0;
                                assert(bin >= 0 && bin < HISTO_LENGTH);
                                rotHist[bin].emplace_back(idx1);
                            }
                            nmatches++;
                        }
                    }
                }

                f1it++;
                f2it++;
            } else if (f1it->first < f2it->first) {
                f1it = vFeatVec1.lower_bound(f2it->first);
            } else {
                f2it = vFeatVec2.lower_bound(f1it->first);
            }
        }

        // Step 6 检查旋转直方图分布，剔除差异较大的匹配
        if (mbCheckOrientation) {
            int ind1 = -1;
            int ind2 = -1;
            int ind3 = -1;
            ComputeThreeMaxima(rotHist, HISTO_LENGTH, ind1, ind2, ind3);
            for (int i = 0; i < HISTO_LENGTH; i++) {
                if (i == ind1 || i == ind2 || i == ind3)
                    continue;
                for (size_t j = 0, jend = rotHist[i].size(); j < jend; j++) {
                    vpMatches12[rotHist[i][j]] = static_cast<MapPoint *>(NULL);
                    nmatches--;
                }
            }
        }
        return nmatches;
    }

    int ORBmatcher::SearchKFAndKFByTriangulation(KeyFrame *pKF1, KeyFrame *pKF2,
                                                 vector<pair<size_t, size_t> > &vMatchedPairs, const bool bOnlyStereo,
                                                 const bool bCoarse) {
        const DBoW2::FeatureVector &vFeatVec1 = pKF1->mFeatVec;
        const DBoW2::FeatureVector &vFeatVec2 = pKF2->mFeatVec;

        //Compute epipole in second image
        // Step 1 计算KF1的相机中心在KF2图像平面的二维像素坐标
        Sophus::SE3f T1w = pKF1->GetPose();
        Sophus::SE3f T2w = pKF2->GetPose();
        Sophus::SE3f Tw2 = pKF2->GetPoseInverse(); // for convenience
        Eigen::Vector3f Cw = pKF1->GetCameraCenter();
        Eigen::Vector3f C2 = T2w * Cw;

        Eigen::Vector2f ep = pKF2->mpCamera->ProjectMPToKP(C2);
        Sophus::SE3f T12;
        Sophus::SE3f Tll, Tlr, Trl, Trr;
        Eigen::Matrix3f R12; // for fastest computation
        Eigen::Vector3f t12; // for fastest computation

        GeometricCamera *pCamera1 = pKF1->mpCamera, *pCamera2 = pKF2->mpCamera;

        T12 = T1w * Tw2;
        R12 = T12.rotationMatrix();
        t12 = T12.translation();

        Eigen::Matrix3f Rll = Tll.rotationMatrix(), Rlr = Tlr.rotationMatrix(), Rrl = Trl.rotationMatrix(), Rrr = Trr.rotationMatrix();
        Eigen::Vector3f tll = Tll.translation(), tlr = Tlr.translation(), trl = Trl.translation(), trr = Trr.translation();

        // Find matches between not tracked keypoints
        // Matching speed-up by ORB Vocabulary
        // Compare only ORB that share the same node

        int nmatches = 0;
        // 记录匹配是否成功，避免重复匹配
        vector<bool> vbMatched2(pKF2->mnKPsLeftNum, false);
        vector<int> vMatches12(pKF1->mnKPsLeftNum, -1);
        // 用于统计匹配点对旋转差的直方图
        vector<int> rotHist[HISTO_LENGTH];
        for (int i = 0; i < HISTO_LENGTH; i++)
            rotHist[i].reserve(500);

        const float factor = HISTO_LENGTH / 360.0f;

        // We perform the matching over ORB that belong to the same vocabulary node (at a certain level)
        // Step 2 利用BoW加速匹配：只对属于同一节点(特定层)的ORB特征进行匹配
        // FeatureVector其实就是一个map类，那就可以直接获取它的迭代器进行遍历
        // FeatureVector的数据结构类似于：{(node1,feature_vector1) (node2,feature_vector2)...}
        // f1it->first对应node编号，f1it->second对应属于该node的所有特特征点编号
        DBoW2::FeatureVector::const_iterator f1it = vFeatVec1.begin();
        DBoW2::FeatureVector::const_iterator f2it = vFeatVec2.begin();
        DBoW2::FeatureVector::const_iterator f1end = vFeatVec1.end();
        DBoW2::FeatureVector::const_iterator f2end = vFeatVec2.end();

        // Step 2.1：遍历pKF1和pKF2中的node节点
        while (f1it != f1end && f2it != f2end) {
            // 如果f1it和f2it属于同一个node节点才会进行匹配，这就是BoW加速匹配原理
            if (f1it->first == f2it->first) {
                // Step 2.2：遍历属于同一node节点(id：f1it->first)下的所有特征点
                for (size_t i1 = 0, iend1 = f1it->second.size(); i1 < iend1; i1++) {
                    // 获取pKF1中属于该node节点的所有特征点索引
                    const size_t idx1 = f1it->second[i1];

                    // Step 2.3：通过特征点索引idx1在pKF1中取出对应的MapPoint
                    MapPoint *pMP1 = pKF1->GetIdxMapPoint(idx1);

                    // If there is already a MapPoint skip
                    // 由于寻找的是未匹配的特征点，所以pMP1应该为NULL
                    if (pMP1) {
                        continue;
                    }
                    // 如果mvuRight中的值大于0，表示是双目，且该特征点有深度值
                    const bool bStereo1 = (pKF1->mvfXInRight[idx1] >= 0);
                    if (bOnlyStereo)
                        if (!bStereo1)
                            continue;

                    // Step 2.4：通过特征点索引idx1在pKF1中取出对应的特征点
                    const cv::KeyPoint &kp1 = pKF1->mvKPsUn[idx1];
                    //if(bRight1) continue;
                    // 通过特征点索引idx1在pKF1中取出对应的特征点的描述子
                    const cv::Mat &d1 = pKF1->mDescriptors.row(idx1);

                    int bestDist = TH_LOW;
                    int bestIdx2 = -1;

                    // Step 2.5：遍历该node节点下(f2it->first)对应KF2中的所有特征点
                    for (size_t i2 = 0, iend2 = f2it->second.size(); i2 < iend2; i2++) {
                        // 获取pKF2中属于该node节点的所有特征点索引
                        size_t idx2 = f2it->second[i2];

                        // 通过特征点索引idx2在pKF2中取出对应的MapPoint
                        MapPoint *pMP2 = pKF2->GetIdxMapPoint(idx2);

                        // If we have already matched or there is a MapPoint skip
                        // 如果pKF2当前特征点索引idx2已经被匹配过或者对应的3d点非空，那么跳过这个索引idx2
                        if (vbMatched2[idx2] || pMP2)
                            continue;

                        const bool bStereo2 = (pKF2->mvfXInRight[idx2] >= 0);

                        if (bOnlyStereo)
                            if (!bStereo2)
                                continue;

                        // 通过特征点索引idx2在pKF2中取出对应的特征点的描述子
                        const cv::Mat &d2 = pKF2->mDescriptors.row(idx2);

                        // Step 2.6 计算idx1与idx2在两个关键帧中对应特征点的描述子距离
                        const int dist = GetDescriptorDistance(d1, d2);

                        if (dist > TH_LOW || dist > bestDist)
                            continue;

                        // 通过特征点索引idx2在pKF2中取出对应的特征点
                        const cv::KeyPoint &kp2 = pKF2->mvKPsUn[idx2];
                        //? 为什么双目就不需要判断像素点到极点的距离的判断？
                        // 因为双目模式下可以左右互匹配恢复三维点
                        if (!bStereo1 && !bStereo2) {
                            const float distex = ep(0) - kp2.pt.x;
                            const float distey = ep(1) - kp2.pt.y;
                            // Step 2.7 极点e2到kp2的像素距离如果小于阈值th,认为kp2对应的MapPoint距离pKF1相机太近，跳过该匹配点对
                            // 作者根据kp2金字塔尺度因子(scale^n，scale=1.2，n为层数)定义阈值th
                            // 金字塔层数从0到7，对应距离 sqrt(100*pKF2->mvfScaleFactors[kp2.octave]) 是10-20个像素
                            //? 对这个阈值的有效性持怀疑态度
                            if (distex * distex + distey * distey < 100 * pKF2->mvScaleFactors[kp2.octave]) {
                                continue;
                            }
                        }

                        // Step 2.8 计算特征点kp2到kp1对应极线的距离是否小于阈值
                        if (bCoarse ||
                            pCamera1->EpipolarConstrain(pCamera2, kp1, kp2, R12, t12, pKF1->mvfLevelSigma2[kp1.octave],
                                                        pKF2->mvfLevelSigma2[kp2.octave])) // MODIFICATION_2
                        {
                            // bestIdx2，bestDist 是 kp1 对应 KF2中的最佳匹配点 index及匹配距离
                            bestIdx2 = idx2;
                            bestDist = dist;
                        }
                    }

                    if (bestIdx2 >= 0) {
                        const cv::KeyPoint &kp2 = pKF2->mvKPsUn[bestIdx2];
                        // 记录匹配结果 
                        vMatches12[idx1] = bestIdx2;
                        // !记录已经匹配，避免重复匹配。原作者漏掉！可以添加下面代码
                        // vbMatched2[bestIdx2]=true;
                        nmatches++;

                        // 记录旋转差直方图信息
                        if (mbCheckOrientation) {
                            // angle：角度，表示匹配点对的方向差。
                            float rot = kp1.angle - kp2.angle;
                            if (rot < 0.0)
                                rot += 360.0f;
                            int bin = round(rot * factor);
                            if (bin == HISTO_LENGTH)
                                bin = 0;
                            assert(bin >= 0 && bin < HISTO_LENGTH);
                            rotHist[bin].emplace_back(idx1);
                        }
                    }
                }

                f1it++;
                f2it++;
            } else if (f1it->first < f2it->first) {
                f1it = vFeatVec1.lower_bound(f2it->first);
            } else {
                f2it = vFeatVec2.lower_bound(f1it->first);
            }
        }

        // Step 3 用旋转差直方图来筛掉错误匹配对
        if (mbCheckOrientation) {
            int ind1 = -1;
            int ind2 = -1;
            int ind3 = -1;

            ComputeThreeMaxima(rotHist, HISTO_LENGTH, ind1, ind2, ind3);

            for (int i = 0; i < HISTO_LENGTH; i++) {
                if (i == ind1 || i == ind2 || i == ind3)
                    continue;
                for (size_t j = 0, jend = rotHist[i].size(); j < jend; j++) {
                    vbMatched2[vMatches12[rotHist[i][j]]] = false;  // !清除匹配关系。原作者漏掉！
                    vMatches12[rotHist[i][j]] = -1;
                    nmatches--;
                }
            }

        }

        // Step 4 存储匹配关系，下标是关键帧1的特征点id，存储的是关键帧2的特征点id
        vMatchedPairs.clear();
        vMatchedPairs.reserve(nmatches);

        for (size_t i = 0, iend = vMatches12.size(); i < iend; i++) {
            if (vMatches12[i] < 0)
                continue;
            vMatchedPairs.emplace_back(make_pair(i, vMatches12[i]));
        }

        return nmatches;
    }

    int ORBmatcher::SearchReplaceKFAndMPsByProject(KeyFrame *pKF, const vector<MapPoint *> &vpMapPoints,
                                                   const float nThProjRad,
                                                   const bool bRight) {
        Sophus::SE3f Tcw = pKF->GetPose();
        int nFused = 0, bestDist, bestIdx;
        const int nMPs = vpMapPoints.size();
        for (int i = 0; i < nMPs; i++) {
            MapPoint *pMP = vpMapPoints[i];
            if (!pMP) {
                continue;
            }
            if (pMP->isBad()) {
                continue;
            }
            if (pMP->IsInKeyFrame(pKF)) {
                continue;
            }
            if (!GetBestKPIdxInKFToMP(pKF, Tcw, pMP, nThProjRad, bestDist, bestIdx, true)) {
                continue;
            }
            if (bestDist <= TH_LOW) {
                MapPoint *pMPinKF = pKF->GetIdxMapPoint(bestIdx);
                if (pMPinKF) {
                    // 如果最佳匹配点有对应有效地图点，选择被观测次数最多的那个替换
                    if (!pMPinKF->isBad()) {
                        if (pMPinKF->GetObsTimes() > pMP->GetObsTimes())
                            pMP->Replace(pMPinKF);
                        else
                            pMPinKF->Replace(pMP);
                    }
                } else {
                    // 如果最佳匹配点没有对应地图点，添加观测信息
                    pMP->AddObsKFAndLRIdx(pKF, bestIdx);
                    pKF->AddMapPoint(pMP, bestIdx);
                }
                nFused++;
            }
        }
        cout << nFused << "*";
        return nFused;
    }

    bool
    ORBmatcher::GetBestKPIdxInKFToMP(KeyFrame *pKF, Sophus::SE3f &Tcw, MapPoint *pMP, float nThProjRad, int &bestDist,
                                     int &bestIdx, bool bProjError) {
        Eigen::Vector3f Ow = Tcw.inverse().translation();
        Eigen::Vector3f p3Dw = pMP->GetWorldPos();
        Eigen::Vector3f p3Dc = Tcw * p3Dw;
        if (p3Dc(2) < 0.0f)
            return false;
        const Eigen::Vector2f uv = pKF->mpCamera->ProjectMPToKP(p3Dc);
        if (!pKF->IsInImage(uv(0), uv(1)))
            return false;
        const float maxDistance = pMP->GetMaxDistanceInvariance();
        const float minDistance = pMP->GetMinDistanceInvariance();
        Eigen::Vector3f PO = p3Dw - Ow;
        const float dist3D = PO.norm();
        if (dist3D < minDistance || dist3D > maxDistance)
            return false;
        Eigen::Vector3f Pn = pMP->GetNormal();
        if (PO.dot(Pn) < 0.5 * dist3D)
            return false;
        const int nPredictedLevel = pMP->PredictScale(dist3D, pKF);
        const float radius = nThProjRad * pKF->mvScaleFactors[nPredictedLevel];
        const vector<size_t> vIndices = pKF->GetFeaturesInArea(uv(0), uv(1), radius);
        if (vIndices.empty())
            return false;

        const cv::Mat dMP = pMP->GetDescriptor();
        const float invz = 1 / p3Dc(2);
        const float ur = uv(0) - pKF->mfBaselineFocal * invz;
        bestDist = INT_MAX;
        bestIdx = -1;
        for (vector<size_t>::const_iterator vit = vIndices.begin(); vit != vIndices.end(); vit++) {
            const size_t Idx = *vit;
            const cv::KeyPoint &kp = pKF->mvKPsUn[Idx];
            const int &kpLevel = kp.octave;
            if (kpLevel < nPredictedLevel - 1 || kpLevel > nPredictedLevel)
                continue;

            if (bProjError) {
                const float &kpx = kp.pt.x;
                const float &kpy = kp.pt.y;
                const float ex = uv(0) - kpx;
                const float ey = uv(1) - kpy;
                if (pKF->mvfXInRight[Idx] >= 0) {
                    const float &kpr = pKF->mvfXInRight[Idx];
                    const float er = ur - kpr;
                    const float e2 = ex * ex + ey * ey + er * er;
                    //自由度为3, 误差小于1个像素,这种事情95%发生的概率对应卡方检验阈值为7.82
                    if (e2 * pKF->mvfInvLevelSigma2[kpLevel] > 7.8)
                        continue;
                } else {
                    const float e2 = ex * ex + ey * ey;
                    // 自由度为2的，卡方检验阈值5.99（假设测量有一个像素的偏差）
                    if (e2 * pKF->mvfInvLevelSigma2[kpLevel] > 5.99)
                        continue;
                }
            }

            const cv::Mat &dKF = pKF->mDescriptors.row(Idx);
            int dist = GetDescriptorDistance(dMP, dKF);
            if (dist < bestDist) {
                bestDist = dist;
                bestIdx = Idx;
            }
        }
        if (bestIdx != -1) {
            return true;
        } else {
            return false;
        }
    }

    /**
     * @brief 闭环矫正中使用。将当前关键帧闭环匹配上的关键帧及其共视关键帧组成的地图点投影到当前关键帧，融合地图点
     * 
     * @param[in] pKF                   当前关键帧
     * @param[in] Scw                   当前关键帧经过闭环Sim3 后的世界到相机坐标系的Sim变换
     * @param[in] vpPoints              与当前关键帧闭环匹配上的关键帧及其共视关键帧组成的地图点
     * @param[in] nThProjRad                    搜索范围系数
     * @param[out] vpReplacePoint       替换的地图点
     * @return int                      融合（替换和新增）的地图点数目
     */
    int ORBmatcher::Fuse(KeyFrame *pKF, Sophus::Sim3f &Scw, const vector<MapPoint *> &vpPoints, float nThProjRad,
                         vector<MapPoint *> &vpReplacePoint) {
        Sophus::SE3f Tcw = Sophus::SE3f(Scw.rotationMatrix(), Scw.translation() / Scw.scale());
        const set<MapPoint *> spAlreadyFound = pKF->GetMapPoints();
        int nFused = 0, bestDist, bestIdx;
        const int nPoints = vpPoints.size();
        for (int iMP = 0; iMP < nPoints; iMP++) {
            MapPoint *pMP = vpPoints[iMP];
            if (!pMP) {
                continue;
            }
            if (pMP->isBad()) {
                continue;
            }
            if (spAlreadyFound.count(pMP)) {
                continue;
            }
            if (!GetBestKPIdxInKFToMP(pKF, Tcw, pMP, nThProjRad, bestDist, bestIdx, false)) {
                continue;
            }
            // If there is already a MapPoint replace otherwise add new measurement
            // Step 7 替换或新增地图点
            if (bestDist <= TH_LOW) {
                MapPoint *pMPinKF = pKF->GetIdxMapPoint(bestIdx);
                if (pMPinKF) {
                    // 如果这个地图点已经存在，则记录要替换信息
                    // 这里不能直接替换，原因是需要对地图点加锁后才能替换，否则可能会crash。所以先记录，在加锁后替换
                    if (!pMPinKF->isBad())
                        vpReplacePoint[iMP] = pMPinKF;
                } else {
                    // 如果这个地图点不存在，直接添加
                    pMP->AddObsKFAndLRIdx(pKF, bestIdx);
                    pKF->AddMapPoint(pMP, bestIdx);
                }
                nFused++;
            }
        }
        cout << nFused << "+";
        // 融合（替换和新增）的地图点数目
        return nFused;
    }


    /**
     * @brief 将上一帧跟踪的地图点投影到当前帧，并且搜索匹配点。用于跟踪前一帧
     * 步骤
     * Step 1 建立旋转直方图，用于检测旋转一致性
     * Step 2 计算当前帧和前一帧的平移向量
     * Step 3 对于前一帧的每一个地图点，通过相机投影模型，得到投影到当前帧的像素坐标
     * Step 4 根据相机的前后前进方向来判断搜索尺度范围
     * Step 5 遍历候选匹配点，寻找距离最小的最佳匹配点 
     * Step 6 计算匹配点旋转角度差所在的直方图
     * Step 7 进行旋转一致检测，剔除不一致的匹配
     * @param[in] CurrentFrame          当前帧
     * @param[in] LastFrame             上一帧
     * @param[in] nThProjRad                    搜索范围阈值，默认单目为7，双目15
     * @param[in] bMono                 是否为单目
     * @return int                      成功匹配的数量
     */
    int ORBmatcher::SearchFrameAndFrameByProject(Frame &CurrentFrame, const Frame &LastFrame, const float nThProjRad,
                                                 const bool bMono) {
        int nmatches = 0;

        // Rotation Histogram (to check rotation consistency)
        // Step 1 建立旋转直方图，用于检测旋转一致性
        vector<int> rotHist[HISTO_LENGTH];
        for (int i = 0; i < HISTO_LENGTH; i++)
            rotHist[i].reserve(500);
        const float factor = HISTO_LENGTH / 360.0f;

        // Step 2 计算当前帧和前一帧的平移向量
        //当前帧的相机位姿
        const Sophus::SE3f Tcw = CurrentFrame.GetPose();
        const Eigen::Vector3f twc = Tcw.inverse().translation();

        const Sophus::SE3f Tlw = LastFrame.GetPose();
        const Eigen::Vector3f tlc = Tlw * twc;

        // 判断前进还是后退
        const bool bForward = tlc(2) > CurrentFrame.mfBaseline && !bMono;     // 非单目情况，如果Z大于基线，则表示相机明显前进
        const bool bBackward = -tlc(2) > CurrentFrame.mfBaseline && !bMono;   // 非单目情况，如果-Z小于基线，则表示相机明显后退

        //  Step 3 对于前一帧的每一个地图点，通过相机投影模型，得到投影到当前帧的像素坐标
        for (int i = 0; i < LastFrame.mnKPsLeftNum; i++) {
            MapPoint *pMP = LastFrame.mvpMPs[i];
            if (pMP) {
                if (!LastFrame.mvbOutlier[i]) {
                    // 对上一帧有效的MapPoints投影到当前帧坐标系
                    Eigen::Vector3f x3Dw = pMP->GetWorldPos();
                    Eigen::Vector3f x3Dc = Tcw * x3Dw;

                    const float xc = x3Dc(0);
                    const float yc = x3Dc(1);
                    const float invzc = 1.0 / x3Dc(2);

                    if (invzc < 0)
                        continue;

                    // 投影到当前帧中
                    Eigen::Vector2f uv = CurrentFrame.mpCamera->ProjectMPToKP(x3Dc);

                    if (uv(0) < CurrentFrame.mfMinX || uv(0) > CurrentFrame.mfMaxX)
                        continue;
                    if (uv(1) < CurrentFrame.mfMinY || uv(1) > CurrentFrame.mfMaxY)
                        continue;
                    // 认为投影前后地图点的尺度信息不变
                    int nLastOctave = LastFrame.mvKPsLeft[i].octave;

                    // Search in a window. Size depends on scale
                    // 单目：th = 7，双目：th = 15
                    float radius = nThProjRad * CurrentFrame.mvfScaleFactors[nLastOctave]; // 尺度越大，搜索范围越大

                    // 记录候选匹配点的id
                    vector<size_t> vIndices2;

                    // Step 4 根据相机的前后前进方向来判断搜索尺度范围。
                    // 以下可以这么理解，例如一个有一定面积的圆点，在某个尺度n下它是一个特征点
                    // 当相机前进时，圆点的面积增大，在某个尺度m下它是一个特征点，由于面积增大，则需要在更高的尺度下才能检测出来
                    // 当相机后退时，圆点的面积减小，在某个尺度m下它是一个特征点，由于面积减小，则需要在更低的尺度下才能检测出来
                    if (bForward)  // 前进,则上一帧兴趣点在所在的尺度nLastOctave<=nCurOctave
                        vIndices2 = CurrentFrame.GetFeaturesInArea(uv(0), uv(1), radius, nLastOctave);
                    else if (bBackward)  // 后退,则上一帧兴趣点在所在的尺度0<=nCurOctave<=nLastOctave
                        vIndices2 = CurrentFrame.GetFeaturesInArea(uv(0), uv(1), radius, 0, nLastOctave);
                    else  // 在[nLastOctave-1, nLastOctave+1]中搜索
                        vIndices2 = CurrentFrame.GetFeaturesInArea(uv(0), uv(1), radius, nLastOctave - 1,
                                                                   nLastOctave + 1);

                    if (vIndices2.empty())
                        continue;

                    const cv::Mat dMP = pMP->GetDescriptor();

                    int bestDist = INT_MAX;
                    int bestIdx2 = -1;

                    // Step 5 遍历候选匹配点，寻找距离最小的最佳匹配点 
                    for (vector<size_t>::const_iterator vit = vIndices2.begin(), vend = vIndices2.end();
                         vit != vend; vit++) {
                        const size_t i2 = *vit;

                        // 如果该特征点已经有对应的MapPoint了,则退出该次循环
                        if (CurrentFrame.mvpMPs[i2])
                            if (CurrentFrame.mvpMPs[i2]->GetObsTimes() > 0)
                                continue;

                        if (CurrentFrame.mvfXInRight[i2] > 0) {
                            // 双目和rgbd的情况，需要保证右图的点也在搜索半径以内
                            const float ur = uv(0) - CurrentFrame.mfBaselineFocal * invzc;
                            const float er = fabs(ur - CurrentFrame.mvfXInRight[i2]);
                            if (er > radius)
                                continue;
                        }

                        const cv::Mat &d = CurrentFrame.mDescriptorsLeft.row(i2);

                        const int dist = GetDescriptorDistance(dMP, d);

                        if (dist < bestDist) {
                            bestDist = dist;
                            bestIdx2 = i2;
                        }
                    }

                    // 最佳匹配距离要小于设定阈值
                    if (bestDist <= TH_HIGH) {
                        CurrentFrame.mvpMPs[bestIdx2] = pMP;
                        nmatches++;

                        // Step 6 计算匹配点旋转角度差所在的直方图
                        if (mbCheckOrientation) {
                            cv::KeyPoint kpLF = LastFrame.mvKPsUn[i];
                            cv::KeyPoint kpCF = CurrentFrame.mvKPsUn[bestIdx2];
                            float rot = kpLF.angle - kpCF.angle;
                            if (rot < 0.0)
                                rot += 360.0f;
                            int bin = round(rot * factor);
                            if (bin == HISTO_LENGTH)
                                bin = 0;
                            assert(bin >= 0 && bin < HISTO_LENGTH);
                            rotHist[bin].emplace_back(bestIdx2);
                        }
                    }
                }
            }
        }

        //Apply rotation consistency
        //  Step 7 进行旋转一致检测，剔除不一致的匹配
        if (mbCheckOrientation) {
            int ind1 = -1;
            int ind2 = -1;
            int ind3 = -1;
            ComputeThreeMaxima(rotHist, HISTO_LENGTH, ind1, ind2, ind3);
            for (int i = 0; i < HISTO_LENGTH; i++) {
                // 对于数量不是前3个的点对，剔除
                if (i != ind1 && i != ind2 && i != ind3) {
                    for (size_t j = 0, jend = rotHist[i].size(); j < jend; j++) {
                        CurrentFrame.mvpMPs[rotHist[i][j]] = static_cast<MapPoint *>(NULL);
                        nmatches--;
                    }
                }
            }
        }
        return nmatches;
    }

    /**
     * @brief 通过投影的方式将关键帧中未匹配的地图点投影到当前帧中,进行匹配，并通过旋转直方图进行筛选
     * 
     * @param[in] CurrentFrame          当前帧
     * @param[in] pKF                   参考关键帧
     * @param[in] sAlreadyFound         已经找到的地图点集合，不会用于PNP
     * @param[in] nThProjRad                    匹配时搜索范围，会乘以金字塔尺度
     * @param[in] ORBdist               匹配的ORB描述子距离应该小于这个阈值    
     * @return int                      成功匹配的数量
     */
    int ORBmatcher::SearchFrameAndKFByProject(Frame &CurrentFrame, KeyFrame *pKF, const set<MapPoint *> &sAlreadyFound,
                                              const float nThProjRad, const int ORBdist) {
        int nmatches = 0;

        const Sophus::SE3f Tcw = CurrentFrame.GetPose();
        Eigen::Vector3f Ow = Tcw.inverse().translation();

        // Rotation Histogram (to check rotation consistency)
        // Step 1 建立旋转直方图，用于检测旋转一致性
        vector<int> rotHist[HISTO_LENGTH];
        for (int i = 0; i < HISTO_LENGTH; i++)
            rotHist[i].reserve(500);
        const float factor = HISTO_LENGTH / 360.0f;

        const vector<MapPoint *> vpMPs = pKF->GetVectorMapPointsInKF();

        // Step 2 遍历关键帧中的每个地图点，通过相机投影模型，得到投影到当前帧的像素坐标
        for (size_t i = 0, iend = vpMPs.size(); i < iend; i++) {
            MapPoint *pMP = vpMPs[i];

            if (pMP) {
                // 地图点存在 并且 不在已有地图点集合里
                if (!pMP->isBad() && !sAlreadyFound.count(pMP)) {
                    //ProjectMono
                    Eigen::Vector3f x3Dw = pMP->GetWorldPos();
                    Eigen::Vector3f x3Dc = Tcw * x3Dw;

                    const Eigen::Vector2f uv = CurrentFrame.mpCamera->ProjectMPToKP(x3Dc);

                    if (uv(0) < CurrentFrame.mfMinX || uv(0) > CurrentFrame.mfMaxX)
                        continue;
                    if (uv(1) < CurrentFrame.mfMinY || uv(1) > CurrentFrame.mfMaxY)
                        continue;

                    // Compute predicted scale level
                    Eigen::Vector3f PO = x3Dw - Ow;
                    float dist3D = PO.norm();

                    const float maxDistance = pMP->GetMaxDistanceInvariance();
                    const float minDistance = pMP->GetMinDistanceInvariance();

                    // Depth must be inside the scale pyramid of the image
                    if (dist3D < minDistance || dist3D > maxDistance)
                        continue;

                    //预测尺度
                    int nPredictedLevel = pMP->PredictScale(dist3D, &CurrentFrame);

                    // Search in a window
                    // 搜索半径和尺度相关
                    const float radius = nThProjRad * CurrentFrame.mvfScaleFactors[nPredictedLevel];

                    //  Step 3 搜索候选匹配点
                    const vector<size_t> vIndices2 = CurrentFrame.GetFeaturesInArea(uv(0), uv(1), radius,
                                                                                    nPredictedLevel - 1,
                                                                                    nPredictedLevel + 1);

                    if (vIndices2.empty())
                        continue;

                    const cv::Mat dMP = pMP->GetDescriptor();

                    int bestDist = INT_MAX;
                    int bestIdx2 = -1;
                    // Step 4 遍历候选匹配点，寻找距离最小的最佳匹配点 
                    for (vector<size_t>::const_iterator vit = vIndices2.begin(); vit != vIndices2.end(); vit++) {
                        const size_t i2 = *vit;
                        if (CurrentFrame.mvpMPs[i2])
                            continue;

                        const cv::Mat &d = CurrentFrame.mDescriptorsLeft.row(i2);

                        const int dist = GetDescriptorDistance(dMP, d);

                        if (dist < bestDist) {
                            bestDist = dist;
                            bestIdx2 = i2;
                        }
                    }

                    if (bestDist <= ORBdist) {
                        CurrentFrame.mvpMPs[bestIdx2] = pMP;
                        nmatches++;
                        // Step 5 计算匹配点旋转角度差所在的直方图
                        if (mbCheckOrientation) {
                            float rot = pKF->mvKPsUn[i].angle - CurrentFrame.mvKPsUn[bestIdx2].angle;
                            if (rot < 0.0)
                                rot += 360.0f;
                            int bin = round(rot * factor);
                            if (bin == HISTO_LENGTH)
                                bin = 0;
                            assert(bin >= 0 && bin < HISTO_LENGTH);
                            rotHist[bin].emplace_back(bestIdx2);
                        }
                    }

                }
            }
        }

        //  Step 6 进行旋转一致检测，剔除不一致的匹配
        if (mbCheckOrientation) {
            int ind1 = -1;
            int ind2 = -1;
            int ind3 = -1;

            ComputeThreeMaxima(rotHist, HISTO_LENGTH, ind1, ind2, ind3);

            for (int i = 0; i < HISTO_LENGTH; i++) {
                if (i != ind1 && i != ind2 && i != ind3) {
                    for (size_t j = 0, jend = rotHist[i].size(); j < jend; j++) {
                        CurrentFrame.mvpMPs[rotHist[i][j]] = NULL;
                        nmatches--;
                    }
                }
            }
        }

        return nmatches;
    }

    /**
     * @brief 筛选出在旋转角度差落在在直方图区间内数量最多的前三个bin的索引
     * 
     * @param[in] histo         匹配特征点对旋转方向差直方图
     * @param[in] L             直方图尺寸
     * @param[in & out] ind1          bin值第一大对应的索引
     * @param[in & out] ind2          bin值第二大对应的索引
     * @param[in & out] ind3          bin值第三大对应的索引
     */
    void ORBmatcher::ComputeThreeMaxima(vector<int> *histo, const int L, int &ind1, int &ind2, int &ind3) {
        int max1 = 0;
        int max2 = 0;
        int max3 = 0;

        for (int i = 0; i < L; i++) {
            const int s = histo[i].size();
            if (s > max1) {
                max3 = max2;
                max2 = max1;
                max1 = s;
                ind3 = ind2;
                ind2 = ind1;
                ind1 = i;
            } else if (s > max2) {
                max3 = max2;
                max2 = s;
                ind3 = ind2;
                ind2 = i;
            } else if (s > max3) {
                max3 = s;
                ind3 = i;
            }
        }

        // 如果差距太大了,说明次优的非常不好,这里就索性放弃了,都置为-1
        if (max2 < 0.1f * (float) max1) {
            ind2 = -1;
            ind3 = -1;
        } else if (max3 < 0.1f * (float) max1) {
            ind3 = -1;
        }
    }


    // Bit set count operation from
    // Hamming distance：两个二进制串之间的汉明距离，指的是其不同位数的个数
    // http://graphics.stanford.edu/~seander/bithacks.html#CountBitsSetParallel
    int ORBmatcher::GetDescriptorDistance(const cv::Mat &a, const cv::Mat &b) {
        const int *pa = a.ptr<int32_t>();
        const int *pb = b.ptr<int32_t>();

        int dist = 0;

        // 8*32=256bit

        for (int i = 0; i < 8; i++, pa++, pb++) {
            unsigned int v = *pa ^ *pb;        // 相等为0,不等为1
            // 下面的操作就是计算其中bit为1的个数了,这个操作看上面的链接就好
            // 其实我觉得也还阔以直接使用8bit的查找表,然后做32次寻址操作就完成了;不过缺点是没有利用好CPU的字长
            v = v - ((v >> 1) & 0x55555555);
            v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
            dist += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
        }

        return dist;
    }

} //namespace ORB_SLAM
