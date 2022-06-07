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

#include "MapDrawer.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include <pangolin/pangolin.h>
#include <mutex>

namespace ORB_SLAM3 {


    MapDrawer::MapDrawer(Atlas *pAtlas, const string &strSettingPath, Settings *settings) : mpAtlas(pAtlas) {
        mKeyFrameSize = settings->mfKeyFrameSize;
        mKeyFrameLineWidth = settings->mfKeyFrameLineWidth;
        mGraphLineWidth = settings->mfGraphLineWidth;
        mPointSize = settings->mfPointSize;
        mCameraSize = settings->mfCameraSize;
        mCameraLineWidth = settings->mfCameraLineWidth;
    }

    void MapDrawer::DrawMapPoints() {
        Map *pActiveMap = mpAtlas->GetCurrentMap();
        if (!pActiveMap)
            return;

        const vector<MapPoint *> &vpAllMPs = pActiveMap->GetAllMapPoints();
        const vector<MapPoint *> &vpRefMPs = pActiveMap->GetReferenceMapPoints();
        set<MapPoint *> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

        if (vpAllMPs.empty())
            return;

        glPointSize(mPointSize);
        glBegin(GL_POINTS);
        glColor3f(0.0, 0.0, 0.0);
        for (size_t i = 0, iend = vpAllMPs.size(); i < iend; i++) {
            if (vpAllMPs[i]->isBad() || spRefMPs.count(vpAllMPs[i]))
                continue;
            Eigen::Matrix<float, 3, 1> pos = vpAllMPs[i]->GetWorldPos();
            glVertex3f(pos(0), pos(1), pos(2));
        }
        glEnd();

        glPointSize(mPointSize);
        glBegin(GL_POINTS);
        glColor3f(1.0, 0.0, 0.0);
        for (set<MapPoint *>::iterator sit = spRefMPs.begin(), send = spRefMPs.end(); sit != send; sit++) {
            if ((*sit)->isBad())
                continue;
            Eigen::Matrix<float, 3, 1> pos = (*sit)->GetWorldPos();
            glVertex3f(pos(0), pos(1), pos(2));

        }
        glEnd();
    }

    void MapDrawer::DrawKeyFramesGraphs(const bool bDrawKF, const bool bDrawCovisGraph, const bool bDrawInertialGraph,
                                        const bool bDrawOptFixKF, const bool bHisoryMapKF) {
        const float &fFW = mKeyFrameSize;
        const float fFH = fFW * 0.75;
        const float fFZ = fFW * 0.6;

        Map *pActiveMap = mpAtlas->GetCurrentMap();
        // DEBUG LBA
        std::set<long unsigned int> sOptVisKFs = pActiveMap->msOptVisKFs;
        std::set<long unsigned int> sFixedKFs = pActiveMap->msFixedKFs;
        std::set<long unsigned int> sOptImuKFs = pActiveMap->msOptImuKFs;

        const vector<KeyFrame *> &vpRefKFs = pActiveMap->GetReferenceKeyFrames();
        set<KeyFrame *> spRefKFs(vpRefKFs.begin(), vpRefKFs.end());


        if (!pActiveMap)
            return;

        const vector<KeyFrame *> vpKFs = pActiveMap->GetAllKeyFrames();

        if (bDrawKF) {
            for (size_t i = 0; i < vpKFs.size(); i++) {
                KeyFrame *pKF = vpKFs[i];
                Eigen::Matrix4f Twc = pKF->GetPoseInverse().matrix();
                glPushMatrix();

                glMultMatrixf((GLfloat *) Twc.data());
                if (!pKF->GetParent()) // It is the first KF in the map
                {
                    glLineWidth(mKeyFrameLineWidth * 5);
                    glColor3f(1.0f, 0.0f, 0.0f);
                    glBegin(GL_LINES);
                } else {
                    //cout << "Child KF: " << vpKFs[i]->mnId << endl;
                    glLineWidth(mKeyFrameLineWidth);
                    if (bDrawOptFixKF) {
                        if (sOptVisKFs.find(pKF->mnId) != sOptVisKFs.end()) {
                            glColor3f(0.0f, 1.0f, 0.0f); // Green -> OptVis KFs
                        } else if (sOptImuKFs.find(pKF->mnId) != sOptImuKFs.end()) {
                            glColor3f(1.0f, 0.0f, 0.0f); // Red -> OptImu KFs
                        } else if (sFixedKFs.find(pKF->mnId) != sFixedKFs.end()) {
                            glColor3f(1.0f, 1.0f, 0.0f); // yellow -> Fixed KFs
                        }else {
                            glColor3f(0.0f, 0.0f, 1.0f); // Basic color
                        }
                    } else {
                        if (spRefKFs.count(pKF)) {
                            glColor3f(1.0f, 0.0f, 0.0f);
                        } else {
                            glColor3f(0.0f, 0.0f, 1.0f);
                        }
                    }
                    glBegin(GL_LINES);
                }

                glVertex3f(0, 0, 0);
                glVertex3f(fFW, fFH, fFZ);
                glVertex3f(0, 0, 0);
                glVertex3f(fFW, -fFH, fFZ);
                glVertex3f(0, 0, 0);
                glVertex3f(-fFW, -fFH, fFZ);
                glVertex3f(0, 0, 0);
                glVertex3f(-fFW, fFH, fFZ);

                glVertex3f(fFW, fFH, fFZ);
                glVertex3f(fFW, -fFH, fFZ);

                glVertex3f(-fFW, fFH, fFZ);
                glVertex3f(-fFW, -fFH, fFZ);

                glVertex3f(-fFW, fFH, fFZ);
                glVertex3f(fFW, fFH, fFZ);

                glVertex3f(-fFW, -fFH, fFZ);
                glVertex3f(fFW, -fFH, fFZ);
                glEnd();

                glPopMatrix();

                glEnd();
            }
        }

        if (bDrawCovisGraph) {
            glLineWidth(mGraphLineWidth);
            glColor4f(0.0f, 1.0f, 0.0f, 0.6f);
            glBegin(GL_LINES);

            for (size_t i = 0; i < vpKFs.size(); i++) {
                // Covisibility Graph
                const vector<KeyFrame *> vCovKFs = vpKFs[i]->GetCovisiblesByWeight(100);
                Eigen::Vector3f Ow = vpKFs[i]->GetCameraCenter();
                if (!vCovKFs.empty()) {
                    for (vector<KeyFrame *>::const_iterator vit = vCovKFs.begin(), vend = vCovKFs.end();
                         vit != vend; vit++) {
                        if ((*vit)->mnId < vpKFs[i]->mnId)
                            continue;
                        Eigen::Vector3f Ow2 = (*vit)->GetCameraCenter();
                        glVertex3f(Ow(0), Ow(1), Ow(2));
                        glVertex3f(Ow2(0), Ow2(1), Ow2(2));
                    }
                }

                // Spanning tree
                KeyFrame *pParent = vpKFs[i]->GetParent();
                if (pParent) {
                    Eigen::Vector3f Owp = pParent->GetCameraCenter();
                    glVertex3f(Ow(0), Ow(1), Ow(2));
                    glVertex3f(Owp(0), Owp(1), Owp(2));
                }

                // Loops
                set<KeyFrame *> sLoopKFs = vpKFs[i]->GetLoopEdges();
                for (set<KeyFrame *>::iterator sit = sLoopKFs.begin(), send = sLoopKFs.end(); sit != send; sit++) {
                    if ((*sit)->mnId < vpKFs[i]->mnId)
                        continue;
                    Eigen::Vector3f Owl = (*sit)->GetCameraCenter();
                    glVertex3f(Ow(0), Ow(1), Ow(2));
                    glVertex3f(Owl(0), Owl(1), Owl(2));
                }
            }

            glEnd();
        }

        if (bDrawInertialGraph && pActiveMap->isImuInitialized()) {
            glLineWidth(mGraphLineWidth);
            glColor4f(1.0f, 0.0f, 0.0f, 0.6f);
            glBegin(GL_LINES);

            //Draw inertial links
            for (size_t i = 0; i < vpKFs.size(); i++) {
                KeyFrame *pKFi = vpKFs[i];
                Eigen::Vector3f Ow = pKFi->GetCameraCenter();
                KeyFrame *pNext = pKFi->mNextKF;
                if (pNext) {
                    Eigen::Vector3f Owp = pNext->GetCameraCenter();
                    glVertex3f(Ow(0), Ow(1), Ow(2));
                    glVertex3f(Owp(0), Owp(1), Owp(2));
                }
            }

            glEnd();
        }

        if (bHisoryMapKF) {
            vector<Map *> vpMaps = mpAtlas->GetAllMaps();
            for (Map *pMap : vpMaps) {
                if (pMap == pActiveMap)
                    continue;
                vector<KeyFrame *> vpKFs = pMap->GetAllKeyFrames();

                for (size_t i = 0; i < vpKFs.size(); i++) {
                    KeyFrame *pKF = vpKFs[i];
                    Eigen::Matrix4f Twc = pKF->GetPoseInverse().matrix();
                    unsigned int index_color = pKF->mnOriginMapId;
                    glPushMatrix();

                    glMultMatrixf((GLfloat *) Twc.data());
                    if (!vpKFs[i]->GetParent()) // It is the first KF in the map
                    {
                        glLineWidth(mKeyFrameLineWidth * 5);
                        glColor3f(1.0f, 0.0f, 0.0f);
                        glBegin(GL_LINES);
                    } else {
                        glLineWidth(mKeyFrameLineWidth);
                        glColor3f(mfFrameColors[index_color][0], mfFrameColors[index_color][1],
                                  mfFrameColors[index_color][2]);
                        glBegin(GL_LINES);
                    }

                    glVertex3f(0, 0, 0);
                    glVertex3f(fFW, fFH, fFZ);
                    glVertex3f(0, 0, 0);
                    glVertex3f(fFW, -fFH, fFZ);
                    glVertex3f(0, 0, 0);
                    glVertex3f(-fFW, -fFH, fFZ);
                    glVertex3f(0, 0, 0);
                    glVertex3f(-fFW, fFH, fFZ);

                    glVertex3f(fFW, fFH, fFZ);
                    glVertex3f(fFW, -fFH, fFZ);

                    glVertex3f(-fFW, fFH, fFZ);
                    glVertex3f(-fFW, -fFH, fFZ);

                    glVertex3f(-fFW, fFH, fFZ);
                    glVertex3f(fFW, fFH, fFZ);

                    glVertex3f(-fFW, -fFH, fFZ);
                    glVertex3f(fFW, -fFH, fFZ);
                    glEnd();

                    glPopMatrix();
                }
            }
        }
    }

    void MapDrawer::DrawCurrentCamera(pangolin::OpenGlMatrix &Twc) {
        const float &w = mCameraSize;
        const float h = w * 0.75;
        const float z = w * 0.6;
        glPushMatrix();

#ifdef HAVE_GLES
        glMultMatrixf(Twc.m);
#else
        glMultMatrixd(Twc.m);
#endif
        glLineWidth(mCameraLineWidth);
        glColor3f(0.0f, 1.0f, 0.0f);
        glBegin(GL_LINES);
        glVertex3f(0, 0, 0);
        glVertex3f(w, h, z);
        glVertex3f(0, 0, 0);
        glVertex3f(w, -h, z);
        glVertex3f(0, 0, 0);
        glVertex3f(-w, -h, z);
        glVertex3f(0, 0, 0);
        glVertex3f(-w, h, z);

        glVertex3f(w, h, z);
        glVertex3f(w, -h, z);

        glVertex3f(-w, h, z);
        glVertex3f(-w, -h, z);

        glVertex3f(-w, h, z);
        glVertex3f(w, h, z);

        glVertex3f(-w, -h, z);
        glVertex3f(w, -h, z);
        glEnd();

        glPopMatrix();
    }


    void MapDrawer::SetCurrentCameraPose(const Sophus::SE3f &Tcw) {
        unique_lock<mutex> lock(mMutexCamera);
        mCameraPose = Tcw.inverse();
    }

    void MapDrawer::GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M, pangolin::OpenGlMatrix &MOw) {
        Eigen::Matrix4f Twc;
        {
            unique_lock<mutex> lock(mMutexCamera);
            Twc = mCameraPose.matrix();
        }

        for (int i = 0; i < 4; i++) {
            M.m[4 * i] = Twc(0, i);
            M.m[4 * i + 1] = Twc(1, i);
            M.m[4 * i + 2] = Twc(2, i);
            M.m[4 * i + 3] = Twc(3, i);
        }

        MOw.SetIdentity();
        MOw.m[12] = Twc(0, 3);
        MOw.m[13] = Twc(1, 3);
        MOw.m[14] = Twc(2, 3);
    }
} //namespace ORB_SLAM
