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

#include "Atlas.h"
#include "Viewer.h"

#include "GeometricCamera.h"
#include "Pinhole.h"
#include "KannalaBrandt8.h"

namespace ORB_SLAM3 {

    Atlas::Atlas() {
        mpCurrentMap = static_cast<Map *>(NULL);
    }

    Atlas::Atlas(int initKFid) : mnLastInitKFidMap(initKFid), mHasViewer(false) {
        mpCurrentMap = static_cast<Map *>(NULL);
        SaveAndCreateNewMap();
    }

    Atlas::~Atlas() {
        for (std::set<Map *>::iterator it = mspMaps.begin(), end = mspMaps.end(); it != end;) {
            Map *pMi = *it;
            if (pMi) {
                delete pMi;
                pMi = static_cast<Map *>(NULL);
                it = mspMaps.erase(it);
            } else
                ++it;
        }
    }

/**
 * @brief 创建新地图，如果当前活跃地图有效，先存储当前地图为不活跃地图，然后新建地图；否则，可以直接新建地图。
 * 
 */
    void Atlas::SaveAndCreateNewMap() {
        // 锁住地图集
        unique_lock<mutex> lock(mMutexAtlas);
        cout << "Creation of new map with id: " << Map::nNextId << endl;
        // 如果当前活跃地图有效，先存储当前地图为不活跃地图后退出
        if (mpCurrentMap) {
            // mnLastInitKFidMap为当前地图创建时第1个关键帧的id，它是在上一个地图最大关键帧id的基础上增加1
            if (!mspMaps.empty() && mnLastInitKFidMap < mpCurrentMap->GetMaxKFId())
                mnLastInitKFidMap = mpCurrentMap->GetMaxKFId() + 1; // The init KF is the next of current maximum

            // 将当前地图储存起来，其实就是把mIsInUse标记为false
            mpCurrentMap->SetStoredMap();
            cout << "Stored map with ID: " << mpCurrentMap->GetId() << endl;

            // if(mHasViewer)
            //     mpViewer->AddMapToCreateThumbnail(mpCurrentMap);
        }
        cout << "Creation of new map with last KF id: " << mnLastInitKFidMap << endl;

        mpCurrentMap = new Map(mnLastInitKFidMap);  //新建地图
        mpCurrentMap->SetCurrentMap();              //设置为活跃地图
        mspMaps.insert(mpCurrentMap);               //插入地图集
    }

    void Atlas::ChangeMap(Map *pMap) {
        unique_lock<mutex> lock(mMutexAtlas);
        cout << "Change to map with id: " << pMap->GetId() << endl;
        if (mpCurrentMap) {
            mpCurrentMap->SetStoredMap();
        }

        mpCurrentMap = pMap;
        mpCurrentMap->SetCurrentMap();
    }

    unsigned long int Atlas::GetLastInitKFid() {
        unique_lock<mutex> lock(mMutexAtlas);
        return mnLastInitKFidMap;
    }

    void Atlas::SetViewer(Viewer *pViewer) {
        mpViewer = pViewer;
        mHasViewer = true;
    }

    void Atlas::AddKeyFrame(KeyFrame *pKF) {
        Map *pMapKF = pKF->GetMap();
        pMapKF->AddKeyFrame(pKF);
    }

    void Atlas::AddMapPoint(MapPoint *pMP) {
        Map *pMapMP = pMP->GetMap();
        pMapMP->AddMapPoint(pMP);
    }

/**
 * @brief 添加相机，跟保存地图相关
 * @param pCam 相机
 */
    GeometricCamera *Atlas::AddCamera(GeometricCamera *pCam) {
        // Check if the camera already exists
        bool bAlreadyInMap = false;
        int index_cam = -1;
        // 遍历地图中现有的相机看看跟输入的相机一不一样，不一样的话则向mvpCameras添加
        for (size_t i = 0; i < mvpCameras.size(); ++i) {
            GeometricCamera *pCam_i = mvpCameras[i];
            if (!pCam)
                std::cout << "Not pCam" << std::endl;
            if (!pCam_i)
                std::cout << "Not pCam_i" << std::endl;
            if (pCam->GetType() != pCam_i->GetType())
                continue;

            if (pCam->GetType() == GeometricCamera::CAM_PINHOLE) {
                if (((Pinhole *) pCam_i)->IsEqual(pCam)) {
                    bAlreadyInMap = true;
                    index_cam = i;
                }
            } else if (pCam->GetType() == GeometricCamera::CAM_FISHEYE) {
                if (((KannalaBrandt8 *) pCam_i)->IsEqual(pCam)) {
                    bAlreadyInMap = true;
                    index_cam = i;
                }
            }
        }

        if (bAlreadyInMap) {
            return mvpCameras[index_cam];
        } else {
            mvpCameras.emplace_back(pCam);
            return pCam;
        }
    }

    std::vector<GeometricCamera *> Atlas::GetAllCameras() {
        return mvpCameras;
    }



    vector<Map *> Atlas::GetAllMaps() {
        unique_lock<mutex> lock(mMutexAtlas);
        struct compFunctor {
            inline bool operator()(Map *elem1, Map *elem2) {
                return elem1->GetId() < elem2->GetId();
            }
        };
        vector<Map *> vMaps(mspMaps.begin(), mspMaps.end());
        sort(vMaps.begin(), vMaps.end(), compFunctor());
        return vMaps;
    }

    int Atlas::CountMaps() {
        unique_lock<mutex> lock(mMutexAtlas);
        return mspMaps.size();
    }


    void Atlas::clearAtlas() {
        unique_lock<mutex> lock(mMutexAtlas);
        /*for(std::set<Map*>::iterator it=mspMaps.begin(), send=mspMaps.end(); it!=send; it++)
        {
            (*it)->clear();
            delete *it;
        }*/
        mspMaps.clear();
        mpCurrentMap = static_cast<Map *>(NULL);
        mnLastInitKFidMap = 0;
    }

    Map *Atlas::GetCurrentMap() {
        unique_lock<mutex> lock(mMutexAtlas);
        if (!mpCurrentMap)
            SaveAndCreateNewMap();
        while (mpCurrentMap->IsBad())
            usleep(5000);
        return mpCurrentMap;
    }

    void Atlas::SetMapBad(Map *pMap) {
        mspMaps.erase(pMap);
        pMap->SetBad();
        mspBadMaps.insert(pMap);
    }

    void Atlas::RemoveBadMaps() {
        /*for(Map* pMap : mspBadMaps)
        {
            delete pMap;
            pMap = static_cast<Map*>(NULL);
        }*/
        mspBadMaps.clear();
    }


} // namespace ORB_SLAM3
