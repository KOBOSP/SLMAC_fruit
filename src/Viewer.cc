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


#include "Viewer.h"
#include <pangolin/pangolin.h>

#include <mutex>

namespace ORB_SLAM3 {

    Viewer::Viewer(System *pSystem, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, Tracking *pTracking,
                   const string &sSettingPath, Settings *settings) :
            mbFrameBoth(true), mpSystem(pSystem), mpFrameDrawer(pFrameDrawer), mpMapDrawer(pMapDrawer), mpTracker(pTracking),
            mbFinishRequested(false), mbFinished(true), mbReseted(true), mbResetRequested(false) {
        mfImgFreq = 1e3 / settings->mfImgFps;
        mfImageFrameScale = settings->mfImageFrameScale;
        mfViewpointX = settings->mfViewPointX;
        mfViewpointY = settings->mfViewPointY;
        mfViewpointZ = settings->mfViewPointZ;
        mfViewpointF = settings->mfViewPointF;
        mfMapWidth = settings->mfMapWidth;
        mfMapHeight = settings->mfMapHeight;
        mbStopTrack = false;
    }


    void Viewer::Run() {
        mbFinished = false;
        mbReseted = false;
        pangolin::CreateWindowAndBind("ORB-SLAM3: Map Viewer", mfMapWidth, mfMapHeight);
        // 3D Mouse handler requires depth testing to be enabled, Issue specific OpenGl we might need
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        pangolin::CreatePanel("menu").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(175));
        pangolin::Var<bool> menuFollowCamera("menu.Follow Camera", false, true);
        pangolin::Var<bool> menuCamView("menu.Camera View", false, false);
        pangolin::Var<bool> menuTopView("menu.Top View", false, false);
        pangolin::Var<bool> menuShowMapPoints("menu.Show MapPoints", true, true);
        pangolin::Var<bool> menuShowKeyFrames("menu.Show KeyFrames", true, true);
        pangolin::Var<bool> menuShowCovisGraph("menu.Covis Graph", false, true);
        pangolin::Var<bool> menuShowImuGraph("menu.Inertial Graph", true, true);
        pangolin::Var<bool> menuStereoFrame("menu.Stereo Frame", false, true);
        pangolin::Var<bool> menuShowOptFixKF("menu.Show OptFixKF", false, true);
        pangolin::Var<bool> menuHisoryMapKF("menu.Show HisoryMapKF", false, true);
        pangolin::Var<bool> menuLocalizationMode("menu.Localization Mode", false, true);
        pangolin::Var<bool> menuStepByStep("menu.Step By Step", false, true);  // false, true
        pangolin::Var<bool> menuNextStep("menu.Next Step", false, false);
        pangolin::Var<bool> menuReset("menu.CheckResetRequest", false, false);
        pangolin::Var<bool> menuFinish("menu.Finish", false, false);
        // Define Camera Render Object (for view / scene browsing)
        pangolin::OpenGlRenderState MapView(
                pangolin::ProjectionMatrix(mfMapWidth, mfMapHeight, mfViewpointF, mfViewpointF, mfMapWidth/2, mfMapHeight/2, 0.1, 1000),
                pangolin::ModelViewLookAt(mfViewpointX, mfViewpointY, mfViewpointZ, 0, 0, 0, 0.0, -1.0, 0.0)
        );

        // Add named OpenGL viewport to window and provide 3D Handler
        pangolin::View &MapHandler = pangolin::CreateDisplay()
                .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1.0*mfMapWidth/mfMapHeight)
                .SetHandler(new pangolin::Handler3D(MapView));

        pangolin::OpenGlMatrix Twc;
        Twc.SetIdentity();
        pangolin::OpenGlMatrix Ow; // Oriented with g in the z axis
        Ow.SetIdentity();
        cv::namedWindow("ORB-SLAM3: Current Frame");

        bool bFollow = true;
        bool bLocalizationMode = false;
        bool bCameraView = true;
        bool bStepByStep = false;
        float trackedImageScale = mpTracker->GetImageScale();

        cout << "Starting the Viewer" << endl;
        while (1) {
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            mpMapDrawer->GetCurrentOpenGLCameraMatrix(Twc, Ow);
            if (mbStopTrack) {
                menuStepByStep = true;
                mbStopTrack = false;
            }

            if (menuFollowCamera && bFollow) {
                if (bCameraView)
                    MapView.Follow(Twc);
                else
                    MapView.Follow(Ow);
            } else if (menuFollowCamera && !bFollow) {
                if (bCameraView) {
                    MapView.SetProjectionMatrix(pangolin::ProjectionMatrix(mfMapWidth, mfMapHeight, mfViewpointF, mfViewpointF, mfMapWidth/2, mfMapHeight/2, 0.1, 1000));
                    MapView.SetModelViewMatrix(pangolin::ModelViewLookAt(mfViewpointX, mfViewpointY, mfViewpointZ, 0, 0, 0, 0.0, -1.0, 0.0));
                    MapView.Follow(Twc);
                } else {
                    MapView.SetProjectionMatrix(pangolin::ProjectionMatrix(mfMapWidth, mfMapHeight, 3000, 3000, mfMapWidth / 2, mfMapHeight / 2, 0.1, 1000));
                    MapView.SetModelViewMatrix(pangolin::ModelViewLookAt(0, 0.01, 10, 0, 0, 0, 0.0, 0.0, 1.0));
                    MapView.Follow(Ow);
                }
                bFollow = true;
            } else if (!menuFollowCamera && bFollow) {
                bFollow = false;
            }

            if (menuCamView) {
                menuCamView = false;
                bCameraView = true;
                MapView.SetProjectionMatrix(pangolin::ProjectionMatrix(mfMapWidth, mfMapHeight, 3000, 3000, mfMapWidth/2, mfMapHeight/2, 0.1, 1000));
                MapView.SetModelViewMatrix(pangolin::ModelViewLookAt(mfViewpointX, mfViewpointY, mfViewpointZ, 0, 0, 0, 0.0, -1.0, 0.0));
                MapView.Follow(Twc);
            }
            if (menuTopView && mpMapDrawer->mpAtlas->isImuInitialized()) {
                menuTopView = false;
                bCameraView = false;
                MapView.SetProjectionMatrix(pangolin::ProjectionMatrix(mfMapWidth, mfMapHeight, 3000, 3000, mfMapWidth / 2, mfMapHeight / 2, 0.1, 1000));
                MapView.SetModelViewMatrix(pangolin::ModelViewLookAt(0, 0.01, 50, 0, 0, 0, 0.0, 0.0, 1.0));
                MapView.Follow(Ow);
            }

            if (menuStereoFrame) {
                mbFrameBoth = true;
            }else{
                mbFrameBoth = false;
            }


            if (menuLocalizationMode && !bLocalizationMode) {
                mpSystem->ActivateLocalizationMode();
                bLocalizationMode = true;
            } else if (!menuLocalizationMode && bLocalizationMode) {
                mpSystem->DeactivateLocalizationMode();
                bLocalizationMode = false;
            }

            if (menuStepByStep && !bStepByStep) {
                mpTracker->SetStepByStep(true);
                bStepByStep = true;
            } else if (!menuStepByStep && bStepByStep) {
                mpTracker->SetStepByStep(false);
                bStepByStep = false;
            }
            if (menuNextStep) {
                mpTracker->mbDoNext = true;
                menuNextStep = false;
            }

            MapHandler.Activate(MapView);
            glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
            mpMapDrawer->DrawCurrentCamera(Twc);
            if (menuShowKeyFrames || menuShowCovisGraph || menuShowImuGraph || menuShowOptFixKF || menuHisoryMapKF)
                mpMapDrawer->DrawKeyFramesGraphs(menuShowKeyFrames, menuShowCovisGraph, menuShowImuGraph,
                                                 menuShowOptFixKF, menuHisoryMapKF);
            if (menuShowMapPoints)
                mpMapDrawer->DrawMapPoints();
            pangolin::FinishFrame();

            cv::Mat FrameShow = mpFrameDrawer->DrawKeyFrame(trackedImageScale, mbFrameBoth);

            int width = FrameShow.cols * mfImageFrameScale;
            int height = FrameShow.rows * mfImageFrameScale;
            cv::resize(FrameShow, FrameShow, cv::Size(width, height));

            cv::imshow("ORB-SLAM3: Current Frame", FrameShow);

            cv::waitKey(mfImgFreq);

            if (menuReset) {
                menuShowCovisGraph = true;
                menuShowImuGraph = true;
                menuShowKeyFrames = true;
                menuShowMapPoints = true;
                menuLocalizationMode = false;
                menuStereoFrame = true;
                menuHisoryMapKF = false;
                if (bLocalizationMode)
                    mpSystem->DeactivateLocalizationMode();
                bLocalizationMode = false;
                bFollow = true;
                menuFollowCamera = true;
                mpSystem->ResetActiveMap();
                menuReset = false;
            }

            if (menuFinish) {
                if (bLocalizationMode)
                    mpSystem->DeactivateLocalizationMode();

                // CheckResetRequest all threads
                mpSystem->ShutDownRequest();

                menuFinish = false;
            }
            cout<<"CheckResetRequest"<<endl;
            if (CheckResetRequest()) {
                while (IsReseted()) {
                    usleep(500);
                }
            }
            cout<<"CheckFinishReqest"<<endl;
            if (CheckFinishReqest())
                break;
        }
        SetFinish();
    }

    void Viewer::RequestFinish() {
        unique_lock<mutex> lock(mMutexFinish);
        mbFinishRequested = true;
    }

    bool Viewer::CheckFinishReqest() {
        unique_lock<mutex> lock(mMutexFinish);
        return mbFinishRequested;
    }

    void Viewer::SetFinish() {
        unique_lock<mutex> lock(mMutexFinish);
        mbFinished = true;
    }

    bool Viewer::IsFinished() {
        unique_lock<mutex> lock(mMutexFinish);
        return mbFinished;
    }

    void Viewer::RequestReset() {
        unique_lock<mutex> lock(mMutexReset);
        if (!mbReseted)
            mbResetRequested = true;
    }

    bool Viewer::IsReseted() {
        unique_lock<mutex> lock(mMutexReset);
        return mbReseted;
    }

    bool Viewer::CheckResetRequest() {
        unique_lock<mutex> lock(mMutexReset);
        unique_lock<mutex> lock2(mMutexFinish);
        if (mbFinishRequested)
            return false;
        else if (mbResetRequested) {
            mbReseted = true;
            mbResetRequested = false;
            return true;
        }
        return false;
    }

    void Viewer::Release() {
        unique_lock<mutex> lock(mMutexReset);
        mbReseted = false;
    }
}
