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

#ifndef CAMERAMODELS_GEOMETRICCAMERA_H
#define CAMERAMODELS_GEOMETRICCAMERA_H

#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <boost/serialization/serialization.hpp>
#include <boost/serialization/access.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/export.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/assume_abstract.hpp>

#include <sophus/se3.hpp>

#include <Eigen/Geometry>

#include "Converter.h"
#include "GeometricTools.h"

namespace ORB_SLAM3 {
    class GeometricCamera {
        friend class boost::serialization::access;
        template<class Archive>
        void serialize(Archive &ar, const unsigned int version) {
            ar & mnId;
            ar & mnType;
            ar & mvParameters;
        }

    public:
        GeometricCamera() {}
        GeometricCamera(const std::vector<float> &_vParameters) : mvParameters(_vParameters) {}
        ~GeometricCamera() {}
        virtual cv::Point2f ProjectMPToKP(const cv::Point3f &p3D) = 0;
        virtual Eigen::Vector2d ProjectMPToKP(const Eigen::Vector3d &v3D) = 0;
        virtual Eigen::Vector2f ProjectMPToKP(const Eigen::Vector3f &v3D) = 0;
        virtual float uncertainty2(const Eigen::Matrix<double, 2, 1> &p2D) = 0;
        virtual Eigen::Vector3f UnprojectEig(const cv::Point2f &p2D) = 0;
        virtual cv::Point3f UnprojectCv(const cv::Point2f &p2D) = 0;
        virtual Eigen::Matrix<double, 2, 3> ProjectJac(const Eigen::Vector3d &v3D) = 0;


        virtual cv::Mat GetKCv() = 0;
        virtual Eigen::Matrix3f GetKEig() = 0;
        virtual bool EpipolarConstrain(GeometricCamera *pCamera2, const cv::KeyPoint &kp1, const cv::KeyPoint &kp2,
                               const Eigen::Matrix3f &R12, const Eigen::Vector3f &t12, const float sigmaLevel,
                               const float unc) = 0;
        virtual float TriangulateMatches(GeometricCamera *pCamera2, const cv::KeyPoint &kp1, const cv::KeyPoint &kp2,
                                 const Eigen::Matrix3f &R12, const Eigen::Vector3f &t12, const float sigmaLevel,
                                 const float unc, Eigen::Vector3f &p3D) = 0;
        float GetParameter(const int i) { return mvParameters[i]; }
        void SetParameter(const float p, const size_t i) { mvParameters[i] = p; }
        size_t ParameterSize() { return mvParameters.size(); }
        unsigned int GetId() { return mnId; }
        unsigned int GetType() { return mnType; }
        const static unsigned int CAM_PINHOLE = 0;
        const static unsigned int CAM_FISHEYE = 1;
        static long unsigned int nNextId;

    protected:
        virtual void Triangulate(const cv::Point2f &p1, const cv::Point2f &p2, const Eigen::Matrix<float, 3, 4> &Tcw1,
                                 const Eigen::Matrix<float, 3, 4> &Tcw2, Eigen::Vector3f &x3D) = 0;
        std::vector<float> mvParameters;
        unsigned int mnId;
        unsigned int mnType;

    };
}


#endif //CAMERAMODELS_GEOMETRICCAMERA_H
