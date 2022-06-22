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

#ifndef CAMERAMODELS_KANNALABRANDT8_H
#define CAMERAMODELS_KANNALABRANDT8_H


#include <assert.h>

#include "GeometricCamera.h"

namespace ORB_SLAM3 {
    class KannalaBrandt8 : public GeometricCamera {

        friend class boost::serialization::access;

        template<class Archive>
        void serialize(Archive &ar, const unsigned int version) {
            ar & boost::serialization::base_object<GeometricCamera>(*this);
            ar & const_cast<float &>(precision);
        }

    public:
        KannalaBrandt8() : precision(1e-6) {
            mvParameters.resize(8);
            mnId = nNextId++;
            mnType = CAM_FISHEYE;
        }

        KannalaBrandt8(const std::vector<float> _vParameters) : GeometricCamera(_vParameters), precision(1e-6),
                                                                mvLappingArea(2, 0) {
            assert(mvParameters.size() == 8);
            mnId = nNextId++;
            mnType = CAM_FISHEYE;
        }

        KannalaBrandt8(const std::vector<float> _vParameters, const float _precision) : GeometricCamera(_vParameters),
                                                                                        precision(_precision),
                                                                                        mvLappingArea(2, 0) {
            assert(mvParameters.size() == 8);
            mnId = nNextId++;
            mnType = CAM_FISHEYE;
        }

        KannalaBrandt8(KannalaBrandt8 *pKannala) : GeometricCamera(pKannala->mvParameters),
                                                   precision(pKannala->precision), mvLappingArea(2, 0) {
            assert(mvParameters.size() == 8);
            mnId = nNextId++;
            mnType = CAM_FISHEYE;
        }

        cv::Point2f ProjectMPToKP(const cv::Point3f &p3D);

        Eigen::Vector2d ProjectMPToKP(const Eigen::Vector3d &v3D);

        Eigen::Vector2f ProjectMPToKP(const Eigen::Vector3f &v3D);

        float uncertainty2(const Eigen::Matrix<double, 2, 1> &p2D);

        Eigen::Vector3f UnprojectEig(const cv::Point2f &p2D);

        cv::Point3f UnprojectCv(const cv::Point2f &p2D);

        Eigen::Matrix<double, 2, 3> ProjectJac(const Eigen::Vector3d &v3D);

        cv::Mat GetKCv();

        Eigen::Matrix3f GetKEig();

        bool EpipolarConstrain(GeometricCamera *pCamera2, const cv::KeyPoint &kp1, const cv::KeyPoint &kp2,
                               const Eigen::Matrix3f &R12, const Eigen::Vector3f &t12, const float sigmaLevel,
                               const float unc);

        float TriangulateMatches(GeometricCamera *pCamera2, const cv::KeyPoint &kp1, const cv::KeyPoint &kp2,
                                 const Eigen::Matrix3f &R12, const Eigen::Vector3f &t12, const float sigmaLevel,
                                 const float unc, Eigen::Vector3f &p3D);

        std::vector<int> mvLappingArea;


        friend std::ostream &operator<<(std::ostream &os, const KannalaBrandt8 &kb);

        friend std::istream &operator>>(std::istream &is, KannalaBrandt8 &kb);

        float GetPrecision() { return precision; }

        bool IsEqual(GeometricCamera *pCam);

    private:
        const float precision;

        //Parameters vector corresponds to
        //[fx, fy, cx, cy, k0, k1, k2, k3]

        void Triangulate(const cv::Point2f &p1, const cv::Point2f &p2, const Eigen::Matrix<float, 3, 4> &Tcw1,
                         const Eigen::Matrix<float, 3, 4> &Tcw2, Eigen::Vector3f &x3D);
    };
}


#endif //CAMERAMODELS_KANNALABRANDT8_H
