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

#include "Pinhole.h"

#include <boost/serialization/export.hpp>

// BOOST_CLASS_EXPORT_IMPLEMENT(ORB_SLAM3::Pinhole)

namespace ORB_SLAM3 {
// BOOST_CLASS_EXPORT_GUID(Pinhole, "Pinhole")

    long unsigned int GeometricCamera::nNextId = 0;

/** 
 * @brief 相机坐标系下的三维点投影到无畸变像素平面
 * @param p3D 三维点
 * @return 像素坐标
 */
    cv::Point2f Pinhole::ProjectMPToKP(const cv::Point3f &p3D) {
        return cv::Point2f(mvParameters[0] * p3D.x / p3D.z + mvParameters[2],
                           mvParameters[1] * p3D.y / p3D.z + mvParameters[3]);
    }

/** 
 * @brief 相机坐标系下的三维点投影到无畸变像素平面
 * @param v3D 三维点
 * @return 像素坐标
 */
    Eigen::Vector2d Pinhole::ProjectMPToKP(const Eigen::Vector3d &v3D) {
        Eigen::Vector2d res;
        res[0] = mvParameters[0] * v3D[0] / v3D[2] + mvParameters[2];
        res[1] = mvParameters[1] * v3D[1] / v3D[2] + mvParameters[3];
        return res;
    }

/** 
 * @brief 相机坐标系下的三维点投影到无畸变像素平面
 * @param v3D 三维点
 * @return 像素坐标
 */
    Eigen::Vector2f Pinhole::ProjectMPToKP(const Eigen::Vector3f &v3D) {
        Eigen::Vector2f res;
        res[0] = mvParameters[0] * v3D[0] / v3D[2] + mvParameters[2];
        res[1] = mvParameters[1] * v3D[1] / v3D[2] + mvParameters[3];
        return res;
    }


/** 
 * @brief 貌似是调试遗留的产物
 */
    float Pinhole::uncertainty2(const Eigen::Matrix<double, 2, 1> &p2D) {
        return 1.0;
    }

/** 
 * @brief 反投影
 * @param p2D 特征点
 * @return 归一化坐标
 */
    Eigen::Vector3f Pinhole::UnprojectEig(const cv::Point2f &p2D) {
        return Eigen::Vector3f(
                (p2D.x - mvParameters[2]) / mvParameters[0],
                (p2D.y - mvParameters[3]) / mvParameters[1],
                1.f);
    }

/** 
 * @brief 反投影
 * @param p2D 特征点
 * @return 归一化坐标
 */
    cv::Point3f Pinhole::UnprojectCv(const cv::Point2f &p2D) {
        return cv::Point3f(
                (p2D.x - mvParameters[2]) / mvParameters[0],
                (p2D.y - mvParameters[3]) / mvParameters[1],
                1.f);
    }

/** 
 * @brief 求解二维像素坐标关于三维点坐标的雅克比矩阵
 * @param v3D 三维点
 * @return 
 */
    Eigen::Matrix<double, 2, 3> Pinhole::ProjectJac(const Eigen::Vector3d &v3D) {
        Eigen::Matrix<double, 2, 3> Jac;
        Jac(0, 0) = mvParameters[0] / v3D[2];
        Jac(0, 1) = 0.f;
        Jac(0, 2) = -mvParameters[0] * v3D[0] / (v3D[2] * v3D[2]);
        Jac(1, 0) = 0.f;
        Jac(1, 1) = mvParameters[1] / v3D[2];
        Jac(1, 2) = -mvParameters[1] * v3D[1] / (v3D[2] * v3D[2]);
        return Jac;
    }

/**
 * @brief 返回内参矩阵
 * @return K
 */
    cv::Mat Pinhole::GetKCv() {
        cv::Mat K = (cv::Mat_<float>(3, 3) << mvParameters[0],
                0.f, mvParameters[2], 0.f, mvParameters[1], mvParameters[3], 0.f, 0.f, 1.f);
        return K;
    }

/**
 * @brief 返回内参矩阵
 * @return K
 */
    Eigen::Matrix3f Pinhole::GetKEig() {
        Eigen::Matrix3f K;
        K << mvParameters[0], 0.f, mvParameters[2], 0.f, mvParameters[1], mvParameters[3], 0.f, 0.f, 1.f;
        return K;
    }

/**
 * @brief 极线约束
 * @param pCamera2 右相机
 * @param kp1 左相机特征点
 * @param kp2 右相机特征点
 * @param R12 2->1的旋转
 * @param t12 2->1的平移
 * @param sigmaLevel 特征点1的尺度的平方
 * @param unc 特征点2的尺度的平方，1.2^2n
 * @return 三维点恢复的成功与否
 */
    bool Pinhole::EpipolarConstrain(
            GeometricCamera *pCamera2, const cv::KeyPoint &kp1, const cv::KeyPoint &kp2,
            const Eigen::Matrix3f &R12, const Eigen::Vector3f &t12, const float sigmaLevel, const float unc) {
        // Compute Fundamental Matrix
        Eigen::Matrix3f t12x = Sophus::SO3f::hat(t12);
        Eigen::Matrix3f K1 = this->GetKEig();
        Eigen::Matrix3f K2 = pCamera2->GetKEig();
        Eigen::Matrix3f F12 = K1.transpose().inverse() * t12x * R12 * K2.inverse();

        // Epipolar line in second image l = x1'F12 = [a mBiasOri c]
        //                      u2,
        // (u1, v1, 1) * F12 * (v2,) = 0   -->  (a, mBiasOri, c) * (u2, v2, 1)^mTs = 0 --> a*u2 + mBiasOri*v2 + c = 0
        //                       1
        const float a = kp1.pt.x * F12(0, 0) + kp1.pt.y * F12(1, 0) + F12(2, 0);
        const float b = kp1.pt.x * F12(0, 1) + kp1.pt.y * F12(1, 1) + F12(2, 1);
        const float c = kp1.pt.x * F12(0, 2) + kp1.pt.y * F12(1, 2) + F12(2, 2);

        // 点到直线距离的公式
        // d = |a*u2 + mBiasOri*v2 + c| / sqrt(a^2 + mBiasOri^2)
        const float num = a * kp2.pt.x + b * kp2.pt.y + c;

        const float den = a * a + b * b;

        if (den == 0)
            return false;

        const float dsqr = num * num / den;

        return dsqr < 3.84 * unc;
    }

    std::ostream &operator<<(std::ostream &os, const Pinhole &ph) {
        os << ph.mvParameters[0] << " " << ph.mvParameters[1] << " " << ph.mvParameters[2] << " " << ph.mvParameters[3];
        return os;
    }

    std::istream &operator>>(std::istream &is, Pinhole &ph) {
        float nextParam;
        for (size_t i = 0; i < 4; i++) {
            assert(is.good());  //Make sure the input stream is good
            is >> nextParam;
            ph.mvParameters[i] = nextParam;

        }
        return is;
    }

    bool Pinhole::IsEqual(GeometricCamera *pCam) {
        if (pCam->GetType() != GeometricCamera::CAM_PINHOLE)
            return false;
        Pinhole *pPinholeCam = (Pinhole *) pCam;
        if (ParameterSize() != pPinholeCam->ParameterSize())
            return false;
        bool bSameCamera = true;
        for (size_t i = 0; i < ParameterSize(); ++i) {
            if (abs(mvParameters[i] - pPinholeCam->GetParameter(i)) > 1e-6) {
                bSameCamera = false;
                break;
            }
        }
        return bSameCamera;
    }
}
