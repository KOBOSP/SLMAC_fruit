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

#include "OptimizableTypes.h"

namespace ORB_SLAM3 {
    bool EdgeLeftCameraOnly::read(std::istream &is) {
        for (int i = 0; i < 2; i++) {
            is >> _measurement[i];
        }
        for (int i = 0; i < 2; i++) {
            for (int j = i; j < 2; j++) {
                is >> information()(i, j);
                if (i != j) {
                    information()(j, i) = information()(i, j);
                }
            }
        }
        return true;
    }

    bool EdgeLeftCameraOnly::write(std::ostream &os) const {

        for (int i = 0; i < 2; i++) {
            os << measurement()[i] << " ";
        }
        for (int i = 0; i < 2; i++) {
            for (int j = i; j < 2; j++) {
                os << " " << information()(i, j);
            }
        }
        return os.good();
    }

/**
 * @brief left reproject error to camera pose
 */
    void EdgeLeftCameraOnly::linearizeOplus() {
        g2o::VertexSE3Expmap *vi = static_cast<g2o::VertexSE3Expmap *>(_vertices[0]);
        Eigen::Vector3d xyz_trans = vi->estimate().map(Xw);

        double x = xyz_trans[0];
        double y = xyz_trans[1];
        double z = xyz_trans[2];

        Eigen::Matrix<double, 3, 6> SE3deriv;
        SE3deriv << 0.f, z, -y, 1.f, 0.f, 0.f,
                -z, 0.f, x, 0.f, 1.f, 0.f,
                y, -x, 0.f, 0.f, 0.f, 1.f;

        _jacobianOplusXi = -pCamera->ProjectJac(xyz_trans) * SE3deriv;//7.46
    }

    bool EdgeRightCameraOnly::read(std::istream &is) {
        for (int i = 0; i < 2; i++) {
            is >> _measurement[i];
        }
        for (int i = 0; i < 2; i++)
            for (int j = i; j < 2; j++) {
                is >> information()(i, j);
                if (i != j)
                    information()(j, i) = information()(i, j);
            }
        return true;
    }

    bool EdgeRightCameraOnly::write(std::ostream &os) const {

        for (int i = 0; i < 2; i++) {
            os << measurement()[i] << " ";
        }

        for (int i = 0; i < 2; i++)
            for (int j = i; j < 2; j++) {
                os << " " << information()(i, j);
            }
        return os.good();
    }

/**
 * @brief  right reproject error to camera pose _jacobianOplusXi
 */
    void EdgeRightCameraOnly::linearizeOplus() {
        // 获得三维点在右相机坐标系下的坐标
        g2o::VertexSE3Expmap *vi = static_cast<g2o::VertexSE3Expmap *>(_vertices[0]);
        g2o::SE3Quat T_lw(vi->estimate());//left camera pose
        Eigen::Vector3d X_l = T_lw.map(Xw);//mappoint in left axis
        Eigen::Vector3d X_r = mTrl.map(T_lw.map(Xw));//mappoint in right axis

        double x_w = X_l[0];
        double y_w = X_l[1];
        double z_w = X_l[2];

        Eigen::Matrix<double, 3, 6> SE3deriv;//7.45
        SE3deriv << 0.f, z_w, -y_w, 1.f, 0.f, 0.f,
                -z_w, 0.f, x_w, 0.f, 1.f, 0.f,
                y_w, -x_w, 0.f, 0.f, 0.f, 1.f;

        /*
        注意这里是对李代数求导，ρlw != tlw 所以不能使用Pl = Rlw*Pw + tlw
        Pl = EXP(ξlw)*Pw    Pr = Rrl * EXP(ξlw) * Pw + trl
        让Pr 对 ξlw 求雅克比
        相当于Rrl*(Pl 对 ξlw的雅克比)
        */
        _jacobianOplusXi = -pCamera->ProjectJac(X_r) * mTrl.rotation().toRotationMatrix() * SE3deriv;
    }

    EdgeLeftCameraAndMP::EdgeLeftCameraAndMP()
            : BaseBinaryEdge<2, Eigen::Vector2d, g2o::VertexSBAPointXYZ, g2o::VertexSE3Expmap>() {
    }

    bool EdgeLeftCameraAndMP::read(std::istream &is) {
        for (int i = 0; i < 2; i++) {
            is >> _measurement[i];
        }
        for (int i = 0; i < 2; i++)
            for (int j = i; j < 2; j++) {
                is >> information()(i, j);
                if (i != j)
                    information()(j, i) = information()(i, j);
            }
        return true;
    }

    bool EdgeLeftCameraAndMP::write(std::ostream &os) const {

        for (int i = 0; i < 2; i++) {
            os << measurement()[i] << " ";
        }

        for (int i = 0; i < 2; i++)
            for (int j = i; j < 2; j++) {
                os << " " << information()(i, j);
            }
        return os.good();
    }

/**
 * @brief 求解二维像素坐标关于camera位姿的雅克比矩阵 _jacobianOplusXj  二维像素坐标关于三维点世界坐标的雅克比矩阵 _jacobianOplusXi
 */
    void EdgeLeftCameraAndMP::linearizeOplus() {
        g2o::VertexSE3Expmap *vj = static_cast<g2o::VertexSE3Expmap *>(_vertices[1]);//camera
        g2o::SE3Quat T(vj->estimate());
        g2o::VertexSBAPointXYZ *vi = static_cast<g2o::VertexSBAPointXYZ *>(_vertices[0]);//mappoint
        Eigen::Vector3d xyz = vi->estimate();
        Eigen::Vector3d xyz_trans = T.map(xyz);

        double x = xyz_trans[0];
        double y = xyz_trans[1];
        double z = xyz_trans[2];

        auto projectJac = -pCamera->ProjectJac(xyz_trans);
        // Pc = Rcw*Pw + tcw  先求Pw改变对Pc的影响，所以直接为Rcw，前面再乘Pc对像素的影响
        _jacobianOplusXi = projectJac * T.rotation().toRotationMatrix();//7.48

        Eigen::Matrix<double, 3, 6> SE3deriv;
        SE3deriv << 0.f, z, -y, 1.f, 0.f, 0.f,
                -z, 0.f, x, 0.f, 1.f, 0.f,
                y, -x, 0.f, 0.f, 0.f, 1.f;
        _jacobianOplusXj = projectJac * SE3deriv;//7.46
    }

    EdgeRightCameraAndMP::EdgeRightCameraAndMP()
            : BaseBinaryEdge<2, Eigen::Vector2d, g2o::VertexSBAPointXYZ, g2o::VertexSE3Expmap>() {
    }

    bool EdgeRightCameraAndMP::read(std::istream &is) {
        for (int i = 0; i < 2; i++) {
            is >> _measurement[i];
        }
        for (int i = 0; i < 2; i++)
            for (int j = i; j < 2; j++) {
                is >> information()(i, j);
                if (i != j)
                    information()(j, i) = information()(i, j);
            }
        return true;
    }

    bool EdgeRightCameraAndMP::write(std::ostream &os) const {

        for (int i = 0; i < 2; i++) {
            os << measurement()[i] << " ";
        }

        for (int i = 0; i < 2; i++)
            for (int j = i; j < 2; j++) {
                os << " " << information()(i, j);
            }
        return os.good();
    }

/**
 * @brief 求解右目二维像素坐标关于位姿的雅克比矩阵 _jacobianOplusXj  右目二维像素坐标关于三维点世界坐标的雅克比矩阵 _jacobianOplusXi
 */
    void EdgeRightCameraAndMP::linearizeOplus() {
        g2o::VertexSE3Expmap *vj = static_cast<g2o::VertexSE3Expmap *>(_vertices[1]);//left camera
        g2o::SE3Quat T_lw(vj->estimate());//left camera pose
        g2o::SE3Quat T_rw = mTrl * T_lw;//right camera pose
        g2o::VertexSBAPointXYZ *vi = static_cast<g2o::VertexSBAPointXYZ *>(_vertices[0]);//mappoint
        Eigen::Vector3d X_w = vi->estimate();//mappoint world
        Eigen::Vector3d X_l = T_lw.map(X_w);//mappoint left
        Eigen::Vector3d X_r = mTrl.map(T_lw.map(X_w));//mappoint right

        _jacobianOplusXi = -pCamera->ProjectJac(X_r) * T_rw.rotation().toRotationMatrix();

        double x = X_l[0];
        double y = X_l[1];
        double z = X_l[2];

        Eigen::Matrix<double, 3, 6> SE3deriv;
        SE3deriv << 0.f, z, -y, 1.f, 0.f, 0.f,
                -z, 0.f, x, 0.f, 1.f, 0.f,
                y, -x, 0.f, 0.f, 0.f, 1.f;
        /*
            注意这里是对李代数求导，ρlw != tlw 所以不能使用Pl = Rlw*Pw + tlw
            Pl = EXP(ξlw)*Pw    Pr = Rrl * EXP(ξlw) * Pw + trl
            让Pr 对 ξlw 求雅克比
            相当于Rrl*(Pl 对 ξlw的雅克比)
            */
        _jacobianOplusXj = -pCamera->ProjectJac(X_r) * mTrl.rotation().toRotationMatrix() * SE3deriv;
    }

    VertexSim3Expmap::VertexSim3Expmap() : BaseVertex<7, g2o::Sim3>() {
        _marginalized = false;
        _fix_scale = false;
    }

    bool VertexSim3Expmap::read(std::istream &is) {
        g2o::Vector7d cam2world;
        for (int i = 0; i < 6; i++) {
            is >> cam2world[i];
        }
        is >> cam2world[6];

        float nextParam;
        for (size_t i = 0; i < pCamera1->ParameterSize(); i++) {
            is >> nextParam;
            pCamera1->SetParameter(nextParam, i);
        }

        for (size_t i = 0; i < pCamera2->ParameterSize(); i++) {
            is >> nextParam;
            pCamera2->SetParameter(nextParam, i);
        }

        setEstimate(g2o::Sim3(cam2world).inverse());
        return true;
    }

    bool VertexSim3Expmap::write(std::ostream &os) const {
        g2o::Sim3 cam2world(estimate().inverse());
        g2o::Vector7d lv = cam2world.log();
        for (int i = 0; i < 7; i++) {
            os << lv[i] << " ";
        }

        for (size_t i = 0; i < pCamera1->ParameterSize(); i++) {
            os << pCamera1->GetParameter(i) << " ";
        }

        for (size_t i = 0; i < pCamera2->ParameterSize(); i++) {
            os << pCamera2->GetParameter(i) << " ";
        }

        return os.good();
    }

    EdgeSim3ProjectXYZ::EdgeSim3ProjectXYZ()
            : g2o::BaseBinaryEdge<2, Eigen::Vector2d, g2o::VertexSBAPointXYZ, VertexSim3Expmap>() {
    }

    bool EdgeSim3ProjectXYZ::read(std::istream &is) {
        for (int i = 0; i < 2; i++) {
            is >> _measurement[i];
        }

        for (int i = 0; i < 2; i++)
            for (int j = i; j < 2; j++) {
                is >> information()(i, j);
                if (i != j)
                    information()(j, i) = information()(i, j);
            }
        return true;
    }

    bool EdgeSim3ProjectXYZ::write(std::ostream &os) const {
        for (int i = 0; i < 2; i++) {
            os << _measurement[i] << " ";
        }

        for (int i = 0; i < 2; i++)
            for (int j = i; j < 2; j++) {
                os << " " << information()(i, j);
            }
        return os.good();
    }

    EdgeInverseSim3ProjectXYZ::EdgeInverseSim3ProjectXYZ()
            : g2o::BaseBinaryEdge<2, Eigen::Vector2d, g2o::VertexSBAPointXYZ, VertexSim3Expmap>() {
    }

    bool EdgeInverseSim3ProjectXYZ::read(std::istream &is) {
        for (int i = 0; i < 2; i++) {
            is >> _measurement[i];
        }

        for (int i = 0; i < 2; i++)
            for (int j = i; j < 2; j++) {
                is >> information()(i, j);
                if (i != j)
                    information()(j, i) = information()(i, j);
            }
        return true;
    }

    bool EdgeInverseSim3ProjectXYZ::write(std::ostream &os) const {
        for (int i = 0; i < 2; i++) {
            os << _measurement[i] << " ";
        }

        for (int i = 0; i < 2; i++)
            for (int j = i; j < 2; j++) {
                os << " " << information()(i, j);
            }
        return os.good();
    }

}
