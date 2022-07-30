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


#ifndef IMUTYPES_H
#define IMUTYPES_H

#include <vector>
#include <utility>
#include <opencv2/core/core.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <sophus/se3.hpp>
#include <mutex>

#include "SerializationUtils.h"

#include <boost/serialization/serialization.hpp>
#include <boost/serialization/vector.hpp>

namespace ORB_SLAM3 {

    namespace IMU {

        const float GRAVITY_VALUE = 9.8;

//IMU measurement (gyro, accelerometer and timestamp)
        class Point {
        public:
            Point(const float &acc_x, const float &acc_y, const float &acc_z,
                  const float &ang_vel_x, const float &ang_vel_y, const float &ang_vel_z,
                  const double &timestamp) : mAcc(acc_x, acc_y, acc_z), mGyr(ang_vel_x, ang_vel_y, ang_vel_z),
                                             mTs(timestamp) {}

            Point(const cv::Point3f Acc, const cv::Point3f Gyr, const double &timestamp) :
                    mAcc(Acc.x, Acc.y, Acc.z), mGyr(Gyr.x, Gyr.y, Gyr.z), mTs(timestamp) {}

        public:
            Eigen::Vector3f mAcc;
            Eigen::Vector3f mGyr;
            double mTs;

            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        };

//IMU biases (gyro and accelerometer)
        class Bias {
            friend class boost::serialization::access;

            template<class Archive>
            void serialize(Archive &ar, const unsigned int version) {
                ar & mBAX;
                ar & mBAY;
                ar & mBAZ;

                ar & mBGX;
                ar & mBGY;
                ar & mBGZ;
            }

        public:
            Bias() : mBAX(0), mBAY(0), mBAZ(0), mBGX(0), mBGY(0), mBGZ(0) {}

            Bias(const float &BAX, const float &BAY, const float &BAZ,
                 const float &BGX, const float &BGY, const float &BGZ) :
                    mBAX(BAX), mBAY(BAY), mBAZ(BAZ), mBGX(BGX), mBGY(BGY), mBGZ(BGZ) {}

            void CopyFrom(Bias &b);

            friend std::ostream &operator<<(std::ostream &out, const Bias &b);

        public:
            float mBAX, mBAY, mBAZ;
            float mBGX, mBGY, mBGZ;

            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        };

//IMU calibration (Tbc, Tcb, noise)
        class Calib {
            friend class boost::serialization::access;

            template<class Archive>
            void serialize(Archive &ar, const unsigned int version) {
                serializeSophusSE3(ar, mTcb, version);
                serializeSophusSE3(ar, mTbc, version);

                ar & boost::serialization::make_array(CovNoise.diagonal().data(), CovNoise.diagonal().size());
                ar & boost::serialization::make_array(CovWalk.diagonal().data(), CovWalk.diagonal().size());

                ar & mbIsSet;
            }

        public:

            Calib(const Sophus::SE3<float> &Tbc, const float &ng, const float &na, const float &ngw, const float &naw) {
                Set(Tbc, ng, na, ngw, naw);
            }

            Calib(const Calib &calib);

            Calib() { mbIsSet = false; }

            //void Set(const cv::Mat &cvTbc, const float &ng, const float &na, const float &ngw, const float &naw);
            void Set(const Sophus::SE3<float> &sophTbc, const float &ng, const float &na, const float &ngw,
                     const float &naw);

        public:
            // Sophus/Eigen implementation
            Sophus::SE3<float> mTcb;
            Sophus::SE3<float> mTbc;
            Eigen::DiagonalMatrix<float, 6> CovNoise, CovWalk;
            bool mbIsSet;
        };

//Integration of 1 gyro measurement
        class IntegratedRotation {
        public:
            IntegratedRotation() {}

            IntegratedRotation(const Eigen::Vector3f &angVel, const Bias &imuBias, const float &time);

        public:
            float deltaT; //integration time
            Eigen::Matrix3f deltaR;
            Eigen::Matrix3f rightJ; // right jacobian
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        };

//Preintegration of Imu Measurements
        class Preintegrated {
            friend class boost::serialization::access;

            template<class Archive>
            void serialize(Archive &ar, const unsigned int version) {
                ar & mfTs;
                ar & boost::serialization::make_array(C.data(), C.size());
                ar & boost::serialization::make_array(mCovNoise.diagonal().data(), mCovNoise.diagonal().size());
                ar & boost::serialization::make_array(mCovWalk.diagonal().data(), mCovWalk.diagonal().size());
                ar & mBiasOri;
                ar & boost::serialization::make_array(dR.data(), dR.size());
                ar & boost::serialization::make_array(dV.data(), dV.size());
                ar & boost::serialization::make_array(dP.data(), dP.size());
                ar & boost::serialization::make_array(JRg.data(), JRg.size());
                ar & boost::serialization::make_array(JVg.data(), JVg.size());
                ar & boost::serialization::make_array(JVa.data(), JVa.size());
                ar & boost::serialization::make_array(JPg.data(), JPg.size());
                ar & boost::serialization::make_array(JPa.data(), JPa.size());
                ar & boost::serialization::make_array(mAvgAcc.data(), mAvgAcc.size());
                ar & boost::serialization::make_array(mAvgGyr.data(), mAvgGyr.size());

                ar & mBiasUpdate;
                ar & boost::serialization::make_array(mBiasDelta.data(), mBiasDelta.size());
                ar & mvMeasurements;
            }

        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW

            Preintegrated(const Bias &InputBias, const Calib &calib);

            Preintegrated(Preintegrated *pImuPre);

            Preintegrated() {}

            ~Preintegrated() {}

            void CopyFrom(Preintegrated *pImuPre);

            void Initialize(const Bias &InputBias);

            void IntegrateNewMeasurement(const Eigen::Vector3f &Acc, const Eigen::Vector3f &Gyr,
                                         const float &TDelta);

            void Reintegrate();

            void MergePrevious(Preintegrated *pPrev);

            void SetNewBias(const Bias &bu_);

            IMU::Bias GetDeltaBias(const Bias &b_);

            Eigen::Matrix3f GetDeltaRotation(const Bias &b_);

            Eigen::Vector3f GetDeltaVelocity(const Bias &b_);

            Eigen::Vector3f GetDeltaPosition(const Bias &b_);

            Eigen::Matrix3f GetUpdatedDeltaRotation();

            Eigen::Vector3f GetUpdatedDeltaVelocity();

            Eigen::Vector3f GetUpdatedDeltaPosition();

            Eigen::Matrix3f GetOriginalDeltaRotation();

            Eigen::Vector3f GetOriginalDeltaVelocity();

            Eigen::Vector3f GetOriginalDeltaPosition();

            Eigen::Matrix<float, 6, 1> GetDeltaBias();

            Bias GetOriginalBias();

            Bias GetUpdatedBias();

            void printMeasurements() const {
                std::cout << "pint meas:\n";
                for (int i = 0; i < mvMeasurements.size(); i++) {
                    std::cout << "meas " << mvMeasurements[i].mTs << std::endl;
                }
                std::cout << "end pint meas:\n";
            }

        public:
            float mfTs;
            Eigen::Matrix<float, 15, 15> C;
            Eigen::DiagonalMatrix<float, 6> mCovNoise, mCovWalk;

            // Values for the original bias (when integration was computed)
            Bias mBiasOri;
            Eigen::Matrix3f dR;
            Eigen::Vector3f dV, dP;
            Eigen::Matrix3f JRg, JVg, JVa, JPg, JPa;
            Eigen::Vector3f mAvgAcc, mAvgGyr;


        private:
            // Updated bias
            Bias mBiasUpdate;    //更新后的零偏
            // Dif between original and updated bias
            // This is used to compute the updated values of the preintegration
            Eigen::Matrix<float, 6, 1> mBiasDelta;

            struct integrable {
                template<class Archive>
                void serialize(Archive &ar, const unsigned int version) {
                    ar & boost::serialization::make_array(mAcc.data(), mAcc.size());
                    ar & boost::serialization::make_array(mGyr.data(), mGyr.size());
                    ar & mTs;
                }

                EIGEN_MAKE_ALIGNED_OPERATOR_NEW

                integrable() {}

                integrable(const Eigen::Vector3f &a_, const Eigen::Vector3f &w_, const float &t_) : mAcc(a_), mGyr(w_),
                                                                                                    mTs(t_) {}

                Eigen::Vector3f mAcc, mGyr;
                float mTs;
            };

            std::vector<integrable> mvMeasurements;

            std::mutex mMutex;
        };

// Lie Algebra Functions
        Eigen::Matrix3f RightJacobianSO3(const float &x, const float &y, const float &z);

        Eigen::Matrix3f RightJacobianSO3(const Eigen::Vector3f &v);

        Eigen::Matrix3f InverseRightJacobianSO3(const float &x, const float &y, const float &z);

        Eigen::Matrix3f InverseRightJacobianSO3(const Eigen::Vector3f &v);

        Eigen::Matrix3f NormalizeRotation(const Eigen::Matrix3f &R);

    }

} //namespace ORB_SLAM2

#endif // IMUTYPES_H
