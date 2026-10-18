/*
 * IC-GVINS: A Robust, Real-time, INS-Centric GNSS-Visual-Inertial Navigation System
 *
 * Copyright (C) 2022 i2Nav Group, Wuhan University
 *
 *     Author : Hailiang Tang
 *    Contact : thl@whu.edu.cn
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef REPROJECTION_FACTOR_H
#define REPROJECTION_FACTOR_H

#include <ceres/ceres.h>

#include <Eigen/Geometry>

#include "ic_gvins/common/rotation.h"

using Eigen::Matrix3d;
using Eigen::Quaterniond;
using Eigen::Vector3d;

class ReprojectionFactor : public ceres::SizedCostFunction<2, 7, 7, 7, 1, 1> {

public:
    ReprojectionFactor() = delete;

    // 标准差为归一化相机下的重投影误差观测, pixel / f
    ReprojectionFactor(Vector3d pts0, Vector3d pts1, Vector3d vel0, Vector3d vel1, double td0, double td1, double std)
        : pts0_(std::move(pts0))
        , pts1_(std::move(pts1))
        , vel0_(std::move(vel0))
        , vel1_(std::move(vel1))
        , td0_(td0)
        , td1_(td1) {

        sqrt_info_.setZero();
        sqrt_info_(0, 0) = 1.0 / std;
        sqrt_info_(1, 1) = 1.0 / std;
    }

    bool Evaluate(const double *const *parameters, double *residuals, double **jacobians) const override {
        // State poses are T_w_b: body/IMU B (FRD) -> fixed world W (origin NED).
        Eigen::Vector3d t_w_b0(parameters[0][0], parameters[0][1], parameters[0][2]);
        Eigen::Quaterniond q_w_b0(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

        Eigen::Vector3d t_w_b1(parameters[1][0], parameters[1][1], parameters[1][2]);
        Eigen::Quaterniond q_w_b1(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);

        // Camera extrinsics are T_b_c: camera optical C (RDF) -> body B.
        Eigen::Vector3d t_b_c(parameters[2][0], parameters[2][1], parameters[2][2]);
        Eigen::Quaterniond q_b_c(parameters[2][6], parameters[2][3], parameters[2][4], parameters[2][5]);

        // 逆深度
        double id0 = parameters[3][0];

        // 时间延时
        double td = parameters[4][0];

        Eigen::Vector3d pts_0_td = pts0_ - (td - td0_) * vel0_;
        Eigen::Vector3d pts_1_td = pts1_ - (td - td1_) * vel1_;

        // C0 -> B0 -> W -> B1 -> C1.
        Eigen::Vector3d p_c0 = pts_0_td / id0;
        Eigen::Vector3d p_b0 = q_b_c * p_c0 + t_b_c;
        Eigen::Vector3d p_w  = q_w_b0 * p_b0 + t_w_b0;
        Eigen::Vector3d p_b1 = q_w_b1.inverse() * (p_w - t_w_b1);
        Eigen::Vector3d p_c1 = q_b_c.inverse() * (p_b1 - t_b_c);

        double d1 = p_c1.z();

        // 残差, 没有考虑参考帧的残差
        Eigen::Map<Eigen::Vector2d> residual(residuals);
        residual = (p_c1 / d1).head(2) - pts_1_td.head(2);
        residual = sqrt_info_ * residual;

        if (jacobians) {
            Eigen::Matrix3d R_w_b0 = q_w_b0.toRotationMatrix();
            Eigen::Matrix3d R_b1_w = q_w_b1.toRotationMatrix().transpose();
            Eigen::Matrix3d R_c_b  = q_b_c.toRotationMatrix().transpose();
            Eigen::Matrix<double, 2, 3> reduce;
            reduce << 1.0 / d1, 0, -p_c1(0) / (d1 * d1), 0, 1.0 / d1, -p_c1(1) / (d1 * d1);

            reduce = sqrt_info_ * reduce;

            if (jacobians[0]) {
                Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_pose_i(jacobians[0]);

                Eigen::Matrix<double, 3, 6> jaco_i;
                jaco_i.leftCols<3>()  = R_c_b * R_b1_w;
                jaco_i.rightCols<3>() = -R_c_b * R_b1_w * R_w_b0 * Rotation::skewSymmetric(p_b0);

                jacobian_pose_i.leftCols<6>() = reduce * jaco_i;
                jacobian_pose_i.rightCols<1>().setZero();
            }

            if (jacobians[1]) {
                Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_pose_j(jacobians[1]);

                Eigen::Matrix<double, 3, 6> jaco_j;
                jaco_j.leftCols<3>()  = -R_c_b * R_b1_w;
                jaco_j.rightCols<3>() = R_c_b * Rotation::skewSymmetric(p_b1);

                jacobian_pose_j.leftCols<6>() = reduce * jaco_j;
                jacobian_pose_j.rightCols<1>().setZero();
            }

            if (jacobians[2]) {
                Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_ex_pose(jacobians[2]);

                Eigen::Matrix<double, 3, 6> jaco_ex;
                jaco_ex.leftCols<3>() = R_c_b * (R_b1_w * R_w_b0 - Eigen::Matrix3d::Identity());
                Eigen::Matrix3d tmp_r = R_c_b * R_b1_w * R_w_b0 * R_c_b.transpose();

                jaco_ex.rightCols<3>() = -tmp_r * Rotation::skewSymmetric(p_c0) +
                                         Rotation::skewSymmetric(tmp_r * p_c0) +
                                         Rotation::skewSymmetric(
                                             R_c_b * (R_b1_w * (R_w_b0 * t_b_c + t_w_b0 - t_w_b1) - t_b_c));

                jacobian_ex_pose.leftCols<6>() = reduce * jaco_ex;
                jacobian_ex_pose.rightCols<1>().setZero();
            }

            if (jacobians[3]) {
                Eigen::Map<Eigen::Vector2d> jacobian_feature(jacobians[3]);
                jacobian_feature =
                    -reduce * R_c_b * R_b1_w * R_w_b0 * R_c_b.transpose() * pts_0_td / (id0 * id0);
            }

            if (jacobians[4]) {
                Eigen::Map<Eigen::Vector2d> jacobian_td(jacobians[4]);
                jacobian_td = -reduce * R_c_b * R_b1_w * R_w_b0 * R_c_b.transpose() * vel0_ / id0 +
                              sqrt_info_ * vel1_.head(2);
            }
        }

        return true;
    }

private:
    // 归一化相机坐标系下的坐标
    Vector3d pts0_;
    Vector3d pts1_;

    Vector3d vel0_, vel1_;
    double td0_, td1_;

    Eigen::Matrix2d sqrt_info_;
};

#endif // REPROJECTION_FACTOR_H
