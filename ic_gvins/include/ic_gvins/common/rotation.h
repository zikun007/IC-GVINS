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
 */

#ifndef ROTATION_H
#define ROTATION_H

#include "ic_gvins/common/angle.h"

#include <Eigen/Geometry>

#include <cmath>

using Eigen::Matrix3d;
using Eigen::Quaterniond;
using Eigen::Vector3d;

// Rotation implementation adapted from open_nav/nav_core revision
// 882e8ba12889d04cbee0cf4a267518360d9df28d.
class Rotation {

public:
    static Quaterniond matrix2quaternion(const Matrix3d &matrix) {
        return Quaterniond(matrix).normalized();
    }

    static Matrix3d quaternion2matrix(const Quaterniond &quaternion) {
        return quaternion.normalized().toRotationMatrix();
    }

    // ZYX rotation, RPY output with heading in [0, 2*pi).
    static Vector3d matrix2euler(const Matrix3d &dcm) {
        Vector3d euler;
        euler.y() = std::atan2(-dcm(2, 0), std::hypot(dcm(2, 1), dcm(2, 2)));
        euler.x() = std::atan2(dcm(2, 1), dcm(2, 2));
        euler.z() = std::abs(std::abs(dcm(2, 0)) - 1.0) < 1e-6 ? 0.0 : std::atan2(dcm(1, 0), dcm(0, 0));
        if (euler.z() < 0.0) {
            euler.z() += 2.0 * PI;
        }
        return euler;
    }

    static Vector3d quaternion2euler(const Quaterniond &quaternion) {
        return matrix2euler(quaternion2matrix(quaternion));
    }

    static Quaterniond rotvec2quaternion(const Vector3d &rotvec) {
        const double angle = rotvec.norm();
        if (angle < 1e-12) {
            return Quaterniond::Identity();
        }
        return Quaterniond(Eigen::AngleAxisd(angle, rotvec / angle));
    }

    static Vector3d quaternion2vector(const Quaterniond &quaternion) {
        const Eigen::AngleAxisd angle_axis(quaternion.normalized());
        if (std::abs(angle_axis.angle()) < 1e-12) {
            return Vector3d::Zero();
        }
        return angle_axis.angle() * angle_axis.axis();
    }

    static Matrix3d euler2matrix(const Vector3d &euler) {
        return (Eigen::AngleAxisd(euler.z(), Vector3d::UnitZ()) * Eigen::AngleAxisd(euler.y(), Vector3d::UnitY()) *
                Eigen::AngleAxisd(euler.x(), Vector3d::UnitX()))
            .toRotationMatrix();
    }

    static Quaterniond euler2quaternion(const Vector3d &euler) {
        return Quaterniond(euler2matrix(euler)).normalized();
    }

    static Matrix3d skewSymmetric(const Vector3d &vector) {
        Matrix3d skew;
        skew << 0.0, -vector.z(), vector.y(), vector.z(), 0.0, -vector.x(), -vector.y(), vector.x(), 0.0;
        return skew;
    }

    static Eigen::Matrix4d quaternionleft(const Quaterniond &q) {
        Eigen::Matrix4d ans;
        ans(0, 0)             = q.w();
        ans.block<1, 3>(0, 1) = -q.vec().transpose();
        ans.block<3, 1>(1, 0) = q.vec();
        ans.block<3, 3>(1, 1) = q.w() * Matrix3d::Identity() + skewSymmetric(q.vec());
        return ans;
    }

    static Eigen::Matrix4d quaternionright(const Quaterniond &q) {
        Eigen::Matrix4d ans;
        ans(0, 0)             = q.w();
        ans.block<1, 3>(0, 1) = -q.vec().transpose();
        ans.block<3, 1>(1, 0) = q.vec();
        ans.block<3, 3>(1, 1) = q.w() * Matrix3d::Identity() - skewSymmetric(q.vec());
        return ans;
    }
};

#endif // ROTATION_H
