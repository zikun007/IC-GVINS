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

#ifndef TYPES_H
#define TYPES_H

#include <Eigen/Geometry>

using Eigen::Matrix3d;
using Eigen::Quaterniond;
using Eigen::Vector3d;

typedef struct GNSS {
    double time;

    Vector3d blh;
    Vector3d std;

    bool isyawvalid;
    double yaw;
} GNSS;

typedef struct PVA {
    double time;

    Vector3d blh;
    Vector3d vel;
    Vector3d att;
} PVA;

typedef struct IMU {
    double time;
    double dt;

    Vector3d dtheta;
    Vector3d dvel;

    double odovel;
} IMU;

// Fixed local navigation/world frame W is NED-aligned at the first GNSS fix.
// R_a_b always maps coordinates from frame b to frame a, and t_a_b is the
// position of frame-b's origin expressed in frame a.
struct LocalBodyPose {
    Matrix3d R_w_b{Matrix3d::Identity()};
    Vector3d t_w_b{Vector3d::Zero()};
};

// This is not a Euclidean rigid transform: position is geodetic BLH while the
// attitude is expressed in the current geographic NED frame N.
struct GlobalBodyPose {
    Matrix3d R_n_b{Matrix3d::Identity()};
    Vector3d blh_b{Vector3d::Zero()};
};

struct CameraPose {
    Matrix3d R_w_c{Matrix3d::Identity()};
    Vector3d t_w_c{Vector3d::Zero()};
};

struct CameraExtrinsics {
    Matrix3d R_b_c{Matrix3d::Identity()};
    Vector3d t_b_c{Vector3d::Zero()};
};

#endif // TYPES_H
