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

#ifndef EARTH_H
#define EARTH_H

#include "ic_gvins/common/angle.h"
#include "ic_gvins/common/types.h"

#include <Eigen/Geometry>

#include <cmath>

using Eigen::Matrix3d;
using Eigen::Quaterniond;
using Eigen::Vector3d;

inline constexpr double WGS84_WIE = 7.2921151467e-5;
inline constexpr double WGS84_RA  = 6378137.0;
inline constexpr double WGS84_RB  = 6356752.3142451793;
inline constexpr double WGS84_E1  = 0.0066943799901413156;
inline constexpr double WGS84_F   = (WGS84_RA - WGS84_RB) / WGS84_RA;
inline constexpr double WGS84_E2  = WGS84_E1 / (1.0 - WGS84_E1);
inline constexpr double WGS84_GM0 = 398600441800000.0;

// Earth-model implementation adapted from open_nav/nav_core revision
// 882e8ba12889d04cbee0cf4a267518360d9df28d.
class Earth {

public:
    static double gravity(const Vector3d &blh) {
        double sin_lat2 = std::sin(blh.x());
        sin_lat2 *= sin_lat2;
        const double height = blh.z();
        return 9.7803267715 * (1.0 + 0.0052790414 * sin_lat2 + 0.0000232718 * sin_lat2 * sin_lat2) +
               height * (4.3977311e-9 * sin_lat2 - 3.0876910891e-6) + 7.211e-13 * height * height;
    }

    static Eigen::Vector2d meridianPrimeVerticalRadius(double lat) {
        const double sin_lat = std::sin(lat);
        const double q       = 1.0 - WGS84_E1 * sin_lat * sin_lat;
        const double root_q  = std::sqrt(q);
        return {WGS84_RA * (1.0 - WGS84_E1) / (q * root_q), WGS84_RA / root_q};
    }

    static double RN(double lat) {
        return meridianPrimeVerticalRadius(lat).y();
    }

    // NED -> ECEF.
    static Matrix3d cne(const Vector3d &blh) {
        const double sin_lat = std::sin(blh.x());
        const double cos_lat = std::cos(blh.x());
        const double sin_lon = std::sin(blh.y());
        const double cos_lon = std::cos(blh.y());

        Matrix3d R_e_n;
        R_e_n << -sin_lat * cos_lon, -sin_lon, -cos_lat * cos_lon, -sin_lat * sin_lon, cos_lon, -cos_lat * sin_lon,
            cos_lat, 0.0, -sin_lat;
        return R_e_n;
    }

    static Quaterniond qne(const Vector3d &blh) {
        return Quaterniond(cne(blh)).normalized();
    }

    static Vector3d blh(const Quaterniond &qne_value, double height) {
        const Matrix3d R_e_n = qne_value.normalized().toRotationMatrix();
        const double lat     = std::atan2(-R_e_n(2, 2), R_e_n(2, 0));
        const double lon     = std::atan2(-R_e_n(0, 1), R_e_n(1, 1));
        return {lat, lon, height};
    }

    static Vector3d blh2ecef(const Vector3d &blh) {
        const double sin_lat = std::sin(blh.x());
        const double cos_lat = std::cos(blh.x());
        const double sin_lon = std::sin(blh.y());
        const double cos_lon = std::cos(blh.y());
        const double rn      = RN(blh.x());

        return {(rn + blh.z()) * cos_lat * cos_lon, (rn + blh.z()) * cos_lat * sin_lon,
                (rn * (1.0 - WGS84_E1) + blh.z()) * sin_lat};
    }

    static Vector3d ecef2blh(const Vector3d &ecef) {
        const double horizontal = std::hypot(ecef.x(), ecef.y());
        if (horizontal < 1e-8) {
            return {ecef.z() >= 0.0 ? PI / 2.0 : -PI / 2.0, 0.0, std::abs(ecef.z()) - WGS84_RB};
        }

        const double lon = std::atan2(ecef.y(), ecef.x());
        double lat       = std::atan2(ecef.z(), horizontal * (1.0 - WGS84_E1));
        double height    = 0.0;

        for (int i = 0; i < 10; ++i) {
            const double rn         = RN(lat);
            const double new_height = horizontal / std::cos(lat) - rn;
            const double new_lat    = std::atan2(ecef.z(), horizontal * (1.0 - WGS84_E1 * rn / (rn + new_height)));
            if (std::abs(new_lat - lat) < 1e-13 && std::abs(new_height - height) < 1e-5) {
                lat    = new_lat;
                height = new_height;
                break;
            }
            lat    = new_lat;
            height = new_height;
        }
        return {lat, lon, height};
    }

    static Matrix3d DRi(const Vector3d &blh) {
        const Eigen::Vector2d rmn = meridianPrimeVerticalRadius(blh.x());
        Matrix3d dri              = Matrix3d::Zero();
        dri(0, 0)                 = 1.0 / (rmn.x() + blh.z());
        dri(1, 1)                 = 1.0 / ((rmn.y() + blh.z()) * std::cos(blh.x()));
        dri(2, 2)                 = -1.0;
        return dri;
    }

    static Matrix3d DR(const Vector3d &blh) {
        const Eigen::Vector2d rmn = meridianPrimeVerticalRadius(blh.x());
        Matrix3d dr               = Matrix3d::Zero();
        dr(0, 0)                  = rmn.x() + blh.z();
        dr(1, 1)                  = (rmn.y() + blh.z()) * std::cos(blh.x());
        dr(2, 2)                  = -1.0;
        return dr;
    }

    static Vector3d local2global(const Vector3d &origin, const Vector3d &local) {
        return ecef2blh(blh2ecef(origin) + cne(origin) * local);
    }

    static Vector3d global2local(const Vector3d &origin, const Vector3d &global) {
        return cne(origin).transpose() * (blh2ecef(global) - blh2ecef(origin));
    }

    static GlobalBodyPose local2global(const Vector3d &origin, const LocalBodyPose &local_pose) {
        GlobalBodyPose global_pose;
        global_pose.blh_b = local2global(origin, local_pose.t_w_b);
        global_pose.R_n_b = cne(global_pose.blh_b).transpose() * cne(origin) * local_pose.R_w_b;
        return global_pose;
    }

    static LocalBodyPose global2local(const Vector3d &origin, const GlobalBodyPose &global_pose) {
        LocalBodyPose local_pose;
        local_pose.t_w_b = global2local(origin, global_pose.blh_b);
        local_pose.R_w_b = cne(origin).transpose() * cne(global_pose.blh_b) * global_pose.R_n_b;
        return local_pose;
    }

    static Vector3d iewe() {
        return {0.0, 0.0, WGS84_WIE};
    }

    static Vector3d iewn(double lat) {
        return {WGS84_WIE * std::cos(lat), 0.0, -WGS84_WIE * std::sin(lat)};
    }

    static Vector3d iewn(const Vector3d &origin, const Vector3d &local) {
        return iewn(local2global(origin, local).x());
    }

    static Vector3d enwn(const Eigen::Vector2d &, const Vector3d &blh, const Vector3d &vel) {
        const Eigen::Vector2d radii = meridianPrimeVerticalRadius(blh.x());
        const double rm             = radii.x() + blh.z();
        const double rn             = radii.y() + blh.z();
        return {vel.y() / rn, -vel.x() / rm, -vel.y() * std::tan(blh.x()) / rn};
    }

    static Vector3d enwn(const Vector3d &origin, const Vector3d &local, const Vector3d &vel) {
        const Vector3d global = local2global(origin, local);
        return enwn(meridianPrimeVerticalRadius(global.x()), global, vel);
    }
};

#endif // EARTH_H
