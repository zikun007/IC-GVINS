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

#include "ic_gvins/ros/drawer_rviz.h"

#include "ic_gvins/common/logging.h"
#include "ic_gvins/common/rotation.h"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

DrawerRviz::DrawerRviz(const rclcpp::Node::SharedPtr &node)
    : node_(node)
    , isfinished_(false)
    , isframerdy_(false)
    , ismaprdy_(false) {

    frame_id_ = "world";

    pose_pub_           = node_->create_publisher<nav_msgs::msg::Odometry>("pose", 2);
    path_pub_           = node_->create_publisher<nav_msgs::msg::Path>("path", 2);
    track_image_pub_    = node_->create_publisher<sensor_msgs::msg::Image>("tracking", 2);
    fixed_points_pub_   = node_->create_publisher<sensor_msgs::msg::PointCloud2>("fixed", 2);
    current_points_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("current", 2);
}

void DrawerRviz::setFinished() {
    isfinished_ = true;
    update_sem_.notify_one();
}

void DrawerRviz::run() {

    while (!isfinished_) {
        // 等待绘图更新信号
        std::unique_lock<std::mutex> lock(update_mutex_);
        update_sem_.wait(lock);

        // 发布跟踪的图像
        if (isframerdy_) {
            publishTrackingImage();

            isframerdy_ = false;
        }

        // 发布轨迹和地图点
        if (ismaprdy_) {
            publishOdometry();

            publishMapPoints();

            ismaprdy_ = false;
        }
    }
}

void DrawerRviz::updateFrame(Frame::Ptr frame) {
    std::unique_lock<std::mutex> lock(image_mutex_);

    frame->image().copyTo(raw_image_);

    isframerdy_ = true;
    update_sem_.notify_one();
}

void DrawerRviz::updateTrackedMapPoints(vector<cv::Point2f> map, vector<cv::Point2f> matched,
                                        vector<MapPointType> mappoint_type) {
    std::unique_lock<std::mutex> lock(image_mutex_);
    pts2d_map_     = std::move(map);
    pts2d_matched_ = std::move(matched);
    mappoint_type_ = std::move(mappoint_type);
}

void DrawerRviz::updateTrackedRefPoints(vector<cv::Point2f> ref, vector<cv::Point2f> cur) {
    std::unique_lock<std::mutex> lock(image_mutex_);
    pts2d_ref_ = std::move(ref);
    pts2d_cur_ = std::move(cur);
}

void DrawerRviz::publishTrackingImage() {
    std::unique_lock<std::mutex> lock(image_mutex_);

    Mat drawed;
    drawTrackingImage(raw_image_, drawed);

    sensor_msgs::msg::Image image;

    image.header.stamp    = node_->now();
    image.header.frame_id = frame_id_;
    image.encoding        = sensor_msgs::image_encodings::BGR8;
    image.height          = drawed.rows;
    image.width           = drawed.cols;

    size_t size = image.height * image.width * 3;
    image.step  = image.width * 3;
    image.data.resize(size);
    memcpy(image.data.data(), drawed.data, size);

    track_image_pub_->publish(image);
}

void DrawerRviz::publishMapPoints() {
    std::unique_lock<std::mutex> lock(map_mutex_);

    auto stamp = node_->now();

    // 当前窗口内的路标点
    sensor_msgs::msg::PointCloud2 current_pointcloud;

    current_pointcloud.header.stamp    = stamp;
    current_pointcloud.header.frame_id = frame_id_;
    current_pointcloud.height          = 1;
    current_pointcloud.is_dense        = true;

    // 收集有效点
    std::vector<Vector3d> current_points;
    for (const auto &local : map_->landmarks()) {
        if (local.second && !local.second->isOutlier()) {
            current_points.push_back(local.second->pos());
        }
    }

    sensor_msgs::PointCloud2Modifier current_modifier(current_pointcloud);
    current_modifier.setPointCloud2FieldsByString(1, "xyz");
    current_modifier.resize(current_points.size());

    sensor_msgs::PointCloud2Iterator<float> cur_x(current_pointcloud, "x");
    sensor_msgs::PointCloud2Iterator<float> cur_y(current_pointcloud, "y");
    sensor_msgs::PointCloud2Iterator<float> cur_z(current_pointcloud, "z");
    for (const auto &p : current_points) {
        *cur_x = static_cast<float>(p.x());
        *cur_y = static_cast<float>(p.y());
        *cur_z = static_cast<float>(p.z());
        ++cur_x;
        ++cur_y;
        ++cur_z;
    }
    current_points_pub_->publish(current_pointcloud);

    // 新加入的地图点
    sensor_msgs::msg::PointCloud2 fixed_pointcloud;

    fixed_pointcloud.header.stamp    = stamp;
    fixed_pointcloud.header.frame_id = frame_id_;
    fixed_pointcloud.height          = 1;
    fixed_pointcloud.is_dense        = true;

    sensor_msgs::PointCloud2Modifier fixed_modifier(fixed_pointcloud);
    fixed_modifier.setPointCloud2FieldsByString(1, "xyz");
    fixed_modifier.resize(fixed_mappoints_.size());

    sensor_msgs::PointCloud2Iterator<float> fixed_x(fixed_pointcloud, "x");
    sensor_msgs::PointCloud2Iterator<float> fixed_y(fixed_pointcloud, "y");
    sensor_msgs::PointCloud2Iterator<float> fixed_z(fixed_pointcloud, "z");
    for (const auto &p : fixed_mappoints_) {
        *fixed_x = static_cast<float>(p.x());
        *fixed_y = static_cast<float>(p.y());
        *fixed_z = static_cast<float>(p.z());
        ++fixed_x;
        ++fixed_y;
        ++fixed_z;
    }
    fixed_points_pub_->publish(fixed_pointcloud);

    fixed_mappoints_.clear();
}

void DrawerRviz::publishOdometry() {
    std::unique_lock<std::mutex> lock(map_mutex_);

    nav_msgs::msg::Odometry odometry;

    auto quaternion = Rotation::matrix2quaternion(pose_w_c_.R_w_c);
    auto stamp      = node_->now();

    // Odometry
    odometry.header.stamp            = stamp;
    odometry.header.frame_id         = frame_id_;
    odometry.pose.pose.position.x    = pose_w_c_.t_w_c.x();
    odometry.pose.pose.position.y    = pose_w_c_.t_w_c.y();
    odometry.pose.pose.position.z    = pose_w_c_.t_w_c.z();
    odometry.pose.pose.orientation.x = quaternion.x();
    odometry.pose.pose.orientation.y = quaternion.y();
    odometry.pose.pose.orientation.z = quaternion.z();
    odometry.pose.pose.orientation.w = quaternion.w();
    pose_pub_->publish(odometry);

    // Path
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header.stamp    = stamp;
    pose_stamped.header.frame_id = frame_id_;
    pose_stamped.pose            = odometry.pose.pose;

    path_.header.stamp    = stamp;
    path_.header.frame_id = frame_id_;
    path_.poses.push_back(pose_stamped);
    path_pub_->publish(path_);
}

void DrawerRviz::addNewFixedMappoint(Vector3d point) {
    std::unique_lock<std::mutex> lock(map_mutex_);

    fixed_mappoints_.push_back(point);
}

void DrawerRviz::updateMap(const CameraPose &pose_w_c) {
    std::unique_lock<std::mutex> lock(map_mutex_);

    pose_w_c_ = pose_w_c;

    ismaprdy_ = true;
    update_sem_.notify_one();
}
