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

#include "ic_gvins/ros/fusion_ros.h"
#include "ic_gvins/ros/drawer_rviz.h"

#include "ic_gvins/common/angle.h"
#include "ic_gvins/common/gpstime.h"
#include "ic_gvins/common/logging.h"
#include "ic_gvins/misc.h"
#include "ic_gvins/tracking/frame.h"

#include <yaml-cpp/yaml.h>

#include <sensor_msgs/image_encodings.hpp>

#include <filesystem>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <memory>
#include <sstream>

FusionROS::FusionROS(const rclcpp::Node::SharedPtr &node)
    : node_(node) {}

void FusionROS::setFinished() {
    if (gvins_ && gvins_->isRunning()) {
        gvins_->setFinished();
    }
}

void FusionROS::run() {
    // message topic
    std::string imu_topic, gnss_topic, image_topic;
    node_->declare_parameter<std::string>("imu_topic", "/imu0");
    node_->declare_parameter<std::string>("gnss_topic", "/gnss0");
    node_->declare_parameter<std::string>("image_topic", "/cam0");
    imu_topic   = node_->get_parameter("imu_topic").as_string();
    gnss_topic  = node_->get_parameter("gnss_topic").as_string();
    image_topic = node_->get_parameter("image_topic").as_string();

    // GVINS parameter
    node_->declare_parameter<std::string>("configfile", "gvins.yaml");
    std::string configfile = node_->get_parameter("configfile").as_string();

    // Load configurations
    YAML::Node config;
    std::vector<double> vecdata;
    try {
        config = YAML::LoadFile(configfile);
    } catch (YAML::Exception &exception) {
        std::cout << "Failed to open configuration file" << std::endl;
        return;
    }
    auto outputpath        = config["outputpath"].as<string>();
    auto is_make_outputdir = config["is_make_outputdir"].as<bool>();

    // Create the output directory
    if (!std::filesystem::is_directory(outputpath)) {
        std::filesystem::create_directories(outputpath);
    }
    if (!std::filesystem::is_directory(outputpath)) {
        std::cout << "Failed to open outputpath" << std::endl;
        return;
    }

    if (is_make_outputdir) {
        const auto now      = std::chrono::system_clock::now();
        const auto now_time = std::chrono::system_clock::to_time_t(now);
        std::tm local_time{};
        localtime_r(&now_time, &local_time);
        std::ostringstream timestamp;
        timestamp << std::put_time(&local_time, "/T%Y%m%d%H%M%S");
        outputpath += timestamp.str();
        std::filesystem::create_directories(outputpath);
    }

    // GNSS outage configurations
    isusegnssoutage_ = config["isusegnssoutage"].as<bool>();
    gnssoutagetime_  = config["gnssoutagetime"].as<double>();
    gnssthreshold_   = config["gnssthreshold"].as<double>();
    gnss_std_floor_  = config["gnss_std_floor"] ? config["gnss_std_floor"].as<double>() : 0.0;

    // Glog output path
    FLAGS_log_dir = outputpath;

    // The GVINS object
    Drawer::Ptr drawer = std::make_shared<DrawerRviz>(node_);
    gvins_             = std::make_shared<GVINS>(configfile, outputpath, drawer);

    // check is initialized
    if (!gvins_->isRunning()) {
        LOGE << "Fusion ROS terminate";
        return;
    }

    // subscribe message
    auto imu_sub   = node_->create_subscription<sensor_msgs::msg::Imu>(imu_topic, 200, std::bind(&FusionROS::imuCallback, this, std::placeholders::_1));
    // Keep enough GNSS samples queued while image tracking/optimization runs.
    auto gnss_sub  = node_->create_subscription<sensor_msgs::msg::NavSatFix>(
        gnss_topic, rclcpp::QoS(100).reliable(), std::bind(&FusionROS::gnssCallback, this, std::placeholders::_1));
    // Image tracking and optimization can briefly exceed the 10 Hz input
    // period.  Keep a sufficiently deep reliable queue so rosbag playback
    // does not silently drop frames during those bursts.
    auto image_sub = node_->create_subscription<sensor_msgs::msg::Image>(
        image_topic, rclcpp::QoS(200).reliable(), std::bind(&FusionROS::imageCallback, this, std::placeholders::_1));

    LOGI << "Waiting ROS message...";

    // enter message loopback
    rclcpp::spin(node_);
}

void FusionROS::imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr &imumsg) {
    imu_pre_ = imu_;

    // Time convertion
    double weeksec;
    int week;
    GpsTime::unix2gps(imumsg->header.stamp.sec, imumsg->header.stamp.nanosec, week, weeksec);

    imu_.time = weeksec;
    // delta time
    imu_.dt = imu_.time - imu_pre_.time;

    // IMU measurements, Front-Right-Down
    imu_.dtheta[0] = imumsg->angular_velocity.x * imu_.dt;
    imu_.dtheta[1] = imumsg->angular_velocity.y * imu_.dt;
    imu_.dtheta[2] = imumsg->angular_velocity.z * imu_.dt;
    imu_.dvel[0]   = imumsg->linear_acceleration.x * imu_.dt;
    imu_.dvel[1]   = imumsg->linear_acceleration.y * imu_.dt;
    imu_.dvel[2]   = imumsg->linear_acceleration.z * imu_.dt;

    // Not ready
    if (imu_pre_.time == 0) {
        return;
    }

    imu_buffer_.push(imu_);
    while (!imu_buffer_.empty()) {
        auto imu = imu_buffer_.front();

        // Add new IMU to GVINS
        if (gvins_->addNewImu(imu)) {
            imu_buffer_.pop();
        } else {
            // Thread lock failed, try next time
            break;
        }
    }
}

void FusionROS::gnssCallback(const sensor_msgs::msg::NavSatFix::ConstSharedPtr &gnssmsg) {
    // Time convertion
    double weeksec;
    int week;
    GpsTime::unix2gps(gnssmsg->header.stamp.sec, gnssmsg->header.stamp.nanosec, week, weeksec);

    gnss_.time = weeksec;

    gnss_.blh[0] = gnssmsg->latitude * D2R;
    gnss_.blh[1] = gnssmsg->longitude * D2R;
    gnss_.blh[2] = gnssmsg->altitude;
    gnss_.std[0] = sqrt(gnssmsg->position_covariance[4]); // N
    gnss_.std[1] = sqrt(gnssmsg->position_covariance[0]); // E
    gnss_.std[2] = sqrt(gnssmsg->position_covariance[8]); // D
    gnss_.std    = gnss_.std.cwiseMax(gnss_std_floor_);

    gnss_.isyawvalid = false;

    // Exception
    if ((gnss_.std[0] == 0) || (gnss_.std[1] == 0) || (gnss_.std[2] == 0)) {
        return;
    }

    // Remove bad GNSS
    bool isoutage = false;
    if ((gnss_.std[0] < gnssthreshold_) && (gnss_.std[1] < gnssthreshold_) && (gnss_.std[2] < gnssthreshold_)) {

        if (isusegnssoutage_ && (weeksec >= gnssoutagetime_)) {
            isoutage = true;
        }

        // add new GNSS to GVINS
        if (!isoutage) {
            gvins_->addNewGnss(gnss_);
        }
    }
}

void FusionROS::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &imagemsg) {
    Mat image;

    // Copy image data
    if (imagemsg->encoding == sensor_msgs::image_encodings::MONO8) {
        image = Mat(static_cast<int>(imagemsg->height), static_cast<int>(imagemsg->width), CV_8UC1);
        memcpy(image.data, imagemsg->data.data(), imagemsg->height * imagemsg->width);
    } else if (imagemsg->encoding == sensor_msgs::image_encodings::BGR8) {
        image = Mat(static_cast<int>(imagemsg->height), static_cast<int>(imagemsg->width), CV_8UC3);
        memcpy(image.data, imagemsg->data.data(), imagemsg->height * imagemsg->width * 3);
    } else if (imagemsg->encoding == sensor_msgs::image_encodings::RGB8) {
        image = Mat(static_cast<int>(imagemsg->height), static_cast<int>(imagemsg->width), CV_8UC3);
        memcpy(image.data, imagemsg->data.data(), imagemsg->height * imagemsg->width * 3);
    }

    // Time convertion
    double weeksec;
    int week;
    GpsTime::unix2gps(imagemsg->header.stamp.sec, imagemsg->header.stamp.nanosec, week, weeksec);

    // Add new Image to GVINS
    frame_ = Frame::createFrame(weeksec, image);

    frame_buffer_.push(frame_);
    while (!frame_buffer_.empty()) {
        auto frame = frame_buffer_.front();
        if (gvins_->addNewFrame(frame)) {
            frame_buffer_.pop();
        } else {
            break;
        }
    }

    LOG_EVERY_N(INFO, 20) << "Raw data time " << Logging::doubleData(imu_.time) << ", "
                          << Logging::doubleData(gnss_.time) << ", " << Logging::doubleData(frame_->stamp());
}

int main(int argc, char *argv[]) {
    // Glog initialization
    Logging::initialization(argv, true, true);

    // ROS node
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("gvins_node");

    auto fusion = std::make_shared<FusionROS>(node);

    std::cout << "Fusion process is started..." << std::endl;

    // Enter message loop
    fusion->run();

    // Stop the GVINS threads after spin returns (SIGINT)
    fusion->setFinished();

    rclcpp::shutdown();

    return 0;
}
