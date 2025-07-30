/*
 * Copyright (C) 2025      Jin Rhee
 * Script for listening and saving OpenVINS' tracked features, IMU biases,
 * and camera calibration info
 * 
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2018-2023 Patrick Geneva
 * Copyright (C) 2018-2023 Guoquan Huang
 * Copyright (C) 2018-2023 OpenVINS Contributors
 * Copyright (C) 2018-2019 Kevin Eckenhoff
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

#include <memory>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "ov_msckf/msg/ov_runtime_status.hpp"
#include "ov_msckf/msg/ov_active_feature.hpp"
#include "ov_msckf/msg/ov_active_feature_array.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "utils/csvfile.h"

using namespace ov_msckf;

class RuntimeSubscriber : public rclcpp::Node
{
  public:
    RuntimeSubscriber() : Node("listen_runtime")
    {
      // Subscribe to OVRuntimeStatus
      this->subOVRuntimeStatus_ = this->create_subscription<ov_msckf::msg::OVRuntimeStatus>(
        "/ov_msckf/runtime_status", 10, std::bind(&RuntimeSubscriber::topic1_callback, this, std::placeholders::_1)
      );

      // Subscribe to OVActiveFeatureArray
      this->subOVActiveFeatureArray_ = this->create_subscription<ov_msckf::msg::OVActiveFeatureArray>(
        "/ov_msckf/active_features", 10, std::bind(&RuntimeSubscriber::topic2_callback, this, std::placeholders::_1)
      );
      
      // Get save file path, create if it doesn't exist
      this->declare_parameter<std::string>("save_path", "unset_save_path");
      this->get_parameter<std::string>("save_path", this->save_path);

      // Name save paths
      std::string bias_path = this->save_path + "_bias.csv";
      std::string msckf_path = this->save_path + "_msckf.csv"; 
      std::string slam_path = this->save_path + "_slam.csv";
      std::string feat_path = this->save_path + "_feat.csv";

      // Open csv files at save_path
      this->bias_csv = std::make_unique<csvfile>(bias_path);
      this->msckf_csv = std::make_unique<csvfile>(msckf_path);
      this->slam_csv = std::make_unique<csvfile>(slam_path);
      this->feat_csv = std::make_unique<csvfile>(feat_path);
      RCLCPP_INFO(this->get_logger(), "Created .csv files");

      // Populate first lines
      *(this->bias_csv) << "sec" << "nanosec" << "frame_id"
                        << "bias_a_x" << "bias_a_y" << "bias_a_z"
                        << "bias_g_x" << "bias_g_y" << "bias_g_z"
                        << "t_offset_imu_cam" << endrow;

      *(this->msckf_csv) << "sec" << "nanosec" << "frame_id"
                         << "msckf_num_points" << "x" << "y" << "z" << endrow;

      *(this->slam_csv) << "sec" << "nanosec" << "frame_id"
                        << "slam_num_points" << "x" << "y" << "z" << endrow;
      
      *(this->feat_csv) << "sec" << "nanosec" << "frame_id" << "feat_num"
                        << "featid"
                        << "posx" << "posy" << "posz" 
                        << "u"    << "v"    << "d"
                        << "..." << endrow;
    }

  private:
    void topic1_callback(const ov_msckf::msg::OVRuntimeStatus::SharedPtr status_msg)
    {
      
      RCLCPP_INFO(this->get_logger(), "Received message on topic1");
      std::string sec = std::to_string(status_msg->header.stamp.sec);
      std::string nanosec = std::to_string(status_msg->header.stamp.nanosec);
      std::string frame_id = status_msg->header.frame_id;

      *(this->bias_csv) << sec << nanosec << frame_id;
      *(this->msckf_csv) << sec << nanosec << frame_id;
      *(this->slam_csv) << sec << nanosec << frame_id;

      // Bias CSV
      std::string bias_a_x = std::to_string(status_msg->bias_a.x);
      std::string bias_a_y = std::to_string(status_msg->bias_a.y);
      std::string bias_a_z = std::to_string(status_msg->bias_a.z);
      std::string bias_g_x = std::to_string(status_msg->bias_g.x);
      std::string bias_g_y = std::to_string(status_msg->bias_g.y);
      std::string bias_g_z = std::to_string(status_msg->bias_g.z);
      std::string t_offset_imu_cam = std::to_string(status_msg->t_offset_imu_cam);

      *(this->bias_csv) << bias_a_x << bias_a_y << bias_a_z << bias_g_x << bias_g_y << bias_g_z << t_offset_imu_cam << endrow;
      
      // MSCKF CSV
      sensor_msgs::msg::PointCloud2 cloud_msckf = status_msg->cloud_msckf_features;

      // Extract XYZ coordinates from cloud_msckf
      if (cloud_msckf.width * cloud_msckf.height == 0) {
        RCLCPP_WARN(this->get_logger(), "cloud_msckf_features is empty");
        *(this->msckf_csv) << "0" << endrow;
      }
      else {
        // Find offsets for x, y, z fields
        int offset_x = -1, offset_y = -1, offset_z = -1;
        for (const auto &field : cloud_msckf.fields) {
          if (field.name == "x") offset_x = field.offset;
          if (field.name == "y") offset_y = field.offset;
          if (field.name == "z") offset_z = field.offset;
        }
        if (offset_x == -1 || offset_y == -1 || offset_z == -1) {
          RCLCPP_ERROR(this->get_logger(), "Could not find x/y/z fields in cloud_msckf_features");
          *(this->msckf_csv) << "0" << endrow;
        }
        else {
          size_t point_step = cloud_msckf.point_step;
          size_t num_points = cloud_msckf.width * cloud_msckf.height;

          *(this->msckf_csv) << num_points;
          std::string x_str;
          std::string y_str;
          std::string z_str;

          for (size_t i = 0; i < num_points; ++i) {
            const uint8_t* ptr = &cloud_msckf.data[i * point_step];
            float x = *reinterpret_cast<const float*>(ptr + offset_x);
            float y = *reinterpret_cast<const float*>(ptr + offset_y);
            float z = *reinterpret_cast<const float*>(ptr + offset_z);
            
            x_str = std::to_string(x);
            y_str = std::to_string(y);
            z_str = std::to_string(z);
            
            // Save xyz to csv
            *(this->msckf_csv) << x_str << y_str << z_str;
            //RCLCPP_INFO(this->get_logger(), "Point %zu: x=%f, y=%f, z=%f", i, x, y, z);
          }
          *(this->msckf_csv) << endrow;
        }
      }
      
      // SLAM CSV
      sensor_msgs::msg::PointCloud2 cloud_slam = status_msg->cloud_slam_features;

      // Extract XYZ coordinates from cloud_slam
      if (cloud_slam.width * cloud_slam.height == 0) {
        RCLCPP_WARN(this->get_logger(), "cloud_slam_features is empty");
        *(this->slam_csv) << "0" << endrow;
      }
      else {
        // Find offsets for x, y, z fields
        int offset_x = -1, offset_y = -1, offset_z = -1;
        for (const auto &field : cloud_slam.fields) {
          if (field.name == "x") offset_x = field.offset;
          if (field.name == "y") offset_y = field.offset;
          if (field.name == "z") offset_z = field.offset;
        }
        if (offset_x == -1 || offset_y == -1 || offset_z == -1) {
          RCLCPP_ERROR(this->get_logger(), "Could not find x/y/z fields in cloud_slam_features");
          *(this->slam_csv) << "0" << endrow;
        }
        else {
          size_t point_step = cloud_slam.point_step;
          size_t num_points = cloud_slam.width * cloud_slam.height;
          
          *(this->slam_csv) << num_points;
          std::string x_str;
          std::string y_str;
          std::string z_str;

          for (size_t i = 0; i < num_points; ++i) {
            const uint8_t* ptr = &cloud_slam.data[i * point_step];
            float x = *reinterpret_cast<const float*>(ptr + offset_x);
            float y = *reinterpret_cast<const float*>(ptr + offset_y);
            float z = *reinterpret_cast<const float*>(ptr + offset_z);
            
            x_str = std::to_string(x);
            y_str = std::to_string(y);
            z_str = std::to_string(z);
            
            // Save xyz to csv
            *(this->slam_csv) << x_str << y_str << z_str;
            //RCLCPP_INFO(this->get_logger(), "Point %zu: x=%f, y=%f, z=%f", i, x, y, z);
          }
          *(this->slam_csv) << endrow;
        }
      }
    }

    void topic2_callback(const ov_msckf::msg::OVActiveFeatureArray::SharedPtr feat_msg) {
      
      RCLCPP_INFO(this->get_logger(), "Received message on topic2");

      std::string sec = std::to_string(feat_msg->header.stamp.sec);
      std::string nanosec = std::to_string(feat_msg->header.stamp.nanosec);
      std::string frame_id = feat_msg->header.frame_id;
      *(this->feat_csv) << sec << nanosec << frame_id;
      
      std::vector<ov_msckf::msg::OVActiveFeature> feat_arr = feat_msg->data;
      std::string feat_num = std::to_string(feat_arr.size());
      *(this->feat_csv) << feat_num;

      for (auto &feat : feat_arr) {
        std::string tempStr;
        tempStr = std::to_string(feat.featid);
        *(this->feat_csv) << tempStr;
        tempStr = std::to_string(feat.posinglobal.x);
        *(this->feat_csv) << tempStr;
        tempStr = std::to_string(feat.posinglobal.y);
        *(this->feat_csv) << tempStr;
        tempStr = std::to_string(feat.posinglobal.z);
        *(this->feat_csv) << tempStr;
        tempStr = std::to_string(feat.uvd.x);
        *(this->feat_csv) << tempStr;
        tempStr = std::to_string(feat.uvd.y);
        *(this->feat_csv) << tempStr;
        tempStr = std::to_string(feat.uvd.z);
        *(this->feat_csv) << tempStr;
      }
      *(this->feat_csv) << endrow;
    }
    
    rclcpp::Subscription<ov_msckf::msg::OVRuntimeStatus>::SharedPtr subOVRuntimeStatus_;
    rclcpp::Subscription<ov_msckf::msg::OVActiveFeatureArray>::SharedPtr subOVActiveFeatureArray_;

    std::string save_path;
    std::unique_ptr<csvfile> bias_csv;
    std::unique_ptr<csvfile> msckf_csv;
    std::unique_ptr<csvfile> slam_csv;
    std::unique_ptr<csvfile> feat_csv;
};

int main(int argc, char **argv) {
  // Ensure we have a path, if the user passes it then we should use it
  std::string save_path = "unset_save_path";
  if (argc > 1) {
    save_path = argv[1];
  }

  // Launch our ros node
  rclcpp::init(argc, argv);
  
  // Create our subscriber node
  auto node = std::make_shared<RuntimeSubscriber>();
  
  // Spin to keep the node alive and process callbacks
  rclcpp::spin(node);
  
  rclcpp::shutdown();
  return 0;
}