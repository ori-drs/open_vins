/*
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

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rcutils/types/uint8_array.h>
#include <rosbag2_cpp/converter_options.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/imgcodecs.hpp>

#include "core/VioManager.h"
#include "core/VioManagerOptions.h"
#include "ros/ROS2Visualizer.h"
#include "utils/dataset_reader.h"

using namespace ov_msckf;

std::shared_ptr<VioManager> sys;
std::shared_ptr<ROS2Visualizer> viz;

struct BagMsg {
  std::string topic;
  rclcpp::Time time;
  std::shared_ptr<rcutils_uint8_array_t> serialized_data;
};

static std::string encoding_from_mat(const cv::Mat &image) {
  const int type = image.type();
  if (type == CV_8UC1)
    return sensor_msgs::image_encodings::MONO8;
  if (type == CV_16UC1)
    return sensor_msgs::image_encodings::MONO16;
  if (type == CV_8UC3)
    return sensor_msgs::image_encodings::BGR8;
  if (type == CV_8UC4)
    return sensor_msgs::image_encodings::BGRA8;
  if (type == CV_16UC3)
    return sensor_msgs::image_encodings::BGR16;
  if (type == CV_16UC4)
    return sensor_msgs::image_encodings::BGRA16;
  return sensor_msgs::image_encodings::BGR8;
}

static sensor_msgs::msg::Image::SharedPtr decode_compressed_image(const sensor_msgs::msg::CompressedImage &msg) {
  if (msg.data.empty())
    return nullptr;
  auto *data_ptr = const_cast<unsigned char *>(msg.data.data());
  cv::Mat raw(1, static_cast<int>(msg.data.size()), CV_8UC1, data_ptr);
  cv::Mat decoded = cv::imdecode(raw, cv::IMREAD_UNCHANGED);
  if (decoded.empty())
    return nullptr;
  const std::string encoding = encoding_from_mat(decoded);
  return cv_bridge::CvImage(msg.header, encoding, decoded).toImageMsg();
}

static sensor_msgs::msg::Image::SharedPtr decode_image_message(
    const BagMsg &bag_msg,
    const std::string &type,
    rclcpp::Serialization<sensor_msgs::msg::Image> &image_serializer,
    rclcpp::Serialization<sensor_msgs::msg::CompressedImage> &compressed_serializer) {
  if (type.empty())
    return nullptr;
  if (type.find("CompressedImage") != std::string::npos) {
    auto compressed = std::make_shared<sensor_msgs::msg::CompressedImage>();
    rclcpp::SerializedMessage serialized_msg(*bag_msg.serialized_data);
    compressed_serializer.deserialize_message(&serialized_msg, compressed.get());
    return decode_compressed_image(*compressed);
  }
  if (type.find("sensor_msgs/msg/Image") == std::string::npos)
    return nullptr;
  auto image = std::make_shared<sensor_msgs::msg::Image>();
  rclcpp::SerializedMessage serialized_msg(*bag_msg.serialized_data);
  image_serializer.deserialize_message(&serialized_msg, image.get());
  return image;
}

// Main function
int main(int argc, char **argv) {

  // Ensure we have a path, if the user passes it then we should use it
  std::string config_path = "unset_path_to_config.yaml";
  if (argc > 1) {
    config_path = argv[1];
  }

  // Launch our ros node
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.allow_undeclared_parameters(true);
  options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<rclcpp::Node>("ros2_serial_msckf", options);
  node->get_parameter("config_path", config_path);

  // Load the config
  auto parser = std::make_shared<ov_core::YamlParser>(config_path);
#if ROS_AVAILABLE == 2
  parser->set_node(node);
#endif

  // Verbosity
  std::string verbosity = "INFO";
  parser->parse_config("verbosity", verbosity);
  ov_core::Printer::setPrintLevel(verbosity);

  // Create our VIO system
  VioManagerOptions params;
  params.print_and_load(parser);
  // params.num_opencv_threads = 0; // uncomment if you want repeatability
  // params.use_multi_threading_pubs = 0; // uncomment if you want repeatability
  params.use_multi_threading_subs = false;
  sys = std::make_shared<VioManager>(params);
  viz = std::make_shared<ROS2Visualizer>(node, sys);

  // Ensure we read in all parameters required
  if (!parser->successful()) {
    PRINT_ERROR(RED "[SERIAL]: unable to parse all parameters, please fix\n" RESET);
    std::exit(EXIT_FAILURE);
  }

  //===================================================================================
  //===================================================================================
  //===================================================================================

  // Our imu topic
  std::string topic_imu = "/imu0";
  node->get_parameter_or("topic_imu", topic_imu, topic_imu);
  parser->parse_external("relative_config_imu", "imu0", "rostopic", topic_imu);
  PRINT_DEBUG("[SERIAL]: imu: %s\n", topic_imu.c_str());

  // Our camera topics
  std::vector<std::string> topic_cameras;
  for (int i = 0; i < params.state_options.num_cameras; i++) {
    std::string cam_topic = "/cam" + std::to_string(i) + "/image_raw";
    node->get_parameter_or("topic_camera" + std::to_string(i), cam_topic, cam_topic);
    parser->parse_external("relative_config_imucam", "cam" + std::to_string(i), "rostopic", cam_topic);
    topic_cameras.emplace_back(cam_topic);
    PRINT_DEBUG("[SERIAL]: cam: %s\n", cam_topic.c_str());
  }

  // Location of the ROS bag we want to read in
  std::string path_to_bag = "/home/patrick/datasets/eth/V1_01_easy";
  node->get_parameter_or("path_bag", path_to_bag, path_to_bag);
  PRINT_DEBUG("[SERIAL]: ros bag path is: %s\n", path_to_bag.c_str());

  // Load groundtruth if we have it
  // NOTE: needs to be a csv ASL format file
  std::map<double, Eigen::Matrix<double, 17, 1>> gt_states;
  std::string path_to_gt;
  if (node->get_parameter("path_gt", path_to_gt)) {
    if (!path_to_gt.empty()) {
      ov_core::DatasetReader::load_gt_file(path_to_gt, gt_states);
      PRINT_DEBUG("[SERIAL]: gt file path is: %s\n", path_to_gt.c_str());
    }
  }

  // Get our start location and how much of the bag we want to play
  // Make the bag duration < 0 to just process to the end of the bag
  double bag_start = 0.0;
  double bag_durr = -1.0;
  node->get_parameter_or("bag_start", bag_start, bag_start);
  node->get_parameter_or("bag_durr", bag_durr, bag_durr);
  PRINT_DEBUG("[SERIAL]: bag start: %.1f\n", bag_start);
  PRINT_DEBUG("[SERIAL]: bag duration: %.1f\n", bag_durr);

  //===================================================================================
  //===================================================================================
  //===================================================================================

  // Load rosbag here, and find messages we can play
  rosbag2_cpp::readers::SequentialReader reader;
  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = path_to_bag;
  storage_options.storage_id = "sqlite3";
  rosbag2_cpp::ConverterOptions converter_options;
  converter_options.input_serialization_format = "cdr";
  converter_options.output_serialization_format = "cdr";

  try {
    reader.open(storage_options, converter_options);
  } catch (const std::exception &e) {
    PRINT_ERROR(RED "[SERIAL]: unable to open bag: %s\n" RESET, e.what());
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }

  std::unordered_map<std::string, std::string> topic_to_type;
  for (const auto &topic_info : reader.get_all_topics_and_types()) {
    topic_to_type[topic_info.name] = topic_info.type;
  }

  bool have_any = false;
  rclcpp::Time time_begin_all(0, 0, RCL_SYSTEM_TIME);
  rclcpp::Time time_end_all(0, 0, RCL_SYSTEM_TIME);
  double max_camera_time = -1.0;
  std::vector<BagMsg> msgs;

  while (reader.has_next()) {
    auto bag_msg = reader.read_next();
    rcutils_time_point_value_t stamp_ns = bag_msg->send_timestamp;
    if (stamp_ns == std::numeric_limits<rcutils_time_point_value_t>::max() || stamp_ns <= 0) {
      stamp_ns = bag_msg->recv_timestamp;
    }
    if (stamp_ns == std::numeric_limits<rcutils_time_point_value_t>::max() || stamp_ns < 0) {
      stamp_ns = 0;
    }
    rclcpp::Time msg_time(stamp_ns, RCL_SYSTEM_TIME);

    if (!have_any) {
      time_begin_all = msg_time;
      time_end_all = msg_time;
      have_any = true;
    } else {
      if (msg_time < time_begin_all)
        time_begin_all = msg_time;
      if (msg_time > time_end_all)
        time_end_all = msg_time;
    }

    if (bag_msg->topic_name == topic_imu) {
      msgs.push_back({bag_msg->topic_name, msg_time, bag_msg->serialized_data});
      continue;
    }

    for (int i = 0; i < params.state_options.num_cameras; i++) {
      if (bag_msg->topic_name == topic_cameras.at(i)) {
        msgs.push_back({bag_msg->topic_name, msg_time, bag_msg->serialized_data});
        max_camera_time = std::max(max_camera_time, msg_time.seconds());
        break;
      }
    }
  }

  if (!have_any || msgs.empty()) {
    PRINT_ERROR(RED "[SERIAL]: No messages to play on specified topics.  Exiting.\n" RESET);
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }

  // Start a few seconds in from the full view time
  // If we have a negative duration then use the full bag length
  rclcpp::Time time_init = time_begin_all + rclcpp::Duration::from_seconds(bag_start);
  rclcpp::Time time_finish = (bag_durr < 0) ? time_end_all : time_init + rclcpp::Duration::from_seconds(bag_durr);
  PRINT_DEBUG("time start = %.6f\n", time_init.seconds());
  PRINT_DEBUG("time end   = %.6f\n", time_finish.seconds());
  PRINT_DEBUG("[SERIAL]: total of %zu messages!\n", msgs.size());

  //===================================================================================
  //===================================================================================
  //===================================================================================

  rclcpp::Serialization<sensor_msgs::msg::Imu> imu_serializer;
  rclcpp::Serialization<sensor_msgs::msg::Image> image_serializer;
  rclcpp::Serialization<sensor_msgs::msg::CompressedImage> compressed_serializer;

  // Loop through our message array, and lets process them
  std::set<int> used_index;
  for (int m = 0; m < static_cast<int>(msgs.size()); m++) {

    // End once we reach the last time, or skip if before beginning time (shouldn't happen)
    if (!rclcpp::ok() || msgs.at(m).time > time_finish ||
        (max_camera_time > 0.0 && msgs.at(m).time.seconds() > max_camera_time))
      break;
    if (msgs.at(m).time < time_init)
      continue;

    // Skip messages that we have already used
    if (used_index.find(m) != used_index.end()) {
      used_index.erase(m);
      continue;
    }

    // IMU processing
    if (msgs.at(m).topic == topic_imu) {
      const auto type_it = topic_to_type.find(msgs.at(m).topic);
      if (type_it == topic_to_type.end() || type_it->second.find("sensor_msgs/msg/Imu") == std::string::npos) {
        PRINT_ERROR(RED "[SERIAL]: IMU topic has unsupported type: %s\n" RESET,
                    (type_it == topic_to_type.end()) ? "<unknown>" : type_it->second.c_str());
        continue;
      }
      auto msg = std::make_shared<sensor_msgs::msg::Imu>();
      rclcpp::SerializedMessage serialized_msg(*msgs.at(m).serialized_data);
      imu_serializer.deserialize_message(&serialized_msg, msg.get());
      viz->callback_inertial(msg);
    }

    // Camera processing
    for (int cam_id = 0; cam_id < params.state_options.num_cameras; cam_id++) {

      // Skip if this message is not a camera topic
      if (msgs.at(m).topic != topic_cameras.at(cam_id))
        continue;

      // We have a matching camera topic here, now find the other cameras for this time
      // For each camera, we will find the nearest timestamp (within 0.02sec) that is greater than the current
      // If we are unable, then this message should just be skipped since it isn't a sync'ed pair!
      std::map<int, int> camid_to_msg_index;
      double meas_time = msgs.at(m).time.seconds();
      for (int cam_idt = 0; cam_idt < params.state_options.num_cameras; cam_idt++) {
        if (cam_idt == cam_id) {
          camid_to_msg_index.insert({cam_id, m});
          continue;
        }
        int cam_idt_idx = -1;
        for (int mt = m; mt < static_cast<int>(msgs.size()); mt++) {
          if (msgs.at(mt).topic != topic_cameras.at(cam_idt))
            continue;
          if (std::abs(msgs.at(mt).time.seconds() - meas_time) < 0.02)
            cam_idt_idx = mt;
          break;
        }
        if (cam_idt_idx != -1) {
          camid_to_msg_index.insert({cam_idt, cam_idt_idx});
        }
      }

      // Skip processing if we were unable to find any messages
      if (static_cast<int>(camid_to_msg_index.size()) != params.state_options.num_cameras) {
        PRINT_DEBUG(YELLOW "[SERIAL]: Unable to find stereo pair for message %d at %.2f into bag (will skip!)\n" RESET, m,
                    meas_time - time_init.seconds());
        continue;
      }

      // Check if we should initialize using the groundtruth
      Eigen::Matrix<double, 17, 1> imustate;
      if (!gt_states.empty() && !sys->initialized() && ov_core::DatasetReader::get_gt_state(meas_time, imustate, gt_states)) {
        // biases are pretty bad normally, so zero them
        // imustate.block(11,0,6,1).setZero();
        sys->initialize_with_gt(imustate);
      }

      // Pass our data into our visualizer callbacks!
      if (params.state_options.num_cameras == 1) {
        const auto &bag_msg0 = msgs.at(camid_to_msg_index.at(0));
        const auto type_it0 = topic_to_type.find(bag_msg0.topic);
        const std::string type0 = (type_it0 == topic_to_type.end()) ? "" : type_it0->second;
        auto msg0 = decode_image_message(bag_msg0, type0, image_serializer, compressed_serializer);
        if (!msg0) {
          PRINT_ERROR(RED "[SERIAL]: Unable to decode image for %s (type: %s) at %.3f\n" RESET, bag_msg0.topic.c_str(),
                      type0.empty() ? "<unknown>" : type0.c_str(), bag_msg0.time.seconds());
          break;
        }
        viz->callback_monocular(msg0, 0);
      } else if (params.state_options.num_cameras == 2) {
        const auto &bag_msg0 = msgs.at(camid_to_msg_index.at(0));
        const auto &bag_msg1 = msgs.at(camid_to_msg_index.at(1));
        const auto type_it0 = topic_to_type.find(bag_msg0.topic);
        const auto type_it1 = topic_to_type.find(bag_msg1.topic);
        const std::string type0 = (type_it0 == topic_to_type.end()) ? "" : type_it0->second;
        const std::string type1 = (type_it1 == topic_to_type.end()) ? "" : type_it1->second;
        auto msg0 = decode_image_message(bag_msg0, type0, image_serializer, compressed_serializer);
        auto msg1 = decode_image_message(bag_msg1, type1, image_serializer, compressed_serializer);
        if (!msg0 || !msg1) {
          PRINT_ERROR(RED "[SERIAL]: Unable to decode stereo images at %.3f\n" RESET, bag_msg0.time.seconds());
          break;
        }
        used_index.insert(camid_to_msg_index.at(0)); // skip this message
        used_index.insert(camid_to_msg_index.at(1)); // skip this message
        viz->callback_stereo(msg0, msg1, 0, 1);
      } else {
        PRINT_ERROR(RED "[SERIAL]: We currently only support 1 or 2 camera serial input....\n" RESET);
        rclcpp::shutdown();
        return EXIT_FAILURE;
      }

      break;
    }
  }

  // Final visualization
  viz->visualize_final();

  // Done!
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
