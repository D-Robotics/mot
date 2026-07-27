// Copyright (c) 2024, D-Robotics.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ai_msgs/msg/perception_targets.hpp"
#include "hobot_mot.h"
#include "mot_data_type.h"

using hobot_mot::DataState;
using hobot_mot::MotBox;
using hobot_mot::MotTrackId;

class TrosMotNode : public rclcpp::Node
{
public:
  explicit TrosMotNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("tros_mot_node", options)
  {
    // Declare parameters
    this->declare_parameter<std::string>("mot_config_path", "config/iou2_method_param.json");
    this->declare_parameter<std::string>("sub_topic", "/hobot_dnn_detection");
    this->declare_parameter<std::string>("pub_topic", "/tros_mot_targets");
    this->declare_parameter<int>("frame_width", 960);
    this->declare_parameter<int>("frame_height", 544);

    // Get parameters
    mot_config_path_ = this->get_parameter("mot_config_path").as_string();
    sub_topic_ = this->get_parameter("sub_topic").as_string();
    pub_topic_ = this->get_parameter("pub_topic").as_string();
    frame_width_ = static_cast<uint32_t>(this->get_parameter("frame_width").as_int());
    frame_height_ = static_cast<uint32_t>(this->get_parameter("frame_height").as_int());

    // Resolve config path via ament index
    resolved_config_path_ = mot_config_path_;
    auto pkg_share_dir = ament_index_cpp::get_package_share_directory("hobot_mot");
    if (!mot_config_path_.empty() && mot_config_path_[0] != '/') {
      resolved_config_path_ = pkg_share_dir + "/config/" + mot_config_path_;
      // Fallback: try direct path relative to share dir
      if (access(resolved_config_path_.c_str(), F_OK) != 0) {
        resolved_config_path_ = pkg_share_dir + "/" + mot_config_path_;
      }
    }

    RCLCPP_INFO(this->get_logger(),
      "TrosMotNode starting"
      "\n       sub_topic: %s"
      "\n       pub_topic: %s"
      "\n  mot_config_path: %s"
      "\n     frame_width: %u"
      "\n    frame_height: %u",
      sub_topic_.c_str(), pub_topic_.c_str(), resolved_config_path_.c_str(),
      frame_width_, frame_height_);

    // Initialize MOT instances (one per target type, matching mono2d_body_detection pattern)
    // A single shared instance would cause track_id_counter to increment across types,
    // and the tracker would mix tracks from different types in its internal tracklet_list.

    // Create subscription
    sub_ = this->create_subscription<ai_msgs::msg::PerceptionTargets>(
      sub_topic_, 10,
      std::bind(&TrosMotNode::targetsCallback, this, std::placeholders::_1));

    // Create publisher
    pub_ = this->create_publisher<ai_msgs::msg::PerceptionTargets>(pub_topic_, 10);

    RCLCPP_INFO(this->get_logger(), "TrosMotNode initialized.");
  }

private:
  void targetsCallback(const ai_msgs::msg::PerceptionTargets::SharedPtr msg)
  {
    // Group input targets by type
    std::unordered_map<std::string, std::vector<size_t>> type_to_indices;
    for (size_t i = 0; i < msg->targets.size(); ++i) {
      const auto & target = msg->targets[i];
      type_to_indices[target.type].push_back(i);
    }

    auto pub_msg = std::make_shared<ai_msgs::msg::PerceptionTargets>();
    pub_msg->header = msg->header;
    pub_msg->fps = msg->fps;

    uint64_t ts_ms = msg->header.stamp.sec * 1000ULL +
                     msg->header.stamp.nanosec / 1000000ULL;
    // Use milliseconds as the time_stamp; the tracker uses time_gap (ms) for frame intervals
    time_t time_stamp = static_cast<time_t>(ts_ms);

    // Process each target type through its own MOT instance
    for (auto & [type, indices] : type_to_indices) {
      // Get or create a dedicated HobotMot instance for this type
      if (mot_map_.find(type) == mot_map_.end()) {
        mot_map_[type] = std::make_shared<HobotMot>(resolved_config_path_);
        RCLCPP_INFO(this->get_logger(), "Created MOT instance for type: %s", type.c_str());
      }
      auto & mot = mot_map_[type];
      // Build input MotBox list
      std::vector<MotBox> in_boxes;
      in_boxes.reserve(indices.size());
      for (size_t idx : indices) {
        const auto & target = msg->targets[idx];
        MotBox box;
        if (!target.rois.empty()) {
          const auto & rect = target.rois[0].rect;
          box.x1 = rect.x_offset;
          box.y1 = rect.y_offset;
          box.x2 = rect.x_offset + rect.width;
          box.y2 = rect.y_offset + rect.height;
        }
        box.score = 1.0f;
        in_boxes.push_back(box);
      }

      // Run MOT
      std::vector<MotBox> out_boxes;
      std::vector<std::shared_ptr<MotTrackId>> disappeared_ids;

      int ret = mot->DoProcess(in_boxes, out_boxes, disappeared_ids,
                                time_stamp, frame_width_, frame_height_);
      if (ret < 0) {
        RCLCPP_ERROR(this->get_logger(), "MOT DoProcess failed for type: %s", type.c_str());
        continue;
      }

      // Map tracked boxes back to targets
      for (size_t i = 0; i < out_boxes.size() && i < indices.size(); ++i) {
        const auto & out_box = out_boxes[i];
        if (out_box.id < 0 || DataState::INVALID == out_box.state_) {
          continue;
        }
        const auto & orig_target = msg->targets[indices[i]];
        ai_msgs::msg::Target target;
        target.type = orig_target.type;
        target.set__track_id(out_box.id);
        // Use original input roi coordinates (MOT only updates track_id,
        // the tracker's internal bbox coordinates may be in normalized space
        // and should not overwrite the original pixel coordinates)
        if (!orig_target.rois.empty()) {
          target.rois.push_back(orig_target.rois[0]);
          for (size_t r = 1; r < orig_target.rois.size(); ++r) {
            target.rois.push_back(orig_target.rois[r]);
          }
        }
        // Preserve attributes, points, captures
        target.attributes = orig_target.attributes;
        target.points = orig_target.points;
        target.captures = orig_target.captures;
        pub_msg->targets.push_back(std::move(target));
      }

      // Map disappeared IDs
      for (const auto & id_info : disappeared_ids) {
        if (id_info->value < 0 || DataState::INVALID == id_info->state_) {
          continue;
        }
        ai_msgs::msg::Target target;
        target.set__type(type);
        target.set__track_id(id_info->value);
        pub_msg->disappeared_targets.push_back(std::move(target));
      }
    }

    // Preserve perf info
    pub_msg->perfs = msg->perfs;

    pub_->publish(*pub_msg);
  }

  std::unordered_map<std::string, std::shared_ptr<HobotMot>> mot_map_;
  std::string mot_config_path_;
  std::string resolved_config_path_;
  std::string sub_topic_;
  std::string pub_topic_;
  uint32_t frame_width_;
  uint32_t frame_height_;

  rclcpp::Subscription<ai_msgs::msg::PerceptionTargets>::SharedPtr sub_;
  rclcpp::Publisher<ai_msgs::msg::PerceptionTargets>::SharedPtr pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TrosMotNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
