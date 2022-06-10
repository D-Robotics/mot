// Copyright (c) 2022，Horizon Robotics.
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

#ifndef ROBOT_HOBOT_MOT_H_
#define ROBOT_HOBOT_MOT_H_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "mot_data_type.h"  // NOLINT

using hobot_mot::MotBox;
using hobot_mot::MotTrackId;

class HobotMotImpl;

class HobotMot {
 public:
  explicit HobotMot(const std::string& config_file_path);
  ~HobotMot();

  // 处理每一帧的检测框结果，输出跟踪后的检测框（分配ID）和消失的ID列表
  // 需要保证输入数据的时序
  // - 参数
  //   - [in] in_box_list 输入检测框列表
  //   - [in&out] out_box_list 输出跟踪后的检测框列表
  //   - [in&out] out_disappeared_ids 输出的消失ID列表
  //   - [in] time_stamp 数据对应的时间戳
  //   - [in] frame_width 检测框对应图片的宽度
  //   - [in] frame_height 检测框对应图片的高度
  int DoProcess(const std::vector<MotBox>& in_box_list,
                std::vector<MotBox>& out_box_list,
                std::vector<std::shared_ptr<MotTrackId>>& out_disappeared_ids,
                const time_t& time_stamp,
                const uint32_t& frame_width = 960,
                const uint32_t& frame_height = 544);

 private:
  std::shared_ptr<HobotMotImpl> mot_impl_ptr_ = nullptr;
};

#endif  // ROBOT_HOBOT_MOT_H_
