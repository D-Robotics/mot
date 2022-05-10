// Copyright (c) 2021 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

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
