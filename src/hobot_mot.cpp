// Copyright (c) 2021 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#include "include/hobot_mot.h"

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "include/MOTMethod.h"

class HobotMotImpl {
 public:
  explicit HobotMotImpl(const std::string& config_file_path) {
    mot_method_ptr_ = std::make_shared<xstream::MOTMethod>();
    mot_method_ptr_->Init(config_file_path);
  }
  ~HobotMotImpl() {
    if (mot_method_ptr_) {
      mot_method_ptr_->Finalize();
    }
  }

  int DoProcess(
      const std::vector<XStreamBBox>* in_box_list,
      std::vector<XStreamBBox>& out_box_list,
      std::vector<std::shared_ptr<XStreamUint32>>& out_disappeared_ids,
      time_t time_stamp,
      uint32_t frame_width,
      uint32_t frame_height) {
    if (!mot_method_ptr_) {
      return -1;
    }
    std::unique_lock<std::mutex> lg(mtx_);
    return mot_method_ptr_->DoProcess(in_box_list,
                                      out_box_list,
                                      out_disappeared_ids,
                                      time_stamp,
                                      frame_width,
                                      frame_height);
  }

 private:
  std::shared_ptr<xstream::MOTMethod> mot_method_ptr_ = nullptr;
  std::mutex mtx_;
};

HobotMot::HobotMot(const std::string& config_file_path) {
  mot_impl_ptr_ = std::make_shared<HobotMotImpl>(config_file_path);
}

int HobotMot::DoProcess(
    const std::vector<XStreamBBox>& box_list,
    std::vector<XStreamBBox>& out_rects,
    std::vector<std::shared_ptr<XStreamUint32>>& out_disappeared_ids,
    const time_t& time_stamp,
    const uint32_t& frame_width,
    const uint32_t& frame_height) {
  if (!mot_impl_ptr_) {
    return -1;
  }
  return mot_impl_ptr_->DoProcess(&box_list,
                                  out_rects,
                                  out_disappeared_ids,
                                  time_stamp,
                                  frame_width,
                                  frame_height);
}

HobotMot::~HobotMot() {}
