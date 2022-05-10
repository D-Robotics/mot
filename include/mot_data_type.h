// Copyright (c) 2021 Horizon Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of Horizon Robotics Inc. This is proprietary information owned by
// Horizon Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of Horizon Robotics Inc.

#ifndef MOTMETHOD_DATA_TYPE_MOT_DATA_TYPE_H_
#define MOTMETHOD_DATA_TYPE_MOT_DATA_TYPE_H_

#include <memory>
#include <string>
#include <vector>

namespace hobot_mot {

/// Data State
enum class DataState {
  /// valid
  VALID = 0,
  /// filtered
  FILTERED = 1,
  /// invisible
  INVISIBLE = 2,
  /// disappeared
  DISAPPEARED = 3,
  /// invalid
  INVALID = 4,
};

struct id_s {
  std::string type_ = "Number";
  int value = -1;
  DataState state_ = DataState::VALID;
};

struct box_s {
  box_s() {}
  box_s(int x1_, int y1_, int x2_, int y2_) {
    x1 = x1_;
    y1 = y1_;
    x2 = x2_;
    y2 = y2_;
  }
  box_s(int x1_, int y1_, int x2_, int y2_, float score_) {
    x1 = x1_;
    y1 = y1_;
    x2 = x2_;
    y2 = y2_;
    score = score_;
  }
  int x1 = 0;
  int y1 = 0;
  int x2 = 0;
  int y2 = 0;
  float score = 1;
  int id = 0;
  DataState state_ = DataState::VALID;
};

using MotBox = box_s;
using MotTrackId = id_s;

}  // namespace hobot_mot

#endif  // MOTMETHOD_DATA_TYPE_MOT_DATA_TYPE_H_
