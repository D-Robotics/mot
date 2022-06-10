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
