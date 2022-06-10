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

#ifndef INCLUDE_MOTMETHOD_MOT_IOU2_H_
#define INCLUDE_MOTMETHOD_MOT_IOU2_H_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "include/MOTMethod.h"
#include "include/data_type/mot_data_type.hpp"
#include "include/error_code.h"
#include "iou_based_mot/target.h"
#include "iou_based_mot/tracker.h"

namespace xstream {

struct IOU2Param;

class IOU2 : public Mot {
 public:
  int MotInit(const std::string &config_file_path) override;

  int Track(const std::vector<XStreamBBox> *box_list,
            std::vector<XStreamBBox> &out_rects,
            std::vector<std::shared_ptr<XStreamUint32>> &out_disappeared_ids,
            time_t time_stamp,
            uint32_t frame_width,
            uint32_t frame_height) override;

  void MotFinalize() override;
  int UpdateParameter(const std::string &content) override;

 private:
  int SetTrackerConfig();

  void RectMsg2Box(std::vector<hobot::iou_mot::sp_BBox> *boxes,
                   const std::vector<XStreamBBox> *rects_msg,
                   const int &img_width,
                   const int &img_height);

  void track_to_rects(
      const time_t &time_stamp,
      const std::vector<hobot::iou_mot::sp_TrackLet> &tracklet_list,
      std::vector<XStreamBBox> *p_out_rects,
      std::vector<std::shared_ptr<XStreamUint32>> *p_disappeared_ids);

  void copy_inrects_to_out(const std::vector<XStreamBBox> *rects_msg,
                           std::vector<XStreamBBox> *p_out_rects,
                           const int &img_width,
                           const int &img_height);

  std::shared_ptr<IOU2Param> GetConfig();

 private:
  std::shared_ptr<hobot::iou_mot::Tracker> tracker_;
};
}  // namespace xstream

#endif  // INCLUDE_MOTMETHOD_MOT_IOU2_H_
