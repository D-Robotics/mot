// Copyright (c) 2024，D-Robotics.
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

#ifndef MOTMETHOD_MOT_IOU_H_
#define MOTMETHOD_MOT_IOU_H_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "MOTMethod/MOTMethod.h"
#include "MOTMethod/error_code.h"
#include "iou_tracking/tracking_data_types.h"

namespace xstream {

struct IOUParam;

class IOU : public Mot {
 public:
  ~IOU(){};
  int MotInit(const std::string &config_file_path) override;

  int Track(const std::vector<BaseDataPtr> &in,
            std::vector<BaseDataPtr> &out) override;

  void MotFinalize() override;

 private:
  typedef std::shared_ptr<BaseDataVector> BaseDataVectorPtr;

  int SetTrackingModuleData();

  int RectMsg2Box(box_s *box, BaseDataVector *rects_msg);

  int TrackData2Rect_KalmanBBox(tracking_module_data_t *tracking_data,
                                std::vector<uint32_t> *ids,
                                std::vector<BaseDataPtr> *p_out_rects);

  int TrackData2Rect_OriginalBBox(tracking_module_data_t *tracking_data,
                                  std::vector<uint32_t> *ids,
                                  const std::vector<BaseDataPtr> &in_rects,
                                  std::vector<BaseDataPtr> *p_out_rects);

  int UpdateState(const uint64_t &frame_id,
                  const std::vector<uint32_t> &ids,
                  BaseDataVectorPtr &disappeared_ids_msg);

  int CreateAbnormalDisappearIds(tracking_module_data_t *tracking_data,
                                 BaseDataVector *out_disappeared_ids);

  std::shared_ptr<IOUParam> GetConfig();

  int UpdateParameter(const std::string &content) override;

  void Reset() override;

  tracking_module_data_t *tracking_module_data_ = nullptr;

  uint64_t frame_count_ = 0;

  struct State {
    uint64_t start_ = 0;
    uint64_t last_ = 0;
    uint64_t count_ = 0;
  };

  typedef std::shared_ptr<State> StatePtr;
  std::map<uint32_t, StatePtr> mot_states_;
};
}  // namespace xstream

#endif  // MOTMETHOD_MOT_IOU_H_
