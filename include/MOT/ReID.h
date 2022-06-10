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

#ifndef MOTMETHOD_MOT_REID_H_
#define MOTMETHOD_MOT_REID_H_

#include <list>
#include <memory>
#include <string>
#include <vector>

#include "MOTMethod/MOTMethod.h"
#include "MOTMethod/data_type/mot_data_type.hpp"
#include "feat_based_mot/tracker/mul_obj_tracker.h"
#include "horizon/vision_type/vision_type.hpp"

namespace xstream {

typedef std::shared_ptr<BaseDataVector> BaseDataVectorPtr;

class ReID : public Mot {
 public:
  int MotInit(const std::string &config_file_path) override;

  int Track(const std::vector<BaseDataPtr> &in,
            std::vector<BaseDataPtr> &out) override;

  void MotFinalize() override{};

 private:
  int UpdateParameter(const std::string &content) override;
  std::shared_ptr<REIDParam> GetConfig();
  void ConvertData(
      const BaseDataVector *body_list,
      const BaseDataVector *kps_list,
      const BaseDataVector *feature_list,
      const int &img_width,
      const int &img_height,
      std::vector<hobot::feat_based_mot::spBBox> *boxes,
      std::vector<hobot::feat_based_mot::spSkeleton> *skeletons,
      std::vector<hobot::feat_based_mot::ReidFeatures> *reid_feature_list);

  void CopyBoxToOutput(BaseDataVector *rects_msg,
                       std::vector<BaseDataPtr> *p_out_rects,
                       const int &img_width,
                       const int &img_height);

  void TrackToOutput(
      const time_t &time_stamp,
      const std::vector<hobot::feat_based_mot::spTracklet> &tracklet_list,
      std::vector<BaseDataPtr> *p_out_rects,
      std::vector<BaseDataPtr> *p_disappeared_ids);
  void DisappearedToOutput(BaseDataVector *p_disappeared_ids_in,
                           std::vector<BaseDataPtr> *p_disappeared_ids);

  std::shared_ptr<hobot::feat_based_mot::MulObjTracker> mul_obj_tracker_;
  int out_fps_ = 0;
  std::chrono::high_resolution_clock::time_point out_start_tp_;
  std::mutex reid_mutex_;
};

}  // namespace xstream

#endif  // MOTMETHOD_MOT_REID_H_
