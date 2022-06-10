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

#ifndef MOTMETHOD_DATA_TYPE_MOT_DATA_TYPE_HPP_
#define MOTMETHOD_DATA_TYPE_MOT_DATA_TYPE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "include/error_code.h"
#include "include/mot_data_type.h"
#include "json/json.h"

using hobot_mot::DataState;
using XStreamBBox = hobot_mot::box_s;
using XStreamUint32 = hobot_mot::id_s;

namespace xstream {

#define SET_MOT_METHOD_PARAM(json_cfg, type, key)           \
  if (json_cfg.isMember(#key) && json_cfg[#key].is##type()) \
  key = json_cfg[#key].as##type()

class MOTParam {
 public:
  explicit MOTParam(const std::string &content) {
    if (!content.empty()) {
      UpdateParameter(content);
    }
  }

  virtual int UpdateParameter(const std::string &content) {
    Json::CharReaderBuilder builder;
    builder["collectComments"] = false;
    JSONCPP_STRING error;
    std::shared_ptr<Json::CharReader> json_reader(builder.newCharReader());
    try {
      bool ret = json_reader->parse(content.c_str(),
                                    content.c_str() + content.size(),
                                    &config_jv,
                                    &error);
      SET_MOT_METHOD_PARAM(config_jv, String, tracker_type);
      if (ret) {
        return XSTREAM_MOT_OK;
      } else {
        return XSTREAM_MOT_ERR_PARAM;
      }
    } catch (std::exception &e) {
      return XSTREAM_MOT_ERR_PARAM;
    }
  }

 protected:
  std::string tracker_type = "";
  Json::Value config_jv;
};

struct IOUParam : MOTParam {
 public:
  explicit IOUParam(const std::string &content = "") : MOTParam(content) {
    UpdateParameter(content);
  }

  int UpdateParameter(const std::string &content) {
    int ret = MOTParam::UpdateParameter(content);
    if (ret != XSTREAM_MOT_OK) return ret;
    Json::CharReaderBuilder builder;
    builder["collectComments"] = false;
    JSONCPP_STRING error;
    std::shared_ptr<Json::CharReader> json_reader(builder.newCharReader());
    try {
      ret = json_reader->parse(content.c_str(),
                               content.c_str() + content.size(),
                               &config_jv,
                               &error);
      SET_MOT_METHOD_PARAM(config_jv, Bool, update_no_target_predict);
      SET_MOT_METHOD_PARAM(config_jv, Bool, support_hungarian);
      SET_MOT_METHOD_PARAM(config_jv, Bool, need_check_merge);
      SET_MOT_METHOD_PARAM(config_jv, String, device);
      SET_MOT_METHOD_PARAM(config_jv, Bool, original_bbox);
      SET_MOT_METHOD_PARAM(config_jv, UInt, max_track_target_num);
      SET_MOT_METHOD_PARAM(config_jv, UInt, max_det_target_num);
      SET_MOT_METHOD_PARAM(config_jv, UInt, vanish_frame_count);
      SET_MOT_METHOD_PARAM(config_jv, UInt, time_gap);
      if (ret) {
        return XSTREAM_MOT_OK;
      } else {
        return XSTREAM_MOT_ERR_PARAM;
      }
    } catch (std::exception &e) {
      return XSTREAM_MOT_ERR_PARAM;
    }
  }

  bool update_no_target_predict = false;
  bool support_hungarian = false;
  bool need_check_merge = false;
  bool original_bbox = true;
  std::string device = "X2";
  uint32_t max_track_target_num = 256;
  uint32_t max_det_target_num = 256;
  uint32_t vanish_frame_count = 30;
  uint32_t time_gap = 40;
};

#ifdef ENABLE_IOU2
struct IOU2Param : MOTParam {
 public:
  explicit IOU2Param(const std::string &content = "") : MOTParam(content) {
    if (!content.empty()) {
      UpdateParameter(content);
    }
  }

  int UpdateParameter(const std::string &content) {
    int ret = MOTParam::UpdateParameter(content);
    if (ret != XSTREAM_MOT_OK) {
      return ret;
    }
    Json::CharReaderBuilder builder;
    builder["collectComments"] = false;
    JSONCPP_STRING error;
    std::shared_ptr<Json::CharReader> json_reader(builder.newCharReader());
    try {
      ret = json_reader->parse(content.c_str(),
                               content.c_str() + content.size(),
                               &config_jv,
                               &error);
      SET_MOT_METHOD_PARAM(config_jv, String, match_type);
      SET_MOT_METHOD_PARAM(config_jv, Bool, use_kalman_filter);
      SET_MOT_METHOD_PARAM(config_jv, Int, missing_time_thres);
      SET_MOT_METHOD_PARAM(config_jv, UInt, vanish_frame_count);
      SET_MOT_METHOD_PARAM(config_jv, UInt, time_gap);
      SET_MOT_METHOD_PARAM(config_jv, Int, remove_obsolete_track);
      SET_MOT_METHOD_PARAM(config_jv, Double, iou_thres);
      SET_MOT_METHOD_PARAM(config_jv, Double, euclidean_thres);
      SET_MOT_METHOD_PARAM(config_jv, Int, use_location_gain);
      SET_MOT_METHOD_PARAM(config_jv, Int, max_trajectory_number);

      SET_MOT_METHOD_PARAM(config_jv, Double, min_score);
      SET_MOT_METHOD_PARAM(config_jv, Double, ignore_overlap_thres);
      if (ret) {
        return XSTREAM_MOT_OK;
      } else {
        return XSTREAM_MOT_ERR_PARAM;
      }
    } catch (std::exception &e) {
      return XSTREAM_MOT_ERR_PARAM;
    }
  }

  std::string match_type = "Euclidean";
  bool use_kalman_filter = false;
  int missing_time_thres = 2;
  uint32_t vanish_frame_count = 50;
  uint32_t time_gap = 40;
  int remove_obsolete_track = 0;
  float iou_thres = 0.2f;
  float euclidean_thres = 200.0f;
  int use_location_gain = 1;
  int max_trajectory_number = 3;

  float min_score = 0.9f;
  float ignore_overlap_thres = 0.9f;
};
#endif  // ENABLE_IOU2

struct REIDParam : MOTParam {
 public:
  explicit REIDParam(const std::string &content = "") : MOTParam(content) {
    UpdateParameter(content);
  }

  int UpdateParameter(const std::string &content) {
    int ret = MOTParam::UpdateParameter(content);
    if (ret != XSTREAM_MOT_OK) return ret;
    Json::CharReaderBuilder builder;
    builder["collectComments"] = false;
    JSONCPP_STRING error;
    std::shared_ptr<Json::CharReader> json_reader(builder.newCharReader());
    try {
      ret = json_reader->parse(content.c_str(),
                               content.c_str() + content.size(),
                               &config_jv,
                               &error);
      SET_MOT_METHOD_PARAM(config_jv, String, match_type);
      SET_MOT_METHOD_PARAM(config_jv, Bool, use_kalman_filter);
      SET_MOT_METHOD_PARAM(config_jv, Int, missing_time_thres);
      SET_MOT_METHOD_PARAM(config_jv, UInt, vanish_frame_count);
      SET_MOT_METHOD_PARAM(config_jv, UInt, time_gap);
      SET_MOT_METHOD_PARAM(config_jv, Int, remove_obsolete_track);
      SET_MOT_METHOD_PARAM(config_jv, Double, iou_thres);
      SET_MOT_METHOD_PARAM(config_jv, Double, euclidean_thres);
      SET_MOT_METHOD_PARAM(config_jv, Int, use_location_gain);
      SET_MOT_METHOD_PARAM(config_jv, Int, max_trajectory_number);

      SET_MOT_METHOD_PARAM(config_jv, Double, min_score);
      SET_MOT_METHOD_PARAM(config_jv, Double, ignore_overlap_thres);
      SET_MOT_METHOD_PARAM(config_jv, String, config_file);
      if (ret) {
        return XSTREAM_MOT_OK;
      } else {
        return XSTREAM_MOT_ERR_PARAM;
      }
    } catch (std::exception &e) {
      return XSTREAM_MOT_ERR_PARAM;
    }
  }

  std::string match_type = "Euclidean";
  bool use_kalman_filter = false;
  int missing_time_thres = 2;
  uint32_t vanish_frame_count = 50;
  uint32_t time_gap = 40;
  int remove_obsolete_track = 0;
  float iou_thres = 0.2f;
  float euclidean_thres = 200.0f;
  int use_location_gain = 1;
  int max_trajectory_number = 3;

  float min_score = 0.9f;
  float ignore_overlap_thres = 0.9f;
  std::string config_file = "track_param.json";
};

}  // namespace xstream

#endif  // MOTMETHOD_DATA_TYPE_MOT_DATA_TYPE_HPP_
