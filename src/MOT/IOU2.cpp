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

#include "include/MOT/IOU2.h"

#include <algorithm>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "include/data_type/mot_data_type.hpp"
#include "include/error_code.h"
// #include "horizon/vision_type/vision_type.hpp"
#include "hobotlog/hobotlog.hpp"
#include "json/json.h"

#define INVALID_ID -1

namespace xstream {

int IOU2::MotInit(const std::string &config_file_path) {
  LOGW << "IOU2 Mot::Init " << config_file_path << std::endl;
  this->tracker_ = std::make_shared<hobot::iou_mot::Tracker>(0);

  config_param_ = std::make_shared<IOU2Param>();
  std::ifstream config_if(config_file_path);
  if (!config_if.good()) {
    LOGW << "IOU2Param: no config, using default parameters" << std::endl;
  } else {
    std::ostringstream buf;
    char ch;
    while (buf && config_if.get(ch)) {
      buf.put(ch);
    }
    config_param_->UpdateParameter(buf.str());
  }
  return SetTrackerConfig();
}

int IOU2::SetTrackerConfig() {
  auto iou2_config = GetConfig();
  auto config = tracker_->GetParamConfig();
  config->track_param.match_type = iou2_config->match_type;
  config->track_param.use_kalman_filter = iou2_config->use_kalman_filter;
  config->track_param.missing_time_thres = iou2_config->missing_time_thres;
  config->track_param.remove_invisible_track_ms =
      iou2_config->vanish_frame_count * iou2_config->time_gap;
  config->track_param.remove_obsolete_track =
      iou2_config->remove_obsolete_track;
  config->track_param.iou_thres = iou2_config->iou_thres;
  config->track_param.euclidean_thres = iou2_config->euclidean_thres;
  config->track_param.use_location_gain = iou2_config->use_location_gain;
  config->track_param.max_trajectory_number =
      iou2_config->max_trajectory_number;

  config->detection_param.min_score = iou2_config->min_score;
  config->detection_param.ignore_overlap_thres =
      iou2_config->ignore_overlap_thres;
  return XSTREAM_MOT_OK;
}
int IOU2::Track(
    const std::vector<XStreamBBox> *box_list,
    std::vector<XStreamBBox> &out_rects,
    std::vector<std::shared_ptr<XStreamUint32>> &out_disappeared_ids,
    time_t time_stamp,
    uint32_t frame_width = 960,
    uint32_t frame_height = 544) {
  size_t item_size = box_list->size();
  LOGI << "data size:" << item_size;

  std::vector<hobot::iou_mot::sp_BBox> boxes;
  RectMsg2Box(&boxes, box_list, frame_width, frame_height);
  tracker_->TrackPro(boxes, time_stamp, frame_width, frame_height);

  /* transfer track targets */
  copy_inrects_to_out(box_list, &out_rects, frame_width, frame_height);
  track_to_rects(
      time_stamp, tracker_->tracklet_list(), &out_rects, &out_disappeared_ids);
  for (const auto &in_rect : *box_list) {
    LOGD << "in_rect.id: " << in_rect.id << ", rect: " << in_rect.x1 << " "
         << in_rect.y1 << " " << in_rect.x2 << " " << in_rect.y2;
  }

  LOGD << "out_rects.size(): " << out_rects.size()
       << ", out_disappeared_ids size: " << out_disappeared_ids.size();
  for (const auto &out_rect : out_rects) {
    LOGD << "out_rect.id: " << out_rect.id << ", rect: " << out_rect.x1 << " "
         << out_rect.y1 << " " << out_rect.x2 << " " << out_rect.y2;
  }

  return XSTREAM_MOT_OK;
}

void IOU2::RectMsg2Box(std::vector<hobot::iou_mot::sp_BBox> *boxes,
                       const std::vector<XStreamBBox> *rects_msg,
                       const int &img_width,
                       const int &img_height) {
  size_t size = rects_msg->size();

  for (size_t i = 0; i < size; i++) {
    auto in_rect = &(*rects_msg)[i];
    // HOBOT_CHECK("BBox" == in_rect->type_);
    if (DataState::VALID != in_rect->state_) continue;
    auto &bbox = in_rect;
    hobot::iou_mot::sp_BBox iou_bbox = std::make_shared<hobot::iou_mot::BBox_s>(
        std::max(0, static_cast<int>(std::lround(bbox->x1))),
        std::max(0, static_cast<int>(std::lround(bbox->y1))),
        std::min(img_width - 1, static_cast<int>(std::lround(bbox->x2))),
        std::min(img_height - 1, static_cast<int>(std::lround(bbox->y2))));
    iou_bbox->score = bbox->score;
    iou_bbox->box_id = i;
    boxes->emplace_back(iou_bbox);
  }
}

void IOU2::copy_inrects_to_out(const std::vector<XStreamBBox> *rects_msg,
                               std::vector<XStreamBBox> *p_out_rects,
                               const int &img_width,
                               const int &img_height) {
  size_t size = rects_msg->size();

  for (size_t i = 0; i < size; i++) {
    auto &inbox = (*rects_msg)[i];
    XStreamBBox outbox;
    outbox.id = -1;
    outbox.x1 = std::max(0, static_cast<int>(std::lround(inbox.x1)));
    outbox.y1 = std::max(0, static_cast<int>(std::lround(inbox.y1)));
    outbox.x2 =
        std::min(img_width - 1, static_cast<int>(std::lround(inbox.x2)));
    outbox.y2 =
        std::min(img_height - 1, static_cast<int>(std::lround(inbox.y2)));
    outbox.score = inbox.score;
    outbox.state_ = DataState::INVALID;
    p_out_rects->push_back(outbox);
  }
}

void IOU2::track_to_rects(
    const time_t &time_stamp,
    const std::vector<hobot::iou_mot::sp_TrackLet> &tracklet_list,
    std::vector<XStreamBBox> *p_out_rects,
    std::vector<std::shared_ptr<XStreamUint32>> *p_disappeared_ids) {
  for (const auto &tracklet : tracklet_list) {
    if (tracklet->state == hobot::iou_mot::TrackLet::Deleted) {
      std::shared_ptr<XStreamUint32> track_id(new XStreamUint32());
      track_id->type_ = "Number";
      track_id->value = tracklet->track_id;
      track_id->state_ = DataState::DISAPPEARED;
      p_disappeared_ids->push_back(track_id);
    } else if (p_out_rects->size()) {
      bool flag = false;
      hobot::iou_mot::sp_Target target_out =
          tracklet->GetTargetOfTimeStamp(time_stamp, flag);
      if (target_out) {
        int box_id = target_out->body_bbox->box_id;
        if (flag && box_id >= 0 &&
            static_cast<uint32_t>(box_id) < p_out_rects->size()) {
          XStreamBBox *bbox = &(*p_out_rects)[box_id];
          bbox->x1 = target_out->body_bbox->x1;
          bbox->x2 = target_out->body_bbox->x2;
          bbox->y1 = target_out->body_bbox->y1;
          bbox->y2 = target_out->body_bbox->y2;
          bbox->id = tracklet->track_id;
          bbox->state_ = DataState::VALID;
        }
      }
      // if (flag) {
      //   int box_id = target_out->body_bbox->box_id;
      //   auto bbox = std::static_pointer_cast<XStreamBBox>(
      //                               (*p_out_rects)[box_id]);
      //   bbox->value.id = tracklet->track_id;
      //   bbox->state_ = DataState::VALID;

      //   HOBOT_CHECK_EQ(bbox->type_, "BBox");
      //   HOBOT_CHECK_EQ(bbox->value.x1, target_out->body_bbox->x1);
      //   HOBOT_CHECK_EQ(bbox->value.y1, target_out->body_bbox->y1);
      //   HOBOT_CHECK_EQ(bbox->value.x2, target_out->body_bbox->x2);
      //   HOBOT_CHECK_EQ(bbox->value.y2, target_out->body_bbox->y2);
      //   HOBOT_CHECK_EQ(bbox->value.score, target_out->body_bbox->score);
      // }
    }
  }
}

std::shared_ptr<IOU2Param> IOU2::GetConfig() {
  auto select_config = std::static_pointer_cast<IOU2Param>(config_param_);
  return select_config;
}

void IOU2::MotFinalize() {}

int IOU2::UpdateParameter(const std::string &content) {
  int ret = config_param_->UpdateParameter(content);
  if (XSTREAM_MOT_OK == ret) {
    ret = SetTrackerConfig();
  }
  return ret;
}
}  // namespace xstream
