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

#include "MOTMethod/MOT/ReID.h"

#include <fstream>
#include <list>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "MOTMethod/data_type/mot_data_type.hpp"
#include "MOTMethod/error_code.h"

namespace xstream {

using XStreamUint32 = XStreamData<uint32_t>;
using XStreamBBox = XStreamData<hobot::vision::BBox>;
using ImageFramePtr = std::shared_ptr<hobot::vision::ImageFrame>;
using XStreamImageFrame = XStreamData<ImageFramePtr>;

int ReID::MotInit(const std::string &config_file_path) {
  config_param_ = std::make_shared<REIDParam>();
  std::ifstream config_if(config_file_path);
  if (!config_if.good()) {
    LOGI << "IOU2Param: no config, using default parameters" << std::endl;
  } else {
    std::ostringstream buf;
    char ch;
    while (buf && config_if.get(ch)) {
      buf.put(ch);
    }
    config_param_->UpdateParameter(buf.str());
  }

  auto config = GetConfig();
  LOGW << "reid method file:" << config_file_path
       << " track config file:" << config->config_file;
  mul_obj_tracker_ = std::make_shared<hobot::feat_based_mot::MulObjTracker>();
  hobot::feat_based_mot::Config::instance().init(config->config_file);
  return 0;
}

int ReID::Track(const std::vector<BaseDataPtr> &in,
                std::vector<BaseDataPtr> &out) {
  std::lock_guard<std::mutex> lock(reid_mutex_);
  HOBOT_CHECK(in.size() >= 5);
  auto img_frame = std::static_pointer_cast<BaseData>(in[0]);
  auto box_list = std::static_pointer_cast<BaseDataVector>(in[1]);
  auto kps_list = std::static_pointer_cast<BaseDataVector>(in[2]);
  auto reid_list = std::static_pointer_cast<BaseDataVector>(in[3]);
  auto disappeared_id_list = std::static_pointer_cast<BaseDataVector>(in[4]);

  HOBOT_CHECK(img_frame) << "Lost image frame";
  HOBOT_CHECK(box_list) << "Lost face_head_body boxes";
  HOBOT_CHECK(kps_list) << "Lost kps";
  HOBOT_CHECK(reid_list) << "Lost reid";
  HOBOT_CHECK("ImageFrame" == img_frame->type_);
  HOBOT_CHECK("BaseDataVector" == box_list->type_);
  HOBOT_CHECK("BaseDataVector" == kps_list->type_);
  HOBOT_CHECK("BaseDataVector" == reid_list->type_);
  HOBOT_CHECK("BaseDataVector" == disappeared_id_list->type_);

  auto pframe = std::static_pointer_cast<XStreamImageFrame>(img_frame);
  time_t time_stamp = pframe->value->time_stamp;
  uint32_t frame_width = pframe->value->Width();
  uint32_t frame_height = pframe->value->Height();

  LOGD << time_stamp << " " << frame_width << " " << frame_height;
  LOGD << box_list->datas_.size();
  LOGD << kps_list->datas_.size();
  LOGD << reid_list->datas_.size();

  auto out_rects = std::make_shared<BaseDataVector>();
  auto out_disappeared_idxs = std::make_shared<BaseDataVector>();

  out.push_back(std::static_pointer_cast<BaseData>(out_rects));
  out.push_back(std::static_pointer_cast<BaseData>(out_disappeared_idxs));
  CopyBoxToOutput(
      box_list.get(), &out_rects->datas_, frame_width, frame_height);
  // convert data to call mot strategy
  if (box_list->datas_.empty() || kps_list->datas_.empty() ||
      reid_list->datas_.empty()) {
    LOGW << "mot reid recv body box, kps, reid list data is empty";
    return 0;
  }

  std::vector<hobot::feat_based_mot::spBBox> boxes;
  std::vector<hobot::feat_based_mot::spSkeleton> skeleton_list;
  std::vector<hobot::feat_based_mot::ReidFeatures> reid_feature_list;

  // convert data to call mot strategy
  ConvertData(box_list.get(),
              kps_list.get(),
              reid_list.get(),
              frame_width,
              frame_height,
              &boxes,
              &skeleton_list,
              &reid_feature_list);
  if (boxes.empty()) {
    LOGI << "mot reid convert data boxes is empty";
    return -1;
  }
  // 与上一帧出现的target id比较
  DisappearedToOutput(disappeared_id_list.get(), &out_disappeared_idxs->datas_);

  // call mot strategy
  int ret = mul_obj_tracker_->TrackPro(boxes,
                                       skeleton_list,
                                       reid_feature_list,
                                       static_cast<int64>(time_stamp),
                                       frame_width,
                                       frame_height,
                                       pframe->value->frame_id);
  if (ret) {
    LOGE << "call reid TrackPro fail";
    return -1;
  }

  // get the output from mot strategy
  std::map<int, int> track_id2det_id_ = mul_obj_tracker_->track_id2det_id();
  bool ready_to_output = mul_obj_tracker_->is_ready_to_output();
  if (ready_to_output) {
    TrackToOutput(time_stamp,
                  mul_obj_tracker_->tracklet_list(),
                  &out_rects->datas_,
                  &out_disappeared_idxs->datas_);
  } else {
    LOGW << "mot reid is no ready to output";
  }

#if 0
  out_fps_++;
  std::chrono::duration<double, std::milli> interval_ms =
      std::chrono::high_resolution_clock::now() - out_start_tp_;
  if (interval_ms.count() >= 1000) {
    LOGE << "mot reid out fps " << out_fps_;
    out_fps_ = 0;
    out_start_tp_ = std::chrono::high_resolution_clock::now();
  }
#endif
  return 0;
}

void ReID::ConvertData(
    const BaseDataVector *body_list,
    const BaseDataVector *kps_list,
    const BaseDataVector *feature_list,
    const int &img_width,
    const int &img_height,
    std::vector<hobot::feat_based_mot::spBBox> *boxes,
    std::vector<hobot::feat_based_mot::spSkeleton> *skeletons,
    std::vector<hobot::feat_based_mot::ReidFeatures> *reid_feature_list) {
  size_t size = body_list->datas_.size();
  for (size_t i = 0; i < size; i++) {
    auto body_box =
        std::static_pointer_cast<xstream::XStreamData<hobot::vision::BBox>>(
            body_list->datas_[i]);
    if (body_box->value.id == -1 || DataState::VALID != body_box->state_) {
      continue;
    }

    if (!feature_list->datas_[i]) continue;
    auto body_feature =
        std::static_pointer_cast<xstream::XStreamData<hobot::vision::Feature>>(
            feature_list->datas_[i]);
    if (body_feature->value.values.empty()) continue;

    // {{ body box
    hobot::feat_based_mot::spBBox iou_bbox =
        std::make_shared<hobot::feat_based_mot::BBox>(
            std::max(0, static_cast<int>(std::lround(body_box->value.x1))),
            std::max(0, static_cast<int>(std::lround(body_box->value.y1))),
            std::min(img_width - 1,
                     static_cast<int>(std::lround(body_box->value.x2))),
            std::min(img_height - 1,
                     static_cast<int>(std::lround(body_box->value.y2))),
            body_box->value.id,
            body_box->value.score);

    // {{ kps
    auto body_kps = std::static_pointer_cast<
        xstream::XStreamData<hobot::vision::Landmarks>>(kps_list->datas_[i]);
    // todo body_kps->value.values point num is 19, old is 17
    int point_num = body_kps->value.values.size() - 2;
    hobot::feat_based_mot::HumanSkeleton human_skeleton;
    human_skeleton.points.resize(point_num);
    human_skeleton.scores.resize(point_num);
    human_skeleton.point_num = static_cast<int>(point_num);
    for (int i = 0; i < point_num; i++) {
      human_skeleton.points[i].x = body_kps->value.values[i].x;
      human_skeleton.points[i].y = body_kps->value.values[i].y;
      human_skeleton.scores[i] = body_kps->value.values[i].score;
    }
    hobot::feat_based_mot::spSkeleton sp_kps =
        std::make_shared<hobot::feat_based_mot::Skeleton>(human_skeleton);
    // }}

    // {{ feature
    hobot::feat_based_mot::ReidFeatures reid_feature;
    reid_feature.resize(body_feature->value.values.size());
    for (int i = 0; i < static_cast<int>(body_feature->value.values.size());
         i++) {
      reid_feature[i] = body_feature->value.values[i];
    }
    // }}

    boxes->emplace_back(iou_bbox);
    skeletons->emplace_back(sp_kps);
    reid_feature_list->emplace_back(reid_feature);
  }
}

void ReID::CopyBoxToOutput(BaseDataVector *rects_msg,
                           std::vector<BaseDataPtr> *p_out_rects,
                           const int &img_width,
                           const int &img_height) {
  size_t size = rects_msg->datas_.size();
  for (size_t i = 0; i < size; i++) {
    auto &in_rect = rects_msg->datas_[i];
    auto inbox = std::static_pointer_cast<XStreamBBox>(in_rect);
    std::shared_ptr<XStreamBBox> outbox = std::make_shared<XStreamBBox>();
    outbox->type_ = "BBox";
    outbox->value.id = inbox->value.id;
    outbox->value.x1 =
        std::max(0, static_cast<int>(std::lround(inbox->value.x1)));
    outbox->value.y1 =
        std::max(0, static_cast<int>(std::lround(inbox->value.y1)));
    outbox->value.x2 =
        std::min(img_width - 1, static_cast<int>(std::lround(inbox->value.x2)));
    outbox->value.y2 = std::min(img_height - 1,
                                static_cast<int>(std::lround(inbox->value.y2)));
    outbox->value.score = inbox->value.score;
    outbox->state_ = inbox->state_;
    p_out_rects->push_back(outbox);
  }
}

void ReID::TrackToOutput(
    const time_t &time_stamp,
    const std::vector<hobot::feat_based_mot::spTracklet> &tracklet_list,
    std::vector<BaseDataPtr> *p_out_rects,
    std::vector<BaseDataPtr> *p_disappeared_ids) {
  std::map<int, int> id_track_list;
  for (const auto &tracklet : tracklet_list) {
    if (tracklet->track_state() == hobot::feat_based_mot::TrackState::Deleted ||
        tracklet->track_state() ==
            hobot::feat_based_mot::TrackState::OutOfDate) {
      std::shared_ptr<XStreamUint32> track_id =
          std::make_shared<XStreamUint32>();
      track_id->type_ = "Number";
      track_id->value = tracklet->track_id();
      track_id->state_ = DataState::DISAPPEARED;
      p_disappeared_ids->push_back(track_id);
    } else {
      hobot::feat_based_mot::spTarget target_out =
          tracklet->GetTargetOf(time_stamp);
      if (target_out) {
        int box_id = target_out->body_bbox()->id;
        if (box_id >= 0) {
          id_track_list[box_id] = tracklet->track_id();
        }
      }
    }  // end of else
  }

  for (size_t i = 0; i < p_out_rects->size(); ++i) {
    auto body_box = std::static_pointer_cast<XStreamBBox>((*p_out_rects)[i]);
    int old_id = body_box->value.id;
    if (id_track_list.find(old_id) != id_track_list.end()) {
      body_box->value.id = id_track_list[old_id];
      body_box->state_ = DataState::VALID;
      LOGI << "mot reid body box id:" << old_id
           << " change to:" << body_box->value.id;
    } else {
      if (body_box->state_ != DataState::INVALID) {
        LOGI << "mot reid body box id:" << old_id
             << " find track to output fail";
        //  body_box->state_ = DataState::INVALID;  // TODO(xue.liang)
      }
    }
  }
}

void ReID::DisappearedToOutput(BaseDataVector *p_disappeared_ids_in,
                               std::vector<BaseDataPtr> *p_disappeared_ids) {
  size_t size = p_disappeared_ids_in->datas_.size();
  for (size_t idx = 0; idx < size; idx++) {
    auto disappeared_id = std::static_pointer_cast<XStreamUint32>(
        p_disappeared_ids_in->datas_.at(idx));
    hobot::feat_based_mot::TrackState state;
    bool find =
        mul_obj_tracker_->GetTrackletState(disappeared_id->value, &state);
    if (!find) {
      std::shared_ptr<XStreamUint32> track_id =
          std::make_shared<XStreamUint32>();
      track_id->type_ = "Number";
      track_id->value = disappeared_id->value;
      track_id->state_ = DataState::DISAPPEARED;
      p_disappeared_ids->push_back(track_id);
      LOGE << "reid disappeared id:" << track_id->value;
      continue;
    }
    if (state == hobot::feat_based_mot::TrackState::Deleted ||
        state == hobot::feat_based_mot::TrackState::OutOfDate) {
      std::shared_ptr<XStreamUint32> track_id =
          std::make_shared<XStreamUint32>();
      track_id->type_ = "Number";
      track_id->value = disappeared_id->value;
      track_id->state_ = DataState::DISAPPEARED;
      p_disappeared_ids->push_back(track_id);
      LOGE << "reid disappeared id:" << track_id->value;
    }
  }
}

std::shared_ptr<REIDParam> ReID::GetConfig() {
  auto select_config = std::static_pointer_cast<REIDParam>(config_param_);
  return select_config;
}

int ReID::UpdateParameter(const std::string &content) {
  return config_param_->UpdateParameter(content);
}

}  // namespace xstream
