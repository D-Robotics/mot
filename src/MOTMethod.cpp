/**
 * Copyright (c) 2018 Horizon Robotics. All rights reserved.
 * @brief     MOT Method
 * @author    chao.yang
 * @email     chao01.yang@horizon.ai
 * @version   0.0.0.1
 * @date      2018.12.15
 */

#include "include/MOTMethod.h"

#include <chrono>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "hobotlog/hobotlog.hpp"
#include "include/error_code.h"
// #include "include/MOT/IOU.h"
#include "include/data_type/mot_data_type.hpp"
// #include "include/MOT/ReID.h"

#ifdef ENABLE_IOU2
#include "include/MOT/IOU2.h"
#endif

namespace xstream {

int MOTMethod::Init(const std::string& config_file_path) {
  SetLogLevel(HOBOT_LOG_WARN);

  LOGW << "MOTMethod::Init " << config_file_path << std::endl;
  std::ifstream config_if(config_file_path);
  if (!config_if.good()) {
    LOGW << "MOTParam: no config, using default IOU2 parameters" << std::endl;
    mot_ = std::make_shared<IOU2>();
    mot_->MotInit(config_file_path);
  } else {
    Json::Value config_jv;
    config_if >> config_jv;
    if (config_jv.isMember("tracker_type") &&
        config_jv["tracker_type"].isString())
      tracker_type_ = config_jv["tracker_type"].asString();
#ifdef ENABLE_IOU2
    LOGI << "tracker_type " << tracker_type_ << std::endl;
#endif
    if (tracker_type_ == "IOU") {
      // mot_ = std::make_shared<IOU>();
      // mot_->MotInit(config_file_path);
#ifdef ENABLE_IOU2
    } else if (tracker_type_ == "IOU_2.0") {
      mot_ = std::make_shared<IOU2>();
      mot_->MotInit(config_file_path);
#endif
    } else if (tracker_type_ == "ReID") {
      LOGW << "new ReID mot";
      // mot_ = std::make_shared<ReID>();
      // mot_->MotInit(config_file_path);
    } else {
      LOGE << "config param error";
      return XSTREAM_MOT_ERR_PARAM;
    }
  }
  return XSTREAM_MOT_OK;
}

int MOTMethod::DoProcess(
    const std::vector<XStreamBBox>* box_list,
    std::vector<XStreamBBox>& out_rects,
    std::vector<std::shared_ptr<XStreamUint32>>& out_disappeared_ids,
    time_t time_stamp,
    uint32_t frame_width,
    uint32_t frame_height) {
  LOGI << "MOTMethod::DoProcess" << std::endl;

  return mot_->Track(box_list,
                     out_rects,
                     out_disappeared_ids,
                     time_stamp,
                     frame_width,
                     frame_height);
}

void MOTMethod::Finalize() {
  mot_->MotFinalize();
  LOGI << "MOTMethod::Finalize" << std::endl;
}

#if 0
static int CopyBaseDataVector(BaseDataVector *in, BaseDataVector *out) {
  if (!in || !out) {
    return XSTREAM_MOT_ERR_PARAM;
  }
  out->datas_ = in->datas_;
  out->state_ = in->state_;
  out->type_ = in->type_;
  out->name_ = in->name_;
  out->c_data_ = in->c_data_;
  out->error_code_ = in->error_code_;
  out->error_detail_ = in->error_detail_;
  return XSTREAM_MOT_OK;
}

int MOTMethod::PassThrough(const std::vector<BaseDataPtr> &in,
                     std::vector<BaseDataPtr> &out) {
  assert(!in.empty());
  auto in_rects = std::static_pointer_cast<BaseDataVector>(in[1]);
  assert("BaseDataVector" == in_rects->type_);

  auto out_rects = std::make_shared<BaseDataVector>();
  int ret = CopyBaseDataVector(in_rects.get(), out_rects.get());
  if (XSTREAM_MOT_OK != ret) {
    return ret;
  }
  auto out_disappeared_ids = std::make_shared<BaseDataVector>();
  if (tracker_type_ == "ReID") {
    auto in_ids = std::static_pointer_cast<BaseDataVector>(in[4]);
    assert("BaseDataVector" == in_ids->type_);
    CopyBaseDataVector(in_ids.get(), out_disappeared_ids.get());
  }
  out.push_back(std::static_pointer_cast<BaseData>(out_rects));
  out.push_back(std::static_pointer_cast<BaseData>(out_disappeared_ids));

  return XSTREAM_MOT_OK;
}
#endif

}  // namespace xstream
