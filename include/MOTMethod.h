/**
 * Copyright (c) 2018 Horizon Robotics. All rights reserved.
 * @brief     MOT Method
 * @author    chao.yang
 * @email     chao01.yang@horizon.ai
 * @version   0.0.0.1
 * @date      2018.12.15
 */

#ifndef XSTREAM_METHOD_ZOO_MOTMETHOD_INCLUDE_MOTMETHOD_MOTMETHOD_H_
#define XSTREAM_METHOD_ZOO_MOTMETHOD_INCLUDE_MOTMETHOD_MOTMETHOD_H_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "include/data_type/mot_data_type.hpp"

namespace xstream {

class MOTParam;

class Mot {
 public:
  virtual ~Mot() {}
  virtual int MotInit(const std::string &config_file_path) = 0;

  virtual void MotFinalize() = 0;

  virtual int Track(
      const std::vector<XStreamBBox> *box_list,
      std::vector<XStreamBBox> &out_rects,
      std::vector<std::shared_ptr<XStreamUint32>> &out_disappeared_ids,
      time_t time_stamp,
      uint32_t frame_width,
      uint32_t frame_height) = 0;

  virtual int UpdateParameter(const std::string &content) = 0;

 protected:
  std::shared_ptr<MOTParam> config_param_;
};

class MOTMethod {
 public:
  int Init(const std::string &config_file_path);

  int DoProcess(
      const std::vector<XStreamBBox> *in_box_list,
      std::vector<XStreamBBox> &out_box_list,
      std::vector<std::shared_ptr<XStreamUint32>> &out_disappeared_ids,
      time_t time_stamp,
      uint32_t frame_width = 960,
      uint32_t frame_height = 544);

  void Finalize();

 private:
#if 0
  int PassThrough(const std::vector<BaseDataPtr> &in,
                  std::vector<BaseDataPtr> &out);
#endif
  std::shared_ptr<Mot> mot_;  // = nullptr;
  std::string tracker_type_;
};
}  // namespace xstream

#endif  // XSTREAM_METHOD_ZOO_MOTMETHOD_INCLUDE_MOTMETHOD_MOTMETHOD_H_
