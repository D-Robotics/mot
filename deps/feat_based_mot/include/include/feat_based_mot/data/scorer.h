//
// Created by xuhang.zhuang on 2019-05-20.
// Copyright (c) 2024 D-Robotics. All rights reserved.
//

#ifndef FEAT_BASED_MOT_DATA_SCORER_H_
#define FEAT_BASED_MOT_DATA_SCORER_H_

#include <string>
#include <utility>

#include "feat_based_mot/data/target.h"
#include "feat_based_mot/data/tracklet.h"
#include "feat_based_mot/data/types.h"

namespace hobot {
namespace feat_based_mot {

typedef std::pair<score_t, std::string> score_ret_t;

/**
 * @brief The base class of the Scorer.
 * The scorer contains all the strategy of score computation:
 *   - Detection vs Track distance
 *   - Track vs Track distance
 */
class Scorer {
 public:
  virtual score_ret_t GetDet2TraScore(const spTarget& target,
                                      const spTracklet& track) = 0;

  virtual score_t GetTra2TraScore(const spTracklet& track1,
                                  const spTracklet& track2) = 0;

  inline void SetCurrTime(frame_id_t frame_id, time_t time_stamp) {
    frame_counter_ = frame_id;
    curr_time_stamp_ = time_stamp;
  }

  inline void SetImgSize(int img_width, int img_height) {
    img_width_ = img_width;
    img_height_ = img_height;
  }

 protected:
  frame_id_t frame_counter_ = -1;
  time_t curr_time_stamp_ = -1;
  int img_width_ = 1920;
  int img_height_ = 1080;

  bool has_calc_fps_ = false;
  float frame_interval_ = 40.0;
};

/**
 * @brief A implementation of scorer dealing with PersonTarget
 * and PersonTracklet
 */
class PersonScorer : public Scorer {
 public:
  score_ret_t GetDet2TraScore(const spTarget& target,
                              const spTracklet& track) override;

  score_t GetTra2TraScore(const spTracklet& track1,
                          const spTracklet& track2) override;
  time_t last_time_ = 0;
  int last_frame_count_ = -1;
  int in_fps_ = 0;
  std::chrono::high_resolution_clock::time_point in_start_tp_;
};

}  // namespace feat_based_mot
}  // namespace hobot

#endif  // FEAT_BASED_MOT_DATA_SCORER_H_
