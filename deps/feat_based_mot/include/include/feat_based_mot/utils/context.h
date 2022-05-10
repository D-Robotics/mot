//
// Created by xuhang.zhuang on 2019-04-28.
// Copyright (c) 2019 Horizon Robotics. All rights reserved.
//

#ifndef FEAT_BASED_MOT_UTILS_CONTEXT_H_
#define FEAT_BASED_MOT_UTILS_CONTEXT_H_

#include "feat_based_mot/data/types.h"

namespace hobot {
namespace feat_based_mot {

class Context {
 public:
  /**
   * @brief Get the singleton.
   * @return The singleton instance.
   */
  inline static Context *instance() {
    static Context *context = nullptr;
    if (nullptr == context) {
      context = new Context();
    }
    return context;
  }

  /**
   * @brief Get a new person ID.
   * @return An auto-increment person ID.
   */
  static track_id_t GetNewTrackId() {
    static track_id_t track_id = 1;
    return track_id++;
  }
};

}  // namespace feat_based_mot
}  // namespace hobot

#endif  // FEAT_BASED_MOT_UTILS_CONTEXT_H_
