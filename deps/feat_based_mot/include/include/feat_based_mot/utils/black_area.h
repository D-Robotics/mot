//
// Created by xuhang.zhuang on 2019-04-22.
// Copyright (c) 2024 D-Robotics. All rights reserved.
//

#ifndef FEAT_BASED_MOT_UTILS_BLACK_AREA_H_
#define FEAT_BASED_MOT_UTILS_BLACK_AREA_H_

#include <string>
#include <vector>
#include "feat_based_mot/data/types.h"

namespace hobot {
namespace feat_based_mot {

std::vector<spBBox> GetBlackAreas(std::string black_area_config_path,
                                  int target_ipc_idx);

}  // namespace feat_based_mot
}  // namespace hobot

#endif  // FEAT_BASED_MOT_UTILS_BLACK_AREA_H_
