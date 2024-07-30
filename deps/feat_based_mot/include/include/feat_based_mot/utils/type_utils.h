//
// Created by xuhang.zhuang on 2019-04-15.
// Copyright (c) 2018 D-Robotics. All rights reserved.
//

#ifndef FEAT_BASED_MOT_UTILS_TYPE_UTILS_H_
#define FEAT_BASED_MOT_UTILS_TYPE_UTILS_H_

#include <memory>
#include <list>
#include <array>
#include <vector>
#include "Eigen/Dense"
#include "feat_based_mot/data/types.h"
#include "feat_based_mot/data/feature.h"

namespace hobot {
namespace feat_based_mot {

// Enum
template <typename Enumeration>
inline auto as_integer(Enumeration const value)
               -> typename std::underlying_type<Enumeration>::type {
  return static_cast<typename std::underlying_type<Enumeration>::type>(value);
}

// BBox related
bool hasIntersect(const spBBox & b1, const spBBox & b2);

spBBox GetIntersectBBox(const spBBox & b1, const spBBox & b2);

float IOU(const spBBox & b1, const spBBox & b2);

float OverlapOverTheFirst(const spBBox & b1, const spBBox & b2);

float GetOverlapRatio(const spBBox & b1, const spBBox & b2);

float GetBBoxDistance(const spBBox & b1, const spBBox & b2);

float GetBBoxDistance_X(const spBBox & b1, const spBBox & b2);

float GetBBoxDistance_Y(const spBBox & b1, const spBBox & b2);

bool IsPointInBBox(const ScorePoint & p, const spBBox & box);

// Skeleton Related
Occlusion GetOcclusion(const spSkeleton& skeleton);

inline bool isOccluded(const spSkeleton& skeleton) {
  return GetOcclusion(skeleton) == Occlusion::Occluded;
}

Orientation GetOrientation(const spSkeleton& skeleton);

void GetPositionSimilarity(const Eigen::MatrixXf& X,
                           const Eigen::MatrixXf& Y,
                           bool b_compute_scale,
                           float* l2_error,
                           float* rotation,
                           float* scale,
                           std::vector<float>* translation);

struct SkeletonSimilarity {
  int common_num;
  float l2_error;
  float rotation;
  float scale;
  std::vector<float> translation;
};

SkeletonSimilarity GetSkeletonSimilarity(const spSkeleton& s1,
                                         const spSkeleton& s2);

struct SkeletonDistance {
  int valid_num;
  float skeleton_dist;
};

SkeletonDistance GetSkeletonDistance(const spSkeleton& s1,
                                     const spSkeleton& s2);

struct SkeletonDiff {
  int diff_num;
  int det_valid_num;
  int track_valid_num;
};

SkeletonDiff GetSkeletonDiff(const spSkeleton& det_skeleton,
                             const spSkeleton& track_skeleton);

// frame
bool have_same_element(const std::vector<int>& vec1,
                       const std::vector<int>& vec2,
                       int allow_same_nums = 0);

void GetNearestFrame(const std::vector<int>& frame1,
                     const std::vector<int>& frame2,
                     frame_id_t* frame_dist,
                     frame_id_t* frame_id1,
                     frame_id_t* frame_id2);

}  // namespace feat_based_mot
}  // namespace hobot

#endif  // FEAT_BASED_MOT_UTILS_TYPE_UTILS_H_
