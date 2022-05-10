//
// Created by xuhang.zhuang on 2019-04-15.
// Copyright (c) 2019 Horizon Robotics. All rights reserved.
//

#ifndef FEAT_BASED_MOT_DATA_TARGET_H_
#define FEAT_BASED_MOT_DATA_TARGET_H_

#include <vector>
#include <memory>
#include <algorithm>
#include "hobotlog/hobotlog.hpp"
#include "feat_based_mot/data/types.h"
#include "feat_based_mot/data/feature.h"

namespace hobot {
namespace feat_based_mot {

/**
 * @brief The basic components that the tracker will handle with. Usually it
 * is representing a person with some available attributes like body detection,
 * skeleton, Re-ID feature, etc.
 */
class Target {
 public:
  /**
   * @brief The constructor constructs a target with boby bbox, skeleton,
   * Re-ID feature and time stamp.
   * @param body_bbox The body detection of the target, it is a smart pointer
   * of a BBox object.
   * @param skeleton The skeleton of the target, it is a smart pointer of a
   * Skeleton object.
   * @param reid_features The Re-ID features of the target.
   * @param time_stamp The time stamp at which the target appears.
   */
  explicit Target(const spBBox & body_bbox,
                  const spSkeleton & skeleton,
                  const ReidFeatures & reid_features,
                  time_t time_stamp,
                  int frame_id,
                  float dist_with_others,
                  float overlap_with_others,
                  int img_width,
                  int img_height) {
    body_bbox_ = body_bbox;
    skeleton_ = skeleton;
    reid_features_ = reid_features;
    time_stamp_ = time_stamp;
    frame_id_ = frame_id;
    dist_with_others_ = dist_with_others;
    overlap_with_others_ = overlap_with_others;
    img_width_ = img_width;
    img_height_ = img_height;
    ParseSkeleton();
    LOGD << "Target::Target(), target body id:"
         << body_bbox_->id << ", frame id:" << frame_id_;
  }

  /**
   * @brief The default destructor
   */
  ~Target() {
    LOGD << "Target::~Target(), target body id:"
         << body_bbox_->id << ", frame id:" << frame_id_;
    reid_features_.clear();
  }

  /**
   * @brief To get the time stamp of the target
   * @return The time stamp of the target
   */
  inline const time_t time_stamp() const {
    return time_stamp_;
  }

  inline const int frame_id() const {
    return frame_id_;
  }
  /**
   * @brief To get the body bbox of the target
   * @return The smart pointer of a body bbox
   */
  inline const spBBox body_bbox() const {
    return body_bbox_;
  }

  /**
   * @brief To get the skeleton of the target
   * @return The smart point of the skeleton object
   */
  inline const spSkeleton skeleton() const {
    return skeleton_;
  }

  /**
   * @brief To get the Re-ID feature of the target
   * @return The Re-ID feature
   */
  inline const ReidFeatures& reid_features() const {
    return reid_features_;
  }

  /**
   * @brief To get the orientation of the target
   * @return The orientation of the target
   */
  inline Orientation orientation() const {
    return orientation_;
  }

  /**
   * @brief To set the orientation of the target.
   * @param ori The new orientation of the target.
   */
  inline void set_orientation(Orientation ori) {
    orientation_ = ori;
  }

  /**
   * @brief To get the occlusion of the target.
   * @return The occlusion of the target.
   */
  inline Occlusion occlusion() const {
    return occlusion_;
  }

  /**
   * @brief Indicating weather the target is occluded.
   * @return true, if the target is occluded, otherwise, return false.
   */
  inline bool is_occluded() {
    return occlusion_ == Occlusion::Occluded;
  }

  /**
   * @brief To set occlusion of the target.
   * @param occ The new occlusion state of the target.
   */
  inline void set_occlusion(Occlusion occ) {
    occlusion_ = occ;
  }

  /**
   * @brief To get the indicator of a valid Re-ID feature
   * @return true, if the target has valid Re-ID feature;
   * false, if the target has invalid Re-ID feature.
   */
  inline bool is_feature_valid() const {
    return feature_valid_;
  }

  /**
   * @brief To set the Re-ID feature of the target as invalid.
   */
  inline void set_feature_invalid() {
    feature_valid_ = false;
  }

  /**
   * @brief To set the Re-ID feature of the target as valid
   */
  inline void set_feature_valid() {
    feature_valid_ = true;
  }

  /**
   * @brief To get the indicator of a valid body bbox.
   * @return true, if the body bbox is valid;
   * false, if the body bbox is invalid.
   */
  inline bool is_det_valid() const {
    return det_valid_;
  }

  /**
   * @brief To set the body bbox of the target as valid.
   */
  inline void set_det_valid() {
    det_valid_ = true;
  }

  /**
   * @brief To set the body bbox of the target as invalid
   */
  inline void set_det_invalid() {
    det_valid_ = false;
  }

//  /**
//   * @brief To get the distance of the nearest target in the same frame.
//   * @return The distance of the nearest target in the same frame.
//   * The default value is -1.
//   */
//  inline float nearest_location_info() const {
//    return nearest_location_info_;
//  }
//
//  /**
//   * @brief To set the distance of the nearest target in the same frame.
//   * @param dist The distance of the nearest target in the same frame.
//   */
//  inline void set_nearest_location_info(float dist) {
//    nearest_location_info_ = dist;
//  }

  /**
   * @brief Parse the skeleton of the target to the following attributes:
   *  1. feature_valid_
   *  2. det_valid_
   *  3. occlusion_
   *  4. orientation_
   */

inline float dist_with_others() {
  return dist_with_others_;
}

inline float overlap_with_others() {
  return overlap_with_others_;
}

 private:
  /**
   * @brief The time stamp when the target appears.
   */
  time_t time_stamp_;
  int frame_id_;

  /**
   * @brief The detection of the body of the target.
   */
  spBBox body_bbox_;

  /**
   * @brief The skeleton of the target.
   */
  spSkeleton skeleton_ = nullptr;

  /**
   * @brief The Re-ID feature of the target.
   */
  feature_t reid_features_;

  /**
   * @brief The distance of the nearest target in the same frame.
   */
  float dist_with_others_ = 1000;
  float overlap_with_others_ = 0;

  /**
   * @brief The height of the image where the target appears
   */
  axis_t img_height_ = 1080;

  /**
   * @brief The width of the image where the target appears.
   */
  axis_t img_width_ = 1920;

  /**
   * @brief Indicating weather the Re-ID feature in this target is valid :
   * true, if the Re-ID feature is valid; false, if the Re-ID feature is
   * invalid
   */
  bool feature_valid_ = true;

  /**
   * @brief Indicating weather the detection of the body bbox is valid :
   * true, if the detection is valid; false, if the detection is invalid.
   */
  bool det_valid_ = true;

  /**
   * @brief The orientation of the target.
   */
  Orientation orientation_ = Orientation::Unknown;

  /**
   * @brief The occlusion state of the target.
   */
  Occlusion occlusion_ = Occlusion::Visible;

  void ParseSkeleton();
};
typedef std::shared_ptr<Target> spTarget;

/**
 * @brief Aggregating the body_bbox, skeleton, and Re-ID feature in the same
 * frame into targets.
 * @param bbox_list The list of body bbox in this frame.
 * @param skeleton_list The list of skeleton in this frame.
 * @param reid_feature_list The list of Re-ID features in this frame.
 * @param time_stamp The time stamp of this frame.
 * @return The list of targets in this frame.
 */
std::vector<spTarget> GetTargetsOnOneFrame(
    const std::vector<spBBox>& bbox_list,
    const std::vector<spSkeleton>& skeleton_list,
    const std::vector<ReidFeatures>& reid_feature_list,
    const time_t& time_stamp);

/**
 * @brief To Determining the detection validity based on overlap with other
 * detection in the same frame. If two detection is too close, one of them will
 * be set as invalid.
 * @param det_list The list of body bbox in this frame
 */
void GetDetsValidState(std::vector<spTarget>* det_list);

/**
 * @brief To get the distance of the nearest target in the same frame.
 * @param det_list The list of detection in this frame.
 */
void GetDetsMinDist(std::vector<spTarget>* det_list);



}  // namespace feat_based_mot
}  // namespace hobot

#endif  // FEAT_BASED_MOT_DATA_TARGET_H_
