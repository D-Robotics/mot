//
// Created by xuhang.zhuang on 2019-04-15.
// Copyright (c) 2024 D-Robotics. All rights reserved.
//

#ifndef FEAT_BASED_MOT_UTILS_CONFIG_H_
#define FEAT_BASED_MOT_UTILS_CONFIG_H_

#include <string>

#include "feat_based_mot/data/types.h"

namespace hobot {
namespace feat_based_mot {

// Detection Params
struct DetectionParam {
  score_t min_score;
  float ignore_overlap_thres;
};

// Reid Params
struct ReidModelParam {
  std::string mxnet_symbol;
  std::string mxnet_params;
  std::string dev_type;
  int dev_id;
  int batch_size;
  int input_height;
  int input_width;
  int input_channel;
  float mean_r;
  float mean_g;
  float mean_b;
  float input_scale;
};

struct ReidParam {
  bool use_reid_model;
  bool use_reid_type;
  ReidModelParam model;
  bool use_clustering_reid;
  bool use_reid_mean;
  bool use_orientation_reid;
  bool use_invalid_reid;
  bool use_occlusion_reid;
  float det2tra_reid_thres;
  float tra2tra_reid_thres;
};

// Skeleton Params
struct SkeletonParam {
  int valid_point_num;
  int occlusion_point_num;
  bool ignore_hip_invalid_det;
  bool except_bottom_margin;
  float valid_thres;
};

// Face Param
struct FaceSelectParam {
  axis_t face_min_width;
  score_t face_det_min_score;
  float yaw_thres;
  float pitch_thres;
  float landmark_score_thres;
  int min_valid_landmark_num;
  float pose_frontal_thres;
  float pose_score_weight;
  float face_size_weight;
  axis_t face_inflexion_width;
  axis_t face_max_width;
};

struct FaceParam {
  std::string face_metric_config;
  float feature_update_thres;
  time_t feature_update_time_ms;
  FaceSelectParam face_select_param;
  float face_rectify_thres;
};

// Strategy Params
struct Det2TraStrategyParam {
  std::string similarity_metric;
  bool use_location_gain;
  bool use_min_filter;

  // location related
  //  axis_t distance_constrain;
  float x_distance_over_img_width;
  float y_distance_over_img_width;
  float kps_distance_over_bbox_width;
};

struct Tra2TraStrategyParam {
  bool use_track_cluster;
  time_t cluster_time_interval_ms;
  bool use_nearest_frame_feature;
  bool use_rectify_by_reid_feature;
  int valid_tracklet_len_thres;
  int invalid_tracklet_len_thres;
  int rectify_location_constrain;
};

struct TrackParam {
  bool use_kalman_filter;
  int missing_time_thres;
  time_t remove_invisible_track_ms;
  time_t remove_out_of_date_track_ms;
  bool remove_obsolete_track;
};

struct StrategyParam {
  // source
  bool use_reid;
  bool use_skeleton;
  bool use_face;
  // output
  time_t output_time_interval_ms;
  // strategy
  Det2TraStrategyParam det2tra_strategy_param;
  Tra2TraStrategyParam tra2tra_strategy_param;
  TrackParam track_param;
};

class Config {
 public:
  static Config& instance();
  ~Config() = default;

  void init(const std::string& config_path);

  inline const DetectionParam detection_param() const {
    return detection_param_;
  }

  inline const SkeletonParam skeleton_param() const { return skeleton_param_; }

  inline const ReidParam reid_param() const { return reid_param_; }

  inline const FaceParam face_param() const { return face_param_; }

  inline const StrategyParam strategy_param() const { return strategy_param_; }

 private:
  Config() = default;

 private:
  static Config* instance_;
  DetectionParam detection_param_;
  SkeletonParam skeleton_param_;
  ReidParam reid_param_;
  FaceParam face_param_;
  StrategyParam strategy_param_;
};

}  // namespace feat_based_mot
}  // namespace hobot

#endif  // FEAT_BASED_MOT_UTILS_CONFIG_H_
