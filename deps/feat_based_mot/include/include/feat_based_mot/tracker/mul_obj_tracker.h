//
// Created by xuhang.zhuang on 2019-04-17.
// Copyright (c) 2024 D-Robotics. All rights reserved.
//

#ifndef FEAT_BASED_MOT_TRACKER_MUL_OBJ_TRACKER_H_
#define FEAT_BASED_MOT_TRACKER_MUL_OBJ_TRACKER_H_

#include <deque>
#include <list>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "feat_based_mot/data/cost_matrix.h"
#include "feat_based_mot/data/scorer.h"
#include "feat_based_mot/data/target.h"
#include "feat_based_mot/data/tracklet.h"
#include "feat_based_mot/data/types.h"
#include "feat_based_mot/utils/config.h"
#include "hobotlog/hobotlog.hpp"
#include "opencv2/core.hpp"

namespace hobot {
namespace feat_based_mot {

class MulObjTracker {
 public:
  MulObjTracker() { this->scorer_ = std::make_shared<PersonScorer>(); }

  ~MulObjTracker() {
    target_list_.clear();
    tracklet_list_.clear();
    track_id2det_id_.clear();
    det_id2track_id_.clear();
  }

  inline const std::vector<spTarget>& target_list() { return target_list_; }

  inline const std::vector<spTracklet>& tracklet_list() {
    return tracklet_list_;
  }

  inline bool is_ready_to_output() { return is_ready_to_output_; }

  inline const std::map<int, int> track_id2det_id() { return track_id2det_id_; }

  inline int64_t frame_num() const { return frame_counter_; }

  inline time_t curr_time_stamp() const { return curr_time_stamp_; }

  inline time_t begin_time_stamp() const { return begin_time_stamp_; }

  inline time_t last_time_stamp() const { return last_output_time_stamp_; }

  // Main Process
  int TrackPro(const std::vector<spBBox>& bbox_list,
               const std::vector<spSkeleton>& skeleton_list,
               const std::vector<ReidFeatures>& reid_feature_list,
               time_t time_stamp, int img_width, int img_height,
               uint64 frameid);

  bool GetTrackletState(const int body_id, TrackState* state);

 private:
  // 1. Match Detection and tracklet
  void Match(std::vector<int>* matchded_det, std::vector<int>* matched_track);

  // 2. Update matched tracklet and Mark unmatched tracklet.
  void UpdateTrack(const std::vector<int>& matched_track);

  // 3. Create new tracklet for unmatched detection
  void CreateNewTrack(std::vector<int>* matched_det);

  // 4. Get Match Relation

  // 5. Post-process
  void RectifyByReid();

  void RemoveOutdateTrack();

  void RemoveObsoloteTracklet();

  // 6. Cluster tracklet

  void SetupCostMatrix();

  void UpdateCostMatrix(int col);

  void HierarchicalCluster();

  int BuildTargetList(const std::vector<spBBox>& bbox_list,
                      const std::vector<spSkeleton>& skeleton_list,
                      const std::vector<ReidFeatures>& reid_feature_list,
                      time_t time_stamp, float img_width, float img_height);

 private:
  // output
  bool is_ready_to_output_ = false;
  std::map<int, int> track_id2det_id_;
  std::map<int, int> det_id2track_id_;
  std::vector<spTracklet> tracklet_list_;
  std::vector<spTarget> target_list_;
  const int max_tracklet_num = 100;

  // track state
  frame_id_t frame_counter_ = 0;
  time_t curr_time_stamp_ = -1;
  time_t begin_time_stamp_ = -1;
  time_t last_output_time_stamp_ = -1;
  axis_t img_width_ = 1920;
  axis_t img_height_ = 1080;

  // for clustering
  bool is_cost_matrix_set_up_ = false;
  time_t last_cluster_time_stamp_ = -1;

  // scorer
  std::shared_ptr<Scorer> scorer_;
  std::shared_ptr<CostMatrix> cost_matrix_;
};

}  // namespace feat_based_mot
}  // namespace hobot

#endif  // FEAT_BASED_MOT_TRACKER_MUL_OBJ_TRACKER_H_
