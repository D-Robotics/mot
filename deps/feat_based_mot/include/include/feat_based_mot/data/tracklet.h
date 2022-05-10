//
// Created by xuhang.zhuang on 2019-04-15.
// Copyright (c) 2019 Horizon Robotics. All rights reserved.
//

#ifndef FEAT_BASED_MOT_DATA_TRACKLET_H_
#define FEAT_BASED_MOT_DATA_TRACKLET_H_

#include <deque>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "feat_based_mot/data/feature.h"
#include "feat_based_mot/data/target.h"
#include "feat_based_mot/data/types.h"
#include "hobotlog/hobotlog.hpp"

namespace hobot {
namespace feat_based_mot {

/**
 * @brief A Tracklet is constructed by multiple targets in different times.
 * A tracklet should have only one target in a frame. Each tracklet will be
 * given a unique id.
 */
class Tracklet {
 public:
  /**
   * @brief The constructor constructs a tracklet with a target and
   * the time stamp and frame id when the target appears.
   * @param target The target to construct the tracklet
   * @param time_stamp The time stamp when the target appears.
   * @param frame_id The frame id when the target appears.
   */
  Tracklet(const spTarget& target, time_t time_stamp, frame_id_t frame_id);
  ~Tracklet();

  /**
   * @brief To get the last valid target of the tracklet
   * @return The last valid target of the tracklet
   */
  inline const spTarget target() const { return target_; }

  /**
   * @brief To get the indicator wheather the tracklet has a target in a
   * specific time stamp.
   * @param time_stamp The query time stamp.
   * @return true, if the tracklet has a target in the query time stamp.
   * otherwise, return false.
   */
  inline bool HasTargetOn(time_t time_stamp) {
    return (target_map_.find(time_stamp) != target_map_.end());
  }

  /**
   * @brief To query a target in the tracklet at a specific time stamp
   * @param time_stamp The query time stamp
   * @return The target of the time stamp.
   * @note Must have target in the query time stamp, use HasTargetOn() to check
   * before using this function.
   */
  inline const spTarget GetTargetOf(time_t time_stamp) {
    // HOBOT_CHECK(target_map_.find(time_stamp) != target_map_.end());
    if (target_map_.find(time_stamp) == target_map_.end()) {
      return nullptr;
    }
    return target_map_[time_stamp];
  }

  /**
   * @brief To get the track id of the tracklet.
   * @return The track id of the tracklet.
   */
  inline track_id_t track_id() const { return track_id_; }

  /**
   * @brief To get the target map of the tracklet
   * @return The target map of the tracklet
   */
  inline const std::map<time_t, spTarget>& target_map() const {
    return target_map_;
  }

  /**
   * @brief To get the trajectory of the tracklet
   * @return The trajectory of the tracklet
   */
  inline const std::deque<std::pair<time_t, spTarget>>& trajectory() const {
    return trajectory_;
  }

  inline const std::vector<int>& frame_id_list() const {
    return frame_id_list_;
  }

  /**
   * @brief To get the track state of the tracklet
   * @return The track state of the tracklet
   */
  inline TrackState track_state() const { return track_state_; }

  /**
   * @brief To set new track state of the tracklet
   * @param new_state The new track state
   */
  inline void set_track_state(TrackState new_state) {
    track_state_ = new_state;
  }

  /**
   * @brief To get the indicator of a valid track.
   * @return ture, if the track is valid; false, if the track is invalid.
   */
  inline bool is_track_valid() const { return is_track_valid_; }

  /**
   * @brief To set tracklet as valid.
   */
  inline void set_track_valid() { is_track_valid_ = true; }

  /**
   * @brief To set tracklet as invalid.
   */
  inline void set_track_invalid() { is_track_valid_ = false; }

  /**
   * @brief To get the begin frame of the tracklet
   * @return The begin frame of the tracklet.
   */
  inline int begin_frame() const { return begin_frame_; }

  /**
   * @brief To get the last frame of the tracklet.
   * @return The last frame of the tracklet.
   */
  inline int last_frame() const { return last_frame_; }

  /**
   * @brief To set the last frame of the tracklet.
   * @param frame_id The new frame id of the last frame.
   */
  inline void set_last_frame(frame_id_t frame_id) { last_frame_ = frame_id; }

  /**
   * @brief To get the last time stamp of the tracklet.
   * @return The last time stamp of the tracklet.
   */
  inline time_t last_time_stamp() const { return last_time_stamp_; }

  /**
   * @brief To set new last time stamp of the tracklet
   * @param time_stamp The new last time stamp
   */
  inline void set_last_time_stamp(time_t time_stamp) {
    last_time_stamp_ = time_stamp;
  }

  /**
   * @brief To get the number of missing frames
   * @return The number of missing frames
   */
  inline int missing_frame_num() const { return missing_frame_num_; }

  /**
   * @brief Add one more missing frame.
   */
  inline void add_missing_frame() { missing_frame_num_++; }

  /**
   * @brief To set the number of missing frame to zero
   */
  inline void set_zero_missing_frame() { missing_frame_num_ = 0; }

  inline float dist_with_others() const { return dist_with_others_; }

  inline float overlap_with_others() const { return overlap_with_other_; }

  /**
   * @brief Indicating weather the tracklet has been marked as deleted.
   * @return true, if the track state is deleted, otherwise, return false.
   */
  inline bool is_deleted() { return track_state_ == TrackState::Deleted; }

  // TODO(xuhang) looks unnecessary to me.
  inline bool is_occluded() const { return is_occluded_; }

  /**
   * @brief To get the match state of the tracklet for debug purpose
   * @return The match state of the tracklet
   */
  inline std::string match_state() const { return match_state_; }

  /**
   * @brief To set new match state for tracklet on the purpose of debug.
   * @param match_state The new match state.
   */
  inline void set_match_state(const std::string& match_state) {
    match_state_ = match_state;
  }

  /**
   * @brief To add a new target to tracklet.
   * @param target The new target.
   * @param time_stamp The time stamp when the target appears.
   * @param frame_id The frame id when the target appears.
   */
  void AddTarget(const spTarget& target, time_t time_stamp, int frame_id);

  void MarkMissed(float time_stamp_now);
  /**
   * @brief To merge a tracklet
   * @param other The tracklet to be merged with
   * @note After merge, the other tracklet will be marked as Deleted!
   */
  void MergeWith(const std::shared_ptr<Tracklet>& other);

  /**
   * @brief To calculate distance between tracklets.
   * @param other The other tracklet
   * @return The tracklet distance between two tracklets.
   */
  float GetTrackletDistWith(const std::shared_ptr<Tracklet>& other,
                            int times = 5);

  // reid feature related
  /**
   * @brief To get the last valid Re-ID feature in the tracklet.
   * @return The last valid Re-ID feature in the tracklet.
   */
  inline const ReidFeatures& reid_features() const { return reid_features_; }

  inline const bool first_feature_valid() const { return first_feature_valid_; }
  // TODO(xuhang) look unnecessary to me.
  /**
   * @brief To indicating whether the latest Re-ID feature is valid
   * @return true, if the latest Re-ID feature is valid. otherwise,
   * return false.
   */
  inline bool feature_valid() const { return feature_valid_; }

  /**
   * @brief To get the last invalid Re-ID feature of the tracklet.
   * @return The last invalid Re-ID feature of the tracklet.
   */
  inline const ReidFeatures& last_invalid_reid_feature() const {
    return last_invalid_reid_feature_;
  }

  /**
   * @brief To get the time stamp of the last invalid Re-ID feature of the
   * tracklet.
   * @return The last time stamp of the last invalid Re-ID feature.
   */
  inline time_t last_invalid_time_stamp() const {
    return last_invalid_time_stamp_;
  }

  inline time_t last_valid_time_stamp() const { return last_valid_time_stamp_; }

  /**
   * @brief To get the frame id of the last invalid Re-ID feature of the
   * tracklet.
   * @return The last frame id of the last invalid Re-ID feature.
   */
  inline int last_invalid_frame() const { return last_invalid_frame_; }

  /**
   * @brief To get the last occlusion Re-ID feature of the tracklet.
   * @return The last occlusion Re-ID feature of the tracklet.
   */
  inline const ReidFeatures& last_occlusion_reid_feature() const {
    return last_occlusion_reid_feature_;
  }

  /**
   * @brief To get the last time stamp of the occlusion Re-ID feature of the
   * tracklet.
   * @return The last time stamp of the occlusion Re-ID feature.
   */
  inline time_t last_occlusion_time_stamp() const {
    return last_occlusion_time_stamp_;
  }

  /**
   * @brief To get the frame id of the last occlusion Re-ID feature. in the
   * tracklet.
   * @return The frame id of the last occlusion Re-ID feature.
   */
  inline frame_id_t last_occlusion_frame() const {
    return last_occlusion_frame_;
  }

  /**
   * @brief To get the mean of the Re-ID features of the tracklet.
   * @return The mean of the Re-ID features.
   */
  inline const ReidFeatures& reid_feature_mean() const {
    return reid_feature_mean_;
  }

  /**
   * @brief To get the number of the Re-ID features in the tracklet.
   * @return The number of the Re-ID features in the tracklet.
   */
  inline int reid_feature_cnt() const { return reid_feature_cnt_; }

  /**
   * @brief To get the mean of the front Re-ID features of the tracklet.
   * @return The mean of the front Re-ID features.
   */
  inline const ReidFeatures& front_feature_mean() const {
    return front_feature_mean_;
  }

  /**
   * @brief To get the number of the front Re-ID features.
   * @return The number of the front Re-ID features.
   */
  inline int front_feature_cnt() const { return front_feature_cnt_; }

  /**
   * @brief To get the mean of the back Re-ID features of the tracklet.
   * @return The mean of the back Re-ID features.
   */
  inline const ReidFeatures& back_feature_mean() const {
    return back_feature_mean_;
  }

  /**
   * @brief To get the number of the back Re-ID features.
   * @return The number of the back Re-ID features.
   */
  inline int back_feature_cnt() const { return back_feature_cnt_; }

  /**
   * @brief To get the mean of the left Re-ID features of the tracklet.
   * @return The mean of the left Re-ID features.
   */
  inline const ReidFeatures& left_feature_mean() const {
    return left_feature_mean_;
  }

  /**
   * @brief To get the number of the left Re-ID features.
   * @return The number of the left Re-ID features.
   */
  inline int left_feature_cnt() const { return left_feature_cnt_; }

  /**
   * @brief To get the mean of the right Re-ID features of the tracklet.
   * @return The mean of the right Re-ID features.
   */
  inline const ReidFeatures& right_feature_mean() const {
    return right_feature_mean_;
  }

  /**
   * @brief To get the number of the right Re-ID features.
   * @return The number of the right Re-ID features.
   */
  inline int right_feature_cnt() const { return right_feature_cnt_; }

  /**
   * @brief To get the feature cluster of the tracklet.
   * @return The feature cluster of the tracklet.
   */
  inline const FeatureCluster& clustering_feature() const {
    return clustering_feature_;
  }

  /**
   * @brief To get the map of the Re-ID features.
   * The key value pair is <frame_id, Re-ID features>.
   * @return The map of the Re-ID features.
   */
  inline const std::map<frame_id_t, ReidFeatures>& reid_feature_map() const {
    return reid_feature_map_;
  }

  /**
   * @brief To get the clustering feature sorted by the number of the Re-ID
   * features in ascending order.
   */
  inline void sort_clustering_feature() { clustering_feature_.sort(); }

  // rectify related
  bool is_rectify_confirmed(frame_id_t curr_frame_id);

  bool is_rectify_invisible(frame_id_t curr_frame_id);

  // Clustering Related
  inline int num_accumulated_targets() const {
    return num_accumulated_targets_;
  }

  inline void set_zero_accumulated_targets() {
    num_accumulated_targets_ = 0;
  }

 private:
  void RemoveOldInfo();

 private:
  /**
   * @brief The unique track id of the tracklet
   */
  track_id_t track_id_;

  /**
   * @brief The last valid target of the tracklet.
   */
  std::shared_ptr<Target> target_;

  /**
   * @brief The map of the targets in the tracklet
   * The key-value pair is <time_stamp, spTarget>
   */
  std::map<time_t, std::shared_ptr<Target>> target_map_;

  /**
   * @brief The trajectory of the tracklet
   */
  std::deque<std::pair<time_t, spTarget>> trajectory_;

  int begin_frame_ = -1;
  int last_frame_ = -1;
  time_t last_time_stamp_ = -1;

  /**
   * @brief The match state for debug usage.
   */
  std::string match_state_;

  float dist_with_others_ = -1;
  float overlap_with_other_ = 0;
  time_t last_invalid_time_stamp_ = -1;
  time_t last_valid_time_stamp_ = -1;
  time_t last_occlusion_time_stamp_ = -1;
  int last_invalid_frame_ = -1;
  int last_occlusion_frame_ = -1;
  std::vector<int> frame_id_list_;
  std::vector<time_t> time_stamp_list_;
  TrackState track_state_ = TrackState::Tentative;
  int missing_frame_num_ = 0;
  bool is_track_valid_ = true;
  bool is_occluded_ = false;
  bool first_feature_valid_ = false;

  // reid feature related
  std::map<frame_id_t, ReidFeatures> reid_feature_map_;
  bool feature_valid_ = false;
  ReidFeatures reid_features_;
  ReidFeatures last_features_;
  ReidFeatures last_invalid_reid_feature_;
  ReidFeatures last_occlusion_reid_feature_;
  ReidFeatures reid_feature_mean_;
  int reid_feature_cnt_ = 0;
  ReidFeatures front_feature_mean_;
  int front_feature_cnt_ = 0;
  ReidFeatures back_feature_mean_;
  int back_feature_cnt_ = 0;
  ReidFeatures left_feature_mean_;
  int left_feature_cnt_ = 0;
  ReidFeatures right_feature_mean_;
  int right_feature_cnt_ = 0;
  FeatureCluster clustering_feature_;

  // Clustering Related
  int num_accumulated_targets_ = 1;
  const int max_track_num = 100;
};
typedef std::shared_ptr<Tracklet> spTracklet;
typedef std::deque<spTracklet> TrackletList;

float GetNearestFramesDistance(const spTracklet& sp_tracklet1,
                               const spTracklet& sp_tracklet2,
                               const std::vector<int>& frames1,
                               const std::vector<int>& frames2,
                               frame_id_t max_frame_interval);

}  // namespace feat_based_mot
}  // namespace hobot

#endif  // FEAT_BASED_MOT_DATA_TRACKLET_H_
