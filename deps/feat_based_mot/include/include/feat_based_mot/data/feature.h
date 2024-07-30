//
// Created by xuhang.zhuang on 2019-04-16.
// Copyright (c) 2024 D-Robotics. All rights reserved.
//

#ifndef FEAT_BASED_MOT_DATA_FEATURE_H_
#define FEAT_BASED_MOT_DATA_FEATURE_H_

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include "hobot/hobot.h"

namespace hobot {
namespace feat_based_mot {

typedef float feature_data_t;
typedef std::vector<feature_data_t> feature_t;
typedef feature_t ReidFeatures;
typedef feature_t FaceFeatures;

/**
 * @brief To get the Euclidean distance of two features.
 * @param f1 The first features.
 * @param f2 The second features.
 * @return The distance between two features.
 */
float GetFeatureDistance(const feature_t& f1, const feature_t& f2);

/**
 * @brief To calculate the weighted mean of two features, the result will
 * be updated into first feature as :
 *   feat1[i] = (feat1[i] * size1 + feat2[i] * size2) / (size1 + size2)
 * @param feat1 The first feature
 * @param size1 The weight of the first feature
 * @param feat2 The second feature
 * @param size2 The weight of the second feature
 */
void MergeFeatureMean(feature_t* feat1, int size1, const feature_t& feat2,
                      int size2);

/**
 * @brief The supportive data structure for feature cluster.
 */
class MEAN_STD {
 public:
  int cnt_;
  bool is_ok_;
  feature_t sum_;
  feature_t sum_square_;

  MEAN_STD() : cnt_(0), is_ok_(false) {}

  explicit MEAN_STD(int dim) : cnt_(0), is_ok_(false) {
    sum_.resize(dim, 0);
    sum_square_.resize(dim, 0);
  }
  ~MEAN_STD() {
    sum_.clear();
    sum_square_.clear();
  }

  inline void Add(const MEAN_STD& mean_std) {
    cnt_ += mean_std.cnt_;
    for (size_t i = 0; i < sum_.size(); i++) {
      sum_[i] += mean_std.sum_[i];
      sum_square_[i] += mean_std.sum_square_[i];
    }
  }

  inline void AddValue(const std::vector<float>& val) {
    cnt_++;
    for (size_t i = 0; i < sum_.size(); i++) {
      sum_[i] += val[i];
      sum_square_[i] += val[i] * val[i];
    }
  }

  inline void RemoveValue(const std::vector<float>& val) {
    cnt_--;
    for (size_t i = 0; i < sum_.size(); i++) {
      sum_[i] -= val[i];
      sum_square_[i] -= val[i] * val[i];
    }
  }

  inline bool GetMean(std::vector<float>* mean) {
    if (0 == cnt_) {
      return false;
    }
    mean->resize(sum_.size());
    if (1 == cnt_) {
      for (size_t i = 0; i < sum_.size(); i++) {
        (*mean)[i] = sum_[i];
      }
      return true;
    }

    for (size_t i = 0; i < sum_.size(); i++) {
      (*mean)[i] = sum_[i] / static_cast<float>(cnt_);
    }
    return true;
  }

  inline bool GetMeanStd(std::vector<float>* mean, std::vector<float>* std) {
    if (0 == cnt_) {
      return false;
    }

    for (size_t i = 0; i < sum_.size(); i++) {
      (*mean)[i] = sum_[i] / static_cast<float>(cnt_);
      (*std)[i] = sum_square_[i] / static_cast<float>(cnt_);
      (*std)[i] -= (*mean)[i] * (*mean)[i];
      (*std)[i] = std::sqrt((*std)[i]);
    }
    return true;
  }
};

/**
 * @brief A feature cluster is a multi-gussian clusters of Re-ID features.
 * Currently the maximun center is 5.
 */
class FeatureCluster {
 public:
  FeatureCluster() = default;
  ~FeatureCluster() { features_.clear(); }

  inline const std::deque<MEAN_STD> features() const { return features_; }

  inline void AddFeature(const feature_t& reid_feature) {
    if (static_cast<int>(features_.size()) < max_feature_buf_) {
      MEAN_STD mean_std(reid_feature.size());
      mean_std.AddValue(reid_feature);
      features_.push_back(mean_std);
    } else {
      float min_dist = std::numeric_limits<float>::max();
      int idx = -1;
      for (size_t i = 0; i < features_.size(); ++i) {
        std::vector<float> cur_feature;
        features_[i].GetMean(&cur_feature);
        float dist = GetFeatureDistance(reid_feature, cur_feature);
        if (dist < min_dist) {
          min_dist = dist;
          idx = i;
        }
      }
      features_[idx].AddValue(reid_feature);
    }
  }

  inline void AddCenter(MEAN_STD* feature) {
    if (static_cast<int>(features_.size()) < max_feature_buf_) {
      features_.push_back(*feature);
    } else {
      float min_dist = std::numeric_limits<float>::max();
      int idx = -1;
      for (size_t i = 0; i < features_.size(); ++i) {
        feature_t curr_feature, add_feature;
        features_[i].GetMean(&curr_feature);
        feature->GetMean(&add_feature);
        float dist = GetFeatureDistance(add_feature, curr_feature);
        if (dist < min_dist) {
          min_dist = dist;
          idx = i;
        }
      }
      features_[idx].Add(*feature);
    }
  }

  inline void MergeFeatureCluster(const FeatureCluster& other) {
    auto& features = other.features();
    for (auto feature : features) {
      this->AddCenter(&feature);
    }
  }

  inline float GetMinDistance(const feature_t& reid_feature) {
    float min_dist = std::numeric_limits<float>::max();
    // keep features_ in a sorted order by cnt
    std::stable_sort(features_.begin(), features_.end(),
                     [](MEAN_STD i, MEAN_STD j) { return i.cnt_ > j.cnt_; });
    for (auto& feature : features_) {
      if (feature.cnt_ <= 5) continue;
      feature_t cur_feature;
      feature.GetMean(&cur_feature);
      float dist = GetFeatureDistance(reid_feature, cur_feature);
      if (dist < min_dist) {
        min_dist = dist;
      }
    }
    return min_dist;
  }

  inline void sort() {
    std::stable_sort(features_.begin(), features_.end(),
                     [](MEAN_STD i, MEAN_STD j) { return i.cnt_ > j.cnt_; });
  }

 private:
  int max_feature_buf_ = 5;
  std::deque<MEAN_STD> features_;
};

/**
 * @brief A helper function to print ReID feature as a string
 * @param reid The ReID feature to be converted.
 * @return A ReID feature string
 */
inline std::string print_reid(const ReidFeatures& reid) {
  std::stringstream ss;
  for (size_t i = 0; i < reid.size(); i++) {
    if (i == reid.size() - 1) {
      ss << reid[i];
    } else {
      ss << reid[i] << " ";
    }
  }
  return ss.str();
}

}  // namespace feat_based_mot
}  // namespace hobot

#endif  // FEAT_BASED_MOT_DATA_FEATURE_H_
