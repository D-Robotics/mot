//
// Created by xuhang.zhuang on 2019-04-15.
// Copyright (c) 2024 D-Robotics. All rights reserved.
//

#ifndef FEAT_BASED_MOT_DATA_TYPES_H_
#define FEAT_BASED_MOT_DATA_TYPES_H_

#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "horizon/vision_type/vision_type.hpp"

namespace hobot {
namespace feat_based_mot {

typedef int64_t time_t;
typedef int frame_id_t;
typedef float axis_t;
typedef float score_t;
typedef int track_id_t;

enum struct Orientation {
  Unknown = 0,
  Front = 1,
  Back = 2,
  LeftSide = 3,
  RightSide = 4
};

enum struct Occlusion { Visible = 0, Occluded = 1 };

enum struct TrackState {
  Tentative = 0,
  Confirmed = 1,
  Invisible = 2,
  OutOfDate = 3,
  Deleted = 4
};

template <typename Dtype>
struct BBox_ {
  inline BBox_() {
    id = score = x1 = x2 = y1 = y2 = attribute = 0;
    category_name = "";
  }
  inline BBox_(Dtype x1_, Dtype y1_, Dtype x2_, Dtype y2_, int id_ = 0,
               float score_ = 0.0f, float attri_ = 0.0f,
               const std::string &category_name_ = "") {
    id = id_;
    x1 = x1_;
    y1 = y1_;
    x2 = x2_;
    y2 = y2_;
    score = score_;
    attribute = attri_;
    category_name = category_name_;
  }

  explicit BBox_(const hobot::vision::BBox &box) {
    this->id = box.id;
    this->x1 = box.x1;
    this->y1 = box.y1;
    this->x2 = box.x2;
    this->y2 = box.y2;
    this->score = box.score;
  }
  ~BBox_() = default;

  inline Dtype width() const { return (x2 - x1); }
  inline Dtype height() const { return (y2 - y1); }
  inline Dtype cx() const { return (x1 + (x2 - x1) / 2); }
  inline Dtype cy() const { return (y1 + (y2 - y1) / 2); }
  inline friend std::ostream &operator<<(std::ostream &out, BBox_ &bbox) {
    out << "( x1: " << bbox.x1 << " y1: " << bbox.y1 << " x2: " << bbox.x2
        << " y2: " << bbox.y2 << " score: " << bbox.score << " )";
    return out;
  }
  inline static bool greater(const BBox_ &a, const BBox_ &b) {
    return a.score > b.score;
  }

  Dtype x1, y1, x2, y2;
  int32_t id = -1;
  float score, attribute;
  std::string category_name = "";
};
typedef BBox_<float> BBox;
typedef std::shared_ptr<BBox> spBBox;

/**
 * @brief A Score Point is a 2D point with a score of confidence.
 */
struct ScorePoint {
  ScorePoint(const axis_t x, const axis_t y, const float score)
      : x(x), y(y), score(score) {}

  ~ScorePoint() = default;

  axis_t x;
  axis_t y;
  score_t score;

  friend std::ostream &operator<<(std::ostream &out, const ScorePoint &point) {
    out << point.x << "," << point.y << "," << point.score;
    return out;
  }
};

template <typename Dtype>
struct HumanSkeleton_ {
  inline HumanSkeleton_() { point_num = 0; }
  explicit HumanSkeleton_(int point_num_) {
    point_num = point_num_;
    points.resize(point_num);
    scores.resize(point_num);
  }
  ~HumanSkeleton_() = default;
  int point_num;
  std::vector<hobot::vision::Point> points;
  std::vector<float> scores;
};
typedef HumanSkeleton_<float> HumanSkeleton;

/**
 * @brief A skeleton is a sequence of Score Point which representing the key
 * point of a human.
 */
struct Skeleton {
 public:
  explicit Skeleton(std::vector<ScorePoint> data) : kps(data) {}

  explicit Skeleton(HumanSkeleton skeleton) {
    for (int i = 0; i < skeleton.point_num; i++) {
      auto kp = ScorePoint(skeleton.points[i].x, skeleton.points[i].y,
                           skeleton.scores[i]);
      kps.push_back(kp);
    }
  }

  ~Skeleton() = default;

  std::vector<ScorePoint> kps;

  friend std::ostream &operator<<(std::ostream &out, const Skeleton &skeleton) {
    auto kps_tmp = skeleton.kps;
    for (size_t i = 0; i < kps_tmp.size(); i++) {
      if (i == kps_tmp.size() - 1) {
        out << kps_tmp[i];
      } else {
        out << kps_tmp[i] << " ";
      }
    }
    return out;
  }
};
typedef std::shared_ptr<Skeleton> spSkeleton;

class Landmark {
 public:
  explicit Landmark(std::vector<ScorePoint> data) : data_(data) {}

  ~Landmark() = default;

  inline const std::vector<ScorePoint> &data() { return data_; }

 private:
  std::vector<ScorePoint> data_;
};

class FacePose {
 public:
  FacePose(float yaw, float pitch, float roll)
      : yaw_(yaw), pitch_(pitch), roll_(roll) {}

  ~FacePose() = default;

  inline const float yaw() { return yaw_; }

  inline const float pitch() { return pitch_; }

  inline const float roll() { return roll_; }

 private:
  float yaw_;
  float pitch_;
  float roll_;
};

}  // namespace feat_based_mot
}  // namespace hobot

#endif  // FEAT_BASED_MOT_DATA_TYPES_H_
