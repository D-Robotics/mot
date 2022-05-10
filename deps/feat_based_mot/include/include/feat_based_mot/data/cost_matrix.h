//
// Created by xuhang.zhuang on 2019-05-20.
// Copyright (c) 2019 Horizon Robotics. All rights reserved.
//

#ifndef FEAT_BASED_MOT_DATA_COST_MATRIX_H_
#define FEAT_BASED_MOT_DATA_COST_MATRIX_H_

#include "feat_based_mot/data/types.h"
#include "Eigen/Dense"

namespace hobot {
namespace feat_based_mot {

/**
 * @brief The cost matrix is to store the cost score between tracks.
 * We use a N * N matrix to represent. The underlying storage use
 * Eigen::Matrix.
 */
class CostMatrix {
 public:
  /**
   * @brief The constructor that will construct a N * N cost matrix.
   * @param track_list_size The dimension of the cost matrix.
   */
  explicit CostMatrix(int track_list_size);
  ~CostMatrix() {
    for (int i = 0; i < track_list_size_; ++i) {
      RemoveCol(i);
    }
  }

  /**
   * @brief Get the dimension of the cost matrix.
   * @return The dimension of the cost matrix.
   */
  inline int track_list_size() const {
    return track_list_size_;
  }

  /**
   * @brief Get the minimum distance of the cost matrix and the corresponding
   * row index and column index.
   * @param col The corresponding column index
   * @param row The corresponding row index
   * @return The minimum distance
   */
  score_t minCoeff(int* col, int* row);

  /**
   * @brief Append one more column at the end of the cost matrix.
   */
  void AppendCol();

  /**
   * @brief Remove the column of the corresponding track index of the cost
   * matrix.
   * @param track_idx The track index which will be removed.
   */
  void RemoveCol(int track_idx);


  /**
   * @brief Set the score of the corresponding column index and row index in
   * cost matrix.
   * @param track_idx_1 The first track index
   * @param track_idx_2 The second track index
   * @param dist The score that will be assigned
   */
  void Set(int track_idx_1, int track_idx_2, score_t dist);

  /**
   * @brief Get the score of the corresponding index pair.
   * @param track_idx_1 The first track index.
   * @param track_idx_2 The seconnd track index.
   * @return The corresponding score.
   */
  score_t Get(int track_idx_1, int track_idx_2);

 private:
  /**
   * @brief The deminsion of the cost matrix
   */
  int track_list_size_ = 0;

  /**
   * @brief The cost matrix
   */
  Eigen::MatrixXd cost_matrix_;
};

}  // namespace feat_based_mot
}  // namespace hobot

#endif  // FEAT_BASED_MOT_DATA_COST_MATRIX_H_
