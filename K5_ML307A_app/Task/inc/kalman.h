/*
 * kalman.h
 *
 *	Created on: Jun 21, 2024
 *		Author: lvvv
 */

#ifndef KALMAN_H_
#define KALMAN_H_

#include "matrix.h"
#include "config.h"

typedef struct 
{
  //卡尔曼模型中的k
  int timestep;

  //状态转移矩阵和观测矩阵的维度
  int state_dimension, observation_dimension;
  
  //状态转移矩阵F_k
  matrix_s* p_state_transition;
  
  //观测矩阵H_k
  matrix_s* p_observation_model;
  
  //状态转移噪声矩阵Q_k
  matrix_s* p_process_noise_covariance;
  
  //观测噪声矩阵R_k
  matrix_s* p_observation_noise_covariance;

  //观测值z_k，每一步由用户修改
  matrix_s* p_observation;
  
  //预测状态x-hat_k|k-1，每一步由滤波器更新
  matrix_s* p_predicted_state;
  
  //预测估计协方差P_k|k-1
  matrix_s* p_predicted_estimate_covariance;
  
  //新息（innovation，y-tilde_k）
  matrix_s* p_innovation;
  
  //新息协方差（S_k）
  matrix_s* p_innovation_covariance;
  
  //新息协方差逆矩阵（S_k^-1）
  matrix_s* p_inverse_innovation_covariance;
  
  // 最优卡尔曼增益(K_k)
  matrix_s* p_optimal_gain;
  
  //状态估计（x-hat_k|k）
  matrix_s* p_state_estimate;
  
  //估计协方差（P_k|k）
  matrix_s* p_estimate_covariance;

  //存储中间计算结果
  matrix_s* p_vertical_scratch;
  
  matrix_s* p_small_square_scratch;
  
  matrix_s* p_big_square_scratch;
  
} kalman_filter_s;


kalman_filter_s* kalman_create(int state_dimension,int observation_dimension);
void kalman_destroy(kalman_filter_s* p_filter);
BOOL kalman_update(kalman_filter_s* p_filter);


#endif /* KALMAN_H_ */

