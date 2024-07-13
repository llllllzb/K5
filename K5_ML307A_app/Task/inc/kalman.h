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
  //������ģ���е�k
  int timestep;

  //״̬ת�ƾ���͹۲�����ά��
  int state_dimension, observation_dimension;
  
  //״̬ת�ƾ���F_k
  matrix_s* p_state_transition;
  
  //�۲����H_k
  matrix_s* p_observation_model;
  
  //״̬ת����������Q_k
  matrix_s* p_process_noise_covariance;
  
  //�۲���������R_k
  matrix_s* p_observation_noise_covariance;

  //�۲�ֵz_k��ÿһ�����û��޸�
  matrix_s* p_observation;
  
  //Ԥ��״̬x-hat_k|k-1��ÿһ�����˲�������
  matrix_s* p_predicted_state;
  
  //Ԥ�����Э����P_k|k-1
  matrix_s* p_predicted_estimate_covariance;
  
  //��Ϣ��innovation��y-tilde_k��
  matrix_s* p_innovation;
  
  //��ϢЭ���S_k��
  matrix_s* p_innovation_covariance;
  
  //��ϢЭ���������S_k^-1��
  matrix_s* p_inverse_innovation_covariance;
  
  // ���ſ���������(K_k)
  matrix_s* p_optimal_gain;
  
  //״̬���ƣ�x-hat_k|k��
  matrix_s* p_state_estimate;
  
  //����Э���P_k|k��
  matrix_s* p_estimate_covariance;

  //�洢�м������
  matrix_s* p_vertical_scratch;
  
  matrix_s* p_small_square_scratch;
  
  matrix_s* p_big_square_scratch;
  
} kalman_filter_s;


kalman_filter_s* kalman_create(int state_dimension,int observation_dimension);
void kalman_destroy(kalman_filter_s* p_filter);
BOOL kalman_update(kalman_filter_s* p_filter);


#endif /* KALMAN_H_ */

