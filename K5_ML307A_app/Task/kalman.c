/*
 * kalman.h
 *
 *	Created on: Jun 21, 2024
 *		Author: lvvv
 *
 * 以下代码搭配matrix.c矩阵算法文件为完整卡尔曼滤波算法
 * 理论上可以用来运用在任何含有不确定信息的动态系统中,对
 * 系统下一步的走向做出有根据的预测。
 * 
 */


#include "kalman.h"
#include "stdlib.h"

static void kalman_predict(kalman_filter_s* p_filter);
static BOOL kalman_estimate(kalman_filter_s* p_filter);


/**************************************************
@bref		创建卡尔曼滤波器
@param
	state_dimension 状态转移矩阵
	observation_dimension 观测矩阵
@return
@note
使用卡尔曼滤波算法时必须先调用此函数创建空间
**************************************************/
kalman_filter_s* kalman_create(int state_dimension, int observation_dimension) 
{
	kalman_filter_s* p_filter = malloc(sizeof(kalman_filter_s));

	p_filter->timestep = 0;
	p_filter->state_dimension = state_dimension;
	p_filter->observation_dimension = observation_dimension;

	p_filter->p_state_transition = matrix_create(state_dimension,state_dimension);
	p_filter->p_observation_model = matrix_create(observation_dimension,state_dimension);
	p_filter->p_process_noise_covariance = matrix_create(state_dimension,state_dimension);
	p_filter->p_observation_noise_covariance = matrix_create(observation_dimension,observation_dimension);

	p_filter->p_observation = matrix_create(observation_dimension, 1);

	p_filter->p_predicted_state = matrix_create(state_dimension, 1);
	p_filter->p_predicted_estimate_covariance = matrix_create(state_dimension,state_dimension);
	p_filter->p_innovation = matrix_create(observation_dimension, 1);
	p_filter->p_innovation_covariance = matrix_create(observation_dimension,observation_dimension);
	p_filter->p_inverse_innovation_covariance = matrix_create(observation_dimension,observation_dimension);
	p_filter->p_optimal_gain = matrix_create(state_dimension,observation_dimension);
	p_filter->p_state_estimate = matrix_create(state_dimension, 1);
	p_filter->p_estimate_covariance = matrix_create(state_dimension,state_dimension);

	p_filter->p_vertical_scratch = matrix_create(state_dimension,observation_dimension);
	p_filter->p_small_square_scratch = matrix_create(observation_dimension,observation_dimension);
	p_filter->p_big_square_scratch = matrix_create(state_dimension,state_dimension);

	return p_filter;
}

/**************************************************
@bref		删除卡尔曼滤波器
@param
	p_filter 卡尔曼滤波器
@return
@note
使用完记得删除 CH582调用malloc函数永远成功 若无空间创建会导致复位
**************************************************/
void kalman_destroy(kalman_filter_s* p_filter) 
{ 
	matrix_destroy(p_filter->p_state_transition);
	matrix_destroy(p_filter->p_observation_model);
	matrix_destroy(p_filter->p_process_noise_covariance);
	matrix_destroy(p_filter->p_observation_noise_covariance);

	matrix_destroy(p_filter->p_observation);

	matrix_destroy(p_filter->p_predicted_state);
	matrix_destroy(p_filter->p_predicted_estimate_covariance);
	matrix_destroy(p_filter->p_innovation);
	matrix_destroy(p_filter->p_innovation_covariance);
	matrix_destroy(p_filter->p_inverse_innovation_covariance);
	matrix_destroy(p_filter->p_optimal_gain);
	matrix_destroy(p_filter->p_state_estimate);
	matrix_destroy(p_filter->p_estimate_covariance);

	matrix_destroy(p_filter->p_vertical_scratch);
	matrix_destroy(p_filter->p_small_square_scratch);
	matrix_destroy(p_filter->p_big_square_scratch);
}

/**************************************************
@bref		执行卡尔曼滤波预测
@param
	p_filter 卡尔曼滤波器
@return
	TRUE:执行成功 FALSE:执行失败
@note
运行卡尔曼滤波的预测和估计,在运行这一步之前要先设置观测值,并且要定义好以下变量:
.state_transition
.observation_model
.state_transition
.state_transition
也建议使用合理的猜想初始化以下变量:
.state_estimate
.estimate_covariance
**************************************************/
BOOL kalman_update(kalman_filter_s* p_filter)
{
	kalman_predict(p_filter);
	return kalman_estimate(p_filter);
}

/**************************************************
@bref		预测状态
@param
	p_filter 卡尔曼滤波器
@return
@note
**************************************************/
static void kalman_predict(kalman_filter_s* p_filter) 
{
	p_filter->timestep++;

	//预测状态
	matrix_multiply(*p_filter->p_state_transition, *p_filter->p_state_estimate,p_filter->p_predicted_state);

	//预测状态估计协方差
	matrix_multiply(*p_filter->p_state_transition, *p_filter->p_estimate_covariance,p_filter->p_big_square_scratch);
	matrix_multiply_by_transpose(*p_filter->p_big_square_scratch, *p_filter->p_state_transition,p_filter->p_predicted_estimate_covariance);
	matrix_add(*p_filter->p_predicted_estimate_covariance, *p_filter->p_process_noise_covariance,p_filter->p_predicted_estimate_covariance);
}

/**************************************************
@bref		估计状态
@param
	p_filter 卡尔曼滤波器
@return
@note
求新息协方差的逆矩阵可能会失败,再算一次吧..
**************************************************/
static BOOL kalman_estimate(kalman_filter_s* p_filter)
{
	//计算新息（innovation）
	matrix_multiply(*p_filter->p_observation_model, *p_filter->p_predicted_state, p_filter->p_innovation);
	matrix_subtraction(*p_filter->p_observation, *p_filter->p_innovation, p_filter->p_innovation);

	//计算新息（innovation）协方差
	matrix_multiply_by_transpose(*p_filter->p_predicted_estimate_covariance, *p_filter->p_observation_model, p_filter->p_vertical_scratch);
	matrix_multiply(*p_filter->p_observation_model, *p_filter->p_vertical_scratch, p_filter->p_innovation_covariance);
	matrix_add(*p_filter->p_innovation_covariance, *p_filter->p_observation_noise_covariance, p_filter->p_innovation_covariance);

	//求新息（innovation）协方差的逆矩阵，可能不可逆
	if(FALSE == matrix_destructive_invert(*p_filter->p_innovation_covariance, p_filter->p_inverse_innovation_covariance))
	{
		LogPrintf(DEBUG_ALL,"Failed to kalman_estimate!");
		matrix_print(*p_filter->p_innovation_covariance);
		return FALSE;
	}

	//计算最优卡尔曼增益
	matrix_multiply(*p_filter->p_vertical_scratch, *p_filter->p_inverse_innovation_covariance, p_filter->p_optimal_gain);

	//估计状态
	matrix_multiply(*p_filter->p_optimal_gain, *p_filter->p_innovation, p_filter->p_state_estimate);
	matrix_add(*p_filter->p_state_estimate, *p_filter->p_predicted_state, p_filter->p_state_estimate);

	//估计状态协方差
	matrix_multiply(*p_filter->p_optimal_gain, *p_filter->p_observation_model, p_filter->p_big_square_scratch);
	matrix_subtract_from_identity(p_filter->p_big_square_scratch);
	matrix_multiply(*p_filter->p_big_square_scratch, *p_filter->p_predicted_estimate_covariance, p_filter->p_estimate_covariance);

	return TRUE;
}

