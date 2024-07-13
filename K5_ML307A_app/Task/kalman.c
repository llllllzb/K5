/*
 * kalman.h
 *
 *	Created on: Jun 21, 2024
 *		Author: lvvv
 *
 * ���´������matrix.c�����㷨�ļ�Ϊ�����������˲��㷨
 * �����Ͽ��������������κκ��в�ȷ����Ϣ�Ķ�̬ϵͳ��,��
 * ϵͳ��һ�������������и��ݵ�Ԥ�⡣
 * 
 */


#include "kalman.h"
#include "stdlib.h"

static void kalman_predict(kalman_filter_s* p_filter);
static BOOL kalman_estimate(kalman_filter_s* p_filter);


/**************************************************
@bref		�����������˲���
@param
	state_dimension ״̬ת�ƾ���
	observation_dimension �۲����
@return
@note
ʹ�ÿ������˲��㷨ʱ�����ȵ��ô˺��������ռ�
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
@bref		ɾ���������˲���
@param
	p_filter �������˲���
@return
@note
ʹ����ǵ�ɾ�� CH582����malloc������Զ�ɹ� ���޿ռ䴴���ᵼ�¸�λ
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
@bref		ִ�п������˲�Ԥ��
@param
	p_filter �������˲���
@return
	TRUE:ִ�гɹ� FALSE:ִ��ʧ��
@note
���п������˲���Ԥ��͹���,��������һ��֮ǰҪ�����ù۲�ֵ,����Ҫ��������±���:
.state_transition
.observation_model
.state_transition
.state_transition
Ҳ����ʹ�ú���Ĳ����ʼ�����±���:
.state_estimate
.estimate_covariance
**************************************************/
BOOL kalman_update(kalman_filter_s* p_filter)
{
	kalman_predict(p_filter);
	return kalman_estimate(p_filter);
}

/**************************************************
@bref		Ԥ��״̬
@param
	p_filter �������˲���
@return
@note
**************************************************/
static void kalman_predict(kalman_filter_s* p_filter) 
{
	p_filter->timestep++;

	//Ԥ��״̬
	matrix_multiply(*p_filter->p_state_transition, *p_filter->p_state_estimate,p_filter->p_predicted_state);

	//Ԥ��״̬����Э����
	matrix_multiply(*p_filter->p_state_transition, *p_filter->p_estimate_covariance,p_filter->p_big_square_scratch);
	matrix_multiply_by_transpose(*p_filter->p_big_square_scratch, *p_filter->p_state_transition,p_filter->p_predicted_estimate_covariance);
	matrix_add(*p_filter->p_predicted_estimate_covariance, *p_filter->p_process_noise_covariance,p_filter->p_predicted_estimate_covariance);
}

/**************************************************
@bref		����״̬
@param
	p_filter �������˲���
@return
@note
����ϢЭ������������ܻ�ʧ��,����һ�ΰ�..
**************************************************/
static BOOL kalman_estimate(kalman_filter_s* p_filter)
{
	//������Ϣ��innovation��
	matrix_multiply(*p_filter->p_observation_model, *p_filter->p_predicted_state, p_filter->p_innovation);
	matrix_subtraction(*p_filter->p_observation, *p_filter->p_innovation, p_filter->p_innovation);

	//������Ϣ��innovation��Э����
	matrix_multiply_by_transpose(*p_filter->p_predicted_estimate_covariance, *p_filter->p_observation_model, p_filter->p_vertical_scratch);
	matrix_multiply(*p_filter->p_observation_model, *p_filter->p_vertical_scratch, p_filter->p_innovation_covariance);
	matrix_add(*p_filter->p_innovation_covariance, *p_filter->p_observation_noise_covariance, p_filter->p_innovation_covariance);

	//����Ϣ��innovation��Э���������󣬿��ܲ�����
	if(FALSE == matrix_destructive_invert(*p_filter->p_innovation_covariance, p_filter->p_inverse_innovation_covariance))
	{
		LogPrintf(DEBUG_ALL,"Failed to kalman_estimate!");
		matrix_print(*p_filter->p_innovation_covariance);
		return FALSE;
	}

	//�������ſ���������
	matrix_multiply(*p_filter->p_vertical_scratch, *p_filter->p_inverse_innovation_covariance, p_filter->p_optimal_gain);

	//����״̬
	matrix_multiply(*p_filter->p_optimal_gain, *p_filter->p_innovation, p_filter->p_state_estimate);
	matrix_add(*p_filter->p_state_estimate, *p_filter->p_predicted_state, p_filter->p_state_estimate);

	//����״̬Э����
	matrix_multiply(*p_filter->p_optimal_gain, *p_filter->p_observation_model, p_filter->p_big_square_scratch);
	matrix_subtract_from_identity(p_filter->p_big_square_scratch);
	matrix_multiply(*p_filter->p_big_square_scratch, *p_filter->p_predicted_estimate_covariance, p_filter->p_estimate_covariance);

	return TRUE;
}

