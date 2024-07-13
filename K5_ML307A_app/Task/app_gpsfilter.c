/*
 * app_gpsfilter.c
 *
 *  Created on: Jun 21, 2024
 *      Author: lvvv
 *
 * ����һ������gps�Ŀ������˲�����Ҫ����1440�ֽڵĶѿռ�
 * �����ʹ�õ�ʱ��Ҫ���鿼��MCU ram�ռ���Դ�Ƿ��㹻
 * 
 * ������CH582оƬ�������������ܵĻ�����ȫ����ʹ������㷨��
 * ����ǿ�����������,Ҫ������������ռ��ռ�òſ�ʹ�ø��㷨
 *
 * ͬʱ,��ʹ�ÿ������˲��㷨֮��,��λ��ĵȴ��ϱ���ʱ�䲻�˹���,10s~20s����,0~10s������Ҳ��,
 * ��������ʵ�ʲ���,�����г��зǳ���gps(��mode,2,60,0)�Ĺ���ģʽЧ������~
 */

#include "app_gpsfilter.h"

//debug����
#define GF_DEBUG


/*-------------------------ƽ��ֵ�㷨--------------------------*/
#if defined(GF_TYPE_AVG_ENABLE)

//ƽ���㷨
static gfInfo_t gfinfo[GF_INFO_MAX_SIZE];
static gf_avg_t gf_avg;
static uint8_t gfinfo_index = 0;

/**************************************************
@bref		ƽ��ֵ���˳�ʼ��
@param
@return
@note
**************************************************/
void gf_avg_init(void)
{
	tmos_memset(&gfinfo, 0, sizeof(gfinfo));
	tmos_memset(&gf_avg, 0, sizeof(gf_avg));
	gfinfo_index = 0;
}

/**************************************************
@bref		gps��Ϣ����
@param
@return     0:����ʧ��    1:����ɹ�
@note
**************************************************/
uint8_t gf_gpsinfo_enter(gpsinfo_s *gpsinfo)
{
	uint8_t i, cnt = 0, valid = 0;
	double lat_avg = 0.0;
	double lng_avg = 0.0;
	double speed_avg = 0.0;
	gfinfo[gfinfo_index].last_lag = gpsinfo->latitude;
	gfinfo[gfinfo_index].last_lng = gpsinfo->longtitude;
	gfinfo[gfinfo_index].last_speed = gpsinfo->speed;
	gfinfo[gfinfo_index].valid = 1;
#ifdef GF_DEBUG
	LogPrintf(DEBUG_ALL, "%s==>[%d]lag:%lf lng:%lf speed:%lf ", __FUNCTION__, gfinfo_index,
															gfinfo[gfinfo_index].last_lag,
															gfinfo[gfinfo_index].last_lng,
															gfinfo[gfinfo_index].last_speed);
#endif
	gfinfo_index = (gfinfo_index + 1) % GF_INFO_MAX_SIZE;
	
	for (i = 0; i < GF_INFO_MAX_SIZE; i++)
		if (gfinfo[i].valid) cnt++;
#ifdef GF_DEBUG
	LogPrintf(DEBUG_ALL, "%s==>cnt:%d", __FUNCTION__, cnt);
#endif

	if (cnt == GF_INFO_MAX_SIZE) valid = 1;
	
	if (valid == 0)
		return 0;
	else
	{
		for (i = 0; i < GF_INFO_MAX_SIZE; i++)
		{
			lat_avg = (lat_avg * i + gfinfo[i].last_lag) / (i + 1);
			lng_avg = (lng_avg * i + gfinfo[i].last_lng) / (i + 1);
			speed_avg = (speed_avg * i + gfinfo[i].last_speed) / (i + 1);
		}
		gf_avg.gf_lat_avg = lat_avg;
		gf_avg.gf_lng_avg = lng_avg;
		gf_avg.gf_speed_avg = speed_avg;
		gf_avg.flag = 1;
#ifdef GF_DEBUG
		LogPrintf(DEBUG_ALL, "%s==>avglag:%lf avglng:%lf avgspeed:%lf ", __FUNCTION__, 
																	gf_avg.gf_lat_avg,
																	gf_avg.gf_lng_avg,
																	gf_avg.gf_speed_avg);
#endif
		return 1;
	}
}

/**************************************************
@bref		��ȡ��ǰ�����gpsƽ��ֵ
@param
@return
@note
**************************************************/
gf_avg_t *gf_get_gpsinfo_avg(void)
{
#ifdef GF_DEBUG
	LogPrintf(DEBUG_ALL, "%s==>avg[%d] avglag:%lf avglng:%lf avgspeed:%lf ", __FUNCTION__, 
																		gf_avg.flag,
																		gf_avg.gf_lat_avg,
																		gf_avg.gf_lng_avg,
																		gf_avg.gf_speed_avg);
#endif
	if (gf_avg.flag)
		return &gf_avg;
	return NULL;
}

#endif

/*-------------------------�������˲��㷨--------------------------*/
#if defined(GF_TYPE_KALMAN_ENABLE)

//�������˲��㷨
static kalman_filter_s* gf_kalman_filter = NULL;

static void set_seconds_per_timestep(kalman_filter_s* p_filter,double seconds_per_timestep) 
{
	double unit_scaler = 0.001;
	p_filter->p_state_transition->data[0][2] = unit_scaler * seconds_per_timestep;
	p_filter->p_state_transition->data[1][3] = unit_scaler * seconds_per_timestep;
}

static kalman_filter_s* alloc_filter_velocity2d(double noise) 
{
	kalman_filter_s* p_filter = kalman_create(4, 2);
  	double pos = 0.000001;
	const double trillion = 1000.0 * 1000.0 * 1000.0 * 1000.0;
	
  	matrix_set_identity(p_filter->p_state_transition);
  	
  	set_seconds_per_timestep(p_filter, 1.0);
		 
	matrix_set(p_filter->p_observation_model,1.0, 0.0, 0.0, 0.0,0.0, 1.0, 0.0, 0.0);

  	matrix_set(p_filter->p_process_noise_covariance,pos, 0.0, 0.0, 0.0, 0.0, pos, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0);

	matrix_set(p_filter->p_observation_noise_covariance,pos * noise, 0.0, 0.0, pos * noise);

	matrix_set(p_filter->p_state_estimate, 0.0, 0.0, 0.0, 0.0);
	
	matrix_set_identity(p_filter->p_estimate_covariance);
	
	matrix_scale(p_filter->p_estimate_covariance, trillion);

	return p_filter;
}


static BOOL update_velocity2d(kalman_filter_s* p_filter,
									  double lat, 
									  double lon,
									  double seconds_since_last_timestep)
{
	set_seconds_per_timestep(p_filter, seconds_since_last_timestep);
	matrix_set(p_filter->p_observation, lat * 1000.0, lon * 1000.0);
	return kalman_update(p_filter);
}


static void get_lat_long(kalman_filter_s* p_filter, double* lat, double* lon)
{
   *lat = p_filter->p_state_estimate->data[0][0] / 1000.0;
   *lon = p_filter->p_state_estimate->data[1][0] / 1000.0;
}


/**************************************************
@bref		����gps�Ŀ������˲���
@param
@return
@note
**************************************************/
void gf_kalman_filter_create(double noise)
{
	if (gf_kalman_filter == NULL)
	{
		gf_kalman_filter = alloc_filter_velocity2d(noise);
		LogPrintf(DEBUG_ALL, "%s==>ok", __FUNCTION__);
	}
}

/**************************************************
@bref		����gps�Ŀ������˲���
@param
@return
@note
**************************************************/
void gf_kalman_filter_destroy(void)
{
	kalman_destroy(gf_kalman_filter);
	gf_kalman_filter = NULL;
	LogPrintf(DEBUG_ALL, "%s==>ok", __FUNCTION__);
	
}

/**************************************************
@bref		ִ��gps�������˲���Ԥ�������
@param
@return
@note
**************************************************/
BOOL gf_kalman_filter_update(double lat, double lon, double seconds_since_last_timestep)
{
	if (NULL == gf_kalman_filter)
	{
		return FALSE;
	}
	return update_velocity2d(gf_kalman_filter, lat, lon, seconds_since_last_timestep);
}

/**************************************************
@bref		��ȡgps�������˲���Ԥ��ľ���γ��
@param
@return
@note
**************************************************/
void gf_kalman_filter_read(double* lat, double* lon)
{
	if (NULL == gf_kalman_filter)
	{
		return;
	}
	get_lat_long(gf_kalman_filter, lat, lon);
}

#endif


 
