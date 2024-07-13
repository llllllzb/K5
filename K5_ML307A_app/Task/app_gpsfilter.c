/*
 * app_gpsfilter.c
 *
 *  Created on: Jun 21, 2024
 *      Author: lvvv
 *
 * 创建一个用于gps的卡尔曼滤波器需要消耗1440字节的堆空间
 * 因此在使用的时候要酌情考虑MCU ram空间资源是否足够
 * 
 * 理论上CH582芯片不开启蓝牙功能的话是完全可以使用这个算法的
 * 如果是开启蓝牙功能,要酌情减少其他空间的占用才可使用该算法
 *
 * 同时,在使用卡尔曼滤波算法之后,定位后的等待上报的时间不宜过程,10s~20s即可,0~10s理论上也可,
 * 具体自行实际测试,用于行车中非常开gps(如mode,2,60,0)的工作模式效果显著~
 */

#include "app_gpsfilter.h"

//debug开关
#define GF_DEBUG


/*-------------------------平均值算法--------------------------*/
#if defined(GF_TYPE_AVG_ENABLE)

//平均算法
static gfInfo_t gfinfo[GF_INFO_MAX_SIZE];
static gf_avg_t gf_avg;
static uint8_t gfinfo_index = 0;

/**************************************************
@bref		平均值过滤初始化
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
@bref		gps信息载入
@param
@return     0:计算失败    1:计算成功
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
@bref		获取当前计算的gps平均值
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

/*-------------------------卡尔曼滤波算法--------------------------*/
#if defined(GF_TYPE_KALMAN_ENABLE)

//卡尔曼滤波算法
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
@bref		创建gps的卡尔曼滤波器
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
@bref		创建gps的卡尔曼滤波器
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
@bref		执行gps卡尔曼滤波器预测与估计
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
@bref		读取gps卡尔曼滤波器预测的经度纬度
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


 
