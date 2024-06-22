/*
 * app_gpsfilter.c
 *
 *  Created on: Jun 21, 2024
 *      Author: lvvv
 */

#include "app_gpsfilter.h"

//debug开关
#define GF_DEBUG

static gfInfo_t gfinfo[GF_INFO_MAX_SIZE];
static gf_avg_t gf_avg;
static uint8_t gfinfo_index = 0;

/*-------------------------平均值算法--------------------------*/

/**************************************************
@bref		平均值过滤初始化
@param
@return
@note
**************************************************/

void gf_init(void)
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














 
