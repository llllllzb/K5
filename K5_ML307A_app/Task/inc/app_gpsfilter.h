/*
 * app_gpsfilter.h
 *
 *  Created on: Jun 21, 2024
 *      Author: lvvv
 */

#ifndef TASK_INC_APP_GPSFILTER_H_
#define TASK_INC_APP_GPSFILTER_H_

#include "app_gps.h"
#include "config.h"
#include "app_sys.h"
#include "kalman.h"
#include "matrix.h"


#define GF_INFO_MAX_SIZE	10

#define GF_TYPE_AVG_ENABLE
#define GF_TYPE_KALMAN_ENABLE

#if defined(GF_TYPE_AVG_ENABLE)
typedef struct
{
	uint8_t valid;
	double last_lag;
	double last_lng;
	double last_speed;
}gfInfo_t;

typedef struct
{
	uint8_t flag;
	double gf_lat_avg;
	double gf_lng_avg;
	double gf_speed_avg;
}gf_avg_t;

void gf_avg_init(void);
uint8_t gf_gpsinfo_enter(gpsinfo_s *gpsinfo);
gf_avg_t *gf_get_gpsinfo_avg(void);

#endif

#if defined(GF_TYPE_KALMAN_ENABLE)

void gf_kalman_filter_create(double noise);
void gf_kalman_filter_destroy(void);
BOOL gf_kalman_filter_update(double lat, double lon, double seconds_since_last_timestep);
void gf_kalman_filter_read(double* lat, double* lon);

#endif

#endif /* TASK_INC_APP_GPSFILTER_H_ */
