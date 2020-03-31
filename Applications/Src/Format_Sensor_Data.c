/*
 * True_Angle_Calculation.c
 *
 *  Created on: 18-Dec-2019
 *  Author: GTsilicon
 *  @Brief: Calculation of the attitude angle with respect to the true north
 */

#include <math.h>
#include <stdint.h>
#include <string.h>
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "Command_Interface.h"

float acx,acy,acz,grx,gry,grz,mgx,mgy,mgz;
float max_mx,max_my,max_mz,min_mx,min_my,min_mz, u_mx,u_my,u_mz;


/**@brief Conversion of the Accelerometer values to "g" , Gyroscope values to "rad/s" and Magnetometr values to miliGauss
 */

void scale_conversion(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz, int16_t mx, int16_t my, int16_t mz)
{
	 acx= ((float)ax *0.732)/1000;
	 acy= ((float)ay *0.732)/1000;
	 acz= ((float)az *0.732)/1000;
	 grx= ((float)gx *70*1.74533e-5);
	 gry= ((float)gy *70*1.74533e-5);
	 grz= ((float)gz *70*1.74533e-5);
	 mgx= ((float)mx*0.58);
	 mgy= ((float)my*0.58);
	 mgz= ((float)mz*0.58);
 //	 NRF_LOG_INFO("Ax " NRF_LOG_FLOAT_MARKER "\r\n", NRF_LOG_FLOAT(acx));
//	 NRF_LOG_INFO("Ay " NRF_LOG_FLOAT_MARKER "\r\n", NRF_LOG_FLOAT(acy));
//	 NRF_LOG_INFO("Az " NRF_LOG_FLOAT_MARKER "\r\n", NRF_LOG_FLOAT(acz));
//	 NRF_LOG_INFO("Gx " NRF_LOG_FLOAT_MARKER "\r\n", NRF_LOG_FLOAT(grx));
//	 NRF_LOG_INFO("Gy " NRF_LOG_FLOAT_MARKER "\r\n", NRF_LOG_FLOAT(gry));
//	 NRF_LOG_INFO("Gz " NRF_LOG_FLOAT_MARKER "\r\n", NRF_LOG_FLOAT(grz));
//	 NRF_LOG_INFO("Mx " NRF_LOG_FLOAT_MARKER "\r\n", NRF_LOG_FLOAT(mgx));
//	 NRF_LOG_INFO("My " NRF_LOG_FLOAT_MARKER "\r\n", NRF_LOG_FLOAT(mgy));
//	 NRF_LOG_INFO("Mz " NRF_LOG_FLOAT_MARKER "\r\n", NRF_LOG_FLOAT(mgz));

}


void hard_iron_correction(void)
{
	if(max_mx<mgx)
		max_mx=mgx;
	if(max_my<mgy)
		max_my=mgy;
	if(max_mz<mgz)
		max_mz=mgz;

	if(min_mx>mgx)
		min_mx=mgx;
	if(min_my>mgy)
		min_my=mgy;
	if(min_mz>mgz)
		min_mz=mgz;

	u_mx=mgx -((max_mx)+(min_mx))/2;
	u_my=mgy -((max_my)+(min_my))/2;
	u_mz=mgz -((max_mz)+(min_mz))/2;

}

/**@brief Calculation of quaternions
 */

void format_sensor_data(void)
{
	read_IMU();
	read_MAG();
	//Convert to appropriate scale needed for the Madgwick's filetr calculations
	scale_conversion(accl_values.x,accl_values.y,accl_values.z,gyro_value.ax,gyro_value.ay,gyro_value.az,mag_values.mx,mag_values.my,mag_values.mz);
	// Hard Iron correction of the magnetomter
	hard_iron_correction();
}


