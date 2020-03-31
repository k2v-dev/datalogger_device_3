/*
 * True_Angle_Calculation.h
 *
 *  Created on: 18-Dec-2019
 *      Author: GTsilicon
 */

#ifndef APPLICATIONS_INC_FORMAT_SENSOR_DATA_H_
#define APPLICATIONS_INC_FORMAT_SENSOR_DATA_H_

#include <stdint.h>
#include <string.h>



extern float acx,acy,acz,grx,gry,grz,u_mx,u_my,u_mz;
void scale_conversion(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz, int16_t mx, int16_t my, int16_t mz);
void format_sensor_data(void);


#endif /* APPLICATIONS_INC_FORMAT_SENSOR_DATA_H_ */
