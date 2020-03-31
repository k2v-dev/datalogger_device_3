/*
 * bsp_LSM9DSmag.h
 *
 *  Created on: 3-DEC-2019
 *      Author: Inertial Elements
 */


#ifndef DRIVERS_BSP_INC_BSP_LSM9DSMAG_H_
#define DRIVERS_BSP_INC_BSP_LSM9DSMAG_H_


/**@brief Structure to hold magnetometer values.
 * Sequence of z, y, and x is important to correspond with
 * the sequence of which z, y, and x data are read from the sensor.
 * All values are unsigned 16 bit integers
*/

typedef struct
{
    int16_t mx;
    int16_t my;
    int16_t mz;
}mag_values_t;


void bsp_lsm9dsMAG_SwitchonMAG(uint8_t odr1);

void bsp_lsm9dsMAG_Switchon_SCALE(uint8_t gauss);




void bsp_lsm9dsMAG_SwitchoffAG(void);


void bsp_lsm9dsMAG_Switchon_zaxis(void);

/*@Brief This function read the magnetometer  values from the sensor
 *param[out]mag_values  It will store the values  of all the axis i.e x,y and z
 *in the pointer specified here
*/

extern uint32_t  bsp_lsm9dsmag_ReadMagData(mag_values_t * mag_values);




#endif /* DRIVERS_BSP_INC_BSP_LSM9DSMAG_H_ */

