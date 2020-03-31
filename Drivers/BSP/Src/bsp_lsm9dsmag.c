/*
 *
 * bsp_lsm9ds1.c
 *
 *  Created on: 03-Dec-2019
 *
 *
 *      Author: Inertial Elements
 *
 *      @brief This file contains all the useful functions for reading setting the parameters of
 *      the accelerometer like sampling frequency, setting sda, scl pins, setting of the interrupt
 *      etc. Apart from setting the parameters it also contains function for reading accelerometer
 *      data.
 */


#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include "cpt_lsm9dsmag.h"
#include "bsp_lsm9dsmag.h"
#include "app_util_platform.h"
#include "cpt_lsm9ds1_reg.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
//#include "nrf52.h"



#define sixbytes 6
#define singlebyte 1



#define BURST_MODE2 0x0C

#define SET_IN
#define ACC_RAW_BUFFER_SIZE 1000


/**Switch on the magnetometer
 *
 */



void bsp_lsm9dsMAG_SwitchonMAG(uint8_t odr1)
{
	// The SCL and the SDA pins are set and the Output data  rate is set at 80 Hz
	//uint32_t errCode;
	 cpt_lsm9dsmag_IMU_mag_On(LSM9DS1_I2C_SCL_PIN, LSM9DS1_I2C_SDA_PIN, odr1); //
//	if(errCode != NRF_SUCCESS) return errCode;
	//Set MAGNETOmeter  range at 16g required for activity detection.
	//cpt_lsm9ds1_WriteSingleRegister()


	 cpt_lsm9dsmag_WriteSingleRegister(CTRL_REG3_M, 0x00); // 0X0C I2C ENABLE

}

void bsp_lsm9dsMAG_SwitchoffAG(void)
{
	cpt_lsm9dsmag_WriteSingleRegister(CTRL_REG9,BURST_MODE2); //0x0C is for burst mode reading,  for i2c a/g disable scale
}

void bsp_lsm9dsMAG_Switchon_SCALE(uint8_t gauss)
{
	cpt_lsm9dsmag_WriteSingleRegister(CTRL_REG2_M, gauss);  //  0x60,scale 16 gauss
}



void bsp_lsm9dsMAG_Switchon_zaxis(void)
{
	cpt_lsm9dsmag_WriteSingleRegister(CTRL_REG4_M, 0x08);  //0x08, Z axis enable
}

/**Read MAG Data
 *
 */


uint32_t  bsp_lsm9dsmag_ReadMagData(mag_values_t * mag_values)

  {

    uint32_t err_code1;
    uint8_t raw_values[6];


    uint8_t *data3;
    data3 = (uint8_t*)mag_values;
    for(uint8_t i = 0; i<6; i++)
    {

    	 err_code1 = cpt_lsm9dsmag_ReadRegisters(OUT_X_L_M+i, raw_values, 6); //0x28 read output data
        *data3 = raw_values[0];
        data3++;
    }
    return NRF_SUCCESS;
}




