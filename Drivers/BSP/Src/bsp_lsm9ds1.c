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
 *      etc. Apart from setting the parameters it also contains function for reading accelerometer and gyroscope
 *      data.
 */


#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include "cpt_lsm9ds1.h"
#include "bsp_lsm9ds1.h"
#include "app_util_platform.h"
#include "cpt_lsm9ds1_reg.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
//#include "nrf52.h"



#define sixbytes 6
#define singlebyte 1

//#define FIFO_SAMPLES 0x20  //FIFO MODE
#define FIFO_SAMPLES 0xC0   //CONTINOUS MODE
#define BURST_MODE 0x08

#define SET_IN
#define ACC_RAW_BUFFER_SIZE 1000

static uint8_t sAccBuffer[ACC_RAW_BUFFER_SIZE];
static uint16_t bytes;


/**Switch on the Accelerometer
 *
 */


void bsp_lsm9ds1_SwitchonAccelo(uint8_t odr)
{
	// The SCL and the SDA pins are set and the Output data  rate is set at 10 Hz
	//uint32_t errCode;
	cpt_LSM9DS1_IMU_AG_On(LSM9DS1_I2C_SCL_PIN, LSM9DS1_I2C_SDA_PIN, odr);
//	if(errCode != NRF_SUCCESS) return errCode;
	//Set the accelerometer  range at 16g required for activity detection.
	//cpt_lsm9ds1_WriteSingleRegister()
	cpt_lsm9ds1_WriteSingleRegister(CTRL_REG9,BURST_MODE);//0x08   is for burst mode reading,  i2c enable
 // cpt_lsm9dsmag_WriteSingleRegister(CTRL_REG3_M, 0x80); // 0X80 I2C mag disable
}

void bsp_lsm9ds1_SwitchonAcc(void)
{
	cpt_lsm9ds1_WriteSingleRegister(CTRL_REG5_XL, 0x38);  // oxB8 accel output register x,y,z enable
	cpt_lsm9ds1_WriteSingleRegister(CTRL_REG7_XL,0x80);


}


void bsp_lsm9ds1_SwitchonGyro(void)
{
	cpt_lsm9ds1_WriteSingleRegister(CTRL_REG4, 0x38);    // 0x38 gyro output register x,y,z enable

}

/**Read Accelerometer Data
 *
 */

//accel_values_t sampl;
uint32_t bsp_lsm9ds1_ReadAcceloData(accel_values_t * accel_values)
{

    uint32_t errCode;
    uint8_t rawValues[sixbytes];

    // Reorganize read sensor values and put them into value struct
    uint8_t *data;
    data = (uint8_t*)accel_values;
    for(uint8_t i = 0; i<sixbytes; i++)
    {
    	errCode = cpt_lsm9ds1_ReadRegisters(OUT_X_L_XL+i,rawValues,sixbytes);
    	//if(errCode != NRF_SUCCESS) return errCode;
    	*data = rawValues[0];
        data++;
    }
  return errCode;
}

float lsm9ds1_from_fs100g_to_mg(int16_t lsb)
{
  return ( (float)lsb / 256.0f ) * 780.0f;
}


/**Read gyroscope Data
 *
 */

//gyro_values_t sampl;
uint32_t bsp_lsm9ds1_ReadGyroData(gyro_values_t * gyro_values)
{
    uint32_t err_code;
    uint8_t raw_values[6];



    // Reorganize read sensor values and put them into value struct
    uint8_t *data;
    data = (uint8_t*)gyro_values;
    for(uint8_t i = 0; i<6; i++)
    {
    	 err_code =cpt_lsm9ds1_ReadRegisters(OUT_X_L_G+i, raw_values, 6);
    	 *data = raw_values[0];
        data++;
    }
  //  NRF_LOG_INFO("REGISTER2");


    //  Measurement data starts at DATAXL, and ends at DATAZ1, 6 bytes long
//    sampl.ax = (data1[1] << 8) | data1[0];          // Read 6 bytes at once
//    sampl.ay = (data1[3] << 8) | data1[2];
//   sampl.az = (data1[5] << 8) | data1[4];
 //  NRF_LOG_INFO("Sample # %d\r\nGX: %06d\r\nGY: %06d\r\nGZ: %06d ",(~sampl.ax) ,(~sampl.ay), (~sampl.az));

    return err_code;
}







