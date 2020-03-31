/*
 * cpt
 * Created on: 3 dec 2019
 * Author: Inertial Elements
 * @file	cpt_lsm9ds1.h
 * @brief	This file drives the LSM9DS1 acccelerometer and gyroscope
 * @defgroup	Changes in the LSM9DS1 cpt files for nrf52 controller & driver
 * @{
 */

#ifndef CPT_LSM9DS1_H_
#define CPT_LSM9DS1_H_

#include "stdint.h"

#ifdef LSM9DS1_I2C
    #include "nrf_drv_twi.h"
#endif



/* Pins to connect LSM9DS1.
 */
#define LSM9DS1_I2C_SCL_PIN 27
#define LSM9DS1_I2C_SDA_PIN 26


#define LSM9DS1_I2C_BUFFER_SIZE     6 // 12 byte buffers will suffice to read accelerometer and gyroscope .
#define LSM9DS1_I2C_TIMEOUT         100000

//
#define TWI_INSTANCE_ID     0


//Functions

/**
 * @brief Function for Writing into the accelerometer and gyroscope register.
 *
 * @param[in] reg      Address of the register is stored here.
 * @param[in] data     The value of the register is stored in the data variable
 *
 * @retval NRF_SUCCESS             If data write was successful.
 */

extern uint32_t cpt_lsm9ds1_WriteSingleRegister(uint8_t reg, uint8_t data);

/**
 * @brief Function for Writing into multiple accelerometer and gyroscope register.
 *
 * @param[in] reg      Address of the register is stored here.
 * @param[in] pData    The value of the register is stored in the data variable
 * @param[in] length   Data length of the input parameter data
 *
 * @retval NRF_SUCCESS             If transfer was successful.
 */

extern uint32_t cpt_lsm9ds1_WriteRegisters(uint8_t reg, uint8_t * pData, uint32_t length);


/**
 * @brief Function for reading accelerometer and gyroscope register
 *
 * @param[in] reg      	Address of the register is stored here.
 * @param[in] p_data    The value of the register is stored in the data variable
 * @param[in] length    Length of the data that will be stored n the
 *
 * @retval The value stored in the register whose address is the input
 * @retval NRF_SUCCESS             If transfer was successful.
 */

extern uint32_t cpt_lsm9ds1_ReadRegisters(uint8_t reg, uint8_t * pData, uint8_t length);

/**
 * @brief Function to switch on the accelerometer and gyroscope
 * @details This function
 * @param[in] scl    Pin at which the SCL of the I2C is connected.
 * @param[in] sda    Pin at which the SDA of the I2C is connected.
 * @param[in] odr    The output sampling rate.
 * Keep the output sampling rate at 119 Hz
 */

extern uint32_t cpt_LSM9DS1_IMU_AG_On(uint8_t scl,uint8_t sda,uint8_t odr);

#endif /* CPT_LSM9DS1_H_ */


