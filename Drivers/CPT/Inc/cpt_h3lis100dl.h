/*
 * cpt_h3lis100dl.h
 *
 *  Created on: 02-Dec-2019
 *      Author: GTsilicon
 */

#ifndef DRIVERS_CPT_INC_CPT_H3LIS100DL_H_
#define DRIVERS_CPT_INC_CPT_H3LIS100DL_H_

#include "stdint.h"

#ifdef H3LIS100DL_I2C
    #include "nrf_drv_twi.h"
#endif



/* Pins to connect H3LIS100DL.
 */
#define H3LIS100DL_I2C_SCL_PIN	14
#define H3LIS100DL_I2C_SDA_PIN	15


#define H3LIS100DL_I2C_BUFFER_SIZE     	6 // 12 byte buffers will suffice to read accelerometer.
#define H3LIS100DL_I2C_TIMEOUT 			100000
#define H3LIS100DL_ADDRESS     			0x18
#define H3LIS100DL_WHO_AM_I             0x0F



//
#define TWI_INSTANCE_ID     1


//Functions
static uint32_t cpt_h3lis100dl_Init(uint8_t sclPin, uint8_t sdaPin);

/**
 * @brief Function for Writing into the accelerometer register.
 *
 * @param[in] reg      Address of the register is stored here.
 * @param[in] data     The value of the register is stored in the data variable
 *
 * @retval NRF_SUCCESS             If data write was successful.
 */
extern uint32_t cpt_h3lis100dl_WriteSingleRegister(uint8_t reg, uint8_t data);

/**
 * @brief Function for Writing into multiple accelerometer register.
 *
 * @param[in] reg      Address of the register is stored here.
 * @param[in] pData    The value of the register is stored in the data variable
 * @param[in] length   Data length of the input parameter data
 *
 * @retval NRF_SUCCESS             If transfer was successful.
 */

extern uint32_t cpt_h3lis100dl_WriteRegisters(uint8_t reg, uint8_t * pData, uint32_t length);


/**
 * @brief Function for reading accelerometer register
 *
 * @param[in] reg      	Address of the register is stored here.
 * @param[in] p_data    The value of the register is stored in the data variable
 * @param[in] length    Length of the data that will be stored n the
 *
 * @retval The value stored in the register whose address is the input
 * @retval NRF_SUCCESS             If transfer was successful.
 */

extern uint32_t cpt_h3lis100dl_ReadRegisters(uint8_t reg, uint8_t * pData, uint8_t length);

/**
 * @brief Function to switch on the accelerometer
 * @details This function
 * @param[in] scl    Pin at which the SCL of the I2C is connected.
 * @param[in] sda    Pin at which the SDA of the I2C is connected.
 * @param[in] odr    The output sampling rate.
 * Keep the output sampling rate at 10 Hz as we need only to study the interrupt
 */

extern uint32_t cpt_accelerometer_On(uint8_t scl,uint8_t sda,uint8_t odr);
/**
 * @brief Function to read the accelrometer interrupt  on GPIOs in interrupt pin1.
 *
 * @param[in] gpioIntPin    Pin at which the Interupt should be read.
 * @retval The gpio pin read
 *
 */
extern uint32_t cpt_gpio_AccInit1(uint8_t gpioIntPin);
/**
 * @brief Function to read the accelrometer interrupt on GPIOs in interrupt pin2.
 *
 * @param[in] gpioIntPin    Pin at which the Interupt should be read.
 * @retval The gpio pin read
 *
 */
extern uint32_t cpt_gpio_AccInit2(uint8_t gpioIntPin);

#endif /* DRIVERS_CPT_INC_CPT_H3LIS100DL_H_ */
