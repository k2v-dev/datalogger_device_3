/*
 * cpt_h3lis100dl.c
 *
 *  Created on: 02-Dec-2019
 *      Author: GTsilicon
 */


#include "stdlib.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "cpt_h3lis100dl.h"
#include "cpt_h3lis100dl_cfg.h"
#include "hal_drv_twi.h"
#include "app_util_platform.h"
#include "nrfx_gpiote.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_drv_gpiote.h"
#include "nrf_gpio.h"



static const nrf_drv_twi_t m_I2cInstance = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);
volatile static bool g_I2cTxDone = false;
volatile static bool g_I2cRxDone = false;

int add;

/**Takes care of the nack bit which comes along with the data in case the driver function
 * fails also takes care of the of different transfer flags once the transfer has taken place
 */

static void cpt_h3lis100dl_I2cEventHandler(nrf_drv_twi_evt_t const * pEvent, void * pContext)
{
    switch(pEvent->type)
    {
        case NRF_DRV_TWI_EVT_DONE:
            switch(pEvent->xfer_desc.type)
            {
                case NRF_DRV_TWI_XFER_TX:
                    g_I2cTxDone = true;
                  //  NRF_LOG_INFO("TWI device detected at address 0x%x.",add);
                    break;
                case NRF_DRV_TWI_XFER_TXTX:
                	g_I2cTxDone = true;
                    break;
                case NRF_DRV_TWI_XFER_RX:
                	g_I2cRxDone = true;
                    break;
                case NRF_DRV_TWI_XFER_TXRX:
                	g_I2cRxDone = true;
                    break;
                default:
                    break;
            }
            break;
        case NRF_DRV_TWI_EVT_ADDRESS_NACK:
        	NRF_LOG_INFO("nACK MISSING");
            break;
        case NRF_DRV_TWI_EVT_DATA_NACK:
        	NRF_LOG_INFO("data nACK MISSING");
            break;
        default:
            break;
    }
}

/**H3LIS100DL_I2C_SCL_PIN
 * @brief TWI initialization.
 * Just the usual way. Nothing special here. The TWI0 functionality has been utilized for instantiate.
 */
static uint32_t cpt_h3lis100dl_Init(uint8_t sclPin, uint8_t sdaPin)
{
    uint32_t errCode=0;

    const nrf_drv_twi_config_t twi_H3LIS100DL_config = {
       .scl                = sclPin,
       .sda                = sdaPin,
       .frequency          = NRF_TWI_FREQ_400K, // 400 kHz I2C frequency
       .interrupt_priority = APP_IRQ_PRIORITY_HIGHEST,// Priority is set at high not highest
       .clear_bus_init     = false
    };

    errCode = nrf_drv_twi_init(&m_I2cInstance , &twi_H3LIS100DL_config, cpt_h3lis100dl_I2cEventHandler, NULL);
    if(errCode != NRF_SUCCESS)
	{
		return errCode;
	}
    nrf_drv_twi_enable(&m_I2cInstance);

	return NRF_SUCCESS;
}

/**
 * @brief The TWI driver is not able to do two transmits without repeating the ADDRESS + Write bit byte
 *  Hence we need to merge the H3LIS100DL register address with the buffer and then transmit all as one transmission
 **/
static void cpt_h3lis100dl_MergeRegisterAndData(uint8_t * newBuffer, uint8_t reg, uint8_t * pData, uint32_t length)
{
    newBuffer[0] = reg;
    memcpy((newBuffer + 1), pData, length);
}


uint32_t cpt_h3lis100dl_WriteRegisters(uint8_t reg, uint8_t * pData, uint32_t length)
{
    // This burst write function is not optimal and needs improvement.
    // The SDK 12.03 TWI driver is not able to do two transmits without repeating the ADDRESS + Write bit byte

    uint32_t errCode=0;
    uint32_t timeout = H3LIS100DL_I2C_TIMEOUT;
    uint8_t i2cTxBuffer[H3LIS100DL_I2C_BUFFER_SIZE];

    // Merging H3LIS100DL register address and p_data into one buffer.

    cpt_h3lis100dl_MergeRegisterAndData(i2cTxBuffer, reg, pData, length);

    // Setting up transfer

    nrf_drv_twi_xfer_desc_t xfer_desc; // Setting Up the transfer function with Slave Address
    xfer_desc.address = H3LIS100DL_ADDRESS;
    xfer_desc.type = NRF_DRV_TWI_XFER_TX;
    xfer_desc.primary_length = length + 1;
    xfer_desc.p_primary_buf = i2cTxBuffer;

    /// Transferring

    errCode = nrf_drv_twi_xfer(&m_I2cInstance, &xfer_desc, 0);

    while((!g_I2cTxDone) && --timeout);
    if(!timeout) return NRF_ERROR_TIMEOUT;
    g_I2cTxDone = false;

    return errCode;
}

uint32_t cpt_h3lis100dl_WriteSingleRegister(uint8_t reg, uint8_t data)
{
    uint32_t errCode=0;
    uint32_t timeout = H3LIS100DL_I2C_TIMEOUT;

    uint8_t packet[2] = {reg, data};

    errCode = nrf_drv_twi_tx(&m_I2cInstance, H3LIS100DL_ADDRESS, packet, 2, false);




    if(errCode != NRF_SUCCESS) return errCode;
    NRF_LOG_INFO("Write1: %06i ",errCode);
    while((!g_I2cTxDone) && --timeout);
    if(!timeout) return NRF_ERROR_TIMEOUT;

    g_I2cTxDone = false;
//    NRF_LOG_INFO("err1: %06i ",errCode);
    return errCode;
}

/*
 * @brief This function is to read from multiple registers.
 */
uint32_t cpt_h3lis100dl_ReadRegisters(uint8_t reg, uint8_t * pData, uint8_t length)
{
    uint32_t errCode=0;
    uint32_t timeout = H3LIS100DL_I2C_TIMEOUT;

    errCode = nrf_drv_twi_tx(&m_I2cInstance, H3LIS100DL_ADDRESS, &reg, 1, false);  if(errCode != NRF_SUCCESS) return errCode;
    while((!g_I2cTxDone) && --timeout);
    if(!timeout) return NRF_ERROR_TIMEOUT;
    g_I2cTxDone = false;

    //Read a byte data from the selected register

    errCode = nrf_drv_twi_rx(&m_I2cInstance, H3LIS100DL_ADDRESS, pData, length);
    if(errCode != NRF_SUCCESS) return errCode;

    timeout = H3LIS100DL_I2C_TIMEOUT;
    while((!g_I2cRxDone) && --timeout);
    if(!timeout) return NRF_ERROR_TIMEOUT;
    g_I2cRxDone = false;
 //   }
  // NRF_LOG_INFO("err2: %06i ",errCode);
    return errCode;
}

/**
 * @brief Switch on the accelerometer by setting the SCL and the SDA pin, and also defining the output sampling
 */
//frequency. This function will be called before setting other registers
uint32_t cpt_accelerometer_On(uint8_t scl,uint8_t sda,uint8_t odr)
{
    uint32_t errCode1=0;
    uint32_t errCode2=0;

	/// Initiate I2C driver dependent on what is defined in the project. In Rj100 project it should be 22,23

	errCode1 = cpt_h3lis100dl_Init(scl,sda);
    if(errCode1 != NRF_SUCCESS) return errCode1;


    uint8_t startBit = 0x27;
   // startBit |= odr;
   // startBit = 0x37;   // start bit
    errCode2 = cpt_h3lis100dl_WriteSingleRegister(CTRL1,startBit);
    NRF_LOG_INFO("eRR2 %d\n",errCode2);
    if(errCode2 != NRF_SUCCESS) return errCode2;
    NRF_LOG_INFO("acc_start");
    return NRF_SUCCESS;
}

uint32_t intcode=1;

static void in_pin_AccHandler1(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
	//TODO Based on Philips library we have define event handler if any
    switch(action)
    {
        case NRF_GPIOTE_POLARITY_LOTOHI:
            NRF_LOG_INFO("Interrupt_Detected--Pin1");
            break;
        default:
        	 //do nothing
            break;
    }
}
/**
 * @brief Initializing the GPIO settings for reading the GPIO interrupt from the accelerometer
 */
//uint32_t sample_number = 0;

uint32_t cpt_gpio_AccInit1(uint8_t gpioIntPin)
{
    ret_code_t errCode=0;
    if(!nrf_drv_gpiote_is_init())
    {
        errCode = nrf_drv_gpiote_init(); //Initializing the GPIO pin for reading the interrupt
    }

    APP_ERROR_CHECK(errCode);
    // Choose high Sense setting for reading the  interrupt

   nrf_drv_gpiote_in_config_t in_config =  	GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);
   	in_config.pull = NRF_GPIO_PIN_PULLDOWN;
    errCode = nrf_drv_gpiote_in_init(gpioIntPin, &in_config, in_pin_AccHandler1);// Config and event on initialization
    APP_ERROR_CHECK(errCode);
    nrf_drv_gpiote_in_event_enable(gpioIntPin, true);
    return errCode;
}



static void in_pin_AccHandler2(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
	//TODO Based on Philips library we have define event handler if any
    switch(action)
    {
        case NRF_GPIOTE_POLARITY_LOTOHI:
            NRF_LOG_INFO("Interrupt_Detected--Pin2");
            break;
        default:
        	 //do nothing
            break;
    }
}
/**
 * @brief Initializing the GPIO settings for reading the GPIO interrupt from the accelerometer
 */
//uint32_t sample_number = 0;

uint32_t cpt_gpio_AccInit2(uint8_t gpioIntPin)
{
    ret_code_t errCode=0;
    if(!nrf_drv_gpiote_is_init())
    {
        errCode = nrf_drv_gpiote_init(); //Initializing the GPIO pin for reading the interrupt
    }

    APP_ERROR_CHECK(errCode);
    // Choose high Sense setting for reading the  interrupt

   nrf_drv_gpiote_in_config_t in_config =  	GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);
   	in_config.pull = NRF_GPIO_PIN_PULLDOWN;
    errCode = nrf_drv_gpiote_in_init(gpioIntPin, &in_config, in_pin_AccHandler2);// Config and event on initialization
    APP_ERROR_CHECK(errCode);
    nrf_drv_gpiote_in_event_enable(gpioIntPin, true);
    return errCode;
}







