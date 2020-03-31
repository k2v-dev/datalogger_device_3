/*
 * bsp_h3lis100dl.c
 *
 *  Created on: Dec 3, 2019
 *      Author: PARTH
 */


#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include "cpt_h3lis100dl.h"
#include "bsp_accl.h"
#include "app_util_platform.h"
#include "cpt_h3lis100dl_cfg.h"
#include "cpt_h3lis100dl.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"



#define interrupt 1
#define sixbytes 6
#define singlebyte 1
#define BURST_MODE 0x00
//#define INTERRUPT_1_PIN1 0xC2
//#define INTERRUPT_2_PIN2 0x10
//#define XACTTHRESHOLD_2G 0x9F
//#define YACTTHRESHOLD_2G 0x9F
//#define ZACTTHRESHOLD_2G 0xBF
//#define DURACT_1S 0xC8
//#define INACTTHRESHOLD_1G_ALLAXIS 0xBF
//#define DURINACT_1S 0xD8
//#define SET_ACTTHRES_ON1AXIS 0x60
//#define SET_INACTTHRES_ON3AXIS 0xC3
#define ACC_RAW_BUFFER_SIZE 1000
/*

static uint8_t sAccBuffer[ACC_RAW_BUFFER_SIZE];
static uint16_t bytes;
static accel_values_t accel_values16bit;
static accel_values_t  accel_values;
*/
/**
 *  \brief  The ring buffer used to store the acceleration samples
 */




/**Switch on the Accelerometer
 *
 */
void bsp_accl_SwitchonAccelo(uint8_t odr)
{
	// The SCL and the SDA pins are set and the Output data  rate is set at 10 Hz
//	uint32_t errCode;
	cpt_accelerometer_On(H3LIS100DL_I2C_SCL_PIN, H3LIS100DL_I2C_SDA_PIN, odr);
//	if(errCode != NRF_SUCCESS) return errCode;
	//Set the accelerometer full scale range at 2g required for activity detection.

	cpt_h3lis100dl_WriteSingleRegister(CTRL4,BURST_MODE);//0x08 is for burst mode reading, 0 for 2g scale
}


/**Read Accelerometer Data
 *
 */

uint32_t bsp_accl_ReadAcceloData(accel3x_values_t * accel_values)
{
    uint32_t errCode;
    uint8_t rawValues[3];
    // Reorganize read sensor values and put them into value struct
    uint8_t *data;
    data = (uint8_t*)accel_values;
    for(uint8_t i = 0; i<3; i++)
    {
    	errCode = cpt_h3lis100dl_ReadRegisters(OUT_X_L+(i*2), rawValues,3);
    	//if(errCode != NRF_SUCCESS) return errCode;
    	//NRF_LOG_INFO("rAWVALUES %x",rawValues[i]);
    	*data = rawValues[0];
        data++;
    }
    return errCode;
}
float h3lis100dl_from_fs100g_to_mg(int8_t lsb)
{
  return ( (float)lsb / 256.0f ) * 780.0f;
}

/*

uint32_t bsp_accl_PushToRawBuf(uint8_t AccBuffer[1000], uint16_t count)
{
    uint32_t errCode;
  //  NRF_LOG_INFO("sTART");
    for (uint16_t i=0;i<1000;i++)
    {
    	AccBuffer[i]= sAccBuffer[i];
    }
    count=bytes ;
    uint32_t timestamp;
    timestamp = get_rtc_counteracc();
    errCode=bsp_accl_ReadAcceloData(&accel_values16bit);
  //  NRF_LOG_INFO("Time %i ",timestamp);
   // NRF_LOG_INFO("Acc %i ",accel_values16bit.z);
    accel_values.x=(accel_values16bit.x >>4);
    accel_values.y=(accel_values16bit.y >>4);
    accel_values.z=(accel_values16bit.z >>4);
    if (bytes >=1000)
    {
    	bytes=0;
    }
    bytes=bytes+10;
   // NRF_LOG_INFO("Acc  %i ",accel_values.z);
    sAccBuffer[bytes-10]=(uint8_t)accel_values.x >>8;
    sAccBuffer[bytes-9]=(uint8_t)accel_values.x;
    sAccBuffer[bytes-8]=(uint8_t)accel_values.y>>8;
    sAccBuffer[bytes-7]=(uint8_t)accel_values.y;
    sAccBuffer[bytes-6]=(uint8_t)accel_values.z>>8;
    sAccBuffer[bytes-5]=(uint8_t)accel_values.z;
    sAccBuffer[bytes-4]=(uint8_t)timestamp>>24;
    sAccBuffer[bytes-3]=(uint8_t)timestamp>>16;
    sAccBuffer[bytes-2]=(uint8_t)timestamp>>8;
    sAccBuffer[bytes-1]=(uint8_t)timestamp;

    return errCode;
}
*/

/*@Brief This function is to set the interrupt pin1 for interrupt1 (ACTIVITY) and interrupt pin2
 * for interrupt2 (INACTIVITY).
 * Interrupt1 for Activity
 * Interrupt2 for Inactivity
*/


void bsp_h3lis100dl_SetInterrupt(void)
{
	// Interrupt1 for Activity on Interrupt 1 pin
	// FIFO enable for interrupt -data ready interrupt

	cpt_h3lis100dl_WriteSingleRegister(CTRL3,0xA4);    //int2 = 0x24

	// inturrupt 1 and 2 configuration

//	cpt_h3lis100dl_WriteSingleRegister(IG_CFG1,0x15);   /// low int
//	cpt_h3lis100dl_WriteSingleRegister(IG_CFG2,0xAA);    /// high int

	// Interrupt threshold set



	//pin config

	bsp_accl_SetInterruptPin();


}
void bsp_h3lis100dl_SetInterrupt1_mode(uint8_t high_low)
{
	cpt_h3lis100dl_WriteSingleRegister(IG_CFG1,high_low);
}


void bsp_h3lis100dl_SetInterrupt2_mode(uint8_t high_low)
{
	cpt_h3lis100dl_WriteSingleRegister(IG_CFG2,high_low);
}


/*@Brief This function is to set the threshold and duration for the Activity detected
* Activity threshold will be set for 2g and the activity duration will be set for 1s
* Activty threshold = Value*(Full Scale/255)
* Activity_duration = Value*(1/odr)
*/
void bsp_accl_SetActivityThres1(uint8_t thres)
{
	cpt_h3lis100dl_WriteSingleRegister(IG_THS1,thres);
}
void bsp_accl_SetActivityThres2(uint8_t thres)
{
	cpt_h3lis100dl_WriteSingleRegister(IG_THS2,thres);
}
void bsp_accl_SetActivityThres1Dur(uint8_t thresDur)
{
	// threshold set
//	cpt_h3lis100dl_WriteSingleRegister(IG_THS1,0x01);



//	cpt_h3lis100dl_WriteSingleRegister(IG_THS2,0x02);

	// inturrupt duration set

	cpt_h3lis100dl_WriteSingleRegister(IG_DUR1,thresDur);


}

void bsp_accl_SetActivityThres2Dur(uint8_t thresDur)
{
	cpt_h3lis100dl_WriteSingleRegister(IG_DUR2,thresDur);
}
/*@Brief This function is to set the threshold and duration for the Inactivity detected.
* Inactivity threshold will be set for 1g and the activity duration will be set for 500ms
* Inactivity threshold = Value*(Full Scale/255)
* Inactivity_duration = Value*(1/odr)
*/
/*
void bsp_accl_SetInactivityThresDur(void)
{
	// Set the threshold on the Inactivity threshold register as 1g

	cpt_h3lis100dl_WriteSingleRegister(IG_THS2,INACTTHRESHOLD_1G_ALLAXIS);

	// Set the duration on the Inactivity  register as 500 ms.

	cpt_h3lis100dl_WriteSingleRegister(IG_DUR2,DURINACT_1S);
}
*/

/*@Brief This function is to set the configuration of the of the Activity Register.
 * The threshold has to to be reached in either of the 3 axis .
 * The threshold is based on the movement only
 *
*/
/*

void bsp_accl_SetActivityConfig(void)
{
	cpt_h3lis100dl_WriteSingleRegister(IG_CFG1,SET_ACTTHRES_ON1AXIS);
}

@Brief This function is to set the configuration of the of the Inactivity Register.
 * The threshold has to to be reached in all of the 3 axis .
 * The threshold is based on the movement in the all the axis.

void bsp_accl_SetInactivityConfig(void)
{
	cpt_h3lis100dl_WriteSingleRegister(IG_CFG2,SET_INACTTHRES_ON3AXIS);
}

*/

/*@Brief This function is to generate interrupt on the Interrupt pin 1 and Interrupt pin 2
 *
*/
void bsp_accl_SetInterruptPin(void)
{
	//bsp_accl_SwitchonAccelo();

	//Set the corresponding interrupt on either of the interrupt pin
//	cpt_gpio_AccInit1(INTERRUPT_1_PIN);
//	cpt_gpio_AccInit2(INTERRUPT_2_PIN);
	 bsp_gpio_IntInit();
}

/**@brief Initialize the gpio to initialize both the  interrupt from the accelerometer
 *
 */


uint32_t bsp_gpio_IntInit(void)
{
	uint32_t errCode;
	errCode = cpt_gpio_AccInit1(INTERRUPT_1_PIN);
	errCode = cpt_gpio_AccInit2(INTERRUPT_2_PIN);
	return errCode;
}




/**Read Interrupt_Status1
 *
 */
/*
uint8_t bsp_accl_ReadFifostatus(void)
{
   // uint32_t errCode;
	//uint8_t ovr;
    uint8_t rawValues[singlebyte];
    for(uint8_t i = 0; i<singlebyte; i++)
    {
    	 cpt_h3lis100dl_ReadRegisters(FIFO_SRC, rawValues,singlebyte);
    }

    return rawValues[0];
}
*/

/**Read Interrupt_Status2
 *
 */

uint8_t bsp_accl_ReadInt1Data(void)
{
   // uint32_t errCode;
    uint8_t rawValues[singlebyte];
    for(uint8_t i = 0; i<singlebyte; i++)
    {
    	 cpt_h3lis100dl_ReadRegisters(IG_SRC1+i, rawValues,singlebyte);

    }

    return rawValues[0];
}



uint8_t bsp_accl_ReadInt2Data(void)
{
   // uint32_t errCode;
    uint8_t rawValues[singlebyte];
    for(uint8_t i = 0; i<singlebyte; i++)
    {
    	 cpt_h3lis100dl_ReadRegisters(IG_SRC2+i, rawValues,singlebyte);

    }

    return rawValues[0];
}




/*

void bsp_set_FIFO(void)
{
	cpt_h3lis100dl_WriteSingleRegister(FIFO_CTRL,FIFO_SET_25_SAMPLES);
}

void bsp_reset_FIFO(void)
{
	cpt_h3lis100dl_WriteSingleRegister(FIFO_CTRL,0);
}
*/




