/*
 * cpt_h3lis100dl_reg.h
 *
 *  Created on: Dec 3, 2019
 *      Author: PARTH
 */

#ifndef DRIVERS_CPT_INC_CPT_H3LIS100DL_CFG_H_
#define DRIVERS_CPT_INC_CPT_H3LIS100DL_CFG_H_

typedef enum
{
	WHO_AM_I = 	0x0F,
	CTRL1 =  	0x20,
	CTRL2 =  	0x21,
	CTRL3 =  	0x22,
	CTRL4 =  	0x23,
	CTRL5 =  	0x24,
	STATUS = 	0x27,
	OUT_X_L = 	0x29,
	OUT_Y_L = 	0x2B,
	OUT_Z_L = 	0x2D,
	IG_CFG1 =   0x30,
	IG_SRC1 = 	0x31,
	IG_THS1 =   0x32,
	IG_DUR1 = 	0x33,
	IG_CFG2 = 	0x34,
	IG_SRC2 = 	0x35,
	IG_THS2 = 	0x36,
	IG_DUR2 = 	0x37,

}AccRegister;

/*
typedef enum
{
	ODR_50  = 0x27,
	ODR_100 = 0x2F,
	ODR_400 = 0x37,
}AccOdr;
*/


/** I2C Device Address 8 bit format  if SA0=0 -> 0x31 if SA0=1 -> 0x33 **/
#define H3LIS100DL_I2C_ADD_L     0x31
#define H3LIS100DL_I2C_ADD_H     0x33


#endif /* DRIVERS_CPT_INC_CPT_H3LIS100DL_CFG_H_ */
