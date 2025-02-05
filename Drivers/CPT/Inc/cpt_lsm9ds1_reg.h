/*******************************************************************************
LSM9DS1_Registers.h
SFE_LSM9DS1 Library - LSM9DS1 Register Map

Original Creation Date: 3 dec, 2019
LSM9DS1_Breakout
This file defines all registers internal to the gyro/accel and magnetometer

******************************************************************************/

#ifndef DRIVERS_CPT_INC_CPT_LSM9DS1_REG_H_
#define DRIVERS_CPT_INC_CPT_LSM9DS1_REG_H_



#include <stdint.h>

/////////////////////////////////////////
// LSM9DS1 Accel/Gyro (XL/G) Registers //
/////////////////////////////////////////



#define ACT_THS				0x04
#define ACT_DUR				0x05
#define INT_GEN_CFG_XL		0x06
#define INT_GEN_THS_X_XL	0x07
#define INT_GEN_THS_Y_XL	0x08
#define INT_GEN_THS_Z_XL	0x09
#define INT_GEN_DUR_XL		0x0A
#define REFERENCE_G			0x0B
#define INT1_CTRL			0x0C
#define INT2_CTRL			0x0D
#define WHO_AM_I_XG			0x0F
#define CTRL_REG1_G			0x10
#define CTRL_REG2_G			0x11
#define CTRL_REG3_G			0x12
#define ORIENT_CFG_G		0x13
#define INT_GEN_SRC_G		0x14
#define OUT_TEMP_L			0x15
#define OUT_TEMP_H			0x16
#define STATUS_REG_0		0x17
#define OUT_X_L_G			0x18
#define OUT_X_H_G			0x19
#define OUT_Y_L_G			0x1A
#define OUT_Y_H_G			0x1B
#define OUT_Z_L_G			0x1C
#define OUT_Z_H_G			0x1D
#define CTRL_REG4			0x1E
#define CTRL_REG5_XL		0x1F
#define CTRL_REG6_XL		0x20
#define CTRL_REG7_XL		0x21
#define CTRL_REG8			0x22
#define CTRL_REG9			0x23
#define CTRL_REG10			0x24
#define INT_GEN_SRC_XL		0x26
#define STATUS_REG_1		0x27
#define OUT_X_L_XL			0x28
#define OUT_X_H_XL			0x29
#define OUT_Y_L_XL			0x2A
#define OUT_Y_H_XL			0x2B
#define OUT_Z_L_XL			0x2C
#define OUT_Z_H_XL			0x2D
#define FIFO_CTRL			0x2E
#define FIFO_SRC			0x2F
#define INT_GEN_CFG_G		0x30
#define INT_GEN_THS_XH_G	0x31
#define INT_GEN_THS_XL_G	0x32
#define INT_GEN_THS_YH_G	0x33
#define INT_GEN_THS_YL_G	0x34
#define INT_GEN_THS_ZH_G	0x35
#define INT_GEN_THS_ZL_G	0x36
#define INT_GEN_DUR_G		0x37

///////////////////////////////
// LSM9DS1 Magneto Registers //
///////////////////////////////
#define OFFSET_X_REG_L_M	0x05
#define OFFSET_X_REG_H_M	0x06
#define OFFSET_Y_REG_L_M	0x07
#define OFFSET_Y_REG_H_M	0x08
#define OFFSET_Z_REG_L_M	0x09
#define OFFSET_Z_REG_H_M	0x0A
#define WHO_AM_I_M			0x0F
#define CTRL_REG1_M			0x20
#define CTRL_REG2_M			0x21
#define CTRL_REG3_M			0x22
#define CTRL_REG4_M			0x23
#define CTRL_REG5_M			0x24
#define STATUS_REG_M		0x27
#define OUT_X_L_M			0x28
#define OUT_X_H_M			0x29
#define OUT_Y_L_M			0x2A
#define OUT_Y_H_M			0x2B
#define OUT_Z_L_M			0x2C
#define OUT_Z_H_M			0x2D
#define INT_CFG_M			0x30
#define INT_SRC_M			0x30
#define INT_THS_L_M			0x32
#define INT_THS_H_M			0x33

////////////////////////////////
// LSM9DS1 WHO_AM_I slave Responses //
////////////////////////////////
#define WHO_AM_I_AG_RSP		0x6A
#define WHO_AM_I_MAG       0x1C


typedef enum
{
// A/G FREQ
	ODR_14  = 0x20,
	ODR_59  = 0x40,
	ODR_119  = 0x60,   //119 hz  accel and gyro
	ODR_238  = 0x80,
	ODR_476  = 0xA0,
	ODR_952  = 0xC0,
//MAG FREQ
	 ODR_1MAG =0x04,
	 ODR_3MAG =0x80,
	 ODR_5MAG =0x0C,
	 ODR_10MAG =0x01,
	 ODR_20MAG =0x14,
	 ODR_40MAG =0x18,
	ODR_80MAG = 0x1C   //80hz magnetometer
       //40hz  magnetometer
}AGMOdr;

typedef enum{

	GAUSS4_MAG = 0x00,
	GAUSS8_MAG = 0x20,
	GAUSS12_MAG = 0x40,
	GAUSS16_MAG = 0x60

}MAGrag;




#endif
