/*
 * bsp_h3lis100dl.h
 *
 *  Created on: Dec 3, 2019
 *      Author: PARTH
 */

#ifndef DRIVERS_BSP_INC_BSP_H3LIS100DL_H_
#define DRIVERS_BSP_INC_BSP_H3LIS100DL_H_

typedef enum
{
	ODR_50  = 0x27,
	ODR_100 = 0x2F,
	ODR_400 = 0x37,
}AccOdr;


typedef enum
{
	OR_LOW		= 0x15,
	OR_HIGH		= 0x2A,
	AND_LOW		= 0x95,
	AND_HIGH	= 0xAA,
}InturruptMode;
#endif /* DRIVERS_BSP_INC_BSP_H3LIS100DL_H_ */
