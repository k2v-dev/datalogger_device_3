/*
 * Command_Interface.h
 *
 *  Created on: 04-Dec-2019
 *      Author: GTsilicon
 */

#include "bsp_lsm9ds1.h"
#include "bsp_lsm9dsmag.h"
#include <stdio.h>
#include <string.h>



#ifndef APPLICATIONS_INC_COMMAND_INTERFACE_H_
#define APPLICATIONS_INC_COMMAND_INTERFACE_H_

#define SEND_DATA_ALL_9AXIS 0x08
#define SEND_DATA_ACC_3AXIS 0x02
#define SEND_DATA_ACC_9AXIS_PRECISION 0x11
#define SEND_DATA_ACC_9AXIS 0x04
#define SEND_DATA_GYRO_9AXIS 0x05
#define SEND_DATA_MAG_9AXIS 0x06
#define SEND_DATA_IMU_9AXIS 0x07
#define STOP_DATA 0x09


#define MAX_DATA_LENGTH 23
uint8_t k;
uint16_t cnt;
extern accel_values_t accl_values;
extern gyro_values_t gyro_value;
extern mag_values_t mag_values;
extern uint8_t data_array_output[MAX_DATA_LENGTH];
extern void finite_state_machine_select_command(void);
void IMU_Initialization(void);
void MAG_Initialization(void);
void read_MAG(void);
void read_IMU(void);

#endif /* APPLICATIONS_INC_COMMAND_INTERFACE_H_ */
