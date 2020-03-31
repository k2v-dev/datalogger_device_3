/*
 * Command_Interface.c
 *
 *  Created on: 04-Dec-2019
 *  Author: GTsilicon
 *  Brief: This page contains information about the different commands and their responses functions
 *
 */


#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "Command_Interface.h"
#include "Ble_Interface.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "bsp_lsm9ds1.h"
#include "bsp_lsm9dsmag.h"
#include "Format_Sensor_Data.h"
#include "time.h"

struct tm * time_stamp;

uint16_t cnt;
uint8_t data_array_output[MAX_DATA_LENGTH];
uint8_t k;
uint8_t length;
uint16_t checksum;
accel_values_t accl_values;
gyro_values_t gyro_value;
mag_values_t mag_values;
uint16_t sample;
uint8_t temp_sec;

/**@brief Function for initialization and reading the IMU values of both accelerometer and the gyroscopes
 *
 *@param[out]: accl_values : This structures stores the values of the accelerometer values in 3-axis.
 *@param[out]: gyro_values : This structures stores the values of the accelerometer values in 3-axis.
 *
 */

void read_IMU(void)
{
	// Initialize the IMU registers for reading the Accelerometer and Gyroscopes
	IMU_Initialization();
	// Accelerometer read functions
	bsp_lsm9ds1_ReadAcceloData(&accl_values);
	//Gyroscope read functions
	bsp_lsm9ds1_ReadGyroData(&gyro_value);
}

/**@brief Function for initialization and reading the Magnetometer values. The initialization the magnetometer is quite diffrent from that
 * of the IMUs so we need to initialize the magnetometer everytime we read the magnetometer.
 *
 *@param[out]: mag_values : This structures stores the values of the accelerometer values in 3-axis.
 *
 */


void read_MAG(void)
{
	// Initialize the MAG registers for reading the Magnetometer
	MAG_Initialization();
	////Magnetometer read function
	bsp_lsm9dsmag_ReadMagData(&mag_values);
}

/**@brief Calculation of packet number that is being send from the BLE. The packet number comes back to 0 at STOP command
 *
*/
void packet_number_calculation(void)
{
	//Byte number 0 as packet number is the first byte of any packet
	k=0;
	// Checksum is also made to 0 at the start of each packet
	checksum=0;
	data_array_output[k++]= (cnt >>8);
	data_array_output[k++]=(cnt & 0xff);

}

/**@brief Calculation of checksum = summation of each bytes present in a packet
 *
*/

void calculate_checksum(void)
{
	length = k;
	for (int i=0;i<length;i++)
	{
		checksum+=data_array_output[i];
	}
	data_array_output[k++]= (checksum >>8);
	data_array_output[k++]= (checksum & 0xff);
}

/**@brief Calculation of the timestamp. Timestamp is the usually the received from the Local Time Information.
 * Timestamp also contains information in tens of miliseconds.
 *
*/
void calculate_timestamp(void)
{
	time_stamp = nrf_cal_get_time();
	data_array_output[k++]=(uint8_t)time_stamp->tm_hour;
	data_array_output[k++]= (uint8_t)time_stamp->tm_min;
	data_array_output[k++]= (uint8_t)time_stamp->tm_sec;
	// Syncing the Milisecond clock with the Local Time Information
	if(temp_sec!=(uint8_t)time_stamp->tm_sec)
	milisec=0;
	temp_sec = (uint8_t)time_stamp->tm_sec;
	data_array_output[k++]= milisec;
	NRF_LOG_INFO("Time:\t%d:%d:%d:%d\r\n",time_stamp->tm_hour,(time_stamp->tm_min),time_stamp->tm_sec,milisec);
}

/**@brief Auxillary function to create data packet in Little Endian Format keeping in mind that the BLE sends data in Little Endian format
 *
*/

void data_packets(uint16_t kx)
{
	data_array_output[k++]= (kx >>8);
	data_array_output[k++]= (kx & 0xff);
}




/**@brief Auxillary function to create data packet in Little Endian Format keeping in mind that the BLE sends data in Little Endian format for float values
 *
*/

void data_packets_float(float kx)
{
	union {
		float a;
		unsigned char bytesf[4];
		} floatvalue;

	floatvalue.a =kx;

	data_array_output[k++]= floatvalue.bytesf[3];
	data_array_output[k++]= floatvalue.bytesf[2];
	data_array_output[k++]= floatvalue.bytesf[1];
	data_array_output[k++]= floatvalue.bytesf[0];
}

/**@brief Command to send data of the 3 AXIS IMU (only accelerometer)
 *
*/

void cmd_send_acc_3_axis(void)
{
	uint16_t ax;
	uint16_t ay;
	uint16_t az;

	packet_number_calculation();
	calculate_timestamp();
	ax=208;
	ay=450;
	az=1024;
	data_packets(ax);
	data_packets(ay);
	data_packets(az);
	calculate_checksum();
}


/**@brief Command to send Accelerometer data of the 9 AXIS IMU
 *
*/
void cmd_send_acc_9_axis(void)
{
	uint16_t ax;
	uint16_t ay;
	uint16_t az;

	packet_number_calculation();
	calculate_timestamp();
	NRF_LOG_INFO("aCC_READ START");
	read_IMU();
	NRF_LOG_INFO("Sample # %d\r\nAX: %06d\r\nAY: %06d\r\nAZ: %06d ",
			 sample++,(~accl_values.x+1) ,(~accl_values.y+1), (~accl_values.z+1)); //accl values
	ax=(~accl_values.x+1);
	ay=(~accl_values.y+1);
	az=(~accl_values.z+1);
	data_packets(ax);
	data_packets(ay);
	data_packets(az);
	calculate_checksum();
}

/**@brief Command to send Magnetometer data of the 9 AXIS IMU
 *
*/

void cmd_send_mag_9_axis(void)
{
	uint16_t mx;
	uint16_t my;
	uint16_t mz;

	packet_number_calculation();
	calculate_timestamp();
	NRF_LOG_INFO("Mag_READ START");
	read_MAG();
	NRF_LOG_INFO("Sample # %d\r\nMX: %06d\r\nMY: %06d\r\nMZ: %06d ",
			 sample++,(~mag_values.mx+1) ,(~mag_values.my+1), (~mag_values.mz+1)); //accl values
	mx=(~accl_values.x+1);
	my=(~accl_values.y+1);
	mz=(~accl_values.z+1);
	data_packets(mx);
	data_packets(my);
	data_packets(mz);
	calculate_checksum();
}

/**@brief Command to send Gyroscope data of the 9 AXIS IMU
 *
*/
void cmd_send_gyro_9_axis(void)
{

	uint16_t gx;
	uint16_t gy;
	uint16_t gz;

	packet_number_calculation();
	calculate_timestamp();
	NRF_LOG_INFO("GYRO_READ START");
	read_IMU();
	NRF_LOG_INFO("Sample # %d\r\nGX: %06d\r\nGY: %06d\r\nGZ: %06d ",
			 sample++,(~gyro_value.ax+1) ,(~gyro_value.ay+1), (~gyro_value.az+1)); //accl values
	gx=(~gyro_value.ax+1);
	gy=(~gyro_value.ay+1);
	gz=(~gyro_value.az+1);
	data_packets(gx);
	data_packets(gy);
	data_packets(gz);
	calculate_checksum();
}



/**@brief Command to send Accelerometer and the Gyroscope data of the 9 AXIS IMU
 *
*/
void cmd_send_imu_9_axis(void)
{
	uint16_t ax;
	uint16_t ay;
	uint16_t az;
	uint16_t gx;
	uint16_t gy;
	uint16_t gz;

	packet_number_calculation();
	calculate_timestamp();
	NRF_LOG_INFO("ACC_GYRO_READ START");
	read_IMU();
	NRF_LOG_INFO("Sample # %d\r\nAX: %06d\r\nAY: %06d\r\nAZ: %06d",
			 sample++,(~accl_values.x+1) ,(~accl_values.y+1), (~accl_values.z+1)); //accl values
	NRF_LOG_INFO("\nGX: %06d\r\nGY: %06d\r\nGZ: %06d ",
				 (~gyro_value.ax+1) ,(~gyro_value.ay+1), (~gyro_value.az+1)); //accl values

	ax=(~accl_values.x+1);
	ay=(~accl_values.y+1);
	az=(~accl_values.z+1);
	gx=(~gyro_value.ax+1);
	gy=(~gyro_value.ay+1);
	gz=(~gyro_value.az+1);

	data_packets(ax);
	data_packets(ay);
	data_packets(az);
	data_packets(gx);
	data_packets(gy);
	data_packets(gz);
	calculate_checksum();
}

/**@brief Command to send Accelerometer, Gyroscope and the Magnetometer data of the 9 AXIS IMU
 *
*/

void cmd_send_all_9_axis(void)
{
	uint16_t ax,ay,az,gx,gy,gz,mx,my,mz;

	packet_number_calculation();
	calculate_timestamp();
	NRF_LOG_INFO("ACC_GYRO_MAG_READ START");
	read_IMU();
	read_MAG();
	NRF_LOG_INFO("Sample # %d\r\nAX: %06d\r\nAY: %06d\r\nAZ: %06d",
			 sample++,(~accl_values.x+1) ,(~accl_values.y+1), (~accl_values.z+1)); //accl values
	NRF_LOG_INFO("\nGX: %06d\r\nGY: %06d\r\nGZ: %06d ",
				 (~gyro_value.ax+1), (~gyro_value.ay+1), (~gyro_value.az+1)); //gyro values
	NRF_LOG_INFO("\r\nMX: %06d\r\nMY: %06d\r\nMZ: %06d ",
				 (~mag_values.mx+1), (~mag_values.my+1), (~mag_values.mz+1)); //mag values
	ax=(~accl_values.x+1);
	ay=(~accl_values.y+1);
	az=(~accl_values.z+1);
	gx=(~gyro_value.ax+1);
	gy=(~gyro_value.ay+1);
	gz=(~gyro_value.az+1);
	mx=(~accl_values.x+1);
	my=(~accl_values.y+1);
	mz=(~accl_values.z+1);
	data_packets(ax);
	data_packets(ay);
	data_packets(az);
	data_packets(gx);
	data_packets(gy);
	data_packets(gz);
	data_packets(mx);
	data_packets(my);
	data_packets(mz);
	calculate_checksum();
}



void cmd_send_9axis_precision(void)
{
	packet_number_calculation();
	calculate_timestamp();
	format_sensor_data();
	data_packets_float(acx);
	data_packets_float(acy);
	data_packets_float(acz);
	data_packets_float(grx);
	data_packets_float(gry);
	data_packets_float(grz);
	data_packets_float(u_mx);
	data_packets_float(u_my);
	data_packets_float(u_mz);
	calculate_checksum();
}

/**@brief Command to stop all the output
 *
*/
void cmd_stop(void)
{
	turn_on_output=false;
	// Set the packet number as 0
	cnt=0;
}

/**@brief FSM to select the command  and the command Id
 *
*/
void finite_state_machine_select_command(void)
{
	switch(rcvd_cmd_id)
	{
		case SEND_DATA_ACC_3AXIS :
			cmd_send_acc_3_axis();
		break;
		case SEND_DATA_ACC_9AXIS:
			cmd_send_acc_9_axis();
		break;
		case SEND_DATA_GYRO_9AXIS:
			cmd_send_gyro_9_axis();
		break;
		case SEND_DATA_IMU_9AXIS:
			cmd_send_imu_9_axis();
		break;
		case SEND_DATA_MAG_9AXIS:
			cmd_send_mag_9_axis();
		break;
		case SEND_DATA_ALL_9AXIS:
			cmd_send_all_9_axis();
		break;
		case SEND_DATA_ACC_9AXIS_PRECISION:
			cmd_send_9axis_precision();
		break;
		case STOP_DATA :
			cmd_stop();
			NRF_LOG_INFO("STOP THE OUTPUT");
		default:
			NRF_LOG_INFO("Check the command");
		break;
	}
}


/**@brief Magnetometer initialization command
 *
*/

void MAG_Initialization(void)
{
	bsp_lsm9dsMAG_SwitchonMAG(0x1C);
	bsp_lsm9dsMAG_Switchon_SCALE(0x60);
	bsp_lsm9dsMAG_Switchon_zaxis();
}


/**@brief Accelerometer and the Gyroscope initialization command
 *
*/
void IMU_Initialization(void)
{
	bsp_lsm9ds1_SwitchonAccelo(0x80);
	bsp_lsm9ds1_SwitchonAcc();
	bsp_lsm9ds1_SwitchonGyro();
}

