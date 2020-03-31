

/*
 * bsp_LSM9DS1.h
 *
 *  Created on: 3-DEC-2019
 *      Author: Inertial Elements
 */





#ifndef DRIVERS_BSP_INC_BSP_LSM9DS1_H_
#define DRIVERS_BSP_INC_BSP_LSM9DS1_H_






/**@brief Structure to hold acceleromter values.
 * Sequence of z, y, and x is important to correspond with
 * the sequence of which z, y, and x data are read from the sensor.
 * All values are signed 16 bit integers
*/
typedef struct
{
	int16_t x;
    int16_t y;
    int16_t z;
}accel_values_t;


typedef struct
{
    int16_t ax;
    int16_t ay;
    int16_t az;
}gyro_values_t;





/*
typedef struct
{
    int8_t axl;
    int8_t axh;
    int8_t ayl;
    int8_t ayh;
    int8_t azl;
    int8_t azh;
}gyro_values_t;

gyro_values_t samp ;
typedef struct
{

	int16_t ax ;
    int16_t ay ;
    int16_t az;

}gyro_value_n;
*/

//typedef int16_t temp_value_t;


/*@Brief This function is used for initialization of the accelerometr
 *
*/
/*
typedef struct accel_sample_data
{
      int16_t x;
      int16_t y;
      int16_t z;
} accel_sample_struct;

accel_sample_struct sample;

accel_sample_struct read_accel_data(void)
{
      // Measurement data starts at DATAX0, and ends at DATAZ1, 6 bytes long
      cmd_array[0] = ADXL345_REG_DATAX0;

      // Read 6 bytes at once
      i2c_transfer(ADXL345_ADDRESS, cmd_array, data_array, 1, 6, I2C_FLAG_WRITE_READ);

      // Now pack the return structure with meaningful data
      sample.x = (data_array[1] << 8) | data_array[0];
      sample.y = (data_array[3] << 8) | data_array[2];
      sample.z = (data_array[5] << 8) | data_array[4];

      return sample;
}
*/
 void bsp_lsm9ds1_SwitchonAccelo(uint8_t odr);

 float lsm9ds1_from_fs100g_to_mg(int16_t lsb);

 void bsp_lsm9ds1_SwitchonGyro(void);


/*@Brief This function read the accelerometer  values from the sensor
 *param[out]accel_values  It will store the values  of all the axis i.e x,y and z
 *in the pointer specified here
*/

uint32_t bsp_lsm9ds1_ReadAcceloData(accel_values_t * accel_values);


/*@Brief This function read the gyrometer  values from the sensor
 *param[out]gyro_values  It will store the values  of all the axis i.e x,y and z
 *in the pointer specified here
*/

uint32_t  bsp_lsm9ds1_ReadGyroData(gyro_values_t * gyro_values);


extern void bsp_lsm9ds1_SwitchonAcc(void);



#endif /* DRIVERS_BSP_INC_BSP_LSM9DS1_H_ */

