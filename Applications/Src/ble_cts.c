/*
 * ble_cts.c
 *
 *  Created on: Dec 14, 2019
 *      Author: gt silicon
 */

#include "sdk_common.h"
//#if NRF_MODULE_ENABLED(BLE_CTS)
#include "ble_cts.h"
#include "ble_cts_c.h"
#include <string.h>
#include "ble_srv_common.h"
#include "ble_date_time.h"
#include "RTC_updated.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_delay.h"


#define INITIAL_VALUE_CTS 0
#define CTS_CURRENT_TIME_EXPECTED_LENGTH  10

#define OPCODE_LENGTH 1                                                              /**< Length of opcode inside Heart Rate Measurement packet. */
#define HANDLE_LENGTH 2                                                              /**< Length of handle inside Heart Rate Measurement packet. */
#define MAX_CTS_LEN      (NRF_SDH_BLE_GATT_MAX_MTU_SIZE - OPCODE_LENGTH - HANDLE_LENGTH) /**< Maximum size of a transmitted Heart Rate Measurement. */
//



/**@brief Function for handling the Connect event.
 *
 * @param[in]   p_cts       Heart Rate Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_connect(ble_cts_t * p_cts, ble_evt_t const * p_ble_evt)
{
    p_cts->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}


/**@brief Function for handling the Disconnect event.
 *
 * @param[in]   p_cts       Heart Rate Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_disconnect(ble_cts_t * p_cts, ble_evt_t const * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_cts->conn_handle = BLE_CONN_HANDLE_INVALID;
}

/**@brief Function for handling write events to the Heart Rate Measurement characteristic.
 *
 * @param[in]   p_cts         Heart Rate Service structure.
 * @param[in]   p_evt_write   Write event received from the BLE stack.
 */
/*
static void on_cts_cccd_write(ble_cts_t * p_cts, ble_gatts_evt_write_t const * p_evt_write)
{
    if (p_evt_write->len == 2)
    {
        // CCCD written, update notification state
        if (p_cts->evt_handler != NULL)
        {
            ble_cts_evt_t evt;

            if (ble_srv_is_notification_enabled(p_evt_write->data))
            {
                evt.evt_type = BLE_CTS_EVT_NOTIFICATION_ENABLED;
            }
            else
            {
                evt.evt_type = BLE_CTS_EVT_NOTIFICATION_DISABLED;
            }

            p_cts->evt_handler(p_cts, &evt);
        }
    }
}
*/


static void on_write(ble_cts_t *p_cts, ble_evt_t * p_ble_evt)
{

        ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

        if (p_evt_write->handle == p_cts->cts_handles.cccd_handle)
        {
            // CCCD written, call application event handler
            if (p_cts->evt_handler != NULL)
            {
                ble_cts_evt_t evt;

                if (ble_srv_is_notification_enabled(p_evt_write->data))
                {
                    evt.evt_type = BLE_CTS_EVT_NOTIFICATION_ENABLED;
                }
                else
                {
                    evt.evt_type = BLE_CTS_EVT_NOTIFICATION_DISABLED;
                }

                p_cts->evt_handler(p_cts, &evt);

            }
        }/*if */
}/*on_write*/



/**@brief Function for handling the Write event.
 *
 * @param[in]   p_cts       Heart Rate Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
/*
static void on_write(ble_cts_t * p_cts, ble_evt_t const * p_ble_evt)
{
    ble_gatts_evt_write_t const * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

    if (p_evt_write->handle == p_cts->cts_handles.cccd_handle)
    {
        on_cts_cccd_write(p_cts, p_evt_write);
    }
}
*/

void ble_cts_on_ble_evt(  ble_evt_t const * p_ble_evt, void * p_context)
{
    ble_cts_t * p_cts = (ble_cts_t *) p_context;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_cts, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_cts, p_ble_evt);
            break;

        case BLE_GATTS_EVT_WRITE:
            on_write(p_cts, p_ble_evt);
            break;

        default:
            // No implementation needed.
            break;
    }
}






/**@brief Function for encoding a Heart Rate Measurement.
 *
 * @param[in]   p_hrs              Heart Rate Service structure.
 * @param[in]   heart_rate         Measurement to be encoded.
 * @param[out]  p_encoded_buffer   Buffer where the encoded data will be written.
 *
 * @return      Size of encoded data.
 */





static uint8_t cts_time_encode(const current_time_char_t * p_time, uint8_t * p_encoded_buffer)

{

    uint8_t len = 0;



    len += ble_date_time_encode(&p_time->exact_time_256.day_date_time.date_time, &p_encoded_buffer[len]);


    	p_encoded_buffer[len++]  = p_time->exact_time_256.day_date_time.day_of_week ;
       p_encoded_buffer[len++] = p_time->exact_time_256.fractions256 ;



    p_encoded_buffer[len++] = (uint8_t) 1; //adjustment reason: external time source



    return len;

}



static uint32_t current_rate_measurement_char_add(ble_cts_t * p_cts,
                                                const ble_cts_init_t * p_cts_init)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;
    uint8_t             encoded_initial_cts[MAX_CTS_LEN];
    uint32_t error;


    if (false != p_cts->is_notification_supported)
    {
     memset(&cccd_md, 0, sizeof(cccd_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    cccd_md.write_perm = p_cts_init-> cts_cccd_wr_sechh.cccd_write_perm;
    cccd_md.vloc       = BLE_GATTS_VLOC_STACK;
    }

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read  = 1;
    char_md.char_props.write = 0;
//    char_md.char_props.notify = 1;
    char_md.char_props.notify = (p_cts->is_notification_supported) ? 1 : 0;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
//    char_md.p_cccd_md         = &cccd_md;
    char_md.p_cccd_md         = (p_cts->is_notification_supported) ? &cccd_md : NULL;
    char_md.p_sccd_md         = NULL;

    BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_CURRENT_TIME_CHAR);


    memset(&attr_md, 0, sizeof(attr_md));

    attr_md.read_perm  = p_cts_init->  bsl_rd_sechh.read_perm;
    attr_md.write_perm = p_cts_init-> cts_cccd_wr_sechh.write_perm;
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len     = cts_time_encode(&p_cts->current_date_time
      		 ,encoded_initial_cts);
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = 10;
    attr_char_value.p_value   = encoded_initial_cts;


    return sd_ble_gatts_characteristic_add(p_cts-> service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_cts->cts_handles);


}


current_time_char_tt currunt_time_updat()
{
	 current_time_char_tt		time_cur;
	 struct tm *get_cur_time;

	 get_cur_time = nrf_cal_get_time();




	 	time_cur.exact_time_256.day_date_time.date_time.hours = (uint8_t)get_cur_time->tm_hour;
	    time_cur.exact_time_256.day_date_time.date_time.minutes = 	(uint8_t)get_cur_time->tm_min;
	    time_cur.exact_time_256.day_date_time.date_time.seconds = (uint8_t)get_cur_time->tm_sec;
	    time_cur.exact_time_256.day_date_time.date_time.day = (uint8_t)get_cur_time->tm_mday;
	    time_cur.exact_time_256.day_date_time.date_time.month = (uint8_t)get_cur_time->tm_mon+1;
	    time_cur.exact_time_256.day_date_time.date_time.year =  (uint16_t)get_cur_time->tm_year +1900;

	/*	 NRF_LOG_INFO("time1 hr %d",get_cur_time->tm_hour);
		 NRF_LOG_INFO("time1 min %d",get_cur_time->tm_min);
		 NRF_LOG_INFO("time1   sec%d",get_cur_time->tm_sec);
		 NRF_LOG_INFO("time1 day %d",get_cur_time->tm_mday);
		 NRF_LOG_INFO("time1 month %d",get_cur_time->tm_mon+1);
		 NRF_LOG_INFO("time1  year%d",(uint16_t)get_cur_time->tm_year);
*/

	/*    time_cur.exact_time_256.day_date_time.date_time.day = 11;
	    time_cur.exact_time_256.day_date_time.date_time.hours = 13;
	    time_cur.exact_time_256.day_date_time.date_time.minutes = 14;
	    time_cur.exact_time_256.day_date_time.date_time.month = 12;
	    time_cur.exact_time_256.day_date_time.date_time.seconds = 34;
	    time_cur.exact_time_256.day_date_time.date_time.year = (uint16_t)2019;
*/

//	    time_cur.exact_time_256.day_date_time.day_of_week =determine_day_of_week_by_gauss(&time_cur.exact_time_256.day_date_time.date_time);
		 time_cur.exact_time_256.day_date_time.day_of_week = 2;



return time_cur;

}


static uint8_t determine_day_of_week_by_gauss(ble_date_time_t *p_date)
{
 int16_t first_2_digits_of_year, last_2_digits_of_year;
 int16_t shift_month, shift_year;

 int16_t temp;

 int16_t day_of_week;

 shift_year = p_date->year;
 if(1== p_date->month || 2 == p_date->month)
  shift_year -= 1;

 first_2_digits_of_year = shift_year/100;
 last_2_digits_of_year = shift_year%100;

 shift_month = (p_date->month + 10)%12;
 if(0 == shift_month)
  shift_month = 12;

 temp = 2.6*shift_month - 0.2;

 day_of_week = (p_date->day + temp + last_2_digits_of_year
  + (last_2_digits_of_year/4) + (first_2_digits_of_year/4) - 2*first_2_digits_of_year);
 day_of_week = day_of_week % 7;

 if(0 == day_of_week)
  day_of_week = 7;

 return (uint8_t)(day_of_week);

}/*determine_day_of_week_by_gauss*/

static uint32_t LOCAL_TIME_INFORMATION_char_add(ble_cts_t * p_cts, const ble_cts_init_t * p_cts_init)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read  = 1;
    char_md.p_char_user_desc = NULL;
    char_md.p_char_pf        = NULL;
    char_md.p_user_desc_md   = NULL;
    char_md.p_cccd_md        = NULL;
    char_md.p_sccd_md        = NULL;

    BLE_UUID_BLE_ASSIGN(ble_uuid,  BLE_UUID_LOCAL_TIME_INFORMATION_CHAR );


    memset(&attr_md, 0, sizeof(attr_md));

    attr_md.read_perm  = p_cts_init-> bsl_rd_sechh.read_perm;
    attr_md.write_perm = p_cts_init-> bsl_rd_sechh.write_perm;
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;

    memset(&attr_char_value, 0, sizeof(attr_char_value));


    uint8_t day_of_week;

   // day_of_week = determine_day_of_week_by_gauss(&p_cts->current_date_time);


    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = sizeof (uint16_t);
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = sizeof (uint16_t);
   // attr_char_value.p_value   = p_cts_init->p_current_rate_location;
    attr_char_value.p_value   = (uint8_t*)&day_of_week;


    return sd_ble_gatts_characteristic_add(p_cts->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_cts->bsll_handles);
}

uint32_t ble_cts_init(ble_cts_t * p_cts, const ble_cts_init_t * p_cts_init)
{
    uint32_t              err_code;
    ble_uuid_t            ble_uuid;
    ble_add_char_params_t add_char_params;
    uint8_t               encoded_initial_cts[MAX_CTS_LEN];

    // Initialize service structure
    p_cts->evt_handler                 = p_cts_init->evt_handler;
    p_cts->conn_handle                 = BLE_CONN_HANDLE_INVALID;
    p_cts->max_cts_len                 = 10;


    p_cts->is_notification_supported = p_cts_init->is_notification_supported;
    p_cts->current_date_time    = p_cts_init->init_date_time;
       // Add service
    BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_CURRENT_TIME_SERVICE);

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                        &ble_uuid,
                                        &p_cts->service_handle);

    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }




    // Add current rate measurement characteristic
       err_code = current_rate_measurement_char_add(p_cts, p_cts_init);
       if (err_code != NRF_SUCCESS)
       {
           return err_code;
       }


    if (p_cts_init->p_current_rate_location != NULL)
    {
        // Add local time location characteristic
        err_code = LOCAL_TIME_INFORMATION_char_add(p_cts, p_cts_init);
        if (err_code != NRF_SUCCESS)
        {
            return err_code;
        }
    }

    return NRF_SUCCESS;

}



uint32_t ble_cts_update(ble_cts_t *p_cts, current_time_char_tt *p_current_date_time)
{
 uint32_t err_code;
 err_code = NRF_SUCCESS;
 ble_gatts_value_t gatts_value;

 if (0 == memcmp(&p_cts->current_date_time, p_current_date_time, sizeof(current_time_char_tt)))
  return NRF_SUCCESS;

 p_cts->current_date_time = *p_current_date_time;


 gatts_value.len     = sizeof(ble_date_time_t);
 gatts_value.offset  = 0;
 gatts_value.p_value = (uint8_t*)p_current_date_time;


 err_code = sd_ble_gatts_value_set(p_cts->conn_handle,
  p_cts->cts_handles.value_handle, &gatts_value);

 if (err_code != NRF_SUCCESS)
  return err_code;

 // Send value if connected and notifying
 if ((p_cts->conn_handle != BLE_CONN_HANDLE_INVALID)
  && p_cts->is_notification_supported)
 {
  ble_gatts_hvx_params_t hvx_params;
  uint16_t len;

  memset(&hvx_params, 0, sizeof(hvx_params));
  len = sizeof(ble_date_time_t);

  hvx_params.handle   = p_cts->cts_handles.value_handle;
  hvx_params.type     = BLE_GATT_HVX_NOTIFICATION;
  hvx_params.offset   = 0;
  hvx_params.p_len    = &len;
  hvx_params.p_data   = (uint8_t*)p_current_date_time;

  err_code = sd_ble_gatts_hvx(p_cts->conn_handle, &hvx_params);
 }
 else
 {
  err_code = NRF_ERROR_INVALID_STATE;
 }



 uint8_t day_of_week;

// day_of_week = determine_day_of_week_by_gauss(p_current_date_time);

 gatts_value.len     = sizeof(uint8_t);
 gatts_value.offset  = 0;
 gatts_value.p_value = (uint8_t*)&day_of_week;

 err_code = sd_ble_gatts_value_set(p_cts->conn_handle,
  p_cts->day_of_week_handles.value_handle, &gatts_value);

 if (err_code != NRF_SUCCESS)
  return err_code;

 return NRF_SUCCESS;
}

void ble_hrs_on_gatt_evt(ble_cts_t * p_cts, nrf_ble_gatt_evt_t const * p_gatt_evt)
{
    if (    (p_cts->conn_handle == p_gatt_evt->conn_handle)
        &&  (p_gatt_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED))
    {
        p_cts->max_cts_len = p_gatt_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
    }
}

