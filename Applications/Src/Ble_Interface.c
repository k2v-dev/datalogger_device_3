/*
 * Ble_Interface.c
 *
 *  Created on: 05-Dec-2019
 *      Author: GTsilicon
 */
/** @file
 *
 * @defgroup ble_sdk_uart_over_ble_main main.c
 * @{
 * @ingroup  ble_sdk_app_nus_eval
 * @brief    UART over BLE application main file.
 *
 * This file contains the source code for a ble applications that uses the Nordic UART service for DEV2 (neck)
 * This application uses the @ref srvlib_conn_params module.
 */



#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "Command_Interface.h"
#include "Ble_Interface.h"
#include "RTC_updated.h"
#include "nrf.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_nus_c.h"
#include "ble_cts_c.h"
#include "ble_cts.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_delay.h"
#include "nrf_sdh_ble.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "nrf_ble_scan.h"
#include "ble_db_discovery.h"
#include "app_timer.h"
#include "ble_nus.h"
#include "app_uart.h"
#include "app_util_platform.h"
#include "bsp_btn_ble.h"
#include "nrf_pwr_mgmt.h"
#include "time.h"
#include "Format_Sensor_Data.h"
#include "app_button.h"
#include "bsp.h"

#include "nrf_drv_gpiote.h"

#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define APP_BLE_CONN_CFG_TAG            1                                           /**< A tag identifying the SoftDevice BLE configuration. */

#define DEVICE_NAME                     "Button_Device3"                               /**< Name of device. Will be included in the advertising data. */
#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */

#define APP_BLE_OBSERVER_PRIO           3                                           /**< Application's BLE observer priority. You shouldn't need to modify this value. */

#define APP_ADV_INTERVAL                64                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */

#define APP_ADV_DURATION                0                                       /**< The advertising duration (180 seconds) in units of 10 milliseconds. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(10, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (10 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(20, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (10 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(6000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                       /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                      /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */
#define RATE				            APP_TIMER_TICKS(10)							/**<  Data rate or Interrupt rate @ Data is send over BLE */


#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define UART_TX_BUF_SIZE                256                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                256                                         /**< UART RX buffer size. */


BLE_NUS_DEF(m_nus, NRF_SDH_BLE_TOTAL_LINK_COUNT);                                   /**< BLE NUS service instance. */
NRF_BLE_GATT_DEF(m_gatt);                                                           /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                             /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                                                 /**< Advertising module instance. */
BLE_DB_DISCOVERY_DEF(m_db_disc);                                                    /**< Database discovery module instance. */

BLE_CTS_C_DEF(m_cts_c);																/**< CTS Service Instantiate. */
static ble_cts_t m_cts;																/**< CTS Service Struct    */


static ble_gap_scan_params_t m_scan_param =                 /**< Scan parameters requested for scanning and connection. */
{
    .active        = 0x01,
    .interval      = NRF_BLE_SCAN_SCAN_INTERVAL,
    .window        = NRF_BLE_SCAN_SCAN_WINDOW,
    .filter_policy = BLE_GAP_SCAN_FP_ACCEPT_ALL,
    .timeout       = NRF_BLE_SCAN_SCAN_DURATION,
    .scan_phys     = BLE_GAP_PHY_AUTO ,
    .extended      = true,
};




static uint16_t   m_conn_handle          = BLE_CONN_HANDLE_INVALID;                 /**< Handle of the current connection. */
static uint16_t   m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3;            /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */
static ble_uuid_t m_adv_uuids[]          =                                          /**< Universally unique service identifier. */
{
		//
		{BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE},
		{BLE_UUID_CURRENT_TIME_SERVICE, BLE_UUID_TYPE_BLE}

};
APP_TIMER_DEF(timer_id);
uint8_t rcvd_cmd_id;
uint8_t rcvd_cmd_rate;
uint16_t check_cheksum;
bool turn_on_output;
bool central;
uint8_t rate_divider;
uint8_t send_counter;
struct tm * current_time;
uint8_t milisec;
bool time_recived =false;

stability status;

/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyse
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}




static void send_data_over_ble(void)
{
	if(turn_on_output)
	{
		NRF_LOG_INFO("Sending data over BLE");
		send_counter ++;
		finite_state_machine_select_command();
		uint16_t length_output = (uint16_t)k;
		if((rate_divider !=0)&&(send_counter == rate_divider))
		{
			ble_nus_data_send(&m_nus,data_array_output,&length_output,m_conn_handle);
			cnt++;
			send_counter=0;
		}
	}

	milisec++;
	//if(milisec==99)
	//milisec=0;

}

/**@brief Function for initializing the timer module.
 */
static void timers_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
    err_code = app_timer_create(&timer_id,
    	                                APP_TIMER_MODE_REPEATED,
										send_data_over_ble);
   // if (NRF_RTC1->COUNTER % 320)
    //	send_data_over_ble();
}

/**@brief Function for the GAP initialization.
 *
 * @details This function will set up all the necessary GAP (Generic Access Profile) parameters of
 *          the device. It also sets the permissions and appearance.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *) DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);
    err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_HEART_RATE_SENSOR_HEART_RATE_BELT);
     APP_ERROR_CHECK(err_code);
    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}

static void application_timers_start(void)
{
	ret_code_t err_code;

	    // Start application timers.
	    err_code = app_timer_start(timer_id,RATE, NULL);
	    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

static bool inspect_checksum(void)
{
  static uint8_t ack[3];
  static uint8_t date[3];
  uint32_t err_code;
  //current_time = nrf_cal_get_time();
	if (check_cheksum ==(rcvd_cmd_id + rcvd_cmd_rate))
	{
		rate_divider=rcvd_cmd_rate;
		ack[0]=rcvd_cmd_id;
		ack[1]= 00;
		ack[2]=rcvd_cmd_id;
		date[0] = (uint8_t)current_time->tm_mday;
		date[1] = (uint8_t)current_time->tm_mon+1;
		date[2] = (uint8_t)current_time->tm_year;
		uint16_t length =3;
		err_code = ble_nus_data_send(&m_nus,ack,&length,m_conn_handle);
		NRF_LOG_INFO("ACK Send");
		err_code = ble_nus_data_send(&m_nus,date,&length,m_conn_handle);
		NRF_LOG_INFO("Date Send");
	//	APP_ERROR_CHECK(err_code);
		if (err_code !=0)
		{
			NRF_LOG_INFO("turn on notification");
			return false;
		}

		//NRF_LOG_INFO("ERR: %d",err_code);
	    return true;
	}
	else
	{
		NRF_LOG_INFO("Wrong Checksum");
		return false;
	}
}


/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *          it to the UART module.
 *
 * @param[in] p_evt       Nordic UART Service event.
 */
/**@snippet [Handling the data received over BLE] */
static void nus_data_handler(ble_nus_evt_t * p_evt)
{

    if (p_evt->type == BLE_NUS_EVT_RX_DATA)
    {
        uint32_t err_code;

        NRF_LOG_INFO("Received data from BLE NUS. Writing data on UART.");
        NRF_LOG_HEXDUMP_DEBUG(p_evt->params.rx_data.p_data, p_evt->params.rx_data.length);

        for (uint32_t i = 0; i < p_evt->params.rx_data.length; i++)
        {
            do
            {
            	NRF_LOG_INFO("dATA: %d",p_evt->params.rx_data.p_data[i]);
            //	err_code = app_uart_put(p_evt->params.rx_data.p_data[i]);0x2902


                if ((err_code != NRF_SUCCESS) && (err_code != NRF_ERROR_BUSY))
                {
                    NRF_LOG_ERROR("Failed receiving NUS message. Error 0x%x. ", err_code);
                    APP_ERROR_CHECK(err_code);
                }
            } while (err_code == NRF_ERROR_BUSY);
        }
//        	rcvd_cmd_id = (((p_evt->params.rx_data.p_data[0]-48)*10)+(p_evt->params.rx_data.p_data[1]-48));
//        	rcvd_cmd_rate = (((p_evt->params.rx_data.p_data[2]-48)*10)+(p_evt->params.rx_data.p_data[3]-48));
//            check_cheksum = (((p_evt->params.rx_data.p_data[4]-48)*1000)+((p_evt->params.rx_data.p_data[5]-48)*100)
//            				+((p_evt->params.rx_data.p_data[6]-48)*10) +(p_evt->params.rx_data.p_data[7]-48));
        	rcvd_cmd_id =(p_evt->params.rx_data.p_data[0]);
        	rcvd_cmd_rate = (p_evt->params.rx_data.p_data[1]);
        	check_cheksum = (((uint16_t)p_evt->params.rx_data.p_data[2] <<8)|(p_evt->params.rx_data.p_data[3]&0xff));
            NRF_LOG_INFO("CKS: %d",check_cheksum);
            if (inspect_checksum())
            {
            	NRF_LOG_INFO("Send_DATA_over Ble");
            	turn_on_output = true;
            }
            else
            {
            	turn_on_output =false;
            }

        if (p_evt->params.rx_data.p_data[p_evt->params.rx_data.length - 1] == '\r')
        {
        	NRF_LOG_INFO("Done");
        }
    }

}


////RTC////////


/**********************************Time Print**********************************************/
void print_current_time(void)
{
	current_time = nrf_cal_get_time();
	NRF_LOG_INFO("Uncalibrated time:\t%s\r\n", nrf_cal_get_time_string(false));
    NRF_LOG_INFO("Calibrated time:\t%s\r\n", nrf_cal_get_time_string(true));
    NRF_LOG_INFO("Date:\t%d/%d/%d\r\n",current_time->tm_mday,(current_time->tm_mon+1),current_time->tm_year);
    NRF_LOG_INFO("Time:\t%d:%d:%d\r\n",current_time->tm_hour,(current_time->tm_min),current_time->tm_sec);
}


/*************************************End*************************************************/

/******************************************CTS********************************************/
/**@brief Function for handling the Current Time Service errors.
 *
 * @param[in]  nrf_error  Error code containing information about what went wrong.
 */
static void current_time_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for handling the Current Time Service errors.
 *
 * @param[in] p_evt  Event received from the Current Time Service client.
 */
static void current_time_print(ble_cts_c_evt_t * p_evt)
{


	uint32_t year, month ,day, hour, minute, second, fraction;
	year =p_evt->params.current_time.exact_time_256.day_date_time.date_time.year;
	month =p_evt->params.current_time.exact_time_256.day_date_time.date_time.month;
	day =p_evt->params.current_time.exact_time_256.day_date_time.date_time.day;
	hour = p_evt->params.current_time.exact_time_256.day_date_time.date_time.hours;
	minute =p_evt->params.current_time.exact_time_256.day_date_time.date_time.minutes;
	second =p_evt->params.current_time.exact_time_256.day_date_time.date_time.seconds;
	fraction = p_evt->params.current_time.exact_time_256.fractions256;
	milisec = (uint8_t)(100*fraction/256);

	nrf_cal_set_time(year,month,day,hour, minute , second);
	print_current_time();
	NRF_LOG_INFO("\r\nCurrent Time:");
    NRF_LOG_INFO("\r\nDate:");

    NRF_LOG_INFO("\tDay of week   %s", (uint32_t)day_of_week[p_evt->
                                                         params.
                                                         current_time.
                                                         exact_time_256.
                                                         day_date_time.
                                                         day_of_week]);

    if (p_evt->params.current_time.exact_time_256.day_date_time.date_time.day == 0)
    {
        NRF_LOG_INFO("\tDay of month  Unknown");
    }
    else
    {
        NRF_LOG_INFO("\tDay of month  %i",
                       p_evt->params.current_time.exact_time_256.day_date_time.date_time.day);
    }

    NRF_LOG_INFO("\tMonth of year %s",
    (uint32_t)month_of_year[p_evt->params.current_time.exact_time_256.day_date_time.date_time.month]);
    if (p_evt->params.current_time.exact_time_256.day_date_time.date_time.year == 0)
    {
        NRF_LOG_INFO("\tYear          Unknown");
    }
    else
    {
        NRF_LOG_INFO("\tYear          %i",
                       p_evt->params.current_time.exact_time_256.day_date_time.date_time.year);
    }
    NRF_LOG_INFO("\r\nTime:");
    NRF_LOG_INFO("\tHours     %i",
                   p_evt->params.current_time.exact_time_256.day_date_time.date_time.hours);
    NRF_LOG_INFO("\tMinutes   %i",
                   p_evt->params.current_time.exact_time_256.day_date_time.date_time.minutes);
    NRF_LOG_INFO("\tSeconds   %i",
                   p_evt->params.current_time.exact_time_256.day_date_time.date_time.seconds);
    NRF_LOG_INFO("\tFractions %i/256 of a second",
                   p_evt->params.current_time.exact_time_256.fractions256);

    NRF_LOG_INFO("\r\nAdjust reason:\r");
    NRF_LOG_INFO("\tDaylight savings %x",
                   p_evt->params.current_time.adjust_reason.change_of_daylight_savings_time);
    NRF_LOG_INFO("\tTime zone        %x",
                   p_evt->params.current_time.adjust_reason.change_of_time_zone);
    NRF_LOG_INFO("\tExternal update  %x",
                   p_evt->params.current_time.adjust_reason.external_reference_time_update);
    NRF_LOG_INFO("\tManual update    %x",
                   p_evt->params.current_time.adjust_reason.manual_time_update);


}


/**@brief Function for handling the Current Time Service client events.
 *
 * @details This function will be called for all events in the Current Time Service client that
 *          are passed to the application.
 *
 * @param[in] p_evt Event received from the Current Time Service client.
 */
static void on_cts_c_evt(ble_cts_c_t * p_cts, ble_cts_c_evt_t * p_evt)
{
    ret_code_t err_code;

    switch (p_evt->evt_type)
    {
        case BLE_CTS_C_EVT_DISCOVERY_COMPLETE:
            NRF_LOG_INFO("Current Time Service discovered on server.");
            err_code = ble_cts_c_handles_assign(&m_cts_c,
                                                p_evt->conn_handle,
                                                &p_evt->params.char_handles);
            APP_ERROR_CHECK(err_code);
            err_code = ble_cts_c_current_time_read(&m_cts_c);
            APP_ERROR_CHECK(err_code);

            break;

        case BLE_CTS_C_EVT_DISCOVERY_FAILED:
            NRF_LOG_INFO("Current Time Service not found on server. ");
            break;

        case BLE_CTS_C_EVT_DISCONN_COMPLETE:
            NRF_LOG_INFO("Disconnect Complete.");
            break;

        case BLE_CTS_C_EVT_CURRENT_TIME:
            NRF_LOG_INFO("Current Time received.");
            current_time_print(p_evt);
            time_recived =  1;



            break;

        case BLE_CTS_C_EVT_INVALID_TIME:
            NRF_LOG_INFO("Invalid Time received.");


            break;

        default:
            break;
    }
}




//struct tm send_time;
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{

    ble_cts_on_ble_evt(&m_cts, p_ble_evt);



}


/*************************************************CTS_Server************************************************/

//uint32_t ble_cts_init(ble_hrs_t * p_hrs, const ble_hrs_init_t * p_hrs_init)
//{
//    return NRF_SUCCESS;
//}

/*************************************************CTS END*************************************************/
/**@snippet [Handling the data received over BLE] */


/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    uint32_t           err_code;
    ble_nus_init_t     nus_init;
    ble_cts_c_init_t   cts_c_init = {0};   // client
    nrf_ble_qwr_init_t qwr_init = {0};
//    ble_cts_init_t   cts_init = {0};		//Server



    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);

    // Initialize NUS.
    memset(&nus_init, 0, sizeof(nus_init));

    nus_init.data_handler = nus_data_handler;

    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);
        // Initialize CTS.
    memset(&cts_c_init, 0, sizeof(cts_c_init));
    cts_c_init.evt_handler   = on_cts_c_evt;
    cts_c_init.error_handler = current_time_error_handler;
    err_code               = ble_cts_c_init(&m_cts_c, &cts_c_init);
    APP_ERROR_CHECK(err_code);


    // Server CTS Initialize

/*
    memset(&cts_init, 0, sizeof(cts_init));

    cts_init.evt_handler = ble_cts_on_ble_evt;
    cts_init.is_notification_supported=true;
    cts_init.p_current_rate_location = currunt_rate;


    // Here the sec level for the current Rate Service can be changed/increased.

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cts_init. cts_cccd_wr_sechh.cccd_write_perm);

    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&cts_init. cts_cccd_wr_sechh.read_perm);

    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&cts_init. cts_cccd_wr_sechh.write_perm);

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cts_init.bsl_rd_sechh.read_perm);

    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&cts_init.bsl_rd_sechh.write_perm);

    currunt_time_updat(&m_cts_c);
if (time_recived == 1)
{
    cts_init.init_date_time =currunt_time_updat();
    cts_init.init_date_time =  ble_cts_update(&m_cts,&cts_init);

    err_code = ble_cts_init(&m_cts, &cts_init);

    APP_ERROR_CHECK(err_code);
}
}
*/


}


static void cts_init(void)
{

	 uint32_t           err_code;
	 ble_cts_init_t   cts_init = {0};		//Server

	    memset(&cts_init, 0, sizeof(cts_init));

	    cts_init.evt_handler = ble_cts_on_ble_evt;
	    cts_init.is_notification_supported=true;
	//    cts_init.p_current_rate_location = currunt_rate;


	    // Here the sec level for the current Rate Service can be changed/increased.

	    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cts_init. cts_cccd_wr_sechh.cccd_write_perm);

	    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&cts_init. cts_cccd_wr_sechh.read_perm);

	    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&cts_init. cts_cccd_wr_sechh.write_perm);

	    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cts_init.bsl_rd_sechh.read_perm);

	    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&cts_init.bsl_rd_sechh.write_perm);



	    cts_init.init_date_time =currunt_time_updat();


	    ble_cts_init(&m_cts, &cts_init);

	    APP_ERROR_CHECK(err_code);


}



/**@brief Function for handling an event from the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module
 *          which are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply setting
 *       the disconnect_on_fail config parameter, but instead we use the event handler
 *       mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling errors from the Connection Parameters module.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    uint32_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    uint32_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            bsp_board_led_on(BSP_BOARD_LED_1);
            APP_ERROR_CHECK(err_code);
            break;
        case BLE_ADV_EVT_IDLE:
            sleep_mode_enter();
            break;
        default:
            break;
    }
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_peripherial_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    uint32_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Client Connected");
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);
            central =false;
            // start discovery of services. CTS statrt
            err_code = ble_db_discovery_start(&m_db_disc, p_ble_evt->evt.gap_evt.conn_handle);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected");
            // LED indication will be changed when advertising starts.
            turn_on_output =false;
            NRF_LOG_INFO("Disconnected, reason %d\r\n",
                                      p_ble_evt->evt.gap_evt.params.disconnected.reason);
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            if (p_ble_evt->evt.gap_evt.conn_handle == m_cts_c.conn_handle)
               {
                   m_cts_c.conn_handle = BLE_CONN_HANDLE_INVALID;
                }
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            break;
    }
}




void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
	 uint16_t conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
	 uint16_t role        = ble_conn_state_role(conn_handle);

	ble_peripherial_evt_handler(p_ble_evt,p_context);

}


/**@brief Function for the SoftDevice initialization.
 *
 * @details This function initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}


/**@brief Function for handling events from the GATT library. */
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    if ((m_conn_handle == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED))
    {
        m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
        NRF_LOG_INFO("Data len is set to 0x%X(%d)", m_ble_nus_max_data_len, m_ble_nus_max_data_len);
    }
    NRF_LOG_DEBUG("ATT MTU exchange completed. central 0x%x peripheral 0x%x",
                  p_gatt->att_mtu_desired_central,
                  p_gatt->att_mtu_desired_periph);
}


/**@brief Function for initializing the GATT library. */
void gatt_init(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
    APP_ERROR_CHECK(err_code);

//    /**server**/
//    err_code = nrf_ble_gatt_att_mtu_central_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
//       APP_ERROR_CHECK(err_code);

}

uint32_t timer;
uint32_t time_gap;

/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
void bsp_event_handler(bsp_event_t event , uint8_t button_action)
{
    uint32_t err_code;
    switch (event)
    {
        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break;

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BSP_EVENT_WHITELIST_OFF:
            if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
            {
                err_code = ble_advertising_restart_without_whitelist(&m_advertising);
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }

            }
            break;
        	case BSP_EVENT_KEY_0:
              if (m_cts_c.conn_handle != BLE_CONN_HANDLE_INVALID)
              {
            	  err_code = ble_cts_c_current_time_read(&m_cts_c);
                  if (err_code == NRF_ERROR_NOT_FOUND)
                  {
                      NRF_LOG_INFO("Current Time Service is not discovered.");
                  }
            }
            break;




            // button box


       case BSP_EVENT_KEY_1:		// red button
    		if(bsp_button_is_pressed(BSP_BOARD_BUTTON_1)){

    			timer =NRF_RTC1->COUNTER;

    		}
    		break;
       case BSP_EVENT_KEY_1_1:		// red button
           		time_gap =NRF_RTC1->COUNTER - timer;
           		NRF_LOG_INFO("Time %d",time_gap);
           		if(time_gap<25000)
           		{
           			status = Poor;
           			NRF_LOG_INFO("Poor %d",Poor);
           		}
           		else
           		{
           		    status = Very_Poor;
           		    NRF_LOG_INFO("Very Poor %d",Very_Poor);
                }
           		bsp_board_led_off(BSP_BOARD_LED_1);
           		nrf_delay_ms(500);
           		bsp_board_led_on(BSP_BOARD_LED_1);
           		nrf_delay_ms(500);
           		bsp_board_led_off(BSP_BOARD_LED_1);
           		nrf_delay_ms(500);
           		bsp_board_led_on(BSP_BOARD_LED_1);
           	//	}
           	break;

        case BSP_EVENT_KEY_2:			//no opinion
        	status = No_Opinion;
        	NRF_LOG_INFO("No opiniton %d",No_Opinion);
        	bsp_board_led_off(BSP_BOARD_LED_1);
        	nrf_delay_ms(500);
        	bsp_board_led_on(BSP_BOARD_LED_1);
        	nrf_delay_ms(500);
        	bsp_board_led_off(BSP_BOARD_LED_1);
        	nrf_delay_ms(500);
        	bsp_board_led_on(BSP_BOARD_LED_1);
        	break;


        case BSP_EVENT_KEY_3:		// red button
     		if(bsp_button_is_pressed(BSP_BOARD_BUTTON_3)){

     			timer =NRF_RTC1->COUNTER;

     		}
     		break;
        case BSP_EVENT_KEY_3_3:		// red button
            		time_gap =NRF_RTC1->COUNTER - timer;
            		NRF_LOG_INFO("Time %d",time_gap);
            		if(time_gap<25000)
            		{
            			status = Good;
            			NRF_LOG_INFO("Good %d",Good);
            		}
            		else
            		{
            		    status = Very_Good;
            		    NRF_LOG_INFO("Very_Good %d",Very_Good);
                 }
            		bsp_board_led_off(BSP_BOARD_LED_1);
            		nrf_delay_ms(500);
            		bsp_board_led_on(BSP_BOARD_LED_1);
            		nrf_delay_ms(500);
            		bsp_board_led_off(BSP_BOARD_LED_1);
            		nrf_delay_ms(500);
            		bsp_board_led_on(BSP_BOARD_LED_1);
            	//	}
            	break;
//        case BSP_EVENT_KEY_3:			// green button
//
//        	if(bsp_button_is_pressed(BSP_BOARD_BUTTON_3)){
//        		nrf_delay_ms(900);
//                status = Good;
//                NRF_LOG_INFO("good %d",Good);
////              bsp_board_led_on(BSP_BOARD_LED_2);
//           }
//
//           if(bsp_button_is_pressed(BSP_BOARD_BUTTON_3)){
//             	nrf_delay_ms(900);
//                status = Very_Good;
//                NRF_LOG_INFO("very good %d",Very_Good);
////              bsp_board_led_on(BSP_BOARD_LED_3);
//           }
//
//        	bsp_board_led_off(BSP_BOARD_LED_1);
//        	nrf_delay_ms(100);
//        	bsp_board_led_on(BSP_BOARD_LED_1);
//        	nrf_delay_ms(100);
//        	bsp_board_led_off(BSP_BOARD_LED_1);
//        	nrf_delay_ms(100);
//        	bsp_board_led_on(BSP_BOARD_LED_1);
//        	break;


        default:
            break;
    }
}
/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    uint32_t               err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

    init.advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance = false;
    init.advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;

    init.srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.srdata.uuids_complete.p_uuids  = m_adv_uuids;

    init.config.ble_adv_fast_enabled  = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout  = APP_ADV_DURATION;
    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}


/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */




static void buttons_leds_init(bool * p_erase_bonds)
{
    bsp_event_t startup_event;

//    uint32_t err_code = bsp_init(BSP_INIT_LEDS   , bsp_event_handler);
//    APP_ERROR_CHECK(err_code);

    uint32_t err_code = bsp_led_btn_init(BSP_INIT_LEDS |BSP_INIT_BUTTONS  , bsp_event_handler);
     APP_ERROR_CHECK(err_code);

     err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);
    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);

}

/**@brief Function for initializing the nrf log module.ble_db_discovery_on_ble_evt
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}


/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    uint32_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling database discovery events.
 *
 * @details This function is a callback function to handle events from the database discovery module.
 *          Depending on the UUIDs that are discovered, this function forwards the events
 *          to their respective services.
 *
 * @param[in] p_event  Pointer to the database discovery event.
 */
static void db_disc_handler(ble_db_discovery_evt_t * p_evt)
{

		NRF_LOG_INFO("cts discovery");
		ble_cts_c_on_db_disc_evt(&m_cts_c, p_evt);

}

/** @brief Function for initializing the database discovery module. */
static void db_discovery_init(void)
{
    ret_code_t err_code = ble_db_discovery_init(db_disc_handler);
    APP_ERROR_CHECK(err_code);
}





/**@brief Application main function.
 */
int ble_init(void)
{
    bool erase_bonds;

    log_init();
    timers_init();
    buttons_leds_init(&erase_bonds);

    db_discovery_init();
    nrf_cal_init();
    nrf_cal_set_callback(calendar_updated, 4);
    power_management_init();
    ble_stack_init();
    gap_params_init();
    gatt_init();
    services_init();
    advertising_init();
    conn_params_init();

   // nus_c_init();
   // scan_init();




    // Start execution.
    NRF_LOG_INFO("Logging for over RTT started.");
    application_timers_start();
    advertising_start();


    // Enter main loop.
    for (;;)
    {
        idle_state_handle();
     //   NRF_LOG_INFO("Calibrated time:\t%s\r\n", nrf_cal_get_time_string(true));
       // nrf_delay_ms(500);

    }
}

/**************************************************UART CODE (if needed in future)***********************************************/

/**@brief   Function for handling app_uart events.
 *
 * @details This function will receive a single character from the app_uart module and append it to
 *          a string. The string will be be sent over BLE when the last character received was a
 *          'new line' '\n' (hex 0x0A) or if the string has reached the maximum data length.
 */
/**@snippet [Handling the data received over UART] */
void uart_event_handle(app_uart_evt_t * p_event)
{
    static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];
    static uint8_t index = 0;
    uint32_t       err_code;

    switch (p_event->evt_type)
    {
        case APP_UART_DATA_READY:
            UNUSED_VARIABLE(app_uart_get(&data_array[index]));
            index++;

            if ((data_array[index - 1] == '\n') ||
                (data_array[index - 1] == '\r') ||
                (index >= m_ble_nus_max_data_len))
            {
                if (index > 1)
                {
                    NRF_LOG_INFO("Ready to send data over BLE NUS");
                    NRF_LOG_HEXDUMP_DEBUG(data_array, index);

                    do
                    {
                        uint16_t length = (uint16_t)index;
                        err_code = ble_nus_data_send(&m_nus, data_array, &length, m_conn_handle);
                        if ((err_code != NRF_ERROR_INVALID_STATE) &&
                            (err_code != NRF_ERROR_RESOURCES) &&
                            (err_code != NRF_ERROR_NOT_FOUND))
                        {
                            APP_ERROR_CHECK(err_code);
                        }
                    } while (err_code == NRF_ERROR_RESOURCES);
                }

                index = 0;
            }
            break;

        case APP_UART_COMMUNICATION_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_communication);
            break;

        case APP_UART_FIFO_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_code);
            break;

        default:
            break;
    }
}
/**@snippet [Handling the data received over UART] */

/**@brief  Function for initializing the UART module.
 */
/**@snippet [UART Initialization] */
static void uart_init(void)
{
    uint32_t                     err_code;
    app_uart_comm_params_t const comm_params =
    {
        .rx_pin_no    = RX_PIN_NUMBER,
        .tx_pin_no    = TX_PIN_NUMBER,
        .rts_pin_no   = RTS_PIN_NUMBER,
        .cts_pin_no   = CTS_PIN_NUMBER,
        .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
        .use_parity   = false,
#if defined (UART_PRESENT)
        .baud_rate    = NRF_UART_BAUDRATE_115200
#else
        .baud_rate    = NRF_UARTE_BAUDRATE_115200
#endif
    };

    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_event_handle,
                       APP_IRQ_PRIORITY_LOWEST,
                       err_code);
    APP_ERROR_CHECK(err_code);
}
/**@snippet [UART Initialization] */



/**
 * @}NRF_SUCCESS
 */
