/*
 * ble_cts.h
 *
 *  Created on: Dec 14, 2019
 *      Author: PARTH
 */

#ifndef APPLICATIONS_INC_BLE_CTS_H_
#define APPLICATIONS_INC_BLE_CTS_H_

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"
#include "nrf_sdh_ble.h"
#include "nrf_ble_gatt.h"




#include "ble_date_time.h"
#include "ble_db_discovery.h"

#define BLE_CTS_MAX_BUFFERED_RR_INTERVALS       20
#define CURRENT_RATE_LOCATION      2


#ifdef __cplusplus
extern "C" {
#endif

/**@brief   Macro for defining a ble_bps instance.
 *
 * @param   _name   Name of the instance.
 * @hideinitializer
 */
#define BLE_CTS_DEF(_name)                                                                        \
static ble_cts_t _name;                                                                           \
NRF_SDH_BLE_OBSERVER(_name ## _obs,                                                                 \
                     BLE_CTS_BLE_OBSERVER_PRIO,                                                   \
					 ble_cts_on_ble_evt, &_name)



/**@brief "Day Date Time" field of the "Exact Time 256" field of the Current Time Characteristic. */

typedef struct
{
    ble_date_time_t date_time;
    uint8_t         day_of_week;
} day_date_time_tt;


/**@brief "Exact Time 256" field of the Current Time Characteristic. */

typedef struct
{
    day_date_time_tt day_date_time;
    uint8_t         fractions256;
} exact_time_256_tt;


/**@brief "Adjust Reason" field of the Current Time Characteristic. */

typedef struct
{
    uint8_t manual_time_update              : 1;
    uint8_t external_reference_time_update  : 1;
    uint8_t change_of_time_zone             : 1;
    uint8_t change_of_daylight_savings_time : 1;
} adjust_reason_tt;


/**@brief Data structure for the Current Time Characteristic. */

typedef struct
{
    exact_time_256_tt exact_time_256;
    adjust_reason_tt  adjust_reason;
} current_time_char_tt;


// Forward declaration of the ble_cts_c_t type.
typedef struct ble_cts_s ble_cts_t;

/*
*@brief Current Time Service client event type.
typedef enum
{
    BLE_CTS_EVT_DISCOVERY_COMPLETE, *< The Current Time Service was found at the peer.
    BLE_CTS_EVT_DISCOVERY_FAILED,   *< The Current Time Service was not found at the peer.
    BLE_CTS_EVT_DISCONN_COMPLETE,   *< Event indicating that the Current Time Service client module has finished processing the BLE_GAP_EVT_DISCONNECTED event. This event is raised only if a valid instance of the Current Time Service was found at the server. The event can be used by the application to do clean up related to the Current Time Service client.
    BLE_CTS_EVT_CURRENT_TIME,       *< A new current time reading has been received.
    BLE_CTS_EVT_INVALID_TIME        *< The current time value received from the peer is invalid.
} ble_cts_evt_type_t;
*/

/**@brief Heart Rate Service event type. */
typedef enum
{
    BLE_CTS_EVT_NOTIFICATION_ENABLED,   /**< Heart Rate value notification enabled event. */
    BLE_CTS_EVT_NOTIFICATION_DISABLED   /**< Heart Rate value notification disabled event. */
} ble_cts_evt_type_t;




/**@brief Current Time Service client event. */
typedef struct
{
    ble_cts_evt_type_t evt_type;    /**< Type of event. */
} ble_cts_evt_t;



// Forward declaration of the ble_cts_t type.
typedef struct ble_cts_s ble_cts_t;




/**@brief Heart Rate Service event handler type. */
typedef void (*ble_cts_evt_handler_t ) (ble_cts_t * p_hrs, ble_cts_evt_t * p_evt);



/**@brief Structure containing the handles related to the current Rate Service found on the peer. */
typedef struct
{
    uint16_t cts_handle;       /**< Handle of the Current Time characteristic as provided by the SoftDevice. */
    uint16_t cts_cccd_handle;  /**< Handle of the CCCD of the Current Time characteristic. */
} ble_cts_handles_t;



/**@brief Current Time Service  init structure. This structure contains all options and data needed for initialization service.*/
typedef struct
{
    ble_cts_evt_handler_t evt_handler;   /**< Event handler to be called for handling events from the Current Time Service client. */
    ble_srv_error_handler_t error_handler; /**< Function to be called if an error occurs. */
    ble_srv_cccd_security_mode_t               cts_cccd_wr_sechh;     /**< Security requirement for writing the HRM characteristic CCCD. */
    ble_srv_security_mode_t               bsl_rd_sechh;                                           /**< Security requirement for reading the BSL characteristic value. */
    uint8_t *                  p_current_rate_location;

    bool       is_notification_supported;    /**< TRUE if notification of current time measurement is supported. */
    current_time_char_tt     init_date_time;
} ble_cts_init_t;




/**@brief Current Time Service server structure. This structure contains status information for services. */
struct ble_cts_s
{
    ble_cts_evt_handler_t   evt_handler;         /**< Event handler to be called for handling events from the Current Time Service client. */
    ble_srv_error_handler_t error_handler;       /**< Function to be called if an error occurs. */
    ble_cts_handles_t       char_handles;        /**< Handles of Current Time Characteristic at the peer (handles are provided by the BLE stack through the DB Discovery module). */
    uint16_t                conn_handle;         /**< Handle of the current connection. BLE_CONN_HANDLE_INVALID if not in a connection. */
    uint16_t                service_handle;                                       /**< Handle of Heart Rate Service (as provided by the BLE stack). */
    ble_gatts_char_handles_t cts_handles;                                          /**< Handles related to the Heart Rate Measurement characteristic. */
    ble_gatts_char_handles_t     bsll_handles;

    uint8_t                      max_cts_len; //**< Current maximum HR measurement length, adjusted according to the current ATT MTU.



   // ble_gatts_char_handles_t      date_time_handles;            /**< Handles related to the current time characteristic. */
  ble_gatts_char_handles_t      day_of_week_handles;    /**< Handles related to the day of week characteristic. */
     bool                          is_notification_supported;      /**< TRUE if notification of current time is supported. */
     current_time_char_tt      current_date_time;             /**< Last current time passed to the current time Service. */

 };



void print_time12(void);

/**@brief Function for initializing the Current Time Service client.
 *
 * @details This function must be used by the application to initialize the Current Time Service client.
 *
 * @param[out] p_cts Current Time Service client structure. This structure must
 *                   be supplied by the application. It is initialized by this
 *                   function and can later be used to identify this particular client
 *                   instance.
 * @param[in]  p_cts_init Information needed to initialize the Current Time Service client.
 *
 * @retval NRF_SUCCESS If the service was initialized successfully.
 */
uint32_t ble_cts_init(ble_cts_t * p_cts, const ble_cts_init_t * p_cts_init);


/**@brief Function for handling the GATT module's events.
 *
 * @details Handles all events from the GATT module of interest to the Heart Rate Service.
 *
 * @param[in]   p_cts      current time Service structure.
 * @param[in]   p_gatt_evt  Event received from the GATT module.
 */
//void ble_cts_on_gatt_evt(ble_cts_t * p_cts, nrf_ble_gatt_evt_t const * p_gatt_evt);



static uint32_t current_rate_measurement_char_add(ble_cts_t * p_cts,
                                                const ble_cts_init_t * p_cts_init);



/**@brief Function for handling the application's BLE stack events.
 *
 * @details This function handles all events from the BLE stack that are of interest to the
 *          Current Time Service client. This is a callback function that must be dispatched
 *          from main application context.
 *
 * @param[in] p_ble_evt     Event received from the BLE stack.
 * @param[in] p_context     Current Time Service client structure.
 */
void ble_cts_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context);

//static uint8_t cts_encode(ble_cts_t * p_cts, uint16_t current_rate, uint8_t * p_encoded_buffer);

//static uint8_t ctss_encode(const current_time_char_t * p_time, uint8_t * p_encoded_buffer);

//static  uint8_t ble_date_time_encode(const ble_date_time_t * p_date_time,
//                                             uint8_t *               p_encoded_data);

uint32_t ble_cts_time_send(ble_cts_t * p_cts, uint16_t time_t);



static void on_write(ble_cts_t *p_cts, ble_evt_t * p_ble_evt);

static uint8_t determine_day_of_week_by_gauss(ble_date_time_t *p_date);

/**@brief Function for reading the peer's Current Time Service Current Time Characteristic.
 *
 * @param[in] p_cts  Current Time Service client structure.
 *
 * @retval NRF_SUCCESS If the operation is successful. Otherwise, an error code is returned.
 */




current_time_char_tt currunt_time_updat();


#ifdef __cplusplus
}
#endif

#endif // BLE_CTS_C_H__

