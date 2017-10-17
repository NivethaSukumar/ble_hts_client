/**
 * Copyright (c) 2012 - 2017, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/**@file
 *
 * @defgroup ble_hts_c Health Thermometer Service Client
 * @{
 * @ingroup  ble_sdk_srv
 * @brief    Health Thermometer Service Client module.
 *
 * @details  This module contains the APIs and types exposed by the Health Thermometer Service Client
 *           module. These APIs and types can be used by the application to perform discovery of
 *           Health Thermometer Service at the peer and interact with it.
 *
 * @warning  Currently this module only has support for Health Thermometer Measurement characteristic. This
 *           means that it will be able to enable notification of the characteristic at the peer and
 *           be able to receive Health Thermometer Measurement notifications from the peer. It does not
 *           support the Body Sensor Location and the Health Thermometer Control Point characteristics.
 *           When a Health Thermometer Measurement is received, this module will decode only the
 *           Health Thermometer Measurement Value (both 8 bit and 16 bit) field from it and provide it to
 *           the application.
 *
 * @note     The application must propagate BLE stack events to this module by calling
 *           ble_hts_c_on_ble_evt().
 *
 */

#ifndef BLE_HTS_C_H__
#define BLE_HTS_C_H__

#include <stdint.h>
#include "ble.h"
#include "ble_db_discovery.h"
#include "sdk_config.h"

#ifdef __cplusplus
extern "C" {
#endif

// Temperature Type measurement locations
#define BLE_HTS_TEMP_TYPE_ARMPIT      1
#define BLE_HTS_TEMP_TYPE_BODY        2
#define BLE_HTS_TEMP_TYPE_EAR         3
#define BLE_HTS_TEMP_TYPE_FINGER      4
#define BLE_HTS_TEMP_TYPE_GI_TRACT    5
#define BLE_HTS_TEMP_TYPE_MOUTH       6
#define BLE_HTS_TEMP_TYPE_RECTUM      7
#define BLE_HTS_TEMP_TYPE_TOE         8
#define BLE_HTS_TEMP_TYPE_EAR_DRUM    9



/**
 * @defgroup hts_c_enums Enumerations
 * @{
 */

/**@brief hts Client event type. */
typedef enum
{
    BLE_HTS_C_EVT_DISCOVERY_COMPLETE = 1,  /**< Event indicating that the Health Thermometer Service has been discovered at the peer. */
    BLE_HTS_C_EVT_HTS_NOTIFICATION,         /**< Event indicating that a notification of the Health Thermometer Measurement characteristic has been received from the peer. */
	BLE_HTS_C_EVT_HTS_READ_RESP
} ble_hts_c_evt_type_t;

/** @} */

/**
 * @defgroup hts_c_structs Structures
 * @{
 */

/**@brief Structure containing the Health Thermometer measurement received from the peer. */
typedef struct
{
    uint16_t temp_value;                                        /**< Health Thermometer Value. */
    uint8_t  temp_type;                                /**< Temperature type. */
//    ble_date_time_t	time_stamp;						/**<Timestamp. */
} ble_hts_t;


/**@brief Structure containing the handles related to the Health Thermometer Service found on the peer. */
typedef struct
{
    uint16_t hts_cccd_handle;  /**< Handle of the CCCD of the Health Thermometer Measurement characteristic. */
    uint16_t hts_handle;       /**< Handle of the Health Thermometer Measurement characteristic as provided by the SoftDevice. */
} hts_db_t;


/**@brief Health Thermometer Event structure. */
typedef struct
{
    ble_hts_c_evt_type_t evt_type;    /**< Type of the event. */
    uint16_t             conn_handle; /**< Connection handle on which the Health Thermometer service was discovered on the peer device..*/
    union
    {
        hts_db_t  hts_db;            /**< Health Thermometer related handles found on the peer device.. This will be filled if the evt_type is @ref BLE_HTS_C_EVT_DISCOVERY_COMPLETE.*/
        ble_hts_t hts;                /**< Health Thermometer measurement received. This will be filled if the evt_type is @ref BLE_hts_C_EVT_HRM_NOTIFICATION. */
        ble_hts_t temp_type;		/**<Health Temperature type. */
    } params;
} ble_hts_c_evt_t;

/** @} */

/**
 * @defgroup hts_c_types Types
 * @{
 */

// Forward declaration of the ble_bas_t type.
typedef struct ble_hts_c_s ble_hts_c_t;

/**@brief   Event handler type.
 *
 * @details This is the type of the event handler that should be provided by the application
 *          of this module in order to receive events.
 */
typedef void (* ble_hts_c_evt_handler_t) (ble_hts_c_t * p_ble_hts_c, ble_hts_c_evt_t * p_evt);

/** @} */

/**
 * @addtogroup hts_c_structs
 * @{
 */

/**@brief Health Thermometer Client structure.
 */
struct ble_hts_c_s
{
    uint16_t                conn_handle;      /**< Connection handle as provided by the SoftDevice. */
    hts_db_t                peer_hts_db;      /**< Handles related to HTS on the peer*/
    ble_hts_c_evt_handler_t evt_handler;      /**< Application event handler to be called when there is an event related to the Health Thermometer service. */
};

/**@brief Health Thermometer Client initialization structure.
 */
typedef struct
{
    ble_hts_c_evt_handler_t evt_handler;  /**< Event handler to be called by the Health Thermometer Client module whenever there is an event related to the Health Thermometer Service. */
} ble_hts_c_init_t;

/** @} */

/**
 * @defgroup hts_c_functions Functions
 * @{
 */

/**@brief     Function for initializing the Health Thermometer client module.
 *
 * @details   This function will register with the DB Discovery module. There it
 *            registers for the Health Thermometer Service. Doing so will make the DB Discovery
 *            module look for the presence of a Health Thermometer Service instance at the peer when a
 *            discovery is started.
 *
 * @param[in] p_ble_hts_c      Pointer to the Health Thermometer client structure.
 * @param[in] p_ble_hts_c_init Pointer to the Health Thermometer initialization structure containing the
 *                             initialization information.
 *
 * @retval    NRF_SUCCESS On successful initialization. Otherwise an error code. This function
 *                        propagates the error code returned by the Database Discovery module API
 *                        @ref ble_db_discovery_evt_register.
 */
uint32_t ble_hts_c_init(ble_hts_c_t * p_ble_hts_c, ble_hts_c_init_t * p_ble_hts_c_init);

/**@brief     Function for handling BLE events from the SoftDevice.
 *
 * @details   This function will handle the BLE events received from the SoftDevice. If a BLE
 *            event is relevant to the Health Thermometer Client module, then it uses it to update
 *            interval variables and, if necessary, send events to the application.
 *
 * @param[in] p_ble_hts_c Pointer to the Health Thermometer client structure.
 * @param[in] p_ble_evt   Pointer to the BLE event.
 */
void ble_hts_c_on_ble_evt(ble_hts_c_t * p_ble_hts_c, const ble_evt_t * p_ble_evt);


/**@brief   Function for requesting the peer to start sending notification of Health Thermometer
 *          Measurement.
 *
 * @details This function will enable to notification of the Health Thermometer Measurement at the peer
 *          by writing to the CCCD of the Health Thermometer Measurement Characteristic.
 *
 * @param   p_ble_hts_c Pointer to the Health Thermometer client structure.
 *
 * @retval  NRF_SUCCESS If the SoftDevice has been requested to write to the CCCD of the peer.
 *                      Otherwise, an error code. This function propagates the error code returned
 *                      by the SoftDevice API @ref sd_ble_gattc_write.
 */
uint32_t ble_hts_c_hts_notif_enable(ble_hts_c_t * p_ble_hts_c);


/**@brief     Function for handling events from the database discovery module.
 *
 * @details   Call this function when getting a callback event from the DB discovery modue.
 *            This function will handle an event from the database discovery module, and determine
 *            if it relates to the discovery of Health Thermometer service at the peer. If so, it will
 *            call the application's event handler indicating that the Health Thermometer service has been
 *            discovered at the peer. It also populates the event with the service related
 *            information before providing it to the application.
 *
 * @param[in] p_ble_hts_c Pointer to the Health Thermometer client structure instance to associate.
 * @param[in] p_evt Pointer to the event received from the database discovery module.
 *
 */
void ble_hts_on_db_disc_evt(ble_hts_c_t * p_ble_hts_c, const ble_db_discovery_evt_t * p_evt);


/**@brief     Function for assigning a handles to a this instance of hts_c.
 *
 * @details   Call this function when a link has been established with a peer to
 *            associate this link to this instance of the module. This makes it
 *            possible to handle several link and associate each link to a particular
 *            instance of this module.The connection handle and attribute handles will be
 *            provided from the discovery event @ref BLE_hts_C_EVT_DISCOVERY_COMPLETE.
 *
 * @param[in] p_ble_hts_c        Pointer to the Health Thermometer client structure instance to associate.
 * @param[in] conn_handle        Connection handle to associated with the given Health Thermometer Client Instance.
 * @param[in] p_peer_hts_handles Attribute handles for the hts server you want this hts_C client to
 *                               interact with.
 */
uint32_t ble_hts_c_handles_assign(ble_hts_c_t *    p_ble_hts_c,
                                  uint16_t         conn_handle,
                                  const hts_db_t * p_peer_hts_handles);

/** @} */ // End tag for Function group.


#ifdef __cplusplus
}
#endif

#endif // BLE_hts_C_H__

/** @} */ // End tag for the file.
