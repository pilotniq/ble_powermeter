/**
 *  Implement a custom BLE service for an AC poweremter, that can report
 * instantaneous power (W) and accumulated energy (Wh)
 */

#ifndef BLE_POWERMETER_H
#define BLE_POWERMETER_H

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"

/**@brief Battery Service event type. */
typedef enum
{
    BLE_POWERMETER_EVT_NOTIFICATION_ENABLED,                             /**< Powermeter value notification enabled event. */
    BLE_POWERMETER_EVT_NOTIFICATION_DISABLED                             /**< Powermeter value notification disabled event. */
} ble_powermeter_evt_type_t;

/**@brief Battery Service event. */
typedef struct
{
    ble_powermeter_evt_type_t evt_type;                                  /**< Type of event. */
} ble_powermeter_evt_t;

typedef struct ble_powermeter_s *ble_powermeter_p;

/**@brief Battery Service event handler type. */
typedef void (*ble_powermeter_evt_handler_t) (ble_powermeter_p powermeter,
                                              ble_powermeter_evt_t * p_evt);

typedef struct
{
    ble_powermeter_evt_handler_t  evt_handler;                    /**< Event handler to be called for handling events in the Battery Service. */
    bool                          support_notification;           /**< TRUE if notification of Battery Level measurement is supported. */
    ble_srv_report_ref_t *        p_report_ref;                   /**< If not NULL, a Report Reference descriptor with the specified value will be added to the Battery Level characteristic */
    // uint16_t                      initial_power;                  /**< Initial battery level */
    ble_srv_cccd_security_mode_t  power_char_attr_md;     /**< Initial security level for battery characteristics attribute */
    ble_gap_conn_sec_mode_t       power_report_read_perm; /**< Initial security level for battery report read attribute */
    ble_srv_cccd_security_mode_t  energy_char_attr_md;     /**< Initial security level for battery characteristics attribute */
    ble_gap_conn_sec_mode_t       energy_report_read_perm; /**< Initial security level for battery report read attribute */
} ble_powermeter_init_t;

/**@brief Battery Service structure. This contains various status information for the service. */
typedef struct ble_powermeter_s
{
    ble_powermeter_evt_handler_t  evt_handler;                    /**< Event handler to be called for handling events in the Battery Service. */
    uint16_t                      service_handle;                 /**< Handle of Battery Service (as provided by the BLE stack). */
    ble_gatts_char_handles_t      power_handles;          /**< Handles related to the Battery Level characteristic. */
    ble_gatts_char_handles_t      energy_handles;          /**< Handles related to the Battery Level characteristic. */
    uint16_t                      power_report_ref_handle;              /**< Handle of the Report Reference descriptor. */
    uint16_t                      energy_report_ref_handle;              /**< Handle of the Report Reference descriptor. */
    uint16_t                      power_last_w;             /**< Last Battery Level measurement passed to the Battery Service. */
    uint32_t                      energy_last_wh;             /**< Last Battery Level measurement passed to the Battery Service. */
    uint16_t                      conn_handle;                    /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection). */
    bool                          is_notification_supported;      /**< TRUE if notification of Battery Level is supported. */
    uint8_t                       uuid_type;
} ble_powermeter_t;

uint32_t ble_powermeter_init(ble_powermeter_t * p_pm,
                             const ble_powermeter_init_t * p_pm_init,
                             const ble_uuid128_t *baseUUID );

uint32_t ble_powermeter_update( ble_powermeter_t * p_pm, uint16_t power_w,
                                uint32_t energy_wh );

#endif
