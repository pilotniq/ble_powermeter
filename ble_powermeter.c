#include "sdk_common.h"
#include "softdevice_handler.h"

#include "ble_powermeter.h"
#include <string.h>
#include "ble_srv_common.h"
#include "app_error.h"

#include "erl_ble_adv.h"
#include "erl_ble.h"

#define INVALID_POWER UINT16_MAX
#define INVALID_ENERGY 0

static uint32_t char_add( ble_powermeter_t * p_pm,
                          const ble_powermeter_init_t * p_pm_init,
                          const ble_srv_cccd_security_mode_t *char_attr_md,
                          uint16_t uuid, uint16_t report_uuid,
                          void *initial_value, uint8_t size,
                          ble_gatts_char_handles_t *handles,
                          uint16_t *report_ref_handle)
{
  uint32_t            err_code;
  ble_gatts_char_md_t char_md;
  ble_gatts_attr_md_t cccd_md;
  ble_gatts_attr_t    attr_char_value;
  ble_uuid_t          ble_uuid;
  ble_gatts_attr_md_t attr_md;
  // uint16_t            initial_power;
  uint8_t             encoded_report_ref[BLE_SRV_ENCODED_REPORT_REF_LEN];
  uint8_t             init_len;

  // Add Battery Level characteristic
  if (p_pm->is_notification_supported)
  {
      memset(&cccd_md, 0, sizeof(cccd_md));

      // According to BAS_SPEC_V10, the read operation on cccd should be possible without
      // authentication.
      BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
      cccd_md.write_perm = char_attr_md->cccd_write_perm;
      cccd_md.vloc       = BLE_GATTS_VLOC_STACK;
  }

  memset(&char_md, 0, sizeof(char_md));

  char_md.char_props.read   = 1;
  char_md.char_props.notify = (p_pm->is_notification_supported) ? 1 : 0;
  char_md.p_char_user_desc  = NULL;
  char_md.p_char_pf         = NULL;
  char_md.p_user_desc_md    = NULL;
  char_md.p_cccd_md         = (p_pm->is_notification_supported) ? &cccd_md : NULL;
  char_md.p_sccd_md         = NULL;

  ble_uuid.type = p_pm->uuid_type;
  ble_uuid.uuid = uuid;

  // BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_BATTERY_LEVEL_CHAR);

  memset(&attr_md, 0, sizeof(attr_md));

  attr_md.read_perm  = char_attr_md->read_perm;
  attr_md.write_perm = char_attr_md->write_perm;
  attr_md.vloc       = BLE_GATTS_VLOC_STACK;
  attr_md.rd_auth    = 0;
  attr_md.wr_auth    = 0;
  attr_md.vlen       = 0;

  memset(&attr_char_value, 0, sizeof(attr_char_value));

  attr_char_value.p_uuid    = &ble_uuid;
  attr_char_value.p_attr_md = &attr_md;
  attr_char_value.init_len  = size;
  attr_char_value.init_offs = 0;
  attr_char_value.max_len   = size;
  attr_char_value.p_value   = initial_value;

  err_code = sd_ble_gatts_characteristic_add(p_pm->service_handle, &char_md,
                                             &attr_char_value,
                                             handles);
  if (err_code != NRF_SUCCESS)
      return err_code;

  if (p_pm_init->p_report_ref != NULL)
  {
      // Add Report Reference descriptor
      // BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_REPORT_REF_DESCR);
      ble_uuid.uuid = report_uuid;

      memset(&attr_md, 0, sizeof(attr_md));

      attr_md.read_perm = p_pm_init->power_report_read_perm;
      BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);

      attr_md.vloc    = BLE_GATTS_VLOC_STACK;
      attr_md.rd_auth = 0;
      attr_md.wr_auth = 0;
      attr_md.vlen    = 0;

      init_len = ble_srv_report_ref_encode(encoded_report_ref, p_pm_init->p_report_ref);

      memset(&attr_char_value, 0, sizeof(attr_char_value));

      attr_char_value.p_uuid    = &ble_uuid;
      attr_char_value.p_attr_md = &attr_md;
      attr_char_value.init_len  = init_len;
      attr_char_value.init_offs = 0;
      attr_char_value.max_len   = attr_char_value.init_len;
      attr_char_value.p_value   = encoded_report_ref;

      // call below crashes
      err_code = sd_ble_gatts_descriptor_add(handles->value_handle,
                                             &attr_char_value,
                                             report_ref_handle);
      if (err_code != NRF_SUCCESS)
      {
          return err_code;
      }
  }
  else
  {
      *report_ref_handle = BLE_GATT_HANDLE_INVALID;
  }

  return NRF_SUCCESS;
}
/**@brief Function for adding the Battery Level characteristic.
 *
 * @param[in]   p_bas        Battery Service structure.
 * @param[in]   p_bas_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t power_char_add( ble_powermeter_t * p_pm,
                                const ble_powermeter_init_t * p_pm_init)
{
  p_pm->power_last_w = INVALID_POWER; // p_bas_init->initial_power;

  return char_add( p_pm, p_pm_init, &(p_pm_init->power_char_attr_md), 0x0002, 0x0003,
              &(p_pm->power_last_w), sizeof( uint16_t ), &(p_pm->power_handles),
              &(p_pm->power_report_ref_handle) );
}

static uint32_t energy_char_add( ble_powermeter_t * p_pm,
                                 const ble_powermeter_init_t * p_pm_init )
{
  p_pm->energy_last_wh = INVALID_ENERGY; // p_bas_init->initial_power;

  return char_add( p_pm, p_pm_init, &(p_pm_init->energy_char_attr_md), 0x0004, 0x0005,
              &(p_pm->energy_last_wh), sizeof( uint32_t ), &(p_pm->energy_handles),
              &(p_pm->energy_report_ref_handle) );
}

/**@brief Function for handling the Connect event.
 *
 * @param[in]   p_bas       Battery Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_connect(ble_powermeter_t *pm, ble_evt_t *evt)
{
    pm->conn_handle = evt->evt.gap_evt.conn_handle;
}


/**@brief Function for handling the Disconnect event.
 *
 * @param[in]   p_bas       Battery Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_disconnect(ble_powermeter_t *pm, ble_evt_t *evt)
{
    UNUSED_PARAMETER(evt);
    pm->conn_handle = BLE_CONN_HANDLE_INVALID;
}


/**@brief Function for handling the Write event.
 *
 * @param[in]   p_bas       Battery Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_write(ble_powermeter_t *pm, ble_evt_t *evt)
{
  #if 0 // write Not Suppoerted
    if (pm->is_notification_supported)
    {
        ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

        if (
            (p_evt_write->handle == p_bas->battery_level_handles.cccd_handle)
            &&
            (p_evt_write->len == 2)
           )
        {
            // CCCD written, call application event handler
            if (p_bas->evt_handler != NULL)
            {
                ble_bas_evt_t evt;

                if (ble_srv_is_notification_enabled(p_evt_write->data))
                {
                    evt.evt_type = BLE_BAS_EVT_NOTIFICATION_ENABLED;
                }
                else
                {
                    evt.evt_type = BLE_BAS_EVT_NOTIFICATION_DISABLED;
                }

                p_bas->evt_handler(p_bas, &evt);
            }
        }
    }
#endif
}


static void on_ble_evt(ble_evt_t *evt, void *context)
{
  ble_powermeter_t *pm = (ble_powermeter_t *) context;

    if (pm == NULL || evt == NULL)
        return;

    switch( evt->header.evt_id )
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(pm, evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(pm, evt);
            break;

        case BLE_GATTS_EVT_WRITE:
            on_write(pm, evt);
            break;

        default:
            // No implementation needed.
            break;
    }
}

uint32_t ble_powermeter_init(ble_powermeter_t * p_pm,
                             const ble_powermeter_init_t * p_pm_init,
                             const ble_uuid128_t *baseUUID)
{
    if (p_pm == NULL || p_pm_init == NULL)
    {
        return NRF_ERROR_NULL;
    }

    uint32_t   err_code;
    ble_uuid_t ble_uuid;
    // uint8_t baseUUIDtype;

    // randomly generated UUID: 89a4c6c1-1f25-46bb-91c4-482542d8b233

    // Initialize service structure
    p_pm->evt_handler               = p_pm_init->evt_handler;
    p_pm->conn_handle               = BLE_CONN_HANDLE_INVALID;
    p_pm->is_notification_supported = p_pm_init->support_notification;
    p_pm->power_last_w               = INVALID_POWER;
    p_pm->energy_last_wh             = INVALID_ENERGY;

    err_code = sd_ble_uuid_vs_add(baseUUID, &(p_pm->uuid_type) );
    APP_ERROR_CHECK(err_code);

    ble_uuid.type = p_pm->uuid_type;
    ble_uuid.uuid = 0x0001; // first use of this sub uuid

    // Add service
    // BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_BATTERY_SERVICE);

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid,
                                        &p_pm->service_handle);
    if (err_code != NRF_SUCCESS)
        return err_code;

    err_code = erl_ble_adv_add_uuid( &ble_uuid );
    if (err_code != NRF_SUCCESS)
        return err_code;

    // Add battery level characteristic
    err_code = power_char_add(p_pm, p_pm_init);
    if (err_code != NRF_SUCCESS)
        return err_code;

    // Add energy characteristic
    err_code = energy_char_add(p_pm, p_pm_init);
    if (err_code != NRF_SUCCESS)
        return err_code;

  err_code = erl_ble_add_ble_evt_func( on_ble_evt, p_pm );
  return err_code;
}

static uint32_t update_param( ble_powermeter_t * p_pm, int value_size,
                              void *new_value_ptr, void *last_value_ptr,
                              ble_gatts_char_handles_t *handles)
{
  uint32_t err_code = NRF_SUCCESS;
  ble_gatts_value_t gatts_value;

  if (p_pm == NULL)
      return NRF_ERROR_NULL;

  if( memcmp( new_value_ptr, last_value_ptr, value_size ) != 0 )
  {
      // Initialize value struct.
      memset(&gatts_value, 0, sizeof(gatts_value));

      gatts_value.len     = value_size;
      gatts_value.offset  = 0;
      gatts_value.p_value = new_value_ptr;

      // Update database.
      err_code = sd_ble_gatts_value_set(p_pm->conn_handle,
                                        handles->value_handle,
                                        &gatts_value);
      // Save new battery value.
      if (err_code == NRF_SUCCESS)
          memcpy( last_value_ptr, new_value_ptr, value_size );
      else
          return err_code;

      // Send value if connected and notifying.
      if ((p_pm->conn_handle != BLE_CONN_HANDLE_INVALID) && p_pm->is_notification_supported)
      {
          ble_gatts_hvx_params_t hvx_params;

          memset(&hvx_params, 0, sizeof(hvx_params));

          hvx_params.handle = handles->value_handle;
          hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
          hvx_params.offset = gatts_value.offset;
          hvx_params.p_len  = &gatts_value.len;
          hvx_params.p_data = gatts_value.p_value;

          err_code = sd_ble_gatts_hvx(p_pm->conn_handle, &hvx_params);
      }
      else
          err_code = NRF_ERROR_INVALID_STATE;
  }

  return err_code;
}

uint32_t ble_powermeter_update(ble_powermeter_t * p_pm, uint16_t power_w,
                               uint32_t energy_wh)
{
  uint32_t err_code;
  // uint16_t encoded_power_w;
  // uint32_t encoded_energy_wh;

  if (p_pm == NULL)
    return NRF_ERROR_NULL;

  // uint16_encode( power_w, (uint8_t *) &encoded_power_w );

  err_code = update_param( p_pm, sizeof( uint16_t ), &power_w,
                           &(p_pm->power_last_w), &(p_pm->power_handles));
  if( err_code != NRF_SUCCESS )
    return err_code;

  // uint32_encode( energy_wh, (uint8_t *) &encoded_energy_wh );

  err_code = update_param( p_pm, sizeof( uint32_t ), &energy_wh,
                           &(p_pm->energy_last_wh), &(p_pm->energy_handles));
  return err_code;
}
